#include "comotion/planning/AOARC.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>

namespace comotion {

namespace {
using Clock = std::chrono::steady_clock;

} // namespace

void AOARC::configureArcAttempt(ARC &planner) const {
    planner.setInitialWindow(initial_window_);
    planner.setExpansionStep(expansion_step_);
    planner.setExpansionPolicy(expansion_policy_);
    planner.setCustomExpansionMultipliers(custom_expansion_multipliers_);
    if (initial_valid_window_expansion_policy_) {
        planner.setInitialValidWindowExpansionPolicy(
            *initial_valid_window_expansion_policy_);
    }
    if (initial_valid_window_expansion_step_) {
        planner.setInitialValidWindowExpansionStep(
            *initial_valid_window_expansion_step_);
    }
    if (initial_valid_window_expansion_multipliers_) {
        planner.setInitialValidWindowExpansionMultipliers(
            *initial_valid_window_expansion_multipliers_);
    }
    planner.setInitialValidWindowExpansionSymmetric(
        initial_valid_window_expansion_symmetric_);
    planner.setLocalCompositeRrtMaxSamples(local_composite_rrt_max_samples_);
    if (local_composite_rrt_range_)
        planner.setLocalCompositeRrtRange(*local_composite_rrt_range_);
    planner.setLocalCompositeRrtUseMakespanMetric(true);
    planner.setLocalSolverMode(local_solver_mode_);
    planner.setLocalPrioritizedStrrtMaxIterations(
        local_prioritized_strrt_max_iterations_);
    planner.setLocalPrioritizedStrrtReturnFirstSolution(
        local_prioritized_strrt_return_first_solution_);
    planner.setLocalPrioritizedStrrtRewiring(
        local_prioritized_strrt_rewiring_);
    planner.setLocalPrioritizedStrrtPersistAtGoal(
        local_prioritized_strrt_persist_at_goal_);
    planner.setBoundedLocalRepairEpsilonTimesteps(
        bounded_local_repair_epsilon_timesteps_);
    planner.setUseCspaceBounds(use_cspace_bounds_);
    planner.setCspaceBoundMargin(cspace_bound_margin_);
    planner.setMinCspaceBoundRange(min_cspace_bound_range_);
    planner.setStrrtSpaceTimeSpanFactor(strrt_space_time_span_factor_);
    planner.setPathSimplificationOptions(simplification_options_);
    if (conflict_simplification_options_) {
        planner.setConflictPathSimplificationOptions(
            *conflict_simplification_options_);
    }
    planner.setSimplifyInitialSolutions(simplify_initial_solutions_);
    planner.setSimplifyConflictSolutions(simplify_conflict_solutions_);
    planner.setVisualizationTraceEnabled(visualization_trace_enabled_);
    planner.setPlanningSeed(planning_seed_);
    planner.setCancellationCallback(cancel_requested_);
    planner.setProblem(problem_);
}

ompl::base::PlannerStatus AOARC::solve(double timeLimit) {
    resetPlannerRunMetrics();
    solution_paths_.clear();

    const auto start = Clock::now();
    std::vector<aorrtc::SolutionEvent> events;

    ARC first;
    configureArcAttempt(first);
    first.clearGlobalMakespanBoundTimesteps();
    const auto first_status = first.solve(timeLimit);
    replaceVisualizationTrace(first.visualizationTrace());
    const double first_elapsed =
        std::chrono::duration<double>(Clock::now() - start).count();
    if (first_status != ompl::base::PlannerStatus::EXACT_SOLUTION) {
        nlohmann::json stats = nlohmann::json::object();
        stats["solution_events"] = nlohmann::json::array();
        stats["first_solution_status"] = first_status.asString();
        setPlannerStatsJson(std::move(stats));
        return first_status;
    }

    solution_paths_ = first.getSolutionPaths();
    const auto fallback_metrics =
        MultiRobotPlanner::computeSolutionMetrics(solution_paths_);
    const std::uint64_t first_sum =
        first.sumOfCostTimesteps() ? *first.sumOfCostTimesteps()
                                   : fallback_metrics.first;
    const std::uint64_t first_makespan =
        first.makespanTimesteps() ? *first.makespanTimesteps()
                                  : fallback_metrics.second;
    setSolutionMetrics(first_sum, first_makespan);

    events.push_back({first_elapsed,
                      aorrtc::timestepsToOmplCost(first_makespan, *problem_),
                      first_sum,
                      first_makespan,
                      "first_solution"});

    std::uint64_t best_sum = first_sum;
    std::uint64_t best_makespan = first_makespan;
    std::vector<Path> best_paths = solution_paths_;
    std::uint64_t num_attempts = 0;
    std::uint64_t num_improvements = 0;
    std::uint64_t num_rejections = 0;
    std::string stop_reason = "time_budget_exhausted";
    constexpr double kAttemptSafetyMarginSeconds = 0.05;

    while (true) {
        const double elapsed =
            std::chrono::duration<double>(Clock::now() - start).count();
        const double remaining = timeLimit - elapsed;
        if (remaining <= kAttemptSafetyMarginSeconds)
            break;
        const double attempt_budget = remaining - kAttemptSafetyMarginSeconds;

        ARC attempt;
        configureArcAttempt(attempt);
        attempt.setGlobalMakespanBoundTimesteps(best_makespan);
        const auto status = attempt.solve(attempt_budget);
        ++num_attempts;

        if (status == ompl::base::PlannerStatus::EXACT_SOLUTION &&
            attempt.makespanTimesteps() && attempt.sumOfCostTimesteps()) {
            const auto candidate_makespan = *attempt.makespanTimesteps();
            const auto candidate_sum = *attempt.sumOfCostTimesteps();
            const bool improved = candidate_makespan < best_makespan;
            if (improved) {
                best_makespan = candidate_makespan;
                best_sum = candidate_sum;
                best_paths = attempt.getSolutionPaths();
                replaceVisualizationTrace(attempt.visualizationTrace());
                ++num_improvements;
                events.push_back(
                    {std::chrono::duration<double>(Clock::now() - start)
                         .count(),
                     aorrtc::timestepsToOmplCost(best_makespan, *problem_),
                     best_sum,
                     best_makespan,
                     "ao_improvement"});
            } else {
                ++num_rejections;
            }
        } else {
            ++num_rejections;
        }
    }

    solution_paths_ = std::move(best_paths);
    setSolutionMetrics(best_sum, best_makespan);

    nlohmann::json stats = nlohmann::json::object();
    stats["solution_events"] = solutionEventsJson(events);
    stats["num_solution_events"] = events.size();
    stats["num_bounded_attempts"] = num_attempts;
    stats["num_improvements"] = num_improvements;
    stats["num_rejections"] = num_rejections;
    stats["ao_stop_reason"] = stop_reason;
    setPlannerStatsJson(std::move(stats));

    return ompl::base::PlannerStatus::EXACT_SOLUTION;
}

nlohmann::json AOARC::solutionEventsJson(
    const std::vector<aorrtc::SolutionEvent> &events) {
    nlohmann::json out = nlohmann::json::array();
    for (const auto &event : events) {
        out.push_back({
            {"elapsed_seconds", event.elapsed_seconds},
            {"ompl_cost", event.ompl_cost},
            {"sum_of_cost_timesteps", event.sum_of_cost_timesteps},
            {"makespan_timesteps", event.makespan_timesteps},
            {"kind", event.kind},
        });
    }
    return out;
}

} // namespace comotion
