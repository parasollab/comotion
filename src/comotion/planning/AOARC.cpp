#include "comotion/planning/AOARC.h"
#include "comotion/planning/PlanningSeed.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

namespace comotion {

namespace {
using Clock = std::chrono::steady_clock;

} // namespace

std::uint64_t AOARC::boundedInitialPathReuseBoundTimesteps(
    std::uint64_t incumbent_makespan, std::uint64_t epsilon_timesteps) {
    if (incumbent_makespan == 0)
        return 0;
    // Makespan is stored in integer arrival timesteps, so even epsilon=0
    // requires one tick of tightening to represent Algorithm 1's strict < B.
    const std::uint64_t reduction =
        std::max<std::uint64_t>(1, epsilon_timesteps);
    return reduction >= incumbent_makespan ? 0
                                           : incumbent_makespan - reduction;
}

void AOARC::setRandomFullRestartProbability(double probability) {
    if (!std::isfinite(probability) || probability < 0.0 ||
        probability > 1.0) {
        throw std::invalid_argument(
            "AO-ARC random full-restart probability must be finite and in "
            "[0, 1]");
    }
    random_full_restart_probability_ = probability;
}

bool AOARC::randomFullRestartForAttempt(std::uint32_t planning_seed,
                                        std::uint64_t attempt_index,
                                        double probability) {
    if (probability <= 0.0)
        return false;
    if (probability >= 1.0)
        return true;
    constexpr long double kDrawRange =
        static_cast<long double>(
            std::numeric_limits<std::uint32_t>::max()) +
        1.0L;
    const auto draw =
        aoArcRandomRestartDecisionWord(planning_seed, attempt_index);
    return static_cast<long double>(draw) <
           static_cast<long double>(probability) * kDrawRange;
}

std::vector<int> AOARC::repairPartnerRobotsWithinDepth(
    const std::vector<int> &original_bound_violators,
    const std::vector<AppliedRepairHistoryEvent> &repair_history,
    std::size_t depth) {
    if (depth == 0 || original_bound_violators.empty())
        return {};

    const std::set<int> original_robots(original_bound_violators.begin(),
                                        original_bound_violators.end());
    std::set<int> reached = original_robots;
    std::set<int> frontier = original_robots;
    for (std::size_t layer = 0; layer < depth && !frontier.empty(); ++layer) {
        std::set<int> next_frontier;
        for (const auto &event : repair_history) {
            const bool touches_frontier =
                std::any_of(event.robots.begin(), event.robots.end(),
                            [&](int robot) {
                                return frontier.count(robot) != 0;
                            });
            if (!touches_frontier)
                continue;
            for (const int robot : event.robots) {
                if (reached.insert(robot).second)
                    next_frontier.insert(robot);
            }
        }
        frontier = std::move(next_frontier);
    }

    std::set<int> expanded_robots = std::move(reached);
    for (const int robot : original_robots)
        expanded_robots.erase(robot);
    return {expanded_robots.begin(), expanded_robots.end()};
}

void AOARC::incorporateAcceptedRepairHistory(
    std::vector<AppliedRepairHistoryEvent> &incumbent_history,
    const std::vector<AppliedRepairHistoryEvent> &accepted_history,
    bool incumbent_paths_retained) {
    if (incumbent_paths_retained) {
        incumbent_history.insert(incumbent_history.end(),
                                 accepted_history.begin(),
                                 accepted_history.end());
    } else {
        incumbent_history = accepted_history;
    }
}

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
        stats["initial_attempt_seed"] = planning_seed_;
        stats["bounded_attempt_seeds"] = nlohmann::json::array();
        stats["selective_bounded_replanning"] =
            selective_bounded_replanning_;
        stats["selective_initial_conflict_scan"] =
            selective_initial_conflict_scan_;
        stats["expand_replanning_from_repair_history"] =
            expandReplanningFromRepairHistory();
        stats["repair_history_replanning_depth"] =
            repair_history_replanning_depth_;
        stats["random_full_restart_probability"] =
            random_full_restart_probability_;
        stats["num_random_full_restarts"] = 0;
        stats["num_random_full_restart_improvements"] = 0;
        stats["total_history_expanded_replanning_robots"] = 0;
        stats["total_repair_history_expanded_paths"] = 0;
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
    std::vector<std::uint32_t> bounded_attempt_seeds;
    nlohmann::json bounded_attempt_summaries = nlohmann::json::array();
    std::uint64_t total_paths_replanned = 0;
    std::uint64_t total_paths_reused = 0;
    std::uint64_t total_conflict_pairs_skipped = 0;
    std::uint64_t total_history_expanded_replanning_robots = 0;
    std::uint64_t num_random_full_restarts = 0;
    std::uint64_t num_random_full_restart_improvements = 0;
    std::vector<AppliedRepairHistoryEvent> best_repair_history =
        first.appliedRepairHistoryEvents();
    const auto repairPartnerEdgeCount = [](const auto &history) {
        std::set<std::pair<int, int>> edges;
        for (const auto &event : history) {
            for (std::size_t i = 0; i < event.robots.size(); ++i) {
                for (std::size_t j = i + 1; j < event.robots.size(); ++j) {
                    edges.emplace(std::min(event.robots[i], event.robots[j]),
                                  std::max(event.robots[i], event.robots[j]));
                }
            }
        }
        return edges.size();
    };
    std::string stop_reason = "time_budget_exhausted";
    constexpr double kAttemptSafetyMarginSeconds = 0.05;

    while (true) {
        if (cancellationRequested()) {
            stop_reason = "cancelled";
            break;
        }
        if (best_makespan == 0) {
            stop_reason = "zero_makespan_optimum";
            break;
        }
        const double elapsed =
            std::chrono::duration<double>(Clock::now() - start).count();
        const double remaining = timeLimit - elapsed;
        if (remaining <= kAttemptSafetyMarginSeconds)
            break;

        ARC attempt;
        configureArcAttempt(attempt);
        const auto attempt_seed =
            aoArcBoundedAttemptPlanningSeed(planning_seed_, num_attempts);
        attempt.setPlanningSeed(attempt_seed);
        attempt.setGlobalMakespanBoundTimesteps(best_makespan);
        const auto random_restart_decision_word =
            aoArcRandomRestartDecisionWord(planning_seed_, num_attempts);
        const bool random_full_restart =
            selective_bounded_replanning_ &&
            randomFullRestartForAttempt(planning_seed_, num_attempts,
                                        random_full_restart_probability_);
        const std::uint64_t reuse_bound =
            boundedInitialPathReuseBoundTimesteps(
                best_makespan,
                bounded_local_repair_epsilon_timesteps_);
        std::vector<int> original_bound_violators;
        for (std::size_t i = 0; i < best_paths.size(); ++i) {
            if (!best_paths[i].empty() &&
                best_paths[i].arrival_timestep() > reuse_bound) {
                original_bound_violators.push_back(static_cast<int>(i));
            }
        }
        std::vector<int> history_expanded_replanning_robots;
        if (selective_bounded_replanning_ && !random_full_restart) {
            attempt.setBoundedInitialPaths(best_paths);
            attempt.setBoundedInitialPathReuseBoundTimesteps(reuse_bound);
            if (repair_history_replanning_depth_ != 0) {
                history_expanded_replanning_robots =
                    repairPartnerRobotsWithinDepth(
                        original_bound_violators, best_repair_history,
                        repair_history_replanning_depth_);
                attempt.setBoundedInitialForcedReplanningRobots(
                    history_expanded_replanning_robots);
            }
        }
        attempt.setSelectiveInitialConflictScan(
            selective_initial_conflict_scan_);
        const double remaining_after_setup =
            timeLimit -
            std::chrono::duration<double>(Clock::now() - start).count();
        if (remaining_after_setup <= kAttemptSafetyMarginSeconds) {
            stop_reason = "time_budget_exhausted_before_bounded_attempt";
            break;
        }
        const double attempt_budget =
            remaining_after_setup - kAttemptSafetyMarginSeconds;
        bounded_attempt_seeds.push_back(attempt_seed);
        const auto status = attempt.solve(attempt_budget);
        ++num_attempts;
        if (random_full_restart)
            ++num_random_full_restarts;

        const auto &attempt_stats = attempt.plannerStatsJson();
        const auto initialization =
            attempt_stats.value("bounded_initialization",
                                nlohmann::json::object());
        const std::uint64_t paths_replanned =
            initialization.value("num_paths_replanned", 0ULL);
        const std::uint64_t paths_reused =
            initialization.value("num_paths_reused", 0ULL);
        const std::uint64_t conflict_pairs_skipped =
            initialization.value("num_conflict_pairs_skipped", 0ULL);
        total_paths_replanned += paths_replanned;
        total_paths_reused += paths_reused;
        total_conflict_pairs_skipped += conflict_pairs_skipped;
        total_history_expanded_replanning_robots +=
            history_expanded_replanning_robots.size();
        nlohmann::json attempt_summary = {
            {"seed", attempt_seed},
            {"status", status.asString()},
            {"random_full_restart", random_full_restart},
            {"random_full_restart_decision_word",
             random_restart_decision_word},
            {"random_full_restart_probability",
             random_full_restart_probability_},
            {"repair_history_replanning_depth",
             repair_history_replanning_depth_},
            {"selective_replanning_applied",
             selective_bounded_replanning_ && !random_full_restart},
            {"improved", false},
            {"incumbent_bound_timesteps", best_makespan},
            {"initial_path_reuse_bound_timesteps", reuse_bound},
            {"num_paths_replanned", paths_replanned},
            {"num_paths_reused", paths_reused},
            {"num_conflict_pairs_skipped", conflict_pairs_skipped},
            {"bound_violating_robot_indices", original_bound_violators},
            {"num_bound_violating_robots", original_bound_violators.size()},
            {"repair_history_expanded_robot_indices",
             history_expanded_replanning_robots},
            {"num_repair_history_expanded_robots",
             history_expanded_replanning_robots.size()},
            {"incumbent_repair_history_event_count",
             best_repair_history.size()},
            {"incumbent_repair_partner_edge_count",
             repairPartnerEdgeCount(best_repair_history)},
            {"num_conflicts", attempt_stats.value("num_conflicts", 0ULL)},
            {"conflict_detection_time_seconds",
             attempt_stats.value(
                 "conflict_detection_times_seconds_wall_clock", 0.0)},
        };

        if (status == ompl::base::PlannerStatus::EXACT_SOLUTION &&
            attempt.makespanTimesteps() && attempt.sumOfCostTimesteps()) {
            const auto candidate_makespan = *attempt.makespanTimesteps();
            const auto candidate_sum = *attempt.sumOfCostTimesteps();
            attempt_summary["candidate_makespan_timesteps"] =
                candidate_makespan;
            attempt_summary["candidate_sum_of_cost_timesteps"] = candidate_sum;
            const bool improved = candidate_makespan < best_makespan;
            attempt_summary["improved"] = improved;
            if (improved) {
                best_makespan = candidate_makespan;
                best_sum = candidate_sum;
                best_paths = attempt.getSolutionPaths();
                const auto &accepted_repair_events =
                    attempt.appliedRepairHistoryEvents();
                const bool incumbent_paths_retained =
                    selective_bounded_replanning_ &&
                    !random_full_restart;
                incorporateAcceptedRepairHistory(
                    best_repair_history, accepted_repair_events,
                    incumbent_paths_retained);
                attempt_summary["repair_history_reset"] =
                    !incumbent_paths_retained;
                if (random_full_restart)
                    ++num_random_full_restart_improvements;
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
        bounded_attempt_summaries.push_back(std::move(attempt_summary));
    }

    solution_paths_ = std::move(best_paths);
    setSolutionMetrics(best_sum, best_makespan);

    nlohmann::json stats = nlohmann::json::object();
    stats["solution_events"] = solutionEventsJson(events);
    stats["num_solution_events"] = events.size();
    stats["num_bounded_attempts"] = num_attempts;
    stats["num_improvements"] = num_improvements;
    stats["num_rejections"] = num_rejections;
    stats["initial_attempt_seed"] = planning_seed_;
    stats["bounded_attempt_seeds"] = bounded_attempt_seeds;
    stats["bounded_attempts"] = std::move(bounded_attempt_summaries);
    stats["selective_bounded_replanning"] =
        selective_bounded_replanning_;
    stats["selective_initial_conflict_scan"] =
        selective_initial_conflict_scan_;
    stats["expand_replanning_from_repair_history"] =
        expandReplanningFromRepairHistory();
    stats["repair_history_replanning_depth"] =
        repair_history_replanning_depth_;
    stats["random_full_restart_probability"] =
        random_full_restart_probability_;
    stats["num_random_full_restarts"] = num_random_full_restarts;
    stats["num_random_full_restart_improvements"] =
        num_random_full_restart_improvements;
    stats["initial_path_reuse_bound_reduction_timesteps"] =
        std::max<std::uint64_t>(1,
                                bounded_local_repair_epsilon_timesteps_);
    stats["total_paths_replanned"] = total_paths_replanned;
    stats["total_paths_reused"] = total_paths_reused;
    stats["total_initial_conflict_pairs_skipped"] =
        total_conflict_pairs_skipped;
    stats["total_history_expanded_replanning_robots"] =
        total_history_expanded_replanning_robots;
    stats["total_repair_history_expanded_paths"] =
        total_history_expanded_replanning_robots;
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
