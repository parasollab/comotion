#include "comotion/planning/CompositeAORRTC.h"

#include <nlohmann/json.hpp>

namespace comotion {

namespace ob = ompl::base;

ompl::base::PlannerStatus CompositeAORRTC::solve(double timeLimit) {
    resetPlannerRunMetrics();
    solution_paths_.clear();
    solution_events_.clear();

    aorrtc::SolveOptions options;
    options.use_makespan_metric = true;
    options.simplify_solution = simplify_solution_;
    options.planning_seed = planning_seed_;
    if (max_internal_samples_ > 0)
        options.max_internal_samples = max_internal_samples_;
    if (max_internal_vertices_ > 0)
        options.max_internal_vertices = max_internal_vertices_;
    options.solution_event_callback = [this](const aorrtc::SolutionEvent &event) {
        solution_events_.push_back(event);
    };

    auto result = aorrtc::solveCompositeAnytime(*problem_, robot_indices_,
                                                timeLimit, options);
    solution_paths_ = std::move(result.paths);
    if (solution_events_.empty())
        solution_events_ = std::move(result.solution_events);
    if (!solution_paths_.empty())
        setSolutionMetricsFromPaths(solution_paths_);

    nlohmann::json stats = nlohmann::json::object();
    stats["solution_events"] = solutionEventsJson(solution_events_);
    stats["num_solution_events"] = solution_events_.size();
    stats["best_ompl_cost"] = result.best_ompl_cost;
    stats["max_internal_samples"] = max_internal_samples_;
    stats["max_internal_vertices"] = max_internal_vertices_;
    setPlannerStatsJson(std::move(stats));

    return result.status;
}

std::vector<Path> CompositeAORRTC::getSolutionPaths() const {
    return solution_paths_;
}

nlohmann::json CompositeAORRTC::solutionEventsJson(
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
