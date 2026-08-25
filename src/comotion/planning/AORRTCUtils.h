#pragma once

#include "comotion/planning/MultiRobotProblem.h"
#include "comotion/planning/PathSimplification.h"

#include <ompl/base/PlannerStatus.h>

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace comotion {
namespace aorrtc {

/// Experimental/advanced API shared by AOARC and CompositeAORRTC.
struct SolutionEvent {
    double elapsed_seconds = 0.0;
    double ompl_cost = 0.0;
    std::uint64_t sum_of_cost_timesteps = 0;
    std::uint64_t makespan_timesteps = 0;
    std::string kind;
};

struct SolveOptions {
    bool use_makespan_metric = true;
    bool simplify_solution = true;
    std::optional<std::uint32_t> planning_seed;
    std::optional<std::uint64_t> cost_bound_timesteps;
    std::optional<std::size_t> max_internal_samples;
    std::optional<std::size_t> max_internal_vertices;
    PathSimplificationOptions simplification_options{};
    std::function<void(const SolutionEvent &)> solution_event_callback;
};

struct SolveResult {
    ompl::base::PlannerStatus status{ompl::base::PlannerStatus::TIMEOUT};
    std::vector<Path> paths;
    double best_ompl_cost = 0.0;
    double simplify_seconds = 0.0;
    std::vector<SolutionEvent> solution_events;
};

double timestepsToOmplCost(std::uint64_t timesteps,
                           const MultiRobotProblem &problem);
std::uint64_t omplCostToTimesteps(double cost,
                                  const MultiRobotProblem &problem);

SolveResult solveSingleRobotBounded(
    const MultiRobotProblem &problem, int robot_index, double time_limit,
    const SolveOptions &options = {});

SolveResult solveCompositeBounded(
    const MultiRobotProblem &problem, const std::vector<int> &robot_indices,
    double time_limit, const SolveOptions &options = {});

SolveResult solveCompositeAnytime(
    const MultiRobotProblem &problem, const std::vector<int> &robot_indices,
    double time_limit, const SolveOptions &options = {});

} // namespace aorrtc
} // namespace comotion
