#pragma once

#include "comotion/planning/MultiRobotPlanner.h"
#include "comotion/planning/PathSimplification.h"
#include <vector>

namespace comotion {

/// OMPL ST-RRT* rewiring mode for each per-robot subproblem.
enum class StrrtRewiring { Off, Radius, KNearest };

/// Baseline planner API.
///
/// Prioritized planning with ST-RRT*: plans for robots sequentially, treating
/// each previously planned robot's path as a dynamic obstacle.
class PrioritizedSTRRT : public MultiRobotPlanner {
public:
    ompl::base::PlannerStatus solve(double timeLimit) override;
    std::vector<Path> getSolutionPaths() const override;
    std::string name() const override { return "PrioritizedSTRRT"; }

    void setPriorityOrder(const std::vector<int> &order) {
        priority_order_ = order;
    }

    // Concatenate groups in the supplied order while optionally shuffling
    // within each group. This supports heterogeneous teams whose robot-class
    // precedence must remain fixed while priorities vary by seed.
    void setPriorityGroups(const std::vector<std::vector<int>> &groups) {
        priority_groups_ = groups;
    }

    void setShufflePriorityOrder(bool v) { shuffle_priority_order_ = v; }

    // Per-robot time limit fraction (of total). Defaults to a dynamic split of
    // remaining wall time across remaining robots when return_first_solution_ is false.
    void setPerRobotTimeFraction(double f) { per_robot_fraction_ = f; }

    // When false, skip collision check against prior robots that have finished
    // (timestep >= their path size). For ARC subproblems: robots leave subproblem.
    void setPersistAtGoal(bool v) { persist_at_goal_ = v; }

    // When false, do not equalize path lengths (for ARC subproblem solver).
    void setEqualizePaths(bool v) { equalize_paths_ = v; }

    // When true (default), ST-RRT* stops after the first feasible path (time limit is a cap, not a fill).
    // Each robot's OMPL solve then receives the full remaining wall time from solve(timeLimit), not a
    // per-robot split; when false, per_robot_fraction_ / equal split applies to each solve.
    void setReturnFirstSolution(bool v) { return_first_solution_ = v; }

    // When true, run OMPL path simplification after each successful solve (default off).
    void setSimplifyAfterPlan(bool v) { simplify_after_plan_ = v; }
    void setPathSimplificationOptions(PathSimplificationOptions options) {
        simplification_options_ =
            detail::normalizePathSimplificationOptions(options);
    }
    PathSimplificationOptions getPathSimplificationOptions() const {
        return simplification_options_;
    }

    // ST-RRT* rewiring during tree extension (default Off).
    void setStrrtRewiring(StrrtRewiring m) { strrt_rewiring_ = m; }

    // When true, leave the OMPL time dimension unbounded and rely on STRRT's
    // batch expansion logic. Default false preserves the current bounded mode.
    void setUseUnboundedTime(bool v) { use_unbounded_time_ = v; }

    // When true in unbounded mode, inflate the initial STRRT batch size based
    // on how many goal-time expansions were effectively skipped by the
    // conflict-informed minimum goal time.
    void setInflateInitialBatchFromMinGoalTime(bool v) {
        inflate_initial_batch_from_min_goal_time_ = v;
    }

    void setStrrtInitialBatchSize(unsigned int v) {
        strrt_initial_batch_size_ = v;
    }

    void setStrrtInitialTimeBoundFactor(double f) {
        strrt_initial_time_bound_factor_ = f;
    }

    void setStrrtTimeBoundFactorIncrease(double f) {
        strrt_time_bound_factor_increase_ = f;
    }

    void setStrrtMaxInflatedBatchMultiplier(unsigned int v) {
        strrt_max_inflated_batch_multiplier_ = v;
    }

    /// Maximum STRRT* solve-loop iterations per robot. Zero means unlimited.
    void setStrrtMaxIterations(unsigned int v) { strrt_max_iterations_ = v; }
    unsigned int strrtMaxIterations() const { return strrt_max_iterations_; }

    /// Upper bound (seconds) on the OMPL SpaceTimeStateSpace time component in
    /// bounded solve mode. Lower bound is fixed at 0 in solve(). Default 100
    /// for non-ARC callers.
    void setSpaceTimeUpperBound(double ub_seconds) {
        space_time_upper_bound_sec_ = ub_seconds;
    }

private:
    std::vector<int> priority_order_;
    std::vector<std::vector<int>> priority_groups_;
    std::vector<Path> solution_paths_;
    std::vector<double> last_robot_solve_times_seconds_;
    bool shuffle_priority_order_ = false;
    double per_robot_fraction_ = 0.0; // 0 = auto-split
    bool persist_at_goal_ = true;
    bool equalize_paths_ = true;
    bool return_first_solution_ = true;
    bool simplify_after_plan_ = false;
    PathSimplificationOptions simplification_options_{};
    StrrtRewiring strrt_rewiring_ = StrrtRewiring::Off;
    bool use_unbounded_time_ = true;
    bool inflate_initial_batch_from_min_goal_time_ = true;
    unsigned int strrt_initial_batch_size_ = 256;
    double strrt_initial_time_bound_factor_ = 4.0;
    double strrt_time_bound_factor_increase_ = 2.0;
    unsigned int strrt_max_inflated_batch_multiplier_ = 64;
    unsigned int strrt_max_iterations_ = 0;
    double space_time_upper_bound_sec_ = 100.0;
};

} // namespace comotion
