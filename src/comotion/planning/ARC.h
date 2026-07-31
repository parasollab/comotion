#pragma once

#include "comotion/collision/ConflictChecker.h"
#include "comotion/planning/MultiRobotPlanner.h"
#include "comotion/planning/PathSimplification.h"
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace comotion {

/// Stable planner API.
///
/// Adaptive Robot Coordination (ARC): computes initial per-robot paths, then
/// iteratively finds and resolves conflicts via local subproblems.
class ARC : public MultiRobotPlanner {
public:
    enum class LocalSolverMode {
        Both,
        PrioritizedStrrtOnly,
        CompositeRrtOnly,
    };
    using CancellationCallback = std::function<bool()>;

    ompl::base::PlannerStatus solve(double timeLimit) override;
    std::vector<Path> getSolutionPaths() const override;
    std::string name() const override { return "ARC"; }

    void setInitialWindow(int w) { initial_window_ = w; }
    void setExpansionStep(int e) { expansion_step_ = e; }

    /// Cap CompositeRRT (RRTConnect) outer-loop iterations per local solve call when
    /// the temporal window does not yet span the full global horizon. On the final
    /// full-window local call, the cap is ignored (time budget only).
    /// Zero disables the cap for all non-final calls as well.
    void setLocalCompositeRrtMaxSamples(unsigned n) { local_composite_rrt_max_samples_ = n; }
    void setLocalCompositeRrtUseMakespanMetric(bool v) {
        local_composite_rrt_use_makespan_metric_ = v;
    }
    void setLocalSolverMode(LocalSolverMode mode) { local_solver_mode_ = mode; }
    LocalSolverMode localSolverMode() const { return local_solver_mode_; }
    /// Per-robot STRRT* solve-loop iteration cap for ARC's local
    /// PrioritizedSTRRT attempt. Zero disables the cap.
    void setLocalPrioritizedStrrtMaxIterations(unsigned int iterations) {
        local_prioritized_strrt_max_iterations_ = iterations;
    }
    unsigned int localPrioritizedStrrtMaxIterations() const {
        return local_prioritized_strrt_max_iterations_;
    }
    void setUseCspaceBounds(bool v) { use_cspace_bounds_ = v; }
    void setCspaceBoundMargin(float m) { cspace_bound_margin_ = m; }
    /// Minimum per-joint bound width for subproblem C-space boxes (stationary robots).
    void setMinCspaceBoundRange(double r) { min_cspace_bound_range_ = r; }
    void setPathSimplificationOptions(PathSimplificationOptions options) {
        simplification_options_ =
            detail::normalizePathSimplificationOptions(options);
    }
    PathSimplificationOptions getPathSimplificationOptions() const {
        return simplification_options_;
    }
    void setSimplificationMaxSteps(unsigned int max_steps) {
        simplification_options_.max_shortcut_steps = std::max(1u, max_steps);
    }
    unsigned int getSimplificationMaxSteps() const {
        return simplification_options_.max_shortcut_steps;
    }
    void setSimplifyInitialSolutions(bool simplify) {
        simplify_initial_solutions_ = simplify;
    }
    bool getSimplifyInitialSolutions() const {
        return simplify_initial_solutions_;
    }
    void setSimplifyConflictSolutions(bool simplify) {
        simplify_conflict_solutions_ = simplify;
    }
    bool getSimplifyConflictSolutions() const {
        return simplify_conflict_solutions_;
    }
    void setSimplifySolution(bool simplify) {
        simplify_initial_solutions_ = simplify;
        simplify_conflict_solutions_ = simplify;
    }

    /// Optional makespan bound in native CoMotion timestep units. When set, ARC uses
    /// bounded AO-RRTC for initial individual paths and local composite repairs.
    void setGlobalMakespanBoundTimesteps(std::uint64_t bound) {
        global_makespan_bound_timesteps_ = bound;
    }
    void clearGlobalMakespanBoundTimesteps() {
        global_makespan_bound_timesteps_.reset();
    }
    void setBoundedLocalRepairEpsilonTimesteps(std::uint64_t epsilon) {
        bounded_local_repair_epsilon_timesteps_ = epsilon;
    }
    std::uint64_t boundedLocalRepairEpsilonTimesteps() const {
        return bounded_local_repair_epsilon_timesteps_;
    }

    /// Multiplier for local STRRT* OMPL time upper bound: (end_t - start_t) timesteps /
    /// resolution (seconds) times this factor. Must be positive (default 4).
    void setStrrtSpaceTimeSpanFactor(double f) {
        strrt_space_time_span_factor_ = (f > 0.0) ? f : 4.0;
    }

    /// Deprecated: ARC no longer caps local PrioritizedSTRRT / CompositeRRT wall time
    /// below the remaining global budget. Local solvers receive the full remaining wall
    /// time (recomputed after each layer). Kept for API compatibility.
    [[deprecated("ARC local solvers use full remaining wall time; this setting is ignored.")]]
    void setLocalSolverMaxBudget(double /*max_seconds*/) {}

    /// Deprecated: unused; local wall budgets are always the remaining global time.
    [[deprecated("ARC local solvers use full remaining wall time; this setting is ignored.")]]
    void setLocalSolverBudgetExpansionIncrement(double /*seconds_per_expansion*/) {}

protected:
    struct ArcPlannerStatsSummary {
        LocalSolverMode local_solver_mode = LocalSolverMode::Both;
        unsigned int local_prioritized_strrt_max_iterations = 0;
        bool local_composite_rrt_use_makespan_metric = false;
        std::uint64_t bounded_local_repair_epsilon_timesteps = 1;
        std::uint64_t num_conflicts = 0;
        std::uint64_t subproblem_attempts = 0;
        std::uint64_t temporal_expansions = 0;
        double initial_solution_times_seconds_wall_clock = 0.0;
        double initial_solution_times_seconds_cpu = 0.0;
        double initial_simplification_times_seconds_wall_clock = 0.0;
        double local_composite_simplification_times_seconds_wall_clock = 0.0;
        double conflict_detection_times_seconds_wall_clock = 0.0;
        double conflict_detection_times_seconds_cpu = 0.0;
        double conflict_resolution_times_seconds_wall_clock = 0.0;
        double conflict_resolution_times_seconds_total = 0.0;
        double conflict_resolution_times_seconds_cpu = 0.0;
        std::uint64_t subproblem_batches = 0;
    };

    struct ProcessTreeCpuUsageSnapshot {
        double self_seconds = 0.0;
        double children_seconds = 0.0;
    };

    using Clock = std::chrono::steady_clock;

    struct RepairWindow {
        int window_start_t = 0;
        int window_end_t = 0;
        std::vector<int> history_event_ids;
    };

    struct AppliedRepairHistoryEvent {
        int event_id = -1;
        std::vector<int> robots;
        int window_start_t = 0;
        int window_end_t = 0;
    };

    struct RepairOutcome {
        bool resolved = false;
        int window_start_t = 0;
        int window_end_t = 0;
        std::vector<int> final_involved_robots;
        std::vector<Path> local_patch_paths;
    };

    struct IndividualPlanResult {
        bool success = false;
        int status_type = 0;
        std::string status_message;
        Path path;
        std::uint64_t arrival_timestep = 0;
        std::uint64_t solve_ns = 0;
        std::uint64_t simplify_ns = 0;
        double cpu_seconds = 0.0;
        std::string error_message;
    };

    void resetArcSolveState();

    IndividualPlanResult
    planIndividualPath(int robot_index, double solve_budget_seconds,
                       std::optional<std::uint32_t> local_seed = std::nullopt);
    void recordInitialIndividualPlanStats(const IndividualPlanResult &result);
    void finishInitialIndividualPaths(std::vector<Path> &working_paths);

    // Plan individual paths for each robot using RRTConnect; each solve uses remaining
    // wall time until the global ARC timeLimit.
    bool planIndividualPaths(const Clock::time_point &solve_start,
                             double timeLimit,
                             std::vector<Path> &working_paths);

    // Attempt to solve a subproblem with the solver hierarchy
    /// On success, if window_start_t_out is non-null, writes native path timestep
    /// index of the local window start (same coordinates as conflict.timestep).
    /// `global_time_limit` is the total wall budget for ARC::solve (seconds from
    /// `solve_start`); remaining time is recomputed from the clock each expansion.
    bool solveSubproblemOnPaths(const SubproblemConflict &conflict,
                                const Clock::time_point &solve_start,
                                double global_time_limit,
                                std::vector<Path> &working_paths,
                                 int *window_start_t_out = nullptr,
                                 int *window_end_t_out = nullptr,
                                 std::vector<Path> *local_paths_out = nullptr,
                                 bool apply_solution_to_paths = true,
                                 CancellationCallback cancel_requested = {});

    // Splice subproblem solution into the global paths
    void spliceSolutionIntoPaths(const std::vector<int> &involved_robots,
                                 int start_t, int end_t,
                                 const std::vector<Path> &local_paths,
                                 std::vector<Path> &working_paths);
    void spliceSolutionIntoPaths(const std::vector<int> &involved_robots,
                                 int start_t, int end_t,
                                 const std::vector<const Path *> &local_paths,
                                 std::vector<Path> &working_paths);

    RepairOutcome resolveConflictOnPaths(
        const SubproblemConflict &conflict,
        const Clock::time_point &solve_start, double global_time_limit,
        std::vector<Path> &working_paths,
        bool apply_solution_to_paths = true,
        CancellationCallback cancel_requested = {});

    ArcPlannerStatsSummary currentArcPlannerStatsSummary() const;
    static nlohmann::json
    plannerStatsJsonFromSummary(
        const ArcPlannerStatsSummary &summary,
        const std::vector<double> *conflict_resolution_times_seconds = nullptr,
        const std::vector<double> *conflict_detection_times_seconds = nullptr);
    static ProcessTreeCpuUsageSnapshot processTreeCpuUsageSnapshot();
    static double elapsedProcessTreeCpuSeconds(
        const ProcessTreeCpuUsageSnapshot &start,
        const ProcessTreeCpuUsageSnapshot &finish);
    static nlohmann::json temporaryConflictFindTimingJson(
        const std::vector<double> &main_process_wall_seconds,
        const std::vector<double> &process_tree_cpu_seconds,
        const std::vector<double> &build_worker_wall_seconds,
        const std::vector<double> &build_worker_cpu_seconds,
        const std::vector<double> &collision_worker_wall_seconds,
        const std::vector<double> &collision_worker_cpu_seconds,
        const std::vector<int> &critical_worker_index,
        const std::vector<double> &critical_worker_build_wall_seconds,
        const std::vector<double> &critical_worker_collision_wall_seconds,
        const std::vector<double> &critical_worker_total_wall_seconds);

    void initializeConflictScanStarts(std::size_t robot_count);
    CompositePathValidationOptions conflictScanOptions() const;
    void applyConflictScanProgress(
        const std::vector<std::size_t> &next_t_begin_by_pair);
    void resetConflictScanStartsForRobots(const std::vector<int> &robots,
                                          int start_t);
    void updateDerivedConflictScanStart();

    void recordAppliedRepairHistory(const std::vector<int> &robots,
                                    int window_start_t, int window_end_t);
    const std::vector<AppliedRepairHistoryEvent> &
    appliedRepairHistoryEvents() const {
        return applied_repair_history_events_;
    }

    virtual SubproblemConflict
    expandConflictForSubproblem(const Conflict &conflict) const;

    // Cascade merge via recursive repair-window closure: start from the
    // conflicting pair and repeatedly union robots with prior pair repair
    // windows that intersect the proposed conflict patch window.
    std::vector<int> subproblemRobotsForConflict(int robot_i, int robot_j,
                                                 int window_start_t,
                                                 int window_end_t,
                                                 std::vector<
                                                     SubproblemConflict::
                                                         ExpansionTraceStep> *
                                                     trace_out = nullptr) const;
    const std::vector<RepairWindow> *
    repairWindowsForRobots(int robot_i, int robot_j) const;
    int conflictWindowStart(const Conflict &conflict) const {
        return std::max(0, conflict.timestep - initial_window_);
    }
    std::vector<Path> solution_paths_;
    std::map<int, std::map<int, std::vector<RepairWindow>>>
        repair_window_schedule_;
    std::vector<AppliedRepairHistoryEvent> applied_repair_history_events_;
    std::size_t conflict_scan_robot_count_ = 0;
    std::vector<int> pair_conflict_scan_start_t_;
    std::vector<std::uint64_t> true_arrival_timesteps_;
    /// Native timestep index where last successful local replan began; next conflict
    /// scan can start there (prefix unchanged). Maintained as the minimum of
    /// pair_conflict_scan_start_t_ for coarse tracing/stat summaries.
    int last_subproblem_window_start_ = -1;
    int initial_window_ = 20;
    int expansion_step_ = 20;
    unsigned local_composite_rrt_max_samples_{0};
    bool local_composite_rrt_use_makespan_metric_{false};
    LocalSolverMode local_solver_mode_ = LocalSolverMode::Both;
    unsigned int local_prioritized_strrt_max_iterations_ = 5;
    bool use_cspace_bounds_ = true;
    float cspace_bound_margin_ = 0.5f;
    double min_cspace_bound_range_ = 0.1;
    bool simplify_initial_solutions_ = true;
    bool simplify_conflict_solutions_ = false;
    PathSimplificationOptions simplification_options_{};
    std::optional<std::uint64_t> global_makespan_bound_timesteps_;
    std::uint64_t bounded_local_repair_epsilon_timesteps_ = 1;
    bool warned_bounded_prioritized_disabled_ = false;
    /// Scales local window duration (seconds) -> SpaceTimeStateSpace upper bound.
    double strrt_space_time_span_factor_ = 4.0;
    std::uint64_t num_conflicts_ = 0;
    std::uint64_t num_subproblem_attempts_ = 0;
    std::uint64_t num_temporal_expansions_ = 0;
    double initial_solution_times_seconds_wall_clock_ = 0.0;
    double initial_solution_times_seconds_cpu_ = 0.0;
    double initial_simplification_times_seconds_wall_clock_ = 0.0;
    double local_composite_simplification_times_seconds_wall_clock_ = 0.0;
    std::vector<double> conflict_detection_times_seconds_;
    std::vector<double> conflict_detection_times_cpu_seconds_;
    // TEMP(ablation): remove these once the conflict-detection table
    // reproduction no longer needs per-round worker build/collision timing.
    std::vector<double> temporary_conflict_find_main_process_wall_seconds_;
    std::vector<double> temporary_conflict_find_process_tree_cpu_seconds_;
    std::vector<double> temporary_conflict_find_build_worker_wall_seconds_;
    std::vector<double> temporary_conflict_find_build_worker_cpu_seconds_;
    std::vector<double> temporary_conflict_find_collision_worker_wall_seconds_;
    std::vector<double> temporary_conflict_find_collision_worker_cpu_seconds_;
    std::vector<int> temporary_conflict_find_critical_worker_index_;
    std::vector<double> temporary_conflict_find_critical_worker_build_wall_seconds_;
    std::vector<double>
        temporary_conflict_find_critical_worker_collision_wall_seconds_;
    std::vector<double> temporary_conflict_find_critical_worker_total_wall_seconds_;
    std::vector<double> conflict_resolution_times_seconds_;
    std::vector<double> conflict_resolution_times_cpu_seconds_;
};

using ArcLocalSolverMode = ARC::LocalSolverMode;

} // namespace comotion
