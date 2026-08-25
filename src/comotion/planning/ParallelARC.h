#pragma once

#include "comotion/planning/ARC.h"

#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <vector>

namespace comotion {

enum class ParallelArcParallelStrategy { Synchronous, Asynchronous };
enum class ParallelArcConflictSelectionStrategy {
    Greedy,
    SpatialDistribution,
};
enum class ParallelArcConflictFindMode { Sequential, SegmentParallel };

/// Experimental/advanced planner API.
///
/// Process-parallel ARC variant for multicore conflict repair experiments.
class ParallelARC : public ARC {
public:
    ompl::base::PlannerStatus solve(double timeLimit) override;
    bool runConflictDetectionAblation(double timeLimit);
    std::string name() const override { return "ParallelARC"; }

    void setWorkerProcesses(unsigned n) { worker_processes_ = n; }
    unsigned workerProcesses() const { return worker_processes_; }
    void setParallelizeInitialIndividualPlans(bool enabled) {
        parallelize_initial_individual_plans_ = enabled;
    }
    void setInitialSolutionOr(bool enabled) {
        initial_solution_or_ = enabled;
    }
    void setParallelStrategy(ParallelArcParallelStrategy strategy) {
        parallel_strategy_ = strategy;
    }
    void setConflictSelectionStrategy(
        ParallelArcConflictSelectionStrategy strategy) {
        conflict_selection_strategy_ = strategy;
    }
    void setConflictFindMode(ParallelArcConflictFindMode mode) {
        conflict_find_mode_ = mode;
    }
    void setConflictFindHorizon(std::size_t horizon) {
        conflict_find_horizon_ = horizon;
    }
    void setConflictFindParallelAssignment(
        ConflictFindParallelAssignment assignment) {
        conflict_find_parallel_assignment_ = assignment;
    }
    void setConflictBatchMode(InterRobotConflictBatchMode mode) {
        conflict_batch_mode_ = mode;
    }
    void setRepairDuplicateAttempts(bool enabled) {
        repair_duplicate_attempts_ = enabled;
    }

protected:
    struct BatchConflictTask {
        SubproblemConflict conflict;
        std::vector<std::uint64_t> base_versions;
    };

    struct ConflictRoundEntry {
        int seed_robot_i = -1;
        int seed_robot_j = -1;
        int conflict_timestep = 0;
        std::vector<int> expanded_team;
        std::vector<int> final_team;
        int window_begin_t = 0;
        int window_end_t = 0;
        std::vector<SubproblemConflict::ExpansionTraceStep> expansion_trace;
        std::size_t attempts_launched = 0;
        std::size_t cancelled_sibling_attempts = 0;
        std::size_t winner_attempt_index = 0;
        int winner_slot_index = -1;
        std::uint32_t winner_planning_seed = 0;
        std::uint64_t winner_worker_wall_ns = 0;
        std::uint64_t patch_fingerprint = 0;
        std::vector<std::uint64_t> local_patch_fingerprints;
        std::vector<std::uint64_t> local_patch_arrival_timesteps;
        std::vector<std::uint64_t> post_apply_global_arrival_timesteps;
    };

    struct ConflictRoundStats {
        std::vector<ConflictRoundEntry> entries;
    };

    std::vector<BatchConflictTask>
    selectConflictBatch(const std::vector<SubproblemConflict> &conflicts) const;

    void resetConflictRoundStats();
    void appendConflictRoundStats(ConflictRoundStats round_stats);
    void printConflictRoundStats(std::ostream &os) const;

private:
    void resetParallelArcRunState();
    void finalizeParallelArcPlannerStats(
        const ArcPlannerStatsSummary &planner_stats_summary,
        const nlohmann::json &repair_failure_snapshot);

    struct InitialIndividualWorkerStats {
        int worker_index = -1;
        std::vector<int> robots;
        std::uint64_t command_write_ns = 0;
        std::uint64_t result_read_ns = 0;
        std::uint64_t command_bytes_written = 0;
        std::uint64_t result_bytes_read = 0;
        std::uint64_t worker_wall_ns = 0;
        std::uint64_t solve_ns = 0;
        std::uint64_t simplify_ns = 0;
        double cpu_seconds = 0.0;
    };

    bool planInitialIndividualPathsWithWorkers(
        const Clock::time_point &solve_start, double timeLimit,
        unsigned worker_count, std::vector<Path> &working_paths);

    unsigned worker_processes_ = 2;
    bool parallelize_initial_individual_plans_ = true;
    bool initial_solution_or_ = false;
    unsigned initial_individual_worker_processes_used_ = 0;
    std::uint64_t initial_individual_result_bytes_read_ = 0;
    std::uint64_t initial_individual_command_bytes_written_ = 0;
    std::uint64_t initial_individual_process_launch_ns_ = 0;
    std::uint64_t initial_individual_process_shutdown_ns_ = 0;
    std::uint64_t initial_individual_command_write_ns_ = 0;
    std::uint64_t initial_individual_result_read_ns_ = 0;
    std::uint64_t initial_individual_parent_wait_ns_ = 0;
    std::uint64_t initial_individual_duplicate_attempts_ = 0;
    std::vector<InitialIndividualWorkerStats> initial_individual_worker_stats_;
    // Only Synchronous is currently supported; Asynchronous is reported as
    // unsupported if selected.
    ParallelArcParallelStrategy parallel_strategy_ =
        ParallelArcParallelStrategy::Synchronous;
    // Only Greedy is currently supported; SpatialDistribution is reported as
    // unsupported if selected.
    ParallelArcConflictSelectionStrategy conflict_selection_strategy_ =
        ParallelArcConflictSelectionStrategy::Greedy;
    ParallelArcConflictFindMode conflict_find_mode_ =
        ParallelArcConflictFindMode::SegmentParallel;
    std::size_t conflict_find_horizon_ = 400;
    ConflictFindParallelAssignment conflict_find_parallel_assignment_ =
        ConflictFindParallelAssignment::Auto;
    InterRobotConflictBatchMode conflict_batch_mode_ =
        InterRobotConflictBatchMode::OptimisticIndependent;
    bool repair_duplicate_attempts_ = true;
    std::vector<ConflictRoundStats> conflict_round_stats_;
};

} // namespace comotion
