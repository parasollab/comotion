#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace comotion {

// Detailed validation timing and work accounting is intended for dedicated
// profiling runs. It is disabled by default because these counters sit on
// collision-checking hot paths. Set COMOTION_VALIDATION_INSTRUMENTATION=1 in
// the process environment to opt in.
bool validationInstrumentationEnabled();

enum class ConflictKind { Vertex };
enum class ConflictScope { Environment, Self, InterRobot };
enum class VampBatchOrdering { Combined, Hierarchical };
enum class VampBatchPacking { Rake, Linear };
enum class InterRobotConflictBatchMode {
    OptimisticIndependent,
    IndependentOnly,
};
enum class ConflictFindParallelAssignment {
    Auto,
    PairCover,
    AllRobotsRoundRobin,
    BalancedPairCover,
    PairFirstGreedy,
    CyclicCoverGreedy,
};

struct VampValidationStrategy {
    VampBatchOrdering ordering = VampBatchOrdering::Combined;
    VampBatchPacking packing = VampBatchPacking::Rake;
};

struct PairPathConflict {
    std::size_t timestep = 0;
    // Motion conflict interpolation parameter; native path-conflict scans report 0.0.
    double alpha = 0.0;
    ConflictKind kind = ConflictKind::Vertex;
    std::vector<double> config_a;
    std::vector<double> config_b;
};

struct GoalHoldConstraint {
    std::size_t min_safe_arrival_timestep = 0;
    bool permanently_blocked = false;
};

struct CompositeConflict {
    ConflictScope scope = ConflictScope::InterRobot;
    int robot_i = -1;
    int robot_j = -1;
    std::size_t timestep = 0;
    // Motion conflict interpolation parameter; native path-conflict scans report 0.0.
    double alpha = 0.0;
    ConflictKind kind = ConflictKind::Vertex;
    std::vector<double> config_i;
    std::vector<double> config_j;
};

struct InterRobotConflictDecision {
    bool accept = true;
    std::vector<int> robots_to_claim;
};

using InterRobotConflictCallback =
    std::function<InterRobotConflictDecision(const CompositeConflict &)>;

/// Optional timing accumulator for parallel composite-path validation.
struct ConflictFindTimingInstrumentation {
    double build_worker_wall_seconds = 0.0;
    double build_worker_cpu_seconds = 0.0;
    double collision_worker_wall_seconds = 0.0;
    double collision_worker_cpu_seconds = 0.0;
    std::vector<double> build_worker_wall_seconds_by_worker;
    std::vector<double> collision_worker_wall_seconds_by_worker;

    void recordWorkerResult(std::size_t worker_index,
                            double worker_build_wall_seconds,
                            double worker_build_cpu_seconds,
                            double worker_collision_wall_seconds,
                            double worker_collision_cpu_seconds) {
        build_worker_wall_seconds += worker_build_wall_seconds;
        build_worker_cpu_seconds += worker_build_cpu_seconds;
        collision_worker_wall_seconds += worker_collision_wall_seconds;
        collision_worker_cpu_seconds += worker_collision_cpu_seconds;
        if (worker_index >= build_worker_wall_seconds_by_worker.size()) {
            build_worker_wall_seconds_by_worker.resize(worker_index + 1, 0.0);
            collision_worker_wall_seconds_by_worker.resize(worker_index + 1,
                                                           0.0);
        }
        build_worker_wall_seconds_by_worker[worker_index] +=
            worker_build_wall_seconds;
        collision_worker_wall_seconds_by_worker[worker_index] +=
            worker_collision_wall_seconds;
    }

    int criticalWorkerIndex() const {
        if (build_worker_wall_seconds_by_worker.empty())
            return -1;
        std::size_t best_index = 0;
        double best_total = build_worker_wall_seconds_by_worker[0] +
                            collision_worker_wall_seconds_by_worker[0];
        for (std::size_t worker_index = 1;
             worker_index < build_worker_wall_seconds_by_worker.size();
             ++worker_index) {
            const double worker_total =
                build_worker_wall_seconds_by_worker[worker_index] +
                collision_worker_wall_seconds_by_worker[worker_index];
            if (worker_total > best_total) {
                best_total = worker_total;
                best_index = worker_index;
            }
        }
        return static_cast<int>(best_index);
    }

    double criticalWorkerBuildWallSeconds() const {
        const int worker_index = criticalWorkerIndex();
        return worker_index < 0
                   ? 0.0
                   : build_worker_wall_seconds_by_worker[static_cast<std::size_t>(
                         worker_index)];
    }

    double criticalWorkerCollisionWallSeconds() const {
        const int worker_index = criticalWorkerIndex();
        return worker_index < 0
                   ? 0.0
                   : collision_worker_wall_seconds_by_worker
                         [static_cast<std::size_t>(worker_index)];
    }

    double criticalWorkerTotalWallSeconds() const {
        return criticalWorkerBuildWallSeconds() +
               criticalWorkerCollisionWallSeconds();
    }
};

struct CompositePathValidationOptions {
    bool check_environment = true;
    bool exhaustive = false;
    int discrete_num_checks_hint = -1;
    std::size_t t_begin = 0;
    std::size_t t_end = std::numeric_limits<std::size_t>::max();
    std::vector<std::size_t> per_path_t_begin;
    std::vector<std::size_t> per_pair_t_begin;
    std::size_t conflict_find_parallel_workers = 1;
    std::size_t conflict_find_parallel_horizon = 0;
    InterRobotConflictBatchMode inter_robot_conflict_batch_mode =
        InterRobotConflictBatchMode::OptimisticIndependent;
    ConflictFindParallelAssignment conflict_find_parallel_assignment =
        ConflictFindParallelAssignment::Auto;
    std::function<bool()> stop_requested;
    /// Receives per-worker path-build and collision-check timing when non-null.
    ConflictFindTimingInstrumentation *conflict_find_timing_instrumentation =
        nullptr;
};

struct ValidationWorkStats {
    std::uint64_t motion_timesteps_possible = 0;
    std::uint64_t motion_timesteps_checked = 0;
    std::uint64_t robot_state_checks_possible = 0;
    std::uint64_t robot_state_checks_completed = 0;
    std::uint64_t robot_pair_checks_possible = 0;
    std::uint64_t robot_pair_checks_completed = 0;
    std::uint64_t simd_packs_checked = 0;
    std::uint64_t simd_lanes_checked = 0;

    ValidationWorkStats &operator+=(const ValidationWorkStats &other) {
        motion_timesteps_possible += other.motion_timesteps_possible;
        motion_timesteps_checked += other.motion_timesteps_checked;
        robot_state_checks_possible += other.robot_state_checks_possible;
        robot_state_checks_completed += other.robot_state_checks_completed;
        robot_pair_checks_possible += other.robot_pair_checks_possible;
        robot_pair_checks_completed += other.robot_pair_checks_completed;
        simd_packs_checked += other.simd_packs_checked;
        simd_lanes_checked += other.simd_lanes_checked;
        return *this;
    }
};

struct ValidationTimingStats {
    double total_validation_time_seconds = 0.0;
    std::uint64_t total_validation_calls = 0;

    double composite_state_seconds = 0.0;
    std::uint64_t composite_state_calls = 0;

    double pair_path_seconds = 0.0;
    std::uint64_t pair_path_calls = 0;

    double pair_path_conflict_seconds = 0.0;
    std::uint64_t pair_path_conflict_calls = 0;

    double goal_hold_constraint_seconds = 0.0;
    std::uint64_t goal_hold_constraint_calls = 0;

    double composite_motion_seconds = 0.0;
    std::uint64_t composite_motion_calls = 0;

    double composite_motion_conflict_seconds = 0.0;
    std::uint64_t composite_motion_conflict_calls = 0;

    double composite_paths_seconds = 0.0;
    std::uint64_t composite_paths_calls = 0;

    double composite_path_conflict_seconds = 0.0;
    std::uint64_t composite_path_conflict_calls = 0;

    double inter_robot_path_conflicts_scan_seconds = 0.0;
    std::uint64_t inter_robot_path_conflicts_scan_calls = 0;

    ValidationWorkStats work;

    ValidationTimingStats &operator+=(const ValidationTimingStats &other) {
        total_validation_time_seconds += other.total_validation_time_seconds;
        total_validation_calls += other.total_validation_calls;
        composite_state_seconds += other.composite_state_seconds;
        composite_state_calls += other.composite_state_calls;
        pair_path_seconds += other.pair_path_seconds;
        pair_path_calls += other.pair_path_calls;
        pair_path_conflict_seconds += other.pair_path_conflict_seconds;
        pair_path_conflict_calls += other.pair_path_conflict_calls;
        goal_hold_constraint_seconds += other.goal_hold_constraint_seconds;
        goal_hold_constraint_calls += other.goal_hold_constraint_calls;
        composite_motion_seconds += other.composite_motion_seconds;
        composite_motion_calls += other.composite_motion_calls;
        composite_motion_conflict_seconds +=
            other.composite_motion_conflict_seconds;
        composite_motion_conflict_calls +=
            other.composite_motion_conflict_calls;
        composite_paths_seconds += other.composite_paths_seconds;
        composite_paths_calls += other.composite_paths_calls;
        composite_path_conflict_seconds +=
            other.composite_path_conflict_seconds;
        composite_path_conflict_calls += other.composite_path_conflict_calls;
        inter_robot_path_conflicts_scan_seconds +=
            other.inter_robot_path_conflicts_scan_seconds;
        inter_robot_path_conflicts_scan_calls +=
            other.inter_robot_path_conflicts_scan_calls;
        work += other.work;
        return *this;
    }
};

inline bool usePairCoverConflictAssignment(
    ConflictFindParallelAssignment assignment, std::size_t worker_count) {
    switch (assignment) {
    case ConflictFindParallelAssignment::Auto:
        return worker_count == 4 || worker_count == 8 || worker_count == 16;
    case ConflictFindParallelAssignment::PairCover:
        return true;
    case ConflictFindParallelAssignment::AllRobotsRoundRobin:
        return false;
    case ConflictFindParallelAssignment::BalancedPairCover:
        return false;
    case ConflictFindParallelAssignment::PairFirstGreedy:
        return false;
    case ConflictFindParallelAssignment::CyclicCoverGreedy:
        return false;
    }
    return false;
}

inline std::size_t pairFrontierSize(std::size_t item_count) {
    return item_count < 2 ? 0 : item_count * (item_count - 1) / 2;
}

inline std::size_t pairFrontierIndex(std::size_t i, std::size_t j,
                                     std::size_t item_count) {
    if (i == j || i >= item_count || j >= item_count) {
        throw std::invalid_argument("pair frontier index out of range");
    }
    if (j < i)
        std::swap(i, j);
    return i * (2 * item_count - i - 1) / 2 + (j - i - 1);
}

} // namespace comotion
