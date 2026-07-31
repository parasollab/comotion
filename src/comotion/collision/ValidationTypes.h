#pragma once

#include <cstddef>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace comotion {

enum class ConflictKind { Vertex };
enum class ConflictScope { Environment, Self, InterRobot };
enum class VampBatchOrdering { Combined, Hierarchical };
enum class VampBatchPacking { Rake, Linear };
enum class InterRobotConflictBatchMode {
    OptimisticIndependent,
    IndependentOnly,
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

struct TemporaryConflictFindInstrumentation {
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
    int discrete_num_checks_hint = -1;
    std::size_t t_begin = 0;
    std::size_t t_end = std::numeric_limits<std::size_t>::max();
    std::vector<std::size_t> per_path_t_begin;
    std::vector<std::size_t> per_pair_t_begin;
    std::size_t conflict_find_parallel_workers = 1;
    std::size_t conflict_find_parallel_horizon = 0;
    InterRobotConflictBatchMode inter_robot_conflict_batch_mode =
        InterRobotConflictBatchMode::OptimisticIndependent;
    std::function<bool()> stop_requested;
    // TEMP(ablation): remove this once the conflict-detection table
    // reproduction no longer needs build-vs-collision timing.
    TemporaryConflictFindInstrumentation *temporary_conflict_find_instrumentation =
        nullptr;
};

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
