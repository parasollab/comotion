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
enum class ConflictFindParallelAssignment {
    Auto,
    PairCover,
    AllRobotsRoundRobin,
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

struct CompositePathValidationOptions {
    bool check_environment = true;
    int discrete_num_checks_hint = -1;
    std::size_t t_begin = 0;
    std::size_t t_end = std::numeric_limits<std::size_t>::max();
    std::vector<std::size_t> per_path_t_begin;
    std::vector<std::size_t> per_pair_t_begin;
    std::size_t conflict_find_parallel_workers = 1;
    std::size_t conflict_find_parallel_horizon = 0;
    ConflictFindParallelAssignment conflict_find_parallel_assignment =
        ConflictFindParallelAssignment::Auto;
    std::function<bool()> stop_requested;
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
