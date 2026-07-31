#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <queue>
#include <stdexcept>
#include <utility>
#include <vector>

namespace comotion {

inline constexpr std::array<int, 7> kParallelArcPaperRobotCounts = {{
    4, 8, 16, 32, 64, 128, 256,
}};

inline constexpr std::array<int, 4> kParallelArcPaperWorkerCounts = {{
    2, 4, 8, 16,
}};

namespace detail {

inline std::vector<std::vector<int>>
balancedEightRobotPairCoverWorkerRobotsB16() {
    return {
        {0, 3, 5}, {0, 1, 7}, {1, 2, 4}, {0, 2, 6},
        {1, 3, 6}, {2, 3, 7}, {4, 5, 6}, {2, 3, 4},
        {3, 5},    {0, 4, 7}, {1, 5, 7}, {2, 5, 6},
        {3, 6, 7}, {3, 7},    {4, 7},    {5, 6},
    };
}

inline std::vector<std::vector<std::pair<int, int>>>
balancedEightRobotPairAssignmentsB16() {
    return {
        {{0, 3}, {0, 5}}, {{0, 1}, {1, 7}}, {{1, 2}, {1, 4}},
        {{0, 2}, {0, 6}}, {{1, 3}, {1, 6}}, {{2, 3}, {2, 7}},
        {{4, 5}, {4, 6}}, {{2, 4}, {3, 4}}, {{3, 5}},
        {{0, 4}, {0, 7}}, {{1, 5}, {5, 7}}, {{2, 5}, {2, 6}},
        {{3, 6}, {6, 7}}, {{3, 7}},         {{4, 7}}, {{5, 6}},
    };
}

inline std::vector<std::vector<int>>
balancedSixtyFourRobotPairCoverWorkerRobotsB16() {
    return {
        {0, 3, 5, 12, 13, 16, 18, 25, 26, 29, 31, 38, 39, 42, 44, 51, 52,
         55, 57},
        {0, 1, 8, 9, 13, 14, 21, 22, 26, 27, 34, 35, 39, 40, 47, 48, 52,
         53, 60, 61},
        {1, 2, 4, 12, 14, 15, 17, 25, 27, 28, 30, 38, 40, 41, 43, 51, 53,
         54, 56},
        {0, 2, 6, 10, 13, 15, 19, 23, 26, 28, 32, 36, 39, 41, 45, 49, 52,
         54, 58, 62},
        {1, 3, 6, 11, 14, 16, 19, 24, 27, 29, 32, 37, 40, 42, 45, 50, 53,
         55, 58, 63},
        {2, 3, 7, 9, 15, 16, 20, 22, 28, 29, 33, 35, 41, 42, 46, 48, 54,
         55, 59, 61},
        {4, 5, 6, 9, 17, 18, 19, 22, 30, 31, 32, 35, 43, 44, 45, 48, 56,
         57, 58, 61},
        {3, 4, 8, 10, 16, 17, 21, 23, 29, 30, 34, 36, 42, 43, 47, 49, 55,
         56, 60, 62},
        {9, 10, 11, 12, 22, 23, 24, 25, 35, 36, 37, 38, 48, 49, 50, 51, 61,
         62, 63},
        {0, 4, 7, 11, 13, 17, 20, 24, 26, 30, 33, 37, 39, 43, 46, 50, 52,
         56, 59, 63},
        {1, 5, 7, 10, 14, 18, 20, 23, 27, 31, 33, 36, 40, 44, 46, 49, 53,
         57, 59, 62},
        {2, 5, 8, 11, 15, 18, 21, 24, 28, 31, 34, 37, 41, 44, 47, 50, 54,
         57, 60, 63},
        {6, 7, 8, 12, 19, 20, 21, 25, 32, 33, 34, 38, 45, 46, 47, 51, 58,
         59, 60},
        {2, 8, 13, 16, 18, 19, 22, 25, 26, 30, 31, 32, 37, 46, 48, 49, 50,
         53, 56, 57},
        {4, 5, 9, 11, 12, 13, 15, 20, 27, 36, 39, 42, 43, 44, 52, 54, 57,
         58, 60, 61},
        {3, 6, 7, 8, 10, 11, 14, 17, 19, 23, 24, 32, 33, 34, 40, 41, 47,
         55, 60, 63},
    };
}

class DinicFlow {
public:
    explicit DinicFlow(int node_count) : graph_(node_count), level_(node_count) {}

    void addEdge(int from, int to, int capacity) {
        const int from_rev = static_cast<int>(graph_[from].size());
        const int to_rev = static_cast<int>(graph_[to].size());
        graph_[from].push_back(Edge{to, to_rev, capacity});
        graph_[to].push_back(Edge{from, from_rev, 0});
    }

    int maxFlow(int source, int sink) {
        int total_flow = 0;
        while (buildLevels(source, sink)) {
            std::vector<int> next_edge(graph_.size(), 0);
            while (const int pushed =
                       pushFlow(source, sink, std::numeric_limits<int>::max(),
                                next_edge)) {
                total_flow += pushed;
            }
        }
        return total_flow;
    }

    struct Edge {
        int to = 0;
        int reverse_index = 0;
        int capacity = 0;
    };

    const std::vector<std::vector<Edge>> &graph() const { return graph_; }

private:
    bool buildLevels(int source, int sink) {
        std::fill(level_.begin(), level_.end(), -1);
        std::queue<int> queue;
        level_[source] = 0;
        queue.push(source);
        while (!queue.empty()) {
            const int node = queue.front();
            queue.pop();
            for (const auto &edge : graph_[node]) {
                if (edge.capacity <= 0 || level_[edge.to] >= 0)
                    continue;
                level_[edge.to] = level_[node] + 1;
                queue.push(edge.to);
            }
        }
        return level_[sink] >= 0;
    }

    int pushFlow(int node, int sink, int flow, std::vector<int> &next_edge) {
        if (node == sink)
            return flow;
        for (int &edge_index = next_edge[static_cast<std::size_t>(node)];
             edge_index < static_cast<int>(graph_[node].size()); ++edge_index) {
            Edge &edge = graph_[node][static_cast<std::size_t>(edge_index)];
            if (edge.capacity <= 0 ||
                level_[edge.to] != level_[node] + 1) {
                continue;
            }
            const int pushed =
                pushFlow(edge.to, sink, std::min(flow, edge.capacity),
                         next_edge);
            if (pushed == 0)
                continue;
            edge.capacity -= pushed;
            graph_[edge.to][static_cast<std::size_t>(edge.reverse_index)]
                .capacity += pushed;
            return pushed;
        }
        return 0;
    }

    std::vector<std::vector<Edge>> graph_;
    std::vector<int> level_;
};

inline std::vector<std::vector<std::pair<int, int>>>
assignPairsWithCapacitatedFlow(
    int n, const std::vector<std::vector<int>> &worker_robots,
    const std::vector<int> &worker_pair_capacities) {
    const int worker_count = static_cast<int>(worker_robots.size());
    if (static_cast<int>(worker_pair_capacities.size()) != worker_count) {
        throw std::invalid_argument(
            "assignPairsWithCapacitatedFlow worker capacity count must match "
            "worker count");
    }

    std::vector<std::vector<char>> worker_has_robot(
        static_cast<std::size_t>(worker_count),
        std::vector<char>(static_cast<std::size_t>(n), 0));
    for (int worker = 0; worker < worker_count; ++worker) {
        for (const int robot :
             worker_robots[static_cast<std::size_t>(worker)]) {
            if (robot < 0 || robot >= n) {
                throw std::invalid_argument(
                    "assignPairsWithCapacitatedFlow robot index out of range");
            }
            worker_has_robot[static_cast<std::size_t>(worker)]
                           [static_cast<std::size_t>(robot)] = 1;
        }
    }

    struct PairInfo {
        int first = 0;
        int second = 0;
    };

    std::vector<PairInfo> pairs;
    pairs.reserve(static_cast<std::size_t>(n * (n - 1) / 2));
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j)
            pairs.push_back({i, j});
    }

    const int source = 0;
    const int pair_node_begin = 1;
    const int worker_node_begin = pair_node_begin + static_cast<int>(pairs.size());
    const int sink = worker_node_begin + worker_count;
    DinicFlow flow(sink + 1);

    for (int worker = 0; worker < worker_count; ++worker) {
        flow.addEdge(worker_node_begin + worker, sink,
                     worker_pair_capacities[static_cast<std::size_t>(worker)]);
    }

    for (int pair_index = 0; pair_index < static_cast<int>(pairs.size());
         ++pair_index) {
        const int pair_node = pair_node_begin + pair_index;
        flow.addEdge(source, pair_node, 1);

        const auto &pair = pairs[static_cast<std::size_t>(pair_index)];
        bool has_cover = false;
        for (int worker = 0; worker < worker_count; ++worker) {
            if (worker_has_robot[static_cast<std::size_t>(worker)]
                               [static_cast<std::size_t>(pair.first)] == 0 ||
                worker_has_robot[static_cast<std::size_t>(worker)]
                               [static_cast<std::size_t>(pair.second)] == 0) {
                continue;
            }
            flow.addEdge(pair_node, worker_node_begin + worker, 1);
            has_cover = true;
        }
        if (!has_cover) {
            throw std::runtime_error(
                "assignPairsWithCapacitatedFlow did not cover every robot pair");
        }
    }

    if (flow.maxFlow(source, sink) != static_cast<int>(pairs.size())) {
        throw std::runtime_error(
            "assignPairsWithCapacitatedFlow could not satisfy worker pair "
            "capacities");
    }

    std::vector<std::vector<std::pair<int, int>>> worker_pairs(
        static_cast<std::size_t>(worker_count));
    for (int pair_index = 0; pair_index < static_cast<int>(pairs.size());
         ++pair_index) {
        const int pair_node = pair_node_begin + pair_index;
        const auto &pair = pairs[static_cast<std::size_t>(pair_index)];
        bool assigned = false;
        for (const auto &edge : flow.graph()[static_cast<std::size_t>(pair_node)]) {
            if (edge.to < worker_node_begin || edge.to >= sink ||
                edge.capacity != 0) {
                continue;
            }
            const int worker = edge.to - worker_node_begin;
            worker_pairs[static_cast<std::size_t>(worker)].push_back(
                {pair.first, pair.second});
            assigned = true;
            break;
        }
        if (!assigned) {
            throw std::runtime_error(
                "assignPairsWithCapacitatedFlow produced an incomplete "
                "assignment");
        }
    }

    return worker_pairs;
}

inline constexpr std::array<std::uint16_t, 1> kPairCoverB2Cycle = {{
    0x0001,
}};

// Balanced period for buckets 0..3. One full cycle has bucket loads 3,3,3,3.
inline constexpr std::array<std::uint16_t, 5> kPairCoverB4Cycle = {{
    0x0003, 0x0005, 0x000e, 0x0009, 0x000e,
}};

// Heuristic-balanced intersecting mask family for buckets 0..7. One full cycle
// has bucket loads 10,10,10,10,10,10,10,10.
inline constexpr std::array<std::uint16_t, 24> kPairCoverB8Cycle = {{
    0x001c, 0x0049, 0x0091, 0x00c4, 0x008a, 0x0027,
    0x0072, 0x00a8, 0x0027, 0x001c, 0x0091, 0x0049,
    0x00c4, 0x0072, 0x008a, 0x0027, 0x001c, 0x0091,
    0x0049, 0x00c4, 0x0072, 0x00a8, 0x0027, 0x0072,
}};

// Cyclic projective-plane order-3 line masks, with a prefix order chosen to keep
// powers-of-two requests balanced. Buckets 13..15 are intentionally unused.
inline constexpr std::array<std::uint16_t, 13> kPairCoverB16Cycle = {{
    0x020b, 0x0416, 0x082c, 0x00b1, 0x02c4, 0x0c41, 0x1058,
    0x1620, 0x1882, 0x0162, 0x0588, 0x0b10, 0x1105,
}};

template <std::size_t CycleSize>
inline std::vector<std::vector<int>>
materializePairCoverCycle(int n, int bucket_count,
                          const std::array<std::uint16_t, CycleSize> &cycle) {
    std::vector<std::vector<int>> buckets(
        static_cast<std::size_t>(bucket_count));
    for (int item = 0; item < n; ++item) {
        const std::uint16_t mask =
            cycle[static_cast<std::size_t>(item) % CycleSize];
        const int mask_bucket_count = bucket_count < 16 ? bucket_count : 16;
        for (int bucket = 0; bucket < mask_bucket_count; ++bucket) {
            if ((mask & (std::uint16_t{1} << bucket)) != 0)
                buckets[static_cast<std::size_t>(bucket)].push_back(item);
        }
    }
    return buckets;
}

inline std::vector<std::vector<int>> materializeStarPairCover(int n,
                                                              int bucket_count) {
    std::vector<std::vector<int>> buckets(
        static_cast<std::size_t>(bucket_count));
    buckets.front().reserve(static_cast<std::size_t>(n));
    for (int item = 0; item < n; ++item)
        buckets.front().push_back(item);
    return buckets;
}

} // namespace detail

struct PairCoverConflictAssignment {
    std::vector<std::vector<int>> worker_robots;
    std::vector<std::vector<std::pair<int, int>>> worker_pairs;
};

/// Returns buckets that cover every unordered pair of integers in [0, n).
///
/// The requested powers-of-two table entries for P-ARC paper team sizes
/// n in {4, 8, 16, 32, 64, 128, 256} and bucket counts 2, 4, 8, and 16 are
/// represented by the fixed mask cycles above.
/// The cycles are validated heuristic covers rather than certified optima. Other
/// positive inputs use the same deterministic cycles when available, or a valid
/// star cover.
inline std::vector<std::vector<int>> pairCoveringDesign(int n, int bucket_count) {
    if (n <= 1)
        throw std::invalid_argument("pairCoveringDesign requires n > 1");
    if (bucket_count <= 0)
        throw std::invalid_argument(
            "pairCoveringDesign requires bucket_count > 0");

    switch (bucket_count) {
    case 2:
        return detail::materializePairCoverCycle(n, bucket_count,
                                                 detail::kPairCoverB2Cycle);
    case 4:
        return detail::materializePairCoverCycle(n, bucket_count,
                                                 detail::kPairCoverB4Cycle);
    case 8:
        return detail::materializePairCoverCycle(n, bucket_count,
                                                 detail::kPairCoverB8Cycle);
    case 16:
        if (n == 8) {
            return detail::balancedEightRobotPairCoverWorkerRobotsB16();
        }
        return detail::materializePairCoverCycle(n, bucket_count,
                                                 detail::kPairCoverB16Cycle);
    default:
        return detail::materializeStarPairCover(n, bucket_count);
    }
}

/// Returns the worker-local robot subsets and pair assignments used by the
/// parallel conflict finder.
///
/// Most cases use the historical greedy "lightest covering bucket" assignment
/// on top of pairCoveringDesign(). The 8-robot / 16-worker paper ablation case
/// uses an explicit balanced assignment so no worker receives more than two
/// pairs while still touching at most three robots. The 64-robot / 16-worker
/// case uses explicit 20-robot supports plus exact-capacity pair assignment so
/// the worker pair loads flatten to the optimal 126 pairs per worker.
inline PairCoverConflictAssignment pairCoverConflictAssignment(int n,
                                                               int worker_count) {
    if (n <= 1)
        throw std::invalid_argument("pairCoverConflictAssignment requires n > 1");
    if (worker_count <= 0) {
        throw std::invalid_argument(
            "pairCoverConflictAssignment requires worker_count > 0");
    }

    PairCoverConflictAssignment assignment;
    if (n == 8 && worker_count == 16) {
        assignment.worker_robots =
            detail::balancedEightRobotPairCoverWorkerRobotsB16();
        assignment.worker_pairs =
            detail::balancedEightRobotPairAssignmentsB16();
        return assignment;
    }
    if (n == 64 && worker_count == 16) {
        static const PairCoverConflictAssignment kBalancedSixtyFourBySixteen =
            []() {
                PairCoverConflictAssignment balanced_assignment;
                balanced_assignment.worker_robots =
                    detail::balancedSixtyFourRobotPairCoverWorkerRobotsB16();
                balanced_assignment.worker_pairs =
                    detail::assignPairsWithCapacitatedFlow(
                        64, balanced_assignment.worker_robots,
                        std::vector<int>(16, 126));
                return balanced_assignment;
            }();
        return kBalancedSixtyFourBySixteen;
    }

    assignment.worker_robots = pairCoveringDesign(n, worker_count);
    assignment.worker_pairs.resize(static_cast<std::size_t>(worker_count));

    std::vector<std::uint64_t> robot_bucket_masks(static_cast<std::size_t>(n), 0);
    for (int worker = 0; worker < worker_count; ++worker) {
        for (const int robot :
             assignment.worker_robots[static_cast<std::size_t>(worker)]) {
            robot_bucket_masks[static_cast<std::size_t>(robot)] |=
                std::uint64_t{1} << worker;
        }
    }

    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            const std::uint64_t covering_workers =
                robot_bucket_masks[static_cast<std::size_t>(i)] &
                robot_bucket_masks[static_cast<std::size_t>(j)];
            if (covering_workers == 0) {
                throw std::runtime_error(
                    "pairCoverConflictAssignment did not cover every robot pair");
            }

            int best_worker = worker_count;
            std::size_t best_load = std::numeric_limits<std::size_t>::max();
            for (int worker = 0; worker < worker_count; ++worker) {
                if ((covering_workers & (std::uint64_t{1} << worker)) == 0)
                    continue;
                const std::size_t load = assignment.worker_pairs
                                             [static_cast<std::size_t>(worker)]
                                                 .size();
                if (load < best_load) {
                    best_worker = worker;
                    best_load = load;
                }
            }

            assignment.worker_pairs[static_cast<std::size_t>(best_worker)]
                .push_back({i, j});
        }
    }

    return assignment;
}

} // namespace comotion
