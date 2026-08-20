#pragma once

#include "comotion/collision/ValidationTypes.h"
#include "comotion/collision/detail/PairCoveringDesign.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <queue>
#include <stdexcept>
#include <utility>
#include <vector>

namespace comotion::detail {

struct BalancedPairCoverAssignment {
    std::vector<std::size_t> worker_by_pair;
    std::vector<std::vector<std::size_t>> worker_robots;
    std::vector<std::size_t> worker_pair_counts;
    std::size_t load_bound = 0;
    std::size_t cover_bound = 0;
    std::size_t controlled_spill_pairs = 0;
};

namespace balanced_pair_cover_detail {

inline std::size_t robotCountForPairCapacity(std::size_t pair_capacity) {
    std::size_t robot_count = 0;
    while (robot_count * (robot_count - (robot_count > 0 ? 1 : 0)) / 2 <
           pair_capacity) {
        ++robot_count;
    }
    return robot_count;
}

inline std::size_t pairCoverIncidenceBound(std::size_t robot_count,
                                           std::size_t worker_count) {
    if (robot_count < 2 || worker_count == 0)
        return robot_count;
    for (std::size_t bound = 2; bound <= robot_count; ++bound) {
        const std::size_t appearances =
            (robot_count - 1 + (bound - 2)) / (bound - 1);
        if (worker_count * bound >= robot_count * appearances)
            return bound;
    }
    return robot_count;
}

struct FlowEdge {
    int to = -1;
    std::size_t capacity = 0;
    int reverse = -1;
};

class Dinic {
public:
    explicit Dinic(std::size_t node_count)
        : graph_(node_count), levels_(node_count), next_(node_count) {}

    std::size_t addEdge(int from, int to, std::size_t capacity) {
        const std::size_t index = graph_[static_cast<std::size_t>(from)].size();
        const int reverse_from =
            static_cast<int>(graph_[static_cast<std::size_t>(to)].size());
        const int reverse_to = static_cast<int>(index);
        graph_[static_cast<std::size_t>(from)].push_back(
            FlowEdge{to, capacity, reverse_from});
        graph_[static_cast<std::size_t>(to)].push_back(
            FlowEdge{from, 0, reverse_to});
        return index;
    }

    std::size_t flow(int source, int sink) {
        std::size_t total = 0;
        while (buildLevels(source, sink)) {
            std::fill(next_.begin(), next_.end(), 0);
            while (const std::size_t pushed =
                       send(source, sink,
                            std::numeric_limits<std::size_t>::max())) {
                total += pushed;
            }
        }
        return total;
    }

    std::size_t edgeFlow(int from, std::size_t edge_index) const {
        const auto &edge = graph_[static_cast<std::size_t>(from)][edge_index];
        return graph_[static_cast<std::size_t>(edge.to)]
                     [static_cast<std::size_t>(edge.reverse)]
                         .capacity;
    }

private:
    bool buildLevels(int source, int sink) {
        std::fill(levels_.begin(), levels_.end(), -1);
        std::queue<int> pending;
        levels_[static_cast<std::size_t>(source)] = 0;
        pending.push(source);
        while (!pending.empty()) {
            const int from = pending.front();
            pending.pop();
            for (const auto &edge : graph_[static_cast<std::size_t>(from)]) {
                if (edge.capacity == 0 ||
                    levels_[static_cast<std::size_t>(edge.to)] >= 0) {
                    continue;
                }
                levels_[static_cast<std::size_t>(edge.to)] =
                    levels_[static_cast<std::size_t>(from)] + 1;
                pending.push(edge.to);
            }
        }
        return levels_[static_cast<std::size_t>(sink)] >= 0;
    }

    std::size_t send(int from, int sink, std::size_t available) {
        if (from == sink)
            return available;
        auto &edge_index = next_[static_cast<std::size_t>(from)];
        while (edge_index < graph_[static_cast<std::size_t>(from)].size()) {
            auto &edge = graph_[static_cast<std::size_t>(from)][edge_index];
            if (edge.capacity > 0 &&
                levels_[static_cast<std::size_t>(edge.to)] ==
                    levels_[static_cast<std::size_t>(from)] + 1) {
                const std::size_t pushed =
                    send(edge.to, sink, std::min(available, edge.capacity));
                if (pushed > 0) {
                    edge.capacity -= pushed;
                    graph_[static_cast<std::size_t>(edge.to)]
                          [static_cast<std::size_t>(edge.reverse)]
                              .capacity += pushed;
                    return pushed;
                }
            }
            ++edge_index;
        }
        return 0;
    }

    std::vector<std::vector<FlowEdge>> graph_;
    std::vector<int> levels_;
    std::vector<std::size_t> next_;
};

struct PairRecord {
    std::size_t pair_index = 0;
    std::size_t robot_i = 0;
    std::size_t robot_j = 0;
};

struct EligibilityGroup {
    std::uint32_t worker_mask = 0;
    std::vector<PairRecord> pairs;
    std::vector<std::size_t> flow_edge_by_worker;
};

inline std::size_t deterministicCoreStride(std::size_t robot_count) {
    if (robot_count <= 2)
        return 1;
    std::size_t stride = std::max<std::size_t>(1, robot_count / 4 - 1);
    while (std::gcd(stride, robot_count) != 1)
        ++stride;
    return stride;
}

} // namespace balanced_pair_cover_detail

inline BalancedPairCoverAssignment
makeBalancedPairCoverAssignment(std::size_t robot_count,
                                std::size_t worker_count) {
    using namespace balanced_pair_cover_detail;
    if (robot_count < 2)
        throw std::invalid_argument(
            "balanced pair cover requires at least two robots");
    if (worker_count == 0 || worker_count > 16)
        throw std::invalid_argument(
            "balanced pair cover requires between 1 and 16 workers");

    const std::size_t pair_count = pairFrontierSize(robot_count);
    const std::size_t base_capacity = pair_count / worker_count;
    const std::size_t extra_capacity = pair_count % worker_count;
    std::vector<std::size_t> capacities(worker_count, base_capacity);
    for (std::size_t worker = 0; worker < extra_capacity; ++worker)
        ++capacities[worker];

    BalancedPairCoverAssignment result;
    result.worker_by_pair.assign(pair_count, worker_count);
    result.worker_pair_counts.assign(worker_count, 0);
    result.load_bound = robotCountForPairCapacity(
        *std::max_element(capacities.begin(), capacities.end()));
    result.cover_bound =
        pairCoverIncidenceBound(robot_count, worker_count);

    std::vector<std::vector<std::size_t>> cores(worker_count);
    if (worker_count == 1) {
        cores.front().reserve(robot_count);
        for (std::size_t robot = 0; robot < robot_count; ++robot)
            cores.front().push_back(robot);
    } else {
        const auto seeded = pairCoveringDesign(
            static_cast<int>(robot_count), static_cast<int>(worker_count));
        for (std::size_t worker = 0; worker < worker_count; ++worker) {
            cores[worker].reserve(seeded[worker].size());
            for (const int robot : seeded[worker])
                cores[worker].push_back(static_cast<std::size_t>(robot));
        }
    }

    const std::size_t target_core_size =
        std::min(robot_count, std::max(result.load_bound, result.cover_bound));
    const std::size_t stride = deterministicCoreStride(robot_count);
    std::size_t empty_core_index = 0;
    for (std::size_t worker = 0; worker < worker_count; ++worker) {
        if (!cores[worker].empty() || capacities[worker] == 0)
            continue;
        std::vector<char> selected(robot_count, 0);
        const std::size_t offset = empty_core_index * target_core_size;
        for (std::size_t slot = 0; slot < target_core_size; ++slot) {
            const std::size_t robot =
                ((offset + slot) * stride) % robot_count;
            if (!selected[robot]) {
                selected[robot] = 1;
                cores[worker].push_back(robot);
            }
        }
        ++empty_core_index;
    }

    std::vector<std::vector<char>> core_contains(
        worker_count, std::vector<char>(robot_count, 0));
    for (std::size_t worker = 0; worker < worker_count; ++worker) {
        for (const std::size_t robot : cores[worker])
            core_contains[worker][robot] = 1;
    }

    std::map<std::uint32_t, std::vector<PairRecord>> pairs_by_mask;
    for (std::size_t i = 0; i < robot_count; ++i) {
        for (std::size_t j = i + 1; j < robot_count; ++j) {
            std::uint32_t mask = 0;
            for (std::size_t worker = 0; worker < worker_count; ++worker) {
                if (core_contains[worker][i] && core_contains[worker][j])
                    mask |= std::uint32_t{1} << worker;
            }
            if (mask == 0)
                throw std::runtime_error(
                    "balanced pair-cover seed did not cover every pair");
            pairs_by_mask[mask].push_back(
                PairRecord{pairFrontierIndex(i, j, robot_count), i, j});
        }
    }

    std::vector<EligibilityGroup> groups;
    groups.reserve(pairs_by_mask.size());
    for (auto &[mask, pairs] : pairs_by_mask) {
        groups.push_back(
            EligibilityGroup{mask, std::move(pairs),
                             std::vector<std::size_t>(worker_count,
                                                      std::numeric_limits<
                                                          std::size_t>::max())});
    }

    const int source = 0;
    const int group_offset = 1;
    const int worker_offset =
        group_offset + static_cast<int>(groups.size());
    const int sink = worker_offset + static_cast<int>(worker_count);
    Dinic flow(static_cast<std::size_t>(sink + 1));
    for (std::size_t group_index = 0; group_index < groups.size();
         ++group_index) {
        const int group_node = group_offset + static_cast<int>(group_index);
        flow.addEdge(source, group_node, groups[group_index].pairs.size());
        for (std::size_t worker = 0; worker < worker_count; ++worker) {
            if ((groups[group_index].worker_mask &
                 (std::uint32_t{1} << worker)) == 0) {
                continue;
            }
            groups[group_index].flow_edge_by_worker[worker] = flow.addEdge(
                group_node, worker_offset + static_cast<int>(worker),
                groups[group_index].pairs.size());
        }
    }
    for (std::size_t worker = 0; worker < worker_count; ++worker) {
        flow.addEdge(worker_offset + static_cast<int>(worker), sink,
                     capacities[worker]);
    }
    (void)flow.flow(source, sink);

    std::vector<PairRecord> residual_pairs;
    for (std::size_t group_index = 0; group_index < groups.size();
         ++group_index) {
        auto &group = groups[group_index];
        std::size_t next_pair = 0;
        const int group_node = group_offset + static_cast<int>(group_index);
        for (std::size_t worker = 0; worker < worker_count; ++worker) {
            const std::size_t edge_index =
                group.flow_edge_by_worker[worker];
            if (edge_index == std::numeric_limits<std::size_t>::max())
                continue;
            const std::size_t assigned = flow.edgeFlow(group_node, edge_index);
            for (std::size_t count = 0; count < assigned; ++count) {
                const auto &pair = group.pairs[next_pair++];
                result.worker_by_pair[pair.pair_index] = worker;
                ++result.worker_pair_counts[worker];
            }
        }
        while (next_pair < group.pairs.size())
            residual_pairs.push_back(group.pairs[next_pair++]);
    }

    std::vector<std::vector<char>> actual_contains(
        worker_count, std::vector<char>(robot_count, 0));
    std::vector<std::size_t> actual_robot_counts(worker_count, 0);
    const auto addActualRobot = [&](std::size_t worker, std::size_t robot) {
        if (!actual_contains[worker][robot]) {
            actual_contains[worker][robot] = 1;
            ++actual_robot_counts[worker];
        }
    };
    for (std::size_t i = 0; i < robot_count; ++i) {
        for (std::size_t j = i + 1; j < robot_count; ++j) {
            const std::size_t pair_index =
                pairFrontierIndex(i, j, robot_count);
            const std::size_t worker = result.worker_by_pair[pair_index];
            if (worker < worker_count) {
                addActualRobot(worker, i);
                addActualRobot(worker, j);
            }
        }
    }

    std::size_t current_max = actual_robot_counts.empty()
                                  ? 0
                                  : *std::max_element(
                                        actual_robot_counts.begin(),
                                        actual_robot_counts.end());
    for (const auto &pair : residual_pairs) {
        std::size_t best_worker = worker_count;
        std::size_t best_projected_max =
            std::numeric_limits<std::size_t>::max();
        std::size_t best_added_robots =
            std::numeric_limits<std::size_t>::max();
        std::size_t best_projected_size =
            std::numeric_limits<std::size_t>::max();
        std::size_t best_remaining_capacity = 0;
        for (std::size_t worker = 0; worker < worker_count; ++worker) {
            if (result.worker_pair_counts[worker] >= capacities[worker])
                continue;
            const std::size_t added =
                static_cast<std::size_t>(!actual_contains[worker][pair.robot_i]) +
                static_cast<std::size_t>(!actual_contains[worker][pair.robot_j]);
            const std::size_t projected_size =
                actual_robot_counts[worker] + added;
            const std::size_t projected_max =
                std::max(current_max, projected_size);
            const std::size_t remaining_capacity =
                capacities[worker] - result.worker_pair_counts[worker];
            if (projected_max < best_projected_max ||
                (projected_max == best_projected_max &&
                 (added < best_added_robots ||
                  (added == best_added_robots &&
                   (projected_size < best_projected_size ||
                    (projected_size == best_projected_size &&
                     (remaining_capacity > best_remaining_capacity ||
                      (remaining_capacity == best_remaining_capacity &&
                       worker < best_worker)))))))) {
                best_worker = worker;
                best_projected_max = projected_max;
                best_added_robots = added;
                best_projected_size = projected_size;
                best_remaining_capacity = remaining_capacity;
            }
        }
        if (best_worker == worker_count)
            throw std::runtime_error(
                "balanced pair-cover assignment exhausted worker capacity");
        result.worker_by_pair[pair.pair_index] = best_worker;
        ++result.worker_pair_counts[best_worker];
        addActualRobot(best_worker, pair.robot_i);
        addActualRobot(best_worker, pair.robot_j);
        current_max = std::max(current_max, actual_robot_counts[best_worker]);
        ++result.controlled_spill_pairs;
    }

    result.worker_robots.resize(worker_count);
    for (std::size_t worker = 0; worker < worker_count; ++worker) {
        result.worker_robots[worker].reserve(actual_robot_counts[worker]);
        for (std::size_t robot = 0; robot < robot_count; ++robot) {
            if (actual_contains[worker][robot])
                result.worker_robots[worker].push_back(robot);
        }
        if (result.worker_pair_counts[worker] != capacities[worker]) {
            throw std::runtime_error(
                "balanced pair-cover assignment did not meet exact quota");
        }
    }
    return result;
}

inline std::shared_ptr<const BalancedPairCoverAssignment>
cachedBalancedPairCoverAssignment(std::size_t robot_count,
                                  std::size_t worker_count) {
    using Key = std::pair<std::size_t, std::size_t>;
    static std::mutex cache_mutex;
    static std::map<Key, std::shared_ptr<const BalancedPairCoverAssignment>>
        cache;
    const Key key{robot_count, worker_count};
    std::lock_guard<std::mutex> lock(cache_mutex);
    const auto found = cache.find(key);
    if (found != cache.end())
        return found->second;
    auto assignment = std::make_shared<const BalancedPairCoverAssignment>(
        makeBalancedPairCoverAssignment(robot_count, worker_count));
    cache.emplace(key, assignment);
    return assignment;
}

} // namespace comotion::detail
