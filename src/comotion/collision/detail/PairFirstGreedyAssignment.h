#pragma once

#include "comotion/collision/ValidationTypes.h"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <utility>
#include <vector>

namespace comotion::detail {

struct PairFirstGreedyAssignment {
    std::vector<std::size_t> worker_by_pair;
    std::vector<std::vector<std::size_t>> worker_robots;
    std::vector<std::size_t> worker_pair_counts;
    std::size_t load_bound = 0;
};

namespace pair_first_greedy_detail {

inline std::size_t robotCountForPairCapacity(std::size_t pair_capacity) {
    std::size_t robot_count = 0;
    while (robot_count * (robot_count - (robot_count > 0 ? 1 : 0)) / 2 <
           pair_capacity) {
        ++robot_count;
    }
    return robot_count;
}

} // namespace pair_first_greedy_detail

// Assign pairs in deterministic pair-frontier order. Pair quotas are hard
// constraints; within those constraints, each greedy decision first minimizes
// the projected maximum worker robot-set size and then endpoint replication.
inline PairFirstGreedyAssignment
makePairFirstGreedyAssignment(std::size_t robot_count,
                              std::size_t worker_count) {
    using namespace pair_first_greedy_detail;
    if (robot_count < 2) {
        throw std::invalid_argument(
            "pair-first greedy assignment requires at least two robots");
    }
    if (worker_count == 0) {
        throw std::invalid_argument(
            "pair-first greedy assignment requires at least one worker");
    }

    const std::size_t pair_count = pairFrontierSize(robot_count);
    const std::size_t base_capacity = pair_count / worker_count;
    const std::size_t extra_capacity = pair_count % worker_count;
    std::vector<std::size_t> capacities(worker_count, base_capacity);
    for (std::size_t worker = 0; worker < extra_capacity; ++worker)
        ++capacities[worker];

    PairFirstGreedyAssignment result;
    result.worker_by_pair.assign(pair_count, worker_count);
    result.worker_pair_counts.assign(worker_count, 0);
    result.worker_robots.resize(worker_count);
    result.load_bound = robotCountForPairCapacity(
        *std::max_element(capacities.begin(), capacities.end()));

    std::vector<std::vector<char>> contains(
        worker_count, std::vector<char>(robot_count, 0));
    std::vector<std::size_t> robot_counts(worker_count, 0);
    std::size_t current_max = 0;

    for (std::size_t i = 0; i < robot_count; ++i) {
        for (std::size_t j = i + 1; j < robot_count; ++j) {
            std::size_t best_worker = worker_count;
            std::size_t best_projected_max =
                std::numeric_limits<std::size_t>::max();
            std::size_t best_added_robots =
                std::numeric_limits<std::size_t>::max();
            std::size_t best_projected_size =
                std::numeric_limits<std::size_t>::max();
            std::size_t best_pair_load =
                std::numeric_limits<std::size_t>::max();

            for (std::size_t worker = 0; worker < worker_count; ++worker) {
                if (result.worker_pair_counts[worker] >= capacities[worker])
                    continue;

                const std::size_t added =
                    static_cast<std::size_t>(!contains[worker][i]) +
                    static_cast<std::size_t>(!contains[worker][j]);
                const std::size_t projected_size =
                    robot_counts[worker] + added;
                const std::size_t projected_max =
                    std::max(current_max, projected_size);
                const std::size_t pair_load =
                    result.worker_pair_counts[worker];

                if (projected_max < best_projected_max ||
                    (projected_max == best_projected_max &&
                     (added < best_added_robots ||
                      (added == best_added_robots &&
                       (projected_size < best_projected_size ||
                        (projected_size == best_projected_size &&
                         (pair_load < best_pair_load ||
                          (pair_load == best_pair_load &&
                           worker < best_worker)))))))) {
                    best_worker = worker;
                    best_projected_max = projected_max;
                    best_added_robots = added;
                    best_projected_size = projected_size;
                    best_pair_load = pair_load;
                }
            }

            if (best_worker == worker_count) {
                throw std::runtime_error(
                    "pair-first greedy assignment exhausted worker capacity");
            }

            const std::size_t pair_index =
                pairFrontierIndex(i, j, robot_count);
            result.worker_by_pair[pair_index] = best_worker;
            ++result.worker_pair_counts[best_worker];
            if (!contains[best_worker][i]) {
                contains[best_worker][i] = 1;
                ++robot_counts[best_worker];
            }
            if (!contains[best_worker][j]) {
                contains[best_worker][j] = 1;
                ++robot_counts[best_worker];
            }
            current_max = std::max(current_max, robot_counts[best_worker]);
        }
    }

    for (std::size_t worker = 0; worker < worker_count; ++worker) {
        if (result.worker_pair_counts[worker] != capacities[worker]) {
            throw std::runtime_error(
                "pair-first greedy assignment did not meet exact quota");
        }
        result.worker_robots[worker].reserve(robot_counts[worker]);
        for (std::size_t robot = 0; robot < robot_count; ++robot) {
            if (contains[worker][robot])
                result.worker_robots[worker].push_back(robot);
        }
    }

    return result;
}

inline std::shared_ptr<const PairFirstGreedyAssignment>
cachedPairFirstGreedyAssignment(std::size_t robot_count,
                                std::size_t worker_count) {
    using Key = std::pair<std::size_t, std::size_t>;
    static std::mutex cache_mutex;
    static std::map<Key, std::shared_ptr<const PairFirstGreedyAssignment>>
        cache;
    const Key key{robot_count, worker_count};
    std::lock_guard<std::mutex> lock(cache_mutex);
    const auto found = cache.find(key);
    if (found != cache.end())
        return found->second;
    auto assignment = std::make_shared<const PairFirstGreedyAssignment>(
        makePairFirstGreedyAssignment(robot_count, worker_count));
    cache.emplace(key, assignment);
    return assignment;
}

} // namespace comotion::detail
