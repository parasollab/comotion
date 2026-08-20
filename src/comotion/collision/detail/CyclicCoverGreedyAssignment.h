#pragma once

#include "comotion/collision/ValidationTypes.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <utility>
#include <vector>

namespace comotion::detail {

struct CyclicCoverGreedyAssignment {
    std::vector<std::size_t> worker_by_pair;
    std::vector<std::vector<std::size_t>> worker_robots;
    std::vector<std::size_t> worker_pair_counts;
    std::size_t target_check_load = 0;
};

struct CyclicCoverGreedyAssignmentStats {
    std::size_t minimum_checks_per_worker = 0;
    std::size_t maximum_checks_per_worker = 0;
    std::size_t check_load_spread = 0;
    std::size_t minimum_robots_per_worker = 0;
    std::size_t maximum_robots_per_worker = 0;
    std::size_t total_robot_memberships = 0;
};

namespace cyclic_cover_greedy_detail {

inline constexpr std::array<std::size_t, 2> kDifferenceCover2 = {0, 1};
inline constexpr std::array<std::size_t, 3> kDifferenceCover4 = {0, 1, 2};
inline constexpr std::array<std::size_t, 4> kDifferenceCover8 = {0, 1, 2, 4};
inline constexpr std::array<std::size_t, 5> kDifferenceCover16 = {0, 1, 5, 6,
                                                                 8};

inline std::vector<std::size_t> differenceCover(std::size_t worker_count) {
    switch (worker_count) {
    case 2:
        return {kDifferenceCover2.begin(), kDifferenceCover2.end()};
    case 4:
        return {kDifferenceCover4.begin(), kDifferenceCover4.end()};
    case 8:
        return {kDifferenceCover8.begin(), kDifferenceCover8.end()};
    case 16:
        return {kDifferenceCover16.begin(), kDifferenceCover16.end()};
    default:
        throw std::invalid_argument(
            "cyclic-cover greedy assignment requires 2, 4, 8, or 16 "
            "workers");
    }
}

inline std::size_t bitCount(std::uint16_t mask) {
    std::size_t count = 0;
    while (mask != 0) {
        count += mask & std::uint16_t{1};
        mask >>= 1;
    }
    return count;
}

struct PairRecord {
    std::size_t first = 0;
    std::size_t second = 0;
    std::uint16_t legal_workers = 0;
    std::size_t legal_worker_count = 0;
};

inline void assignPair(CyclicCoverGreedyAssignment &result,
                       std::size_t robot_count, std::size_t first,
                       std::size_t second, std::size_t worker,
                       std::size_t worker_count) {
    const std::size_t canonical_first = std::min(first, second);
    const std::size_t canonical_second = std::max(first, second);
    const std::size_t pair_index =
        pairFrontierIndex(canonical_first, canonical_second, robot_count);
    if (result.worker_by_pair[pair_index] != worker_count) {
        throw std::runtime_error(
            "cyclic-cover greedy assignment assigned a pair more than once");
    }
    result.worker_by_pair[pair_index] = worker;
    ++result.worker_pair_counts[worker];
}

inline void deriveWorkerRobots(CyclicCoverGreedyAssignment &result,
                               std::size_t robot_count,
                               std::size_t worker_count) {
    std::vector<std::vector<char>> contains(
        worker_count, std::vector<char>(robot_count, 0));
    for (std::size_t first = 0; first < robot_count; ++first) {
        for (std::size_t second = first + 1; second < robot_count; ++second) {
            const std::size_t pair_index =
                pairFrontierIndex(first, second, robot_count);
            const std::size_t worker = result.worker_by_pair[pair_index];
            if (worker >= worker_count) {
                throw std::runtime_error(
                    "cyclic-cover greedy assignment did not assign every "
                    "pair");
            }
            contains[worker][first] = 1;
            contains[worker][second] = 1;
        }
    }

    result.worker_robots.assign(worker_count, {});
    for (std::size_t worker = 0; worker < worker_count; ++worker) {
        result.worker_robots[worker].reserve(robot_count);
        for (std::size_t robot = 0; robot < robot_count; ++robot) {
            if (contains[worker][robot])
                result.worker_robots[worker].push_back(robot);
        }
    }
}

inline void assignWithStarPacking(CyclicCoverGreedyAssignment &result,
                                  std::size_t robot_count,
                                  std::size_t worker_count) {
    using Pair = std::pair<std::size_t, std::size_t>;
    std::vector<std::vector<std::size_t>> outgoing(robot_count);
    for (std::size_t first = 0; first < robot_count; ++first) {
        for (std::size_t second = first + 1; second < robot_count; ++second) {
            if (second - first <= robot_count / 2)
                outgoing[first].push_back(second);
            else
                outgoing[second].push_back(first);
        }
    }

    std::vector<std::vector<Pair>> stars;
    for (std::size_t center = 0; center < robot_count; ++center) {
        auto &neighbors = outgoing[center];
        std::sort(neighbors.begin(), neighbors.end());
        for (std::size_t begin = 0; begin < neighbors.size();
             begin += result.target_check_load) {
            const std::size_t end = std::min(
                neighbors.size(), begin + result.target_check_load);
            std::vector<Pair> star;
            star.reserve(end - begin);
            for (std::size_t index = begin; index < end; ++index) {
                star.emplace_back(std::min(center, neighbors[index]),
                                  std::max(center, neighbors[index]));
            }
            stars.push_back(std::move(star));
        }
    }

    if (stars.size() > worker_count) {
        throw std::runtime_error(
            "cyclic-cover greedy star packing requires more stars than "
            "workers");
    }
    for (std::size_t worker = 0; worker < stars.size(); ++worker) {
        for (const auto &[first, second] : stars[worker]) {
            assignPair(result, robot_count, first, second, worker,
                       worker_count);
        }
    }
}

inline void assignWithCyclicCover(CyclicCoverGreedyAssignment &result,
                                  std::size_t robot_count,
                                  std::size_t worker_count,
                                  const std::vector<std::size_t> &cover) {
    std::vector<std::uint16_t> candidates(robot_count, 0);
    for (std::size_t robot = 0; robot < robot_count; ++robot) {
        const std::size_t shift = robot % worker_count;
        for (const std::size_t difference : cover) {
            const std::size_t worker = (shift + difference) % worker_count;
            candidates[robot] |=
                static_cast<std::uint16_t>(std::uint16_t{1} << worker);
        }
    }

    std::vector<PairRecord> pairs;
    pairs.reserve(pairFrontierSize(robot_count));
    for (std::size_t first = 0; first < robot_count; ++first) {
        for (std::size_t second = first + 1; second < robot_count; ++second) {
            const std::uint16_t legal_workers =
                candidates[first] & candidates[second];
            if (legal_workers == 0) {
                throw std::runtime_error(
                    "cyclic difference cover left a pair without a legal "
                    "worker");
            }
            pairs.push_back(PairRecord{first, second, legal_workers,
                                       bitCount(legal_workers)});
        }
    }

    std::sort(pairs.begin(), pairs.end(),
              [](const PairRecord &left, const PairRecord &right) {
                  if (left.legal_worker_count != right.legal_worker_count) {
                      return left.legal_worker_count <
                             right.legal_worker_count;
                  }
                  if (left.first != right.first)
                      return left.first < right.first;
                  return left.second < right.second;
              });

    for (const PairRecord &pair : pairs) {
        const std::size_t shift = pair.first % worker_count;
        std::size_t selected_worker = worker_count;
        std::size_t selected_load = std::numeric_limits<std::size_t>::max();
        std::size_t selected_distance =
            std::numeric_limits<std::size_t>::max();
        for (std::size_t worker = 0; worker < worker_count; ++worker) {
            if ((pair.legal_workers &
                 static_cast<std::uint16_t>(std::uint16_t{1} << worker)) ==
                0) {
                continue;
            }
            const std::size_t load = result.worker_pair_counts[worker];
            const std::size_t distance =
                (worker + worker_count - shift) % worker_count;
            if (load < selected_load ||
                (load == selected_load && distance < selected_distance)) {
                selected_worker = worker;
                selected_load = load;
                selected_distance = distance;
            }
        }
        if (selected_worker == worker_count) {
            throw std::runtime_error(
                "cyclic-cover greedy assignment could not select a worker");
        }
        assignPair(result, robot_count, pair.first, pair.second,
                   selected_worker, worker_count);
    }
}

} // namespace cyclic_cover_greedy_detail

// Assign every unordered pair exactly once. Small teams use one star per
// worker. Otherwise, fixed cyclic difference covers guarantee that every pair
// has a legal worker because D-D spans all worker shifts.
inline CyclicCoverGreedyAssignment
makeCyclicCoverGreedyAssignment(std::size_t robot_count,
                                std::size_t worker_count) {
    using namespace cyclic_cover_greedy_detail;
    if (robot_count < 2) {
        throw std::invalid_argument(
            "cyclic-cover greedy assignment requires at least two robots");
    }

    // Fetching the cover also rejects unsupported worker counts before either
    // branch is entered.
    const std::vector<std::size_t> cover = differenceCover(worker_count);
    const std::size_t pair_count = pairFrontierSize(robot_count);

    CyclicCoverGreedyAssignment result;
    result.worker_by_pair.assign(pair_count, worker_count);
    result.worker_pair_counts.assign(worker_count, 0);
    result.target_check_load =
        pair_count / worker_count + (pair_count % worker_count != 0 ? 1 : 0);

    if (robot_count < worker_count) {
        assignWithStarPacking(result, robot_count, worker_count);
    } else {
        assignWithCyclicCover(result, robot_count, worker_count, cover);
    }

    // Candidate sets are only an eligibility mechanism. Build final robot
    // lists from actual pair endpoints so unused candidates are discarded.
    deriveWorkerRobots(result, robot_count, worker_count);
    return result;
}

inline CyclicCoverGreedyAssignmentStats cyclicCoverGreedyAssignmentStats(
    const CyclicCoverGreedyAssignment &assignment) {
    CyclicCoverGreedyAssignmentStats stats;
    if (assignment.worker_pair_counts.empty())
        return stats;

    const auto [minimum_checks, maximum_checks] =
        std::minmax_element(assignment.worker_pair_counts.begin(),
                            assignment.worker_pair_counts.end());
    stats.minimum_checks_per_worker = *minimum_checks;
    stats.maximum_checks_per_worker = *maximum_checks;
    stats.check_load_spread = *maximum_checks - *minimum_checks;

    std::size_t minimum_robots = std::numeric_limits<std::size_t>::max();
    for (const auto &robots : assignment.worker_robots) {
        minimum_robots = std::min(minimum_robots, robots.size());
        stats.maximum_robots_per_worker =
            std::max(stats.maximum_robots_per_worker, robots.size());
        stats.total_robot_memberships += robots.size();
    }
    stats.minimum_robots_per_worker =
        minimum_robots == std::numeric_limits<std::size_t>::max()
            ? 0
            : minimum_robots;
    return stats;
}

inline std::shared_ptr<const CyclicCoverGreedyAssignment>
cachedCyclicCoverGreedyAssignment(std::size_t robot_count,
                                  std::size_t worker_count) {
    using Key = std::pair<std::size_t, std::size_t>;
    static std::mutex cache_mutex;
    static std::map<Key, std::shared_ptr<const CyclicCoverGreedyAssignment>>
        cache;
    const Key key{robot_count, worker_count};
    std::lock_guard<std::mutex> lock(cache_mutex);
    const auto found = cache.find(key);
    if (found != cache.end())
        return found->second;
    auto assignment = std::make_shared<const CyclicCoverGreedyAssignment>(
        makeCyclicCoverGreedyAssignment(robot_count, worker_count));
    cache.emplace(key, assignment);
    return assignment;
}

} // namespace comotion::detail
