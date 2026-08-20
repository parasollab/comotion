#include "comotion/collision/CollisionChecker.h"
#include "comotion/collision/ConflictChecker.h"
#include "comotion/collision/detail/BalancedPairCoverAssignment.h"
#include "comotion/collision/detail/CyclicCoverGreedyAssignment.h"
#include "comotion/collision/detail/PairFirstGreedyAssignment.h"
#include "comotion/collision/detail/PairCoveringDesign.h"
#include "comotion/planning/Path.h"
#include "comotion/robot/FlyingSphere.h"

#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace {

bool expectTrue(const std::string &label, bool value) {
    if (!value) {
        std::cerr << "vamp_conflict_scan_regression: " << label << "\n";
        return false;
    }
    return true;
}

comotion::Path makePathWithConflictsAt(std::size_t length,
                                   const std::vector<std::size_t> &timesteps) {
    comotion::Path path(length, {5.0, 0.0, 0.0});
    for (const auto timestep : timesteps) {
        if (timestep < path.size())
            path[timestep] = {0.0, 0.0, 0.0};
    }
    return path;
}

comotion::Path makeConstantPath(std::size_t length,
                            const std::vector<double> &config) {
    return comotion::Path(length, config);
}

std::shared_ptr<comotion::FlyingSphere> makeSphereRobot() {
    return std::make_shared<comotion::FlyingSphere>(0.4, -10.0, 10.0);
}

bool runCase(std::size_t t_begin, std::size_t expected_timestep) {
    auto robot_a = makeSphereRobot();
    auto robot_b = makeSphereRobot();
    std::vector<const comotion::RobotModel *> robots{robot_a.get(), robot_b.get()};
    std::vector<comotion::Path> paths{
        makePathWithConflictsAt(24, {5, 16}),
        makeConstantPath(24, {0.0, 0.0, 0.0}),
    };

    comotion::CompositePathValidationOptions options;
    options.check_environment = false;
    options.t_begin = t_begin;

    comotion::CollisionChecker pair_checker(comotion::CollisionChecker::Backend::Vamp);
    const auto pair = pair_checker.findFirstPairPathConflict(
        *robot_a, paths[0], *robot_b, paths[1], t_begin, paths[0].size());
    if (!expectTrue("findFirstPair returned a conflict", pair.has_value()))
        return false;
    if (!expectTrue("findFirstPair timestep is chronological first",
                    pair->timestep == expected_timestep)) {
        return false;
    }

    comotion::CollisionChecker first_checker(comotion::CollisionChecker::Backend::Vamp);
    const auto first = first_checker.findFirstCompositePathConflict(
        paths, robots, options);
    if (!expectTrue("findFirst returned a conflict", first.has_value()))
        return false;
    if (!expectTrue("findFirst timestep is chronological first",
                    first->timestep == expected_timestep &&
                        first->robot_i == 0 && first->robot_j == 1)) {
        return false;
    }

    comotion::CollisionChecker conflict_checker_backend(
        comotion::CollisionChecker::Backend::Vamp);
    comotion::ConflictChecker conflict_checker(conflict_checker_backend);
    const auto single = conflict_checker.findConflict(
        paths, robots, options, 0);
    if (!expectTrue("ConflictChecker::findConflict returned a conflict",
                    single.has_value()))
        return false;
    if (!expectTrue("findConflict matches findFirst",
                    single->timestep == static_cast<int>(expected_timestep) &&
                        single->robot_i == first->robot_i &&
                        single->robot_j == first->robot_j)) {
        return false;
    }

    comotion::CollisionChecker multi_backend(comotion::CollisionChecker::Backend::Vamp);
    comotion::ConflictChecker multi_checker(multi_backend);
    const auto conflicts = multi_checker.findConflicts(
        paths, robots, options, 0, 1, false, {});
    if (!expectTrue("findConflicts N=1 returned one conflict",
                    conflicts.size() == 1))
        return false;
    const auto &conflict = conflicts.front();
    if (!expectTrue("findConflicts N=1 matches findFirst",
                    conflict.conflict_timestep ==
                            static_cast<int>(expected_timestep) &&
                        conflict.seed_robot_i == first->robot_i &&
                        conflict.seed_robot_j == first->robot_j &&
                        conflict.robots == std::vector<int>({0, 1}) &&
                        conflict.window_begin_t ==
                            static_cast<int>(expected_timestep) &&
                        conflict.window_end_t ==
                            static_cast<int>(expected_timestep))) {
        return false;
    }
    return true;
}

bool runCrossPairOrderingCase() {
    auto robot_a = makeSphereRobot();
    auto robot_b = makeSphereRobot();
    auto robot_c = makeSphereRobot();
    std::vector<const comotion::RobotModel *> robots{
        robot_a.get(), robot_b.get(), robot_c.get()};

    auto path_a = makeConstantPath(24, {10.0, 0.0, 0.0});
    auto path_b = makeConstantPath(24, {20.0, 0.0, 0.0});
    auto path_c = makeConstantPath(24, {30.0, 0.0, 0.0});
    path_a[16] = {0.0, 0.0, 0.0};
    path_b[5] = {0.0, 0.0, 0.0};
    path_b[16] = {0.0, 0.0, 0.0};
    path_c[5] = {0.0, 0.0, 0.0};
    std::vector<comotion::Path> paths{path_a, path_b, path_c};

    comotion::CompositePathValidationOptions options;
    options.check_environment = false;
    options.t_begin = 0;

    comotion::CollisionChecker first_checker(comotion::CollisionChecker::Backend::Vamp);
    const auto first = first_checker.findFirstCompositePathConflict(
        paths, robots, options);
    if (!expectTrue("cross-pair findFirst returned a conflict",
                    first.has_value()))
        return false;
    if (!expectTrue("cross-pair findFirst returns earliest pair",
                    first->timestep == 5 && first->robot_i == 1 &&
                        first->robot_j == 2)) {
        return false;
    }

    comotion::CollisionChecker multi_backend(comotion::CollisionChecker::Backend::Vamp);
    comotion::ConflictChecker multi_checker(multi_backend);
    const auto conflicts = multi_checker.findConflicts(
        paths, robots, options, 0, 1, false, {});
    if (!expectTrue("cross-pair findConflicts N=1 returned one conflict",
                    conflicts.size() == 1))
        return false;
    const auto &conflict = conflicts.front();
    if (!expectTrue("cross-pair findConflicts N=1 matches findFirst",
                    conflict.conflict_timestep == 5 &&
                        conflict.seed_robot_i == first->robot_i &&
                        conflict.seed_robot_j == first->robot_j &&
                        conflict.robots == std::vector<int>({1, 2}))) {
        return false;
    }
    return true;
}

bool runExpandedUniqueCase() {
    constexpr std::size_t kLength = 24;
    std::vector<std::shared_ptr<comotion::FlyingSphere>> robot_storage;
    for (int i = 0; i < 4; ++i)
        robot_storage.push_back(makeSphereRobot());

    std::vector<const comotion::RobotModel *> robots;
    for (const auto &robot : robot_storage)
        robots.push_back(robot.get());

    auto path_a = makeConstantPath(kLength, {0.0, 0.0, 0.0});
    auto path_b = makeConstantPath(kLength, {10.0, 0.0, 0.0});
    auto path_c = makeConstantPath(kLength, {20.0, 0.0, 0.0});
    auto path_d = makeConstantPath(kLength, {5.0, 0.0, 0.0});
    path_b[5] = {0.0, 0.0, 0.0};
    path_c[6] = {5.0, 0.0, 0.0};
    std::vector<comotion::Path> paths{path_a, path_b, path_c, path_d};

    comotion::CompositePathValidationOptions options;
    options.check_environment = false;

    comotion::CollisionChecker backend(comotion::CollisionChecker::Backend::Vamp);
    comotion::ConflictChecker checker(backend);
    const auto conflicts = checker.findConflicts(
        paths, robots, options, 0, 0, true,
        [](const comotion::Conflict &conflict) {
            comotion::SubproblemConflict expanded;
            expanded.robots = {conflict.robot_i, conflict.robot_j};
            if (conflict.robot_i == 0 && conflict.robot_j == 1)
                expanded.robots = {0, 1, 2};
            expanded.conflict_timestep = conflict.timestep;
            expanded.window_begin_t = conflict.timestep;
            expanded.window_end_t = conflict.timestep;
            expanded.seed_robot_i = conflict.robot_i;
            expanded.seed_robot_j = conflict.robot_j;
            expanded.alpha = conflict.alpha;
            expanded.kind = conflict.kind;
            expanded.config_i = conflict.config_i;
            expanded.config_j = conflict.config_j;
            return expanded;
        });

    if (!expectTrue("expanded unique conflict count", conflicts.size() == 1))
        return false;
    return expectTrue("expanded unique claims transitive robot",
                      conflicts.front().robots == std::vector<int>({0, 1, 2}) &&
                          conflicts.front().conflict_timestep == 5);
}

bool runPairCoverCoverageCase() {
    for (const int workers : {2, 4, 8, 16}) {
        for (const int robots : {4, 8, 16, 32, 64, 128, 256}) {
            const auto buckets =
                comotion::detail::pairCoveringDesign(robots, workers);
            for (int i = 0; i < robots; ++i) {
                for (int j = i + 1; j < robots; ++j) {
                    bool covered = false;
                    for (const auto &bucket : buckets) {
                        const bool has_i =
                            std::find(bucket.begin(), bucket.end(), i) !=
                            bucket.end();
                        const bool has_j =
                            std::find(bucket.begin(), bucket.end(), j) !=
                            bucket.end();
                        covered = covered || (has_i && has_j);
                    }
                    if (!expectTrue("pair-cover covers every robot pair",
                                    covered))
                        return false;
                }
            }
        }
    }
    return true;
}

bool runBalancedPairCoverCase() {
    for (const std::size_t workers : {std::size_t{1}, std::size_t{2},
                                      std::size_t{4}, std::size_t{8},
                                      std::size_t{16}}) {
        for (const std::size_t robots : {std::size_t{2}, std::size_t{3},
                                         std::size_t{16}, std::size_t{64},
                                         std::size_t{256}}) {
            const auto assignment =
                comotion::detail::makeBalancedPairCoverAssignment(robots,
                                                                   workers);
            const auto repeated =
                comotion::detail::makeBalancedPairCoverAssignment(robots,
                                                                   workers);
            if (!expectTrue("balanced pair-cover deterministic owners",
                            assignment.worker_by_pair ==
                                repeated.worker_by_pair)) {
                return false;
            }
            if (!expectTrue("balanced pair-cover deterministic robots",
                            assignment.worker_robots ==
                                repeated.worker_robots)) {
                return false;
            }

            const std::size_t pair_count =
                comotion::pairFrontierSize(robots);
            const std::size_t base = pair_count / workers;
            const std::size_t extra = pair_count % workers;
            std::vector<std::size_t> observed(workers, 0);
            std::vector<std::vector<char>> contains(
                workers, std::vector<char>(robots, 0));
            for (std::size_t worker = 0; worker < workers; ++worker) {
                for (const std::size_t robot :
                     assignment.worker_robots[worker]) {
                    if (robot >= robots)
                        return false;
                    contains[worker][robot] = 1;
                }
            }
            for (std::size_t i = 0; i < robots; ++i) {
                for (std::size_t j = i + 1; j < robots; ++j) {
                    const std::size_t pair_index =
                        comotion::pairFrontierIndex(i, j, robots);
                    const std::size_t worker =
                        assignment.worker_by_pair[pair_index];
                    if (!expectTrue("balanced pair-cover worker valid",
                                    worker < workers)) {
                        return false;
                    }
                    ++observed[worker];
                    if (!expectTrue(
                            "balanced pair-cover worker owns endpoints",
                            contains[worker][i] && contains[worker][j])) {
                        return false;
                    }
                }
            }
            for (std::size_t worker = 0; worker < workers; ++worker) {
                const std::size_t expected = base + (worker < extra ? 1 : 0);
                if (!expectTrue("balanced pair-cover exact quota",
                                observed[worker] == expected &&
                                    assignment.worker_pair_counts[worker] ==
                                        expected)) {
                    return false;
                }
            }
        }
    }
    return true;
}

bool runPairFirstGreedyCase() {
    for (const std::size_t workers : {std::size_t{1}, std::size_t{2},
                                      std::size_t{3}, std::size_t{4},
                                      std::size_t{5}, std::size_t{8},
                                      std::size_t{16}, std::size_t{17}}) {
        for (const std::size_t robots : {std::size_t{2}, std::size_t{3},
                                         std::size_t{16}, std::size_t{64},
                                         std::size_t{256}}) {
            const auto assignment =
                comotion::detail::makePairFirstGreedyAssignment(robots,
                                                                 workers);
            const auto repeated =
                comotion::detail::makePairFirstGreedyAssignment(robots,
                                                                 workers);
            if (!expectTrue("pair-first greedy deterministic owners",
                            assignment.worker_by_pair ==
                                repeated.worker_by_pair)) {
                return false;
            }
            if (!expectTrue("pair-first greedy deterministic robots",
                            assignment.worker_robots ==
                                repeated.worker_robots)) {
                return false;
            }

            const std::size_t pair_count =
                comotion::pairFrontierSize(robots);
            const std::size_t base = pair_count / workers;
            const std::size_t extra = pair_count % workers;
            std::vector<std::size_t> observed(workers, 0);
            std::vector<std::vector<char>> contains(
                workers, std::vector<char>(robots, 0));
            for (std::size_t worker = 0; worker < workers; ++worker) {
                if (!expectTrue(
                        "pair-first greedy robot list sorted",
                        std::is_sorted(
                            assignment.worker_robots[worker].begin(),
                            assignment.worker_robots[worker].end()))) {
                    return false;
                }
                if (!expectTrue(
                        "pair-first greedy robot list unique",
                        std::adjacent_find(
                            assignment.worker_robots[worker].begin(),
                            assignment.worker_robots[worker].end()) ==
                            assignment.worker_robots[worker].end())) {
                    return false;
                }
                for (const std::size_t robot :
                     assignment.worker_robots[worker]) {
                    if (!expectTrue("pair-first greedy robot valid",
                                    robot < robots)) {
                        return false;
                    }
                    contains[worker][robot] = 1;
                }
            }
            std::vector<std::vector<char>> expected_contains(
                workers, std::vector<char>(robots, 0));
            for (std::size_t i = 0; i < robots; ++i) {
                for (std::size_t j = i + 1; j < robots; ++j) {
                    const std::size_t pair_index =
                        comotion::pairFrontierIndex(i, j, robots);
                    const std::size_t worker =
                        assignment.worker_by_pair[pair_index];
                    if (!expectTrue("pair-first greedy worker valid",
                                    worker < workers)) {
                        return false;
                    }
                    ++observed[worker];
                    expected_contains[worker][i] = 1;
                    expected_contains[worker][j] = 1;
                    if (!expectTrue(
                            "pair-first greedy worker owns endpoints",
                            contains[worker][i] && contains[worker][j])) {
                        return false;
                    }
                }
            }
            for (std::size_t worker = 0; worker < workers; ++worker) {
                const std::size_t expected = base + (worker < extra ? 1 : 0);
                if (!expectTrue("pair-first greedy exact quota",
                                observed[worker] == expected &&
                                    assignment.worker_pair_counts[worker] ==
                                        expected)) {
                    return false;
                }
                for (std::size_t robot = 0; robot < robots; ++robot) {
                    if (!expectTrue(
                            "pair-first greedy robot set is exact endpoint "
                            "union",
                            contains[worker][robot] ==
                                expected_contains[worker][robot])) {
                        return false;
                    }
                }
            }
        }
    }

    const auto golden =
        comotion::detail::makePairFirstGreedyAssignment(6, 2);
    const std::vector<std::size_t> expected_owners = {
        0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 0, 0};
    if (!expectTrue("pair-first greedy scoring order",
                    golden.worker_by_pair == expected_owners)) {
        return false;
    }
    return true;
}

bool runCyclicCoverGreedyCase() {
    for (const std::size_t workers : {std::size_t{2}, std::size_t{4},
                                      std::size_t{8}, std::size_t{16}}) {
        const auto cover =
            comotion::detail::cyclic_cover_greedy_detail::differenceCover(
                workers);
        for (std::size_t first_shift = 0; first_shift < workers;
             ++first_shift) {
            for (std::size_t second_shift = 0; second_shift < workers;
                 ++second_shift) {
                bool intersects = false;
                for (const std::size_t first_difference : cover) {
                    for (const std::size_t second_difference : cover) {
                        intersects =
                            intersects ||
                            (first_shift + first_difference) % workers ==
                                (second_shift + second_difference) % workers;
                    }
                }
                if (!expectTrue(
                        "cyclic-cover shifted candidate sets intersect",
                        intersects)) {
                    return false;
                }
            }
        }

        for (const std::size_t robots : {
                 std::size_t{4}, std::size_t{8}, std::size_t{16},
                 std::size_t{32}, std::size_t{64}, std::size_t{128},
                 std::size_t{256}}) {
            const auto assignment =
                comotion::detail::makeCyclicCoverGreedyAssignment(robots,
                                                                   workers);
            const auto repeated =
                comotion::detail::makeCyclicCoverGreedyAssignment(robots,
                                                                   workers);
            if (!expectTrue("cyclic-cover greedy deterministic owners",
                            assignment.worker_by_pair ==
                                repeated.worker_by_pair) ||
                !expectTrue("cyclic-cover greedy deterministic robots",
                            assignment.worker_robots ==
                                repeated.worker_robots)) {
                return false;
            }

            const std::size_t pair_count =
                comotion::pairFrontierSize(robots);
            const std::size_t target_load =
                (pair_count + workers - 1) / workers;
            if (!expectTrue("cyclic-cover greedy worker result count",
                            assignment.worker_robots.size() == workers &&
                                assignment.worker_pair_counts.size() ==
                                    workers) ||
                !expectTrue("cyclic-cover greedy pair frontier size",
                            assignment.worker_by_pair.size() == pair_count) ||
                !expectTrue("cyclic-cover greedy target load",
                            assignment.target_check_load == target_load)) {
                return false;
            }

            std::vector<std::size_t> observed(workers, 0);
            std::vector<std::vector<char>> expected_contains(
                workers, std::vector<char>(robots, 0));
            for (std::size_t first = 0; first < robots; ++first) {
                for (std::size_t second = first + 1; second < robots;
                     ++second) {
                    const std::size_t pair_index =
                        comotion::pairFrontierIndex(first, second, robots);
                    const std::size_t worker =
                        assignment.worker_by_pair[pair_index];
                    if (!expectTrue("cyclic-cover greedy worker valid",
                                    worker < workers)) {
                        return false;
                    }
                    ++observed[worker];
                    expected_contains[worker][first] = 1;
                    expected_contains[worker][second] = 1;
                }
            }

            std::size_t observed_total = 0;
            std::size_t observed_max_load = 0;
            std::size_t observed_max_robots = 0;
            for (std::size_t worker = 0; worker < workers; ++worker) {
                observed_total += observed[worker];
                observed_max_load =
                    std::max(observed_max_load, observed[worker]);
                observed_max_robots = std::max(
                    observed_max_robots,
                    assignment.worker_robots[worker].size());
                if (!expectTrue(
                        "cyclic-cover greedy pair counts match owners",
                        observed[worker] ==
                            assignment.worker_pair_counts[worker]) ||
                    !expectTrue(
                        "cyclic-cover greedy robot list sorted",
                        std::is_sorted(
                            assignment.worker_robots[worker].begin(),
                            assignment.worker_robots[worker].end())) ||
                    !expectTrue(
                        "cyclic-cover greedy robot list unique",
                        std::adjacent_find(
                            assignment.worker_robots[worker].begin(),
                            assignment.worker_robots[worker].end()) ==
                            assignment.worker_robots[worker].end())) {
                    return false;
                }

                std::vector<char> actual_contains(robots, 0);
                for (const std::size_t robot :
                     assignment.worker_robots[worker]) {
                    if (!expectTrue("cyclic-cover greedy robot valid",
                                    robot < robots)) {
                        return false;
                    }
                    actual_contains[robot] = 1;
                }
                if (!expectTrue(
                        "cyclic-cover greedy robot set is exact endpoint "
                        "union",
                        actual_contains == expected_contains[worker])) {
                    return false;
                }
            }

            const std::size_t robot_bound =
                robots < workers ? target_load + 1
                                 : cover.size() * robots / workers;
            if (!expectTrue("cyclic-cover greedy assigns every pair once",
                            observed_total == pair_count) ||
                !expectTrue("cyclic-cover greedy ideal maximum pair load",
                            observed_max_load == target_load) ||
                !expectTrue("cyclic-cover greedy robot bound",
                            observed_max_robots <= robot_bound)) {
                return false;
            }

            const auto stats =
                comotion::detail::cyclicCoverGreedyAssignmentStats(
                    assignment);
            if (!expectTrue("cyclic-cover greedy statistics maximum load",
                            stats.maximum_checks_per_worker ==
                                observed_max_load) ||
                !expectTrue("cyclic-cover greedy statistics maximum robots",
                            stats.maximum_robots_per_worker ==
                                observed_max_robots)) {
                return false;
            }
        }
    }

    struct FocusedExpectation {
        std::size_t robots;
        std::size_t workers;
        std::size_t maximum_checks;
        std::size_t maximum_robots;
    };
    for (const auto expected : {
             FocusedExpectation{4, 8, 1, 2},
             FocusedExpectation{4, 16, 1, 2},
             FocusedExpectation{8, 16, 2, 3},
             FocusedExpectation{16, 16, 8, 5},
             FocusedExpectation{256, 16, 2040, 80},
             FocusedExpectation{256, 8, 4080, 128},
             FocusedExpectation{256, 2, 16320, 256},
         }) {
        const auto assignment =
            comotion::detail::makeCyclicCoverGreedyAssignment(
                expected.robots, expected.workers);
        const auto stats =
            comotion::detail::cyclicCoverGreedyAssignmentStats(assignment);
        if (!expectTrue("cyclic-cover greedy focused maximum checks",
                        stats.maximum_checks_per_worker ==
                            expected.maximum_checks) ||
            !expectTrue("cyclic-cover greedy focused maximum robots",
                        stats.maximum_robots_per_worker <=
                            expected.maximum_robots)) {
            return false;
        }
    }
    return true;
}

bool runParallelMatchesSequentialCase() {
    constexpr std::size_t kRobotCount = 16;
    constexpr std::size_t kLength = 32;
    std::vector<std::shared_ptr<comotion::FlyingSphere>> robot_storage;
    std::vector<const comotion::RobotModel *> robots;
    std::vector<comotion::Path> paths;
    for (std::size_t robot = 0; robot < kRobotCount; ++robot) {
        robot_storage.push_back(makeSphereRobot());
        robots.push_back(robot_storage.back().get());
        paths.push_back(makeConstantPath(
            kLength, {10.0 * static_cast<double>(robot), 0.0, 0.0}));
    }
    paths[1][5] = paths[0][5];
    paths[3][6] = paths[2][6];
    paths[5][7] = paths[4][7];

    comotion::CompositePathValidationOptions sequential_options;
    sequential_options.check_environment = false;
    sequential_options.inter_robot_conflict_batch_mode =
        comotion::InterRobotConflictBatchMode::OptimisticIndependent;

    comotion::CollisionChecker sequential_backend(
        comotion::CollisionChecker::Backend::Vamp);
    comotion::ConflictChecker sequential_checker(sequential_backend);
    const auto sequential = sequential_checker.findConflicts(
        paths, robots, sequential_options, 0, 4, true, {});

    auto parallel_options = sequential_options;
    parallel_options.conflict_find_parallel_workers = 4;
    parallel_options.conflict_find_parallel_horizon = 8;
    parallel_options.conflict_find_parallel_assignment =
        comotion::ConflictFindParallelAssignment::PairCover;
    comotion::CollisionChecker parallel_backend(
        comotion::CollisionChecker::Backend::Vamp);
    comotion::ConflictChecker parallel_checker(parallel_backend);
    const auto parallel = parallel_checker.findConflicts(
        paths, robots, parallel_options, 0, 4, true, {});

    if (!expectTrue("parallel pair-cover conflict count matches sequential",
                    parallel.size() == sequential.size()))
        return false;
    for (std::size_t i = 0; i < sequential.size(); ++i) {
        if (!expectTrue(
                "parallel pair-cover conflicts match sequential",
                parallel[i].seed_robot_i == sequential[i].seed_robot_i &&
                    parallel[i].seed_robot_j == sequential[i].seed_robot_j &&
                    parallel[i].conflict_timestep ==
                        sequential[i].conflict_timestep))
            return false;
    }

    parallel_options.conflict_find_parallel_assignment =
        comotion::ConflictFindParallelAssignment::BalancedPairCover;
    comotion::CollisionChecker balanced_backend(
        comotion::CollisionChecker::Backend::Vamp);
    comotion::ConflictChecker balanced_checker(balanced_backend);
    const auto balanced = balanced_checker.findConflicts(
        paths, robots, parallel_options, 0, 4, true, {});
    if (!expectTrue("balanced pair-cover conflict count matches sequential",
                    balanced.size() == sequential.size())) {
        return false;
    }
    for (std::size_t i = 0; i < sequential.size(); ++i) {
        if (!expectTrue(
                "balanced pair-cover conflicts match sequential",
                balanced[i].seed_robot_i == sequential[i].seed_robot_i &&
                    balanced[i].seed_robot_j == sequential[i].seed_robot_j &&
                    balanced[i].conflict_timestep ==
                        sequential[i].conflict_timestep)) {
            return false;
        }
    }

    parallel_options.conflict_find_parallel_assignment =
        comotion::ConflictFindParallelAssignment::PairFirstGreedy;
    comotion::CollisionChecker pair_first_backend(
        comotion::CollisionChecker::Backend::Vamp);
    comotion::ConflictChecker pair_first_checker(pair_first_backend);
    const auto pair_first = pair_first_checker.findConflicts(
        paths, robots, parallel_options, 0, 4, true, {});
    if (!expectTrue("pair-first greedy conflict count matches sequential",
                    pair_first.size() == sequential.size())) {
        return false;
    }
    for (std::size_t i = 0; i < sequential.size(); ++i) {
        if (!expectTrue(
                "pair-first greedy conflicts match sequential",
                pair_first[i].seed_robot_i == sequential[i].seed_robot_i &&
                    pair_first[i].seed_robot_j ==
                        sequential[i].seed_robot_j &&
                    pair_first[i].conflict_timestep ==
                        sequential[i].conflict_timestep)) {
            return false;
        }
    }

    parallel_options.conflict_find_parallel_assignment =
        comotion::ConflictFindParallelAssignment::CyclicCoverGreedy;
    comotion::CollisionChecker cyclic_backend(
        comotion::CollisionChecker::Backend::Vamp);
    comotion::ConflictChecker cyclic_checker(cyclic_backend);
    const auto cyclic = cyclic_checker.findConflicts(
        paths, robots, parallel_options, 0, 4, true, {});
    if (!expectTrue("cyclic-cover greedy conflict count matches sequential",
                    cyclic.size() == sequential.size())) {
        return false;
    }
    for (std::size_t i = 0; i < sequential.size(); ++i) {
        if (!expectTrue(
                "cyclic-cover greedy conflicts match sequential",
                cyclic[i].seed_robot_i == sequential[i].seed_robot_i &&
                    cyclic[i].seed_robot_j == sequential[i].seed_robot_j &&
                    cyclic[i].conflict_timestep ==
                        sequential[i].conflict_timestep)) {
            return false;
        }
    }
    return true;
}

} // namespace

int main() {
    if (!runCase(0, 5))
        return 1;
    if (!runCase(6, 16))
        return 1;
    if (!runCrossPairOrderingCase())
        return 1;
    if (!runExpandedUniqueCase())
        return 1;
    if (!runPairCoverCoverageCase())
        return 1;
    if (!runBalancedPairCoverCase())
        return 1;
    if (!runPairFirstGreedyCase())
        return 1;
    if (!runCyclicCoverGreedyCase())
        return 1;
    if (!runParallelMatchesSequentialCase())
        return 1;

    std::cout << "vamp_conflict_scan_regression: OK\n";
    return 0;
}
