#include "comotion/collision/ConflictChecker.h"
#include "comotion/robot/FlyingSphere.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

namespace fs = std::filesystem;

bool expectTrue(const char *label, bool condition) {
    if (!condition) {
        std::cerr << "find_conflict_pairs_regression: " << label << "\n";
        return false;
    }
    return true;
}

comotion::Path makeConstantPath(std::size_t length,
                            const std::vector<double> &config) {
    comotion::Path path;
    path.reserve(length);
    path.waypoint_timesteps_.reserve(length);
    for (std::size_t i = 0; i < length; ++i) {
        path.push_back(config);
        path.waypoint_timesteps_.push_back(i);
    }
    return path;
}

std::shared_ptr<comotion::RobotModel> makeSphereRobot() {
    return std::make_shared<comotion::FlyingSphere>(0.5, -1000.0, 1000.0);
}

comotion::SubproblemConflict
makeExpandedConflict(const comotion::Conflict &conflict) {
    comotion::SubproblemConflict expanded;
    expanded.robots = {conflict.robot_i, conflict.robot_j};
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
}

bool testAllPairsAndCap() {
    constexpr std::size_t kLength = 32;
    std::vector<std::shared_ptr<comotion::RobotModel>> robots;
    for (int i = 0; i < 5; ++i)
        robots.push_back(makeSphereRobot());

    std::vector<comotion::Path> paths;
    paths.push_back(makeConstantPath(kLength, {0.0, 0.0, 0.0}));
    paths.push_back(makeConstantPath(kLength, {100.0, 0.0, 0.0}));
    paths.push_back(makeConstantPath(kLength, {200.0, 0.0, 0.0}));
    paths.push_back(makeConstantPath(kLength, {0.0, 100.0, 0.0}));
    paths.push_back(makeConstantPath(kLength, {100.0, 100.0, 0.0}));

    paths[1][10] = {0.0, 0.0, 0.0};
    paths[1][20] = {20.0, 0.0, 0.0};
    paths[2][20] = {20.0, 0.0, 0.0};
    paths[4][5] = {0.0, 100.0, 0.0};

    std::vector<const comotion::RobotModel *> ptrs;
    ptrs.reserve(robots.size());
    for (const auto &robot : robots)
        ptrs.push_back(robot.get());

    comotion::CollisionChecker collision_checker(
        comotion::CollisionChecker::Backend::Spheres);
    comotion::ConflictChecker checker(collision_checker);

    const auto conflicts = checker.findConflicts(paths, ptrs, 0, 0, false);
    if (!expectTrue("all-pairs conflict count", conflicts.size() == 3))
        return false;
    if (!expectTrue("earliest conflict first",
                    conflicts[0].robots == std::vector<int>({3, 4}) &&
                        conflicts[0].conflict_timestep == 5 &&
                        conflicts[0].window_begin_t == 5 &&
                        conflicts[0].window_end_t == 5)) {
        return false;
    }
    if (!expectTrue("second conflict order",
                    conflicts[1].robots == std::vector<int>({0, 1}) &&
                        conflicts[1].conflict_timestep == 10)) {
        return false;
    }
    if (!expectTrue("third conflict order",
                    conflicts[2].robots == std::vector<int>({1, 2}) &&
                        conflicts[2].conflict_timestep == 20)) {
        return false;
    }

    const auto capped = checker.findConflicts(paths, ptrs, 0, 2, false);
    if (!expectTrue("capped conflict count", capped.size() == 2))
        return false;
    return expectTrue(
        "cap keeps first conflicts in composite timestep order",
        capped[0].conflict_timestep == 5 &&
            capped[1].conflict_timestep == 10);
}

bool testUniqueKeepsNonOverlappingRobotSets() {
    constexpr std::size_t kLength = 32;
    std::vector<std::shared_ptr<comotion::RobotModel>> robots;
    for (int i = 0; i < 4; ++i)
        robots.push_back(makeSphereRobot());

    std::vector<comotion::Path> paths;
    paths.push_back(makeConstantPath(kLength, {0.0, 0.0, 0.0}));
    paths.push_back(makeConstantPath(kLength, {100.0, 0.0, 0.0}));
    paths.push_back(makeConstantPath(kLength, {200.0, 0.0, 0.0}));
    paths.push_back(makeConstantPath(kLength, {300.0, 0.0, 0.0}));

    paths[1][10] = {0.0, 0.0, 0.0};
    paths[3][20] = {200.0, 0.0, 0.0};

    std::vector<const comotion::RobotModel *> ptrs;
    ptrs.reserve(robots.size());
    for (const auto &robot : robots)
        ptrs.push_back(robot.get());

    comotion::CollisionChecker collision_checker(
        comotion::CollisionChecker::Backend::Spheres);
    comotion::ConflictChecker checker(collision_checker);

    const auto all_conflicts = checker.findConflicts(paths, ptrs, 0, 0, false);
    if (!expectTrue("non-unique pair count", all_conflicts.size() == 2))
        return false;

    const auto unique_conflicts = checker.findConflicts(paths, ptrs, 0, 0, true);
    if (!expectTrue("unique non-overlap count", unique_conflicts.size() == 2))
        return false;
    return expectTrue("unique non-overlap keeps both pairs",
                      unique_conflicts[0].robots == std::vector<int>({0, 1}) &&
                          unique_conflicts[1].robots ==
                              std::vector<int>({2, 3}));
}

bool testUniqueDiscardsLaterOverlappingWindowlessConflict() {
    constexpr std::size_t kLength = 32;
    std::vector<std::shared_ptr<comotion::RobotModel>> robots;
    for (int i = 0; i < 3; ++i)
        robots.push_back(makeSphereRobot());

    std::vector<comotion::Path> paths;
    paths.push_back(makeConstantPath(kLength, {0.0, 0.0, 0.0}));
    paths.push_back(makeConstantPath(kLength, {100.0, 0.0, 0.0}));
    paths.push_back(makeConstantPath(kLength, {200.0, 0.0, 0.0}));

    paths[1][10] = {0.0, 0.0, 0.0};
    paths[2][20] = {0.0, 0.0, 0.0};

    std::vector<const comotion::RobotModel *> ptrs;
    ptrs.reserve(robots.size());
    for (const auto &robot : robots)
        ptrs.push_back(robot.get());

    comotion::CollisionChecker collision_checker(
        comotion::CollisionChecker::Backend::Spheres);
    comotion::ConflictChecker checker(collision_checker);

    const auto unique_conflicts = checker.findConflicts(
        paths, ptrs, 0, 0, true,
        [](const comotion::Conflict &conflict) {
            auto expanded = makeExpandedConflict(conflict);
            expanded.robots = {0, 1, 2};
            expanded.window_begin_t = conflict.timestep;
            expanded.window_end_t = conflict.timestep;
            return expanded;
        });
    if (!expectTrue("unique discard later count", unique_conflicts.size() == 1))
        return false;
    return expectTrue("unique discard later keeps earliest expanded conflict",
                      unique_conflicts[0].conflict_timestep == 10 &&
                          unique_conflicts[0].robots ==
                              std::vector<int>({0, 1, 2}));
}

bool testUniqueDiscardsLaterClaimedConflictEvenWithWiderWindow() {
    constexpr std::size_t kLength = 40;
    std::vector<std::shared_ptr<comotion::RobotModel>> robots;
    for (int i = 0; i < 4; ++i)
        robots.push_back(makeSphereRobot());

    std::vector<comotion::Path> paths;
    paths.push_back(makeConstantPath(kLength, {0.0, 0.0, 0.0}));
    paths.push_back(makeConstantPath(kLength, {100.0, 0.0, 0.0}));
    paths.push_back(makeConstantPath(kLength, {200.0, 0.0, 0.0}));
    paths.push_back(makeConstantPath(kLength, {300.0, 0.0, 0.0}));

    paths[1][10] = {0.0, 0.0, 0.0};
    paths[2][11] = {0.0, 0.0, 0.0};
    paths[3][11] = {0.0, 0.0, 0.0};

    std::vector<const comotion::RobotModel *> ptrs;
    ptrs.reserve(robots.size());
    for (const auto &robot : robots)
        ptrs.push_back(robot.get());

    comotion::CollisionChecker collision_checker(
        comotion::CollisionChecker::Backend::Spheres);
    comotion::ConflictChecker checker(collision_checker);

    const auto unique_conflicts = checker.findConflicts(
        paths, ptrs, 0, 0, true,
        [](const comotion::Conflict &conflict) {
            auto expanded = makeExpandedConflict(conflict);
            if (conflict.robot_i == 0 && conflict.robot_j == 1) {
                expanded.robots = {0, 1, 2};
                expanded.window_begin_t = 8;
                expanded.window_end_t = 12;
            } else if (conflict.robot_i == 2 && conflict.robot_j == 3) {
                expanded.robots = {2, 3};
                expanded.window_begin_t = 0;
                expanded.window_end_t = 30;
            }
            return expanded;
        });
    if (!expectTrue("unique claimed discard count", unique_conflicts.size() == 1))
        return false;
    return expectTrue("unique claimed discard keeps earliest representative",
                      unique_conflicts[0].robots == std::vector<int>({0, 1, 2}) &&
                          unique_conflicts[0].window_begin_t == 8 &&
                          unique_conflicts[0].window_end_t == 12 &&
                          unique_conflicts[0].conflict_timestep == 10);
}

bool testFindConflictPerPathStarts() {
    constexpr std::size_t kLength = 32;
    std::vector<std::shared_ptr<comotion::RobotModel>> robots;
    for (int i = 0; i < 3; ++i)
        robots.push_back(makeSphereRobot());

    std::vector<comotion::Path> paths;
    paths.push_back(makeConstantPath(kLength, {0.0, 0.0, 0.0}));
    paths.push_back(makeConstantPath(kLength, {100.0, 0.0, 0.0}));
    paths.push_back(makeConstantPath(kLength, {200.0, 0.0, 0.0}));
    paths[1][10] = {0.0, 0.0, 0.0};

    std::vector<const comotion::RobotModel *> ptrs;
    ptrs.reserve(robots.size());
    for (const auto &robot : robots)
        ptrs.push_back(robot.get());

    comotion::CompositePathValidationOptions options;
    options.check_environment = false;
    options.per_path_t_begin = {0, 0, 15};

    comotion::CollisionChecker collision_checker(
        comotion::CollisionChecker::Backend::Spheres);
    comotion::ConflictChecker checker(collision_checker);

    std::vector<std::size_t> next_t_begin_by_robot;
    const auto conflict =
        checker.findConflict(paths, ptrs, options, 0, &next_t_begin_by_robot);
    if (!expectTrue("findConflict per-path start conflict found",
                    conflict.has_value()))
        return false;
    if (!expectTrue("findConflict per-path start robots",
                    conflict->robot_i == 0 && conflict->robot_j == 1 &&
                        conflict->timestep == 10)) {
        return false;
    }
    if (!expectTrue("findConflict next_t_begin size",
                    next_t_begin_by_robot.size() == paths.size())) {
        return false;
    }
    return expectTrue("findConflict next_t_begin tracks scanned frontier",
                      next_t_begin_by_robot[0] == 10 &&
                          next_t_begin_by_robot[1] == 10 &&
                          next_t_begin_by_robot[2] == 15);
}

bool testFindConflictPairsPerPathStartsAndProgress() {
    constexpr std::size_t kLength = 40;
    std::vector<std::shared_ptr<comotion::RobotModel>> robots;
    for (int i = 0; i < 5; ++i)
        robots.push_back(makeSphereRobot());

    std::vector<comotion::Path> paths;
    paths.push_back(makeConstantPath(kLength, {0.0, 0.0, 0.0}));
    paths.push_back(makeConstantPath(kLength, {100.0, 0.0, 0.0}));
    paths.push_back(makeConstantPath(kLength, {200.0, 0.0, 0.0}));
    paths.push_back(makeConstantPath(kLength, {300.0, 0.0, 0.0}));
    paths.push_back(makeConstantPath(kLength, {400.0, 0.0, 0.0}));

    paths[1][10] = {0.0, 0.0, 0.0};
    paths[2][12] = {20.0, 0.0, 0.0};
    paths[3][12] = {20.0, 0.0, 0.0};
    paths[2][20] = {30.0, 0.0, 0.0};
    paths[3][20] = {30.0, 0.0, 0.0};

    std::vector<const comotion::RobotModel *> ptrs;
    ptrs.reserve(robots.size());
    for (const auto &robot : robots)
        ptrs.push_back(robot.get());

    comotion::CompositePathValidationOptions options;
    options.check_environment = false;
    options.per_path_t_begin = {0, 0, 15, 15, 25};

    comotion::CollisionChecker collision_checker(
        comotion::CollisionChecker::Backend::Spheres);
    comotion::ConflictChecker checker(collision_checker);

    std::vector<std::size_t> next_t_begin_by_robot;
    const auto conflicts = checker.findConflicts(
        paths, ptrs, options, 0, 2, true, {}, &next_t_begin_by_robot);
    if (!expectTrue("per-path-start conflict count", conflicts.size() == 2))
        return false;
    if (!expectTrue("first per-path-start conflict",
                    conflicts[0].robots == std::vector<int>({0, 1}) &&
                        conflicts[0].conflict_timestep == 10)) {
        return false;
    }
    if (!expectTrue("second per-path-start conflict skips pre-activation conflict",
                    conflicts[1].robots == std::vector<int>({2, 3}) &&
                        conflicts[1].conflict_timestep == 20)) {
        return false;
    }
    if (!expectTrue("per-path-start next_t_begin size",
                    next_t_begin_by_robot.size() == paths.size())) {
        return false;
    }
    return expectTrue("per-path-start next_t_begin freezes accepted robot sets",
                      next_t_begin_by_robot[0] == 10 &&
                          next_t_begin_by_robot[1] == 10 &&
                          next_t_begin_by_robot[2] == 20 &&
                          next_t_begin_by_robot[3] == 20 &&
                          next_t_begin_by_robot[4] == 25);
}

bool testFindConflictPairsPerPairStarts() {
    constexpr std::size_t kLength = 32;
    std::vector<std::shared_ptr<comotion::RobotModel>> robots;
    for (int i = 0; i < 3; ++i)
        robots.push_back(makeSphereRobot());

    std::vector<comotion::Path> paths;
    paths.push_back(makeConstantPath(kLength, {0.0, 0.0, 0.0}));
    paths.push_back(makeConstantPath(kLength, {100.0, 0.0, 0.0}));
    paths.push_back(makeConstantPath(kLength, {200.0, 0.0, 0.0}));
    paths[2][10] = {0.0, 0.0, 0.0};
    paths[1][12] = {0.0, 0.0, 0.0};

    std::vector<const comotion::RobotModel *> ptrs;
    ptrs.reserve(robots.size());
    for (const auto &robot : robots)
        ptrs.push_back(robot.get());

    comotion::CompositePathValidationOptions options;
    options.check_environment = false;
    options.per_pair_t_begin.assign(comotion::pairFrontierSize(paths.size()), 0);
    options.per_pair_t_begin[comotion::pairFrontierIndex(0, 2, paths.size())] = 15;

    comotion::CollisionChecker collision_checker(
        comotion::CollisionChecker::Backend::Spheres);
    comotion::ConflictChecker checker(collision_checker);

    const auto conflicts = checker.findConflicts(
        paths, ptrs, options, 0, 1, false);
    if (!expectTrue("per-pair start conflict count", conflicts.size() == 1))
        return false;
    return expectTrue("per-pair start skips only selected pair",
                      conflicts[0].robots == std::vector<int>({0, 1}) &&
                          conflicts[0].conflict_timestep == 12);
}

bool testFindConflictPairsPerPairProgressAndUniqueSkips() {
    constexpr std::size_t kLength = 30;
    std::vector<std::shared_ptr<comotion::RobotModel>> robots;
    for (int i = 0; i < 4; ++i)
        robots.push_back(makeSphereRobot());

    std::vector<comotion::Path> paths;
    paths.push_back(makeConstantPath(kLength, {0.0, 0.0, 0.0}));
    paths.push_back(makeConstantPath(kLength, {100.0, 0.0, 0.0}));
    paths.push_back(makeConstantPath(kLength, {200.0, 0.0, 0.0}));
    paths.push_back(makeConstantPath(kLength, {300.0, 0.0, 0.0}));
    paths[1][10] = {0.0, 0.0, 0.0};
    paths[2][22] = {0.0, 0.0, 0.0};

    std::vector<const comotion::RobotModel *> ptrs;
    ptrs.reserve(robots.size());
    for (const auto &robot : robots)
        ptrs.push_back(robot.get());

    comotion::CompositePathValidationOptions options;
    options.check_environment = false;
    options.per_pair_t_begin.assign(comotion::pairFrontierSize(paths.size()), 0);
    options.per_pair_t_begin[comotion::pairFrontierIndex(0, 2, paths.size())] = 20;

    comotion::CollisionChecker collision_checker(
        comotion::CollisionChecker::Backend::Spheres);
    comotion::ConflictChecker checker(collision_checker);

    std::vector<std::size_t> next_t_begin_by_robot;
    std::vector<std::size_t> next_t_begin_by_pair;
    const auto conflicts = checker.findConflicts(
        paths, ptrs, options, 0, 0, true, {}, &next_t_begin_by_robot,
        &next_t_begin_by_pair);
    if (!expectTrue("per-pair progress unique conflict count",
                    conflicts.size() == 1))
        return false;
    if (!expectTrue("per-pair progress accepted conflict",
                    conflicts[0].robots == std::vector<int>({0, 1}) &&
                        conflicts[0].conflict_timestep == 10)) {
        return false;
    }
    if (!expectTrue("per-pair progress output size",
                    next_t_begin_by_pair.size() ==
                        comotion::pairFrontierSize(paths.size()))) {
        return false;
    }

    return expectTrue(
        "per-pair progress freezes accepted and unique-skipped pairs",
        next_t_begin_by_pair[comotion::pairFrontierIndex(0, 1, paths.size())] ==
                10 &&
            next_t_begin_by_pair[comotion::pairFrontierIndex(0, 2,
                                                         paths.size())] == 20 &&
            next_t_begin_by_pair[comotion::pairFrontierIndex(2, 3,
                                                         paths.size())] ==
                kLength);
}

bool sameConflicts(const std::vector<comotion::SubproblemConflict> &lhs,
                   const std::vector<comotion::SubproblemConflict> &rhs) {
    if (lhs.size() != rhs.size())
        return false;
    for (std::size_t i = 0; i < lhs.size(); ++i) {
        if (lhs[i].robots != rhs[i].robots ||
            lhs[i].conflict_timestep != rhs[i].conflict_timestep ||
            lhs[i].seed_robot_i != rhs[i].seed_robot_i ||
            lhs[i].seed_robot_j != rhs[i].seed_robot_j) {
            return false;
        }
    }
    return true;
}

bool testIndependentOnlyRejectsOptimisticConflictChain() {
    constexpr std::size_t kLength = 32;
    std::vector<std::shared_ptr<comotion::RobotModel>> robots;
    for (int i = 0; i < 4; ++i)
        robots.push_back(makeSphereRobot());

    std::vector<comotion::Path> paths;
    paths.push_back(makeConstantPath(kLength, {0.0, 0.0, 0.0}));
    paths.push_back(makeConstantPath(kLength, {100.0, 0.0, 0.0}));
    paths.push_back(makeConstantPath(kLength, {200.0, 0.0, 0.0}));
    paths.push_back(makeConstantPath(kLength, {300.0, 0.0, 0.0}));

    paths[1][10] = {0.0, 0.0, 0.0};
    paths[2][11] = {0.0, 0.0, 0.0};
    paths[3][12] = {200.0, 0.0, 0.0};

    std::vector<const comotion::RobotModel *> ptrs;
    ptrs.reserve(robots.size());
    for (const auto &robot : robots)
        ptrs.push_back(robot.get());

    comotion::CompositePathValidationOptions optimistic_options;
    optimistic_options.check_environment = false;

    comotion::CollisionChecker collision_checker(
        comotion::CollisionChecker::Backend::Spheres);
    comotion::ConflictChecker checker(collision_checker);

    const auto optimistic_conflicts = checker.findConflicts(
        paths, ptrs, optimistic_options, 0, 4, true);
    if (!expectTrue("optimistic batch admits later independent chain conflict",
                    optimistic_conflicts.size() == 2)) {
        return false;
    }
    if (!expectTrue("optimistic chain conflict reaches newly claimed robot",
                    optimistic_conflicts[1].robots ==
                        std::vector<int>({2, 3}))) {
        return false;
    }

    auto independent_options = optimistic_options;
    independent_options.inter_robot_conflict_batch_mode =
        comotion::InterRobotConflictBatchMode::IndependentOnly;
    const auto independent_conflicts = checker.findConflicts(
        paths, ptrs, independent_options, 0, 4, true);
    if (!expectTrue("independent-only batch rejects conflict chain",
                    independent_conflicts.size() == 1)) {
        return false;
    }
    if (!expectTrue("independent-only keeps earliest chain seed conflict",
                    independent_conflicts[0].robots ==
                        std::vector<int>({0, 1}))) {
        return false;
    }

    independent_options.conflict_find_parallel_workers = 2;
    independent_options.conflict_find_parallel_horizon = 16;
    const auto parallel_independent_conflicts = checker.findConflicts(
        paths, ptrs, independent_options, 0, 4, true);
    if (!expectTrue("parallel independent-only rejects conflict chain",
                    parallel_independent_conflicts.size() == 1)) {
        return false;
    }
    return expectTrue("parallel independent-only matches earliest chain seed",
                      parallel_independent_conflicts[0].robots ==
                          std::vector<int>({0, 1}));
}

bool testSegmentParallelFindConflictsMatchesSequential() {
    constexpr std::size_t kLength = 32;
    std::vector<std::shared_ptr<comotion::RobotModel>> robots;
    for (int i = 0; i < 8; ++i)
        robots.push_back(makeSphereRobot());

    std::vector<comotion::Path> paths;
    for (int i = 0; i < 8; ++i) {
        paths.push_back(makeConstantPath(
            kLength, {100.0 * static_cast<double>(i), 0.0, 0.0}));
    }

    paths[1][5] = paths[0][5];
    paths[3][5] = paths[2][5];
    paths[5][8] = paths[4][8];
    paths[7][9] = paths[6][9];

    std::vector<const comotion::RobotModel *> ptrs;
    ptrs.reserve(robots.size());
    for (const auto &robot : robots)
        ptrs.push_back(robot.get());

    const auto expand = [](const comotion::Conflict &conflict) {
        auto expanded = makeExpandedConflict(conflict);
        if (conflict.robot_i == 0 && conflict.robot_j == 1)
            expanded.robots = {0, 1, 4};
        return expanded;
    };

    comotion::CompositePathValidationOptions sequential_options;
    sequential_options.check_environment = false;
    sequential_options.per_pair_t_begin.assign(
        comotion::pairFrontierSize(paths.size()), 0);
    sequential_options.per_pair_t_begin[comotion::pairFrontierIndex(
        6, 7, paths.size())] = 7;

    comotion::CollisionChecker collision_checker(
        comotion::CollisionChecker::Backend::Spheres);
    comotion::ConflictChecker checker(collision_checker);

    const auto sequential_two = checker.findConflicts(
        paths, ptrs, sequential_options, 0, 2, true, expand);
    const auto sequential_four = checker.findConflicts(
        paths, ptrs, sequential_options, 0, 4, true, expand);
    const auto sequential_eight = checker.findConflicts(
        paths, ptrs, sequential_options, 0, 8, true, expand);

    for (const auto &[workers, horizon] :
         std::vector<std::pair<std::size_t, std::size_t>>{
             {2, 4}, {4, 16}, {8, 16}}) {
        const auto *expected = &sequential_eight;
        if (workers == 2)
            expected = &sequential_two;
        else if (workers == 4)
            expected = &sequential_four;

        comotion::CompositePathValidationOptions parallel_options =
            sequential_options;
        parallel_options.conflict_find_parallel_workers = workers;
        parallel_options.conflict_find_parallel_horizon = horizon;
        std::vector<std::size_t> next_t_begin_by_pair;
        const auto parallel_conflicts = checker.findConflicts(
            paths, ptrs, parallel_options, 0, workers, true, expand, nullptr,
            &next_t_begin_by_pair);
        const std::string label = "segment-parallel N=" +
                                  std::to_string(workers) +
                                  " matches sequential";
        if (!expectTrue(label.c_str(),
                        sameConflicts(*expected, parallel_conflicts))) {
            return false;
        }
        if (!expectTrue("segment-parallel next pair progress size",
                        next_t_begin_by_pair.size() ==
                            comotion::pairFrontierSize(paths.size()))) {
            return false;
        }
        if (horizon > 8) {
            if (!expectTrue(
                    "segment-parallel claimed pair does not advance past claim",
                    next_t_begin_by_pair[comotion::pairFrontierIndex(
                        4, 5, paths.size())] <= 5)) {
                return false;
            }
            if (!expectTrue(
                    "segment-parallel raw later conflict frontier is preserved",
                    next_t_begin_by_pair[comotion::pairFrontierIndex(
                        6, 7, paths.size())] <= 9)) {
                return false;
            }
        }
    }

    const auto repeat_options = [&]() {
        comotion::CompositePathValidationOptions options = sequential_options;
        options.conflict_find_parallel_workers = 4;
        options.conflict_find_parallel_horizon = 16;
        return options;
    }();
    const auto first = checker.findConflicts(paths, ptrs, repeat_options, 0, 4,
                                             true, expand);
    const auto second = checker.findConflicts(paths, ptrs, repeat_options, 0, 4,
                                              true, expand);
    if (!expectTrue("segment-parallel W=4 repeat runs are deterministic",
                    sameConflicts(first, second))) {
        return false;
    }

    comotion::CompositePathValidationOptions repeat_eight_options =
        sequential_options;
    repeat_eight_options.conflict_find_parallel_workers = 8;
    repeat_eight_options.conflict_find_parallel_horizon = 16;
    const auto first_eight = checker.findConflicts(
        paths, ptrs, repeat_eight_options, 0, 8, true, expand);
    const auto second_eight = checker.findConflicts(
        paths, ptrs, repeat_eight_options, 0, 8, true, expand);
    return expectTrue("segment-parallel W=8 repeat runs are deterministic",
                      sameConflicts(first_eight, second_eight));
}

bool testFclSegmentParallelFindConflictsMatchesSequential() {
    constexpr std::size_t kLength = 32;
    std::vector<std::shared_ptr<comotion::RobotModel>> robots;
    for (int i = 0; i < 8; ++i)
        robots.push_back(makeSphereRobot());

    std::vector<comotion::Path> paths;
    for (int i = 0; i < 8; ++i) {
        paths.push_back(makeConstantPath(
            kLength, {100.0 * static_cast<double>(i), 0.0, 0.0}));
    }

    paths[1][5] = paths[0][5];
    paths[3][5] = paths[2][5];
    paths[5][8] = paths[4][8];
    paths[7][9] = paths[6][9];

    std::vector<const comotion::RobotModel *> ptrs;
    ptrs.reserve(robots.size());
    for (const auto &robot : robots)
        ptrs.push_back(robot.get());

    const auto expand = [](const comotion::Conflict &conflict) {
        auto expanded = makeExpandedConflict(conflict);
        if (conflict.robot_i == 0 && conflict.robot_j == 1)
            expanded.robots = {0, 1, 4};
        return expanded;
    };

    comotion::CompositePathValidationOptions sequential_options;
    sequential_options.check_environment = false;
    sequential_options.per_pair_t_begin.assign(
        comotion::pairFrontierSize(paths.size()), 0);
    sequential_options.per_pair_t_begin[comotion::pairFrontierIndex(
        6, 7, paths.size())] = 7;

    comotion::CollisionChecker collision_checker(
        comotion::CollisionChecker::Backend::Fcl);
    comotion::ConflictChecker checker(collision_checker);

    const auto sequential_two = checker.findConflicts(
        paths, ptrs, sequential_options, 0, 2, true, expand);
    const auto sequential_four = checker.findConflicts(
        paths, ptrs, sequential_options, 0, 4, true, expand);
    const auto sequential_eight = checker.findConflicts(
        paths, ptrs, sequential_options, 0, 8, true, expand);

    for (const auto &[workers, horizon] :
         std::vector<std::pair<std::size_t, std::size_t>>{
             {2, 4}, {4, 16}, {8, 16}}) {
        const auto *expected = &sequential_eight;
        if (workers == 2)
            expected = &sequential_two;
        else if (workers == 4)
            expected = &sequential_four;

        comotion::CompositePathValidationOptions parallel_options =
            sequential_options;
        parallel_options.conflict_find_parallel_workers = workers;
        parallel_options.conflict_find_parallel_horizon = horizon;
        std::vector<std::size_t> next_t_begin_by_pair;
        const auto parallel_conflicts = checker.findConflicts(
            paths, ptrs, parallel_options, 0, workers, true, expand, nullptr,
            &next_t_begin_by_pair);
        const std::string label = "FCL segment-parallel N=" +
                                  std::to_string(workers) +
                                  " matches sequential";
        if (!expectTrue(label.c_str(),
                        sameConflicts(*expected, parallel_conflicts))) {
            return false;
        }
        if (!expectTrue("FCL segment-parallel next pair progress size",
                        next_t_begin_by_pair.size() ==
                            comotion::pairFrontierSize(paths.size()))) {
            return false;
        }
        if (horizon > 8) {
            if (!expectTrue(
                    "FCL segment-parallel claimed pair does not advance past claim",
                    next_t_begin_by_pair[comotion::pairFrontierIndex(
                        4, 5, paths.size())] <= 5)) {
                return false;
            }
            if (!expectTrue(
                    "FCL segment-parallel raw later conflict frontier is preserved",
                    next_t_begin_by_pair[comotion::pairFrontierIndex(
                        6, 7, paths.size())] <= 9)) {
                return false;
            }
        }
    }

    const auto repeat_options = [&]() {
        comotion::CompositePathValidationOptions options = sequential_options;
        options.conflict_find_parallel_workers = 4;
        options.conflict_find_parallel_horizon = 16;
        return options;
    }();
    const auto first = checker.findConflicts(paths, ptrs, repeat_options, 0, 4,
                                             true, expand);
    const auto second = checker.findConflicts(paths, ptrs, repeat_options, 0, 4,
                                              true, expand);
    if (!expectTrue("FCL segment-parallel W=4 repeat runs are deterministic",
                    sameConflicts(first, second))) {
        return false;
    }

    comotion::CompositePathValidationOptions repeat_eight_options =
        sequential_options;
    repeat_eight_options.conflict_find_parallel_workers = 8;
    repeat_eight_options.conflict_find_parallel_horizon = 16;
    const auto first_eight = checker.findConflicts(
        paths, ptrs, repeat_eight_options, 0, 8, true, expand);
    const auto second_eight = checker.findConflicts(
        paths, ptrs, repeat_eight_options, 0, 8, true, expand);
    return expectTrue("FCL segment-parallel W=8 repeat runs are deterministic",
                      sameConflicts(first_eight, second_eight));
}

void writeFclPrimitiveConflictUrdf(const fs::path &path) {
    std::ofstream out(path);
    if (!out)
        throw std::runtime_error("cannot write " + path.string());
    out << "<?xml version=\"1.0\"?>\n"
        << "<robot name=\"fcl_parallel_box\">\n"
        << "  <link name=\"base\"/>\n"
        << "  <joint name=\"slide_x\" type=\"prismatic\">\n"
        << "    <parent link=\"base\"/>\n"
        << "    <child link=\"box_link\"/>\n"
        << "    <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n"
        << "    <axis xyz=\"1 0 0\"/>\n"
        << "    <limit lower=\"-10\" upper=\"10\" velocity=\"1\"/>\n"
        << "  </joint>\n"
        << "  <link name=\"box_link\">\n"
        << "    <collision>\n"
        << "      <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n"
        << "      <geometry><box size=\"1 1 1\"/></geometry>\n"
        << "    </collision>\n"
        << "  </link>\n"
        << "</robot>\n";
}

bool testFclSegmentParallelPrimitiveGeometryMatchesSequential() {
    const fs::path dir =
        fs::temp_directory_path() / "comotion_fcl_parallel_conflict_regression";
    fs::create_directories(dir);
    const fs::path urdf = dir / "fcl_parallel_box.urdf";
    writeFclPrimitiveConflictUrdf(urdf);

    std::vector<std::shared_ptr<comotion::RobotModel>> robots;
    for (int i = 0; i < 4; ++i) {
        auto robot = std::make_shared<comotion::RobotModel>();
        robot->loadURDF(urdf.string());
        robots.push_back(std::move(robot));
    }

    constexpr std::size_t kLength = 20;
    std::vector<comotion::Path> paths;
    paths.push_back(makeConstantPath(kLength, {0.0}));
    paths.push_back(makeConstantPath(kLength, {4.0}));
    paths.push_back(makeConstantPath(kLength, {8.0}));
    paths.push_back(makeConstantPath(kLength, {12.0}));
    paths[1][6] = {0.35};
    paths[3][9] = {8.35};

    std::vector<const comotion::RobotModel *> ptrs;
    ptrs.reserve(robots.size());
    for (const auto &robot : robots)
        ptrs.push_back(robot.get());

    comotion::CompositePathValidationOptions sequential_options;
    sequential_options.check_environment = false;

    comotion::CollisionChecker collision_checker(
        comotion::CollisionChecker::Backend::Fcl);
    comotion::ConflictChecker checker(collision_checker);

    const auto expected = checker.findConflicts(paths, ptrs, sequential_options,
                                                0, 2, true);
    if (!expectTrue("FCL primitive sequential conflict count",
                    expected.size() == 2)) {
        return false;
    }

    comotion::CompositePathValidationOptions parallel_options =
        sequential_options;
    parallel_options.conflict_find_parallel_workers = 2;
    parallel_options.conflict_find_parallel_horizon = 8;
    std::vector<std::size_t> next_t_begin_by_pair;
    const auto parallel = checker.findConflicts(
        paths, ptrs, parallel_options, 0, 2, true, {}, nullptr,
        &next_t_begin_by_pair);
    if (!expectTrue("FCL primitive parallel matches sequential",
                    sameConflicts(expected, parallel))) {
        return false;
    }
    if (!expectTrue("FCL primitive next pair progress size",
                    next_t_begin_by_pair.size() ==
                        comotion::pairFrontierSize(paths.size()))) {
        return false;
    }
    return expectTrue(
        "FCL primitive accepted pair frontiers stop at conflicts",
        next_t_begin_by_pair[comotion::pairFrontierIndex(0, 1,
                                                         paths.size())] <= 6 &&
            next_t_begin_by_pair[comotion::pairFrontierIndex(2, 3,
                                                             paths.size())] <= 9);
}

bool testVampSegmentParallelFindConflictsMatchesSequential() {
#if COMOTION_HAVE_VAMP
    constexpr std::size_t kLength = 32;
    std::vector<std::shared_ptr<comotion::RobotModel>> robots;
    for (int i = 0; i < 8; ++i)
        robots.push_back(makeSphereRobot());

    std::vector<comotion::Path> paths;
    for (int i = 0; i < 8; ++i) {
        paths.push_back(makeConstantPath(
            kLength, {100.0 * static_cast<double>(i), 0.0, 0.0}));
    }

    paths[1][5] = paths[0][5];
    paths[3][5] = paths[2][5];
    paths[5][8] = paths[4][8];
    paths[7][9] = paths[6][9];

    std::vector<const comotion::RobotModel *> ptrs;
    ptrs.reserve(robots.size());
    for (const auto &robot : robots)
        ptrs.push_back(robot.get());

    const auto expand = [](const comotion::Conflict &conflict) {
        auto expanded = makeExpandedConflict(conflict);
        if (conflict.robot_i == 0 && conflict.robot_j == 1)
            expanded.robots = {0, 1, 4};
        return expanded;
    };

    comotion::CompositePathValidationOptions sequential_options;
    sequential_options.check_environment = false;
    sequential_options.per_pair_t_begin.assign(
        comotion::pairFrontierSize(paths.size()), 0);
    sequential_options.per_pair_t_begin[comotion::pairFrontierIndex(
        6, 7, paths.size())] = 7;

    comotion::CollisionChecker collision_checker(
        comotion::CollisionChecker::Backend::Vamp);
    comotion::ConflictChecker checker(collision_checker);

    const auto sequential_two = checker.findConflicts(
        paths, ptrs, sequential_options, 0, 2, true, expand);
    const auto sequential_four = checker.findConflicts(
        paths, ptrs, sequential_options, 0, 4, true, expand);

    for (const auto &[workers, horizon] :
         std::vector<std::pair<std::size_t, std::size_t>>{
             {2, 4}, {4, 16}}) {
        const auto *expected = workers == 2 ? &sequential_two : &sequential_four;
        comotion::CompositePathValidationOptions parallel_options =
            sequential_options;
        parallel_options.conflict_find_parallel_workers = workers;
        parallel_options.conflict_find_parallel_horizon = horizon;
        std::vector<std::size_t> next_t_begin_by_pair;
        const auto parallel_conflicts = checker.findConflicts(
            paths, ptrs, parallel_options, 0, workers, true, expand, nullptr,
            &next_t_begin_by_pair);
        const std::string label = "VAMP segment-parallel N=" +
                                  std::to_string(workers) +
                                  " matches sequential";
        if (!expectTrue(label.c_str(),
                        sameConflicts(*expected, parallel_conflicts))) {
            return false;
        }
        if (!expectTrue("VAMP segment-parallel next pair progress size",
                        next_t_begin_by_pair.size() ==
                            comotion::pairFrontierSize(paths.size()))) {
            return false;
        }
        if (horizon > 8 &&
            !expectTrue(
                "VAMP segment-parallel raw later conflict frontier preserved",
                next_t_begin_by_pair[comotion::pairFrontierIndex(
                    6, 7, paths.size())] <= 9)) {
            return false;
        }
    }

    comotion::CompositePathValidationOptions repeat_options = sequential_options;
    repeat_options.conflict_find_parallel_workers = 4;
    repeat_options.conflict_find_parallel_horizon = 16;
    const auto first = checker.findConflicts(paths, ptrs, repeat_options, 0, 4,
                                             true, expand);
    const auto second = checker.findConflicts(paths, ptrs, repeat_options, 0, 4,
                                              true, expand);
    if (!expectTrue("VAMP segment-parallel repeat runs are deterministic",
                    sameConflicts(first, second))) {
        return false;
    }

    return true;
#else
    return true;
#endif
}

} // namespace

int main() {
    if (!testAllPairsAndCap())
        return 1;
    if (!testUniqueKeepsNonOverlappingRobotSets())
        return 1;
    if (!testUniqueDiscardsLaterOverlappingWindowlessConflict())
        return 1;
    if (!testUniqueDiscardsLaterClaimedConflictEvenWithWiderWindow())
        return 1;
    if (!testFindConflictPerPathStarts())
        return 1;
    if (!testFindConflictPairsPerPathStartsAndProgress())
        return 1;
    if (!testFindConflictPairsPerPairStarts())
        return 1;
    if (!testFindConflictPairsPerPairProgressAndUniqueSkips())
        return 1;
    if (!testIndependentOnlyRejectsOptimisticConflictChain())
        return 1;
    if (!testSegmentParallelFindConflictsMatchesSequential())
        return 1;
    if (!testFclSegmentParallelFindConflictsMatchesSequential())
        return 1;
    if (!testFclSegmentParallelPrimitiveGeometryMatchesSequential())
        return 1;
    if (!testVampSegmentParallelFindConflictsMatchesSequential())
        return 1;

    std::cout << "find_conflict_pairs_regression: OK\n";
    return 0;
}
