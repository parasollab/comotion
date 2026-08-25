#include "comotion/collision/CollisionChecker.h"
#include "comotion/collision/ConflictChecker.h"
#include "comotion/planning/MultiRobotProblem.h"
#include "comotion/planning/PrioritizedSTRRT.h"
#include "comotion/planning/Path.h"
#include "comotion/planning/detail/StrrtBatchInflation.h"
#include "comotion/robot/FlyingSphere.h"

#include <ompl/base/MotionValidator.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SpaceTimeStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/STRRTstar.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <vector>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace {

std::shared_ptr<comotion::FlyingSphere> makeSphereRobot(double radius = 0.6) {
    return std::make_shared<comotion::FlyingSphere>(
        radius, std::vector<double>{-10.0, -10.0, -10.0},
        std::vector<double>{10.0, 10.0, 10.0});
}

bool expectTrue(const std::string &label, bool value) {
    if (!value) {
        std::cerr << "prioritized_strrt_goal_hold_regression: " << label
                  << " expected true\n";
        return false;
    }
    return true;
}

bool expectFalse(const std::string &label, bool value) {
    if (value) {
        std::cerr << "prioritized_strrt_goal_hold_regression: " << label
                  << " expected false\n";
        return false;
    }
    return true;
}

bool expectEq(const std::string &label, std::size_t actual,
              std::size_t expected) {
    if (actual != expected) {
        std::cerr << "prioritized_strrt_goal_hold_regression: " << label
                  << " expected " << expected << " got " << actual << "\n";
        return false;
    }
    return true;
}

bool expectNear(const std::string &label, double actual, double expected,
                double tolerance) {
    if (std::fabs(actual - expected) <= tolerance)
        return true;
    std::cerr << "prioritized_strrt_goal_hold_regression: " << label
              << " expected " << expected << " +/- " << tolerance
              << " got " << actual << "\n";
    return false;
}

bool expectVectorEq(const std::string &label, const std::vector<int> &actual,
                    const std::vector<int> &expected) {
    if (actual == expected)
        return true;
    std::cerr << "prioritized_strrt_goal_hold_regression: " << label
              << " vectors differ\n";
    return false;
}

bool isPermutationOfRobotIds(std::vector<int> order, int n) {
    if (order.size() != static_cast<std::size_t>(n))
        return false;
    std::sort(order.begin(), order.end());
    for (int i = 0; i < n; ++i) {
        if (order[static_cast<std::size_t>(i)] != i)
            return false;
    }
    return true;
}

std::shared_ptr<comotion::MultiRobotProblem> makeIndependentSphereProblem(int n) {
    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(16);
    problem->setVmax(3.0);
    for (int i = 0; i < n; ++i) {
        const double y = static_cast<double>(i) * 3.0;
        problem->addRobot(makeSphereRobot(0.1), {-4.0, y, 0.0},
                          {-3.0, y, 0.0});
    }
    return problem;
}

std::vector<int> shuffledPriorityOrderForSeed(std::uint32_t seed) {
    auto problem = makeIndependentSphereProblem(4);

    comotion::PrioritizedSTRRT planner;
    planner.setProblem(problem);
    planner.setPlanningSeed(seed);
    planner.setShufflePriorityOrder(true);
    planner.setEqualizePaths(false);
    planner.setUseUnboundedTime(false);
    planner.setSpaceTimeUpperBound(2.0);
    planner.setStrrtInitialBatchSize(64);

    const auto status = planner.solve(5.0);
    if (status != ob::PlannerStatus::EXACT_SOLUTION) {
        std::cerr << "prioritized_strrt_goal_hold_regression: expected exact "
                     "shuffle solution, got "
                  << status.asString() << "\n";
        return {};
    }

    const auto &stats = planner.plannerStatsJson();
    if (!expectTrue("shuffle stats include priority_order",
                    stats.contains("priority_order")))
        return {};
    if (!expectTrue("shuffle stats record shuffle enabled",
                    stats.value("shuffle_priority_order", false)))
        return {};
    if (!expectTrue("shuffle stats record return-first default",
                    stats.value("return_first_solution", false)))
        return {};

    return stats["priority_order"].get<std::vector<int>>();
}

class SpaceTimeMotionValidator final : public ob::MotionValidator {
public:
    explicit SpaceTimeMotionValidator(const ob::SpaceInformationPtr &si,
                                      double v_max)
        : ob::MotionValidator(si),
          v_max_(v_max),
          state_space_(si->getStateSpace().get()) {}

    bool checkMotion(const ob::State *s1, const ob::State *s2) const override {
        if (!si_->isValid(s2))
            return false;

        auto *space = state_space_->as<ob::SpaceTimeStateSpace>();
        const double delta_pos = space->distanceSpace(s1, s2);
        const double t1 = ob::SpaceTimeStateSpace::getStateTime(s1);
        const double t2 = ob::SpaceTimeStateSpace::getStateTime(s2);
        const double delta_t = t2 - t1;
        return delta_t > 0.0 && delta_pos / delta_t <= v_max_;
    }

    bool checkMotion(const ob::State *s1, const ob::State *s2,
                     std::pair<ob::State *, double> &lastValid) const override {
        lastValid.first = nullptr;
        lastValid.second = 0.0;
        return checkMotion(s1, s2);
    }

private:
    double v_max_;
    ob::StateSpace *state_space_;
};

bool testGoalHoldConstraintSphere() {
    comotion::CollisionChecker checker(comotion::CollisionChecker::Backend::Spheres);
    auto goal_robot = makeSphereRobot();
    auto prior_robot = makeSphereRobot();

    comotion::Path prior_path;
    prior_path.push_back({0.0, 0.0, 0.0});
    prior_path.push_back({1.0, 0.0, 0.0});
    prior_path.push_back({2.0, 0.0, 0.0});
    prior_path.push_back({3.0, 0.0, 0.0});
    prior_path.push_back({4.0, 0.0, 0.0});

    auto constraint = checker.computeGoalHoldConstraint(
        *goal_robot, std::vector<double>{2.0, 0.0, 0.0}, *prior_robot,
        prior_path);
    if (!expectFalse("goal hold sphere permanently blocked",
                     constraint.permanently_blocked))
        return false;
    if (!expectEq("goal hold sphere min safe arrival",
                  constraint.min_safe_arrival_timestep, 4))
        return false;

    comotion::Path blocked_path;
    blocked_path.push_back({0.0, 0.0, 0.0});
    blocked_path.push_back({1.0, 0.0, 0.0});
    blocked_path.push_back({2.0, 0.0, 0.0});

    auto blocked = checker.computeGoalHoldConstraint(
        *goal_robot, std::vector<double>{2.0, 0.0, 0.0}, *prior_robot,
        blocked_path);
    return expectTrue("goal hold sphere permanently blocked",
                      blocked.permanently_blocked);
}

bool testPrioritizedStrrtDelaysArrival() {
    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(32);
    problem->setVmax(2.0);

    problem->addRobot(makeSphereRobot(), {-3.0, 0.0, 0.0}, {3.0, 0.0, 0.0});
    problem->addRobot(makeSphereRobot(), {0.0, 3.0, 0.0}, {0.0, 2.5, 0.0});

    comotion::PrioritizedSTRRT planner;
    planner.setProblem(problem);
    planner.setEqualizePaths(false);
    planner.setPriorityOrder({0, 1});
    planner.setSpaceTimeUpperBound(4.0);

    const auto status = planner.solve(10.0);
    if (status != ob::PlannerStatus::EXACT_SOLUTION) {
        std::cerr << "prioritized_strrt_goal_hold_regression: expected exact "
                     "solution, got "
                  << status.asString() << "\n";
        return false;
    }

    auto paths = planner.getSolutionPaths();
    if (!expectEq("solution path count", paths.size(), 2))
        return false;
    if (!expectTrue("lower-priority path has timesteps",
                    paths[1].has_timesteps()))
        return false;

    const auto constraint = problem->collisionChecker().computeGoalHoldConstraint(
        *problem->robot(1).model, problem->robot(1).goal,
        *problem->robot(0).model, paths[0]);
    if (!expectFalse("lower-priority goal should be unblockable",
                     constraint.permanently_blocked))
        return false;

    const std::size_t arrival_ts = paths[1].arrival_timestep();
    if (arrival_ts < constraint.min_safe_arrival_timestep) {
        std::cerr << "prioritized_strrt_goal_hold_regression: lower-priority "
                     "arrival "
                  << arrival_ts << " earlier than safe threshold "
                  << constraint.min_safe_arrival_timestep << "\n";
        return false;
    }

    auto equalized = paths;
    comotion::equalizePaths(equalized);
    comotion::ConflictChecker conflict_checker(problem->collisionChecker());
    auto ptrs = problem->robotModelPtrs();
    auto conflict = conflict_checker.findConflict(equalized, ptrs);
    if (conflict) {
        std::cerr << "prioritized_strrt_goal_hold_regression: unexpected "
                     "conflict at timestep "
                  << conflict->timestep << "\n";
        return false;
    }

    return true;
}

bool testPrioritizedStrrtSolutionMetricsUsePreEqualizationCosts() {
    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(32);
    problem->setVmax(2.0);

    problem->addRobot(makeSphereRobot(), {-3.0, 0.0, 0.0}, {3.0, 0.0, 0.0});
    problem->addRobot(makeSphereRobot(), {0.0, 3.0, 0.0}, {0.0, 2.5, 0.0});

    comotion::PrioritizedSTRRT planner;
    planner.setProblem(problem);
    planner.setPlanningSeed(2026);
    planner.setPriorityOrder({0, 1});
    planner.setSpaceTimeUpperBound(4.0);
    comotion::PathSimplificationOptions simplification_options;
    simplification_options.max_shortcut_steps = 3;
    simplification_options.max_empty_steps = 2;
    simplification_options.max_smooth_steps = 1;
    planner.setPathSimplificationOptions(simplification_options);

    const auto status = planner.solve(10.0);
    if (status != ob::PlannerStatus::EXACT_SOLUTION) {
        std::cerr << "prioritized_strrt_goal_hold_regression: expected exact "
                     "solution for pre-equalization metrics test, got "
                  << status.asString() << "\n";
        return false;
    }

    if (!expectTrue("sum_of_cost metric populated",
                    planner.sumOfCostTimesteps().has_value()))
        return false;
    if (!expectTrue("makespan metric populated",
                    planner.makespanTimesteps().has_value()))
        return false;

    const auto paths = planner.getSolutionPaths();
    std::uint64_t equalized_sum = 0;
    std::uint64_t equalized_makespan = 0;
    for (const auto &path : paths) {
        if (!expectTrue("equalized solution path has timesteps",
                        path.has_timesteps()))
            return false;
        const auto arrival_timestep =
            static_cast<std::uint64_t>(path.arrival_timestep());
        equalized_sum += arrival_timestep;
        equalized_makespan = std::max(equalized_makespan, arrival_timestep);
    }

    if (!expectTrue("pre-equalization sum_of_cost is lower than equalized sum",
                    *planner.sumOfCostTimesteps() < equalized_sum))
        return false;
    if (!expectTrue("planner makespan matches equalized makespan",
                    *planner.makespanTimesteps() == equalized_makespan))
        return false;

    const auto &planner_stats = planner.plannerStatsJson();
    if (!expectTrue("planner stats include robot_solve_times_seconds",
                    planner_stats.contains("robot_solve_times_seconds")))
        return false;
    if (!expectEq("planner stats record one solve time per robot",
                  planner_stats["robot_solve_times_seconds"].size(), 2))
        return false;
    if (!expectTrue(
            "planner stats record deterministic component seeds",
            planner_stats["planning_seed"].get<std::uint32_t>() == 2026 &&
                planner_stats["state_sampler_seeds"].size() == 2 &&
                planner_stats["time_sampler_seeds"].size() == 2 &&
                planner_stats["planner_local_seeds"].size() == 2 &&
                planner_stats["conditional_sampler_seeds"].size() == 2 &&
                planner_stats["simplifier_local_seeds"].size() == 2))
        return false;
    if (!expectTrue(
            "planner stats record path simplification controls",
            planner_stats["path_simplification"]["max_shortcut_steps"]
                    .get<unsigned int>() == 3 &&
                planner_stats["path_simplification"]["max_empty_steps"]
                        .get<unsigned int>() == 2 &&
                planner_stats["path_simplification"]["max_smooth_steps"]
                        .get<unsigned int>() == 1))
        return false;

    return true;
}

bool testPrioritizedStrrtSeededShuffleStats() {
    const auto order_a = shuffledPriorityOrderForSeed(123);
    const auto order_b = shuffledPriorityOrderForSeed(123);
    const auto order_c = shuffledPriorityOrderForSeed(124);

    if (!expectTrue("shuffle seed 123 is a valid permutation",
                    isPermutationOfRobotIds(order_a, 4)))
        return false;
    if (!expectVectorEq("shuffle is deterministic for same seed", order_b,
                        order_a))
        return false;
    if (!expectTrue("shuffle seed 124 is a valid permutation",
                    isPermutationOfRobotIds(order_c, 4)))
        return false;
    if (!expectTrue("different shuffle seeds can change order",
                    order_c != order_a))
        return false;

    return true;
}

bool testPrioritizedStrrtBudgetPolicies() {
    {
        auto problem = makeIndependentSphereProblem(3);

        comotion::PrioritizedSTRRT planner;
        planner.setProblem(problem);
        planner.setEqualizePaths(false);
        planner.setReturnFirstSolution(false);
        planner.setUseUnboundedTime(false);
        planner.setSpaceTimeUpperBound(2.0);
        planner.setStrrtInitialBatchSize(64);
        planner.setStrrtMaxIterations(256);

        const auto status = planner.solve(6.0);
        if (status != ob::PlannerStatus::EXACT_SOLUTION) {
            std::cerr << "prioritized_strrt_goal_hold_regression: expected exact "
                         "dynamic-budget solution, got "
                      << status.asString() << "\n";
            return false;
        }

        if (!expectTrue("dynamic-budget stats record return_first false",
                        !planner.plannerStatsJson().value(
                            "return_first_solution", true)))
            return false;
    }

    {
        auto problem = makeIndependentSphereProblem(3);

        comotion::PrioritizedSTRRT planner;
        planner.setProblem(problem);
        planner.setEqualizePaths(false);
        planner.setReturnFirstSolution(true);
        planner.setUseUnboundedTime(false);
        planner.setSpaceTimeUpperBound(2.0);
        planner.setStrrtInitialBatchSize(64);
        planner.setStrrtMaxIterations(256);

        const auto status = planner.solve(6.0);
        if (status != ob::PlannerStatus::EXACT_SOLUTION) {
            std::cerr << "prioritized_strrt_goal_hold_regression: expected exact "
                         "return-first budget solution, got "
                      << status.asString() << "\n";
            return false;
        }

        if (!expectTrue("return-first stats record return_first true",
                        planner.plannerStatsJson().value(
                            "return_first_solution", false)))
            return false;
    }

    return true;
}

bool testPrioritizedStrrtPermanentBlock() {
    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(32);
    problem->setVmax(2.0);

    problem->addRobot(makeSphereRobot(), {1.5, 0.0, 0.0}, {2.0, 0.0, 0.0});
    problem->addRobot(makeSphereRobot(), {2.0, 4.5, 0.0}, {2.0, 0.0, 0.0});

    comotion::PrioritizedSTRRT planner;
    planner.setProblem(problem);
    planner.setPriorityOrder({0, 1});

    const auto status = planner.solve(5.0);
    if (status != ob::PlannerStatus::INVALID_GOAL) {
        std::cerr << "prioritized_strrt_goal_hold_regression: expected "
                     "INVALID_GOAL, got "
                  << status.asString() << "\n";
        return false;
    }
    return true;
}

bool testPrioritizedStrrtCanDisableGoalPersistence() {
    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(32);
    problem->setVmax(2.0);

    problem->addRobot(makeSphereRobot(), {1.5, 0.0, 0.0}, {2.0, 0.0, 0.0});
    problem->addRobot(makeSphereRobot(), {2.0, 4.5, 0.0}, {2.0, 0.0, 0.0});

    if (!expectFalse("shared goal would collide if the prior robot persisted",
                     problem->collisionChecker().isValidPair(
                         *problem->robot(0).model, problem->robot(0).goal,
                         *problem->robot(1).model, problem->robot(1).goal))) {
        return false;
    }

    comotion::PrioritizedSTRRT planner;
    planner.setProblem(problem);
    planner.setPriorityOrder({0, 1});
    planner.setPersistAtGoal(false);
    planner.setEqualizePaths(false);
    planner.setSpaceTimeUpperBound(20.0);

    const auto status = planner.solve(10.0);
    if (status != ob::PlannerStatus::EXACT_SOLUTION) {
        std::cerr << "prioritized_strrt_goal_hold_regression: expected exact "
                     "solution with goal persistence disabled, got "
                  << status.asString() << "\n";
        return false;
    }

    const auto paths = planner.getSolutionPaths();
    if (!expectEq("disappear solution path count", paths.size(), 2))
        return false;
    if (!expectTrue("disappear prior path has timesteps", paths[0].has_timesteps()))
        return false;
    if (!expectTrue("disappear lower-priority path has timesteps",
                    paths[1].has_timesteps()))
        return false;
    if (!expectTrue("disappear solve returns ragged paths",
                    paths[0].size() != paths[1].size()))
        return false;

    const auto lower_arrival = paths[1].arrival_timestep();
    if (!expectTrue("lower-priority robot reaches shared goal after prior path ends",
                    lower_arrival >= paths[0].size())) {
        std::cerr << "prioritized_strrt_goal_hold_regression: prior size="
                  << paths[0].size()
                  << " prior arrival=" << paths[0].arrival_timestep()
                  << " lower size=" << paths[1].size()
                  << " lower arrival=" << lower_arrival << "\n";
        return false;
    }

    for (std::size_t t = 0; t < paths[0].size() && t < paths[1].size(); ++t) {
        if (!problem->collisionChecker().isValidPair(
                *problem->robot(0).model, paths[0][t],
                *problem->robot(1).model, paths[1][t])) {
            std::cerr << "prioritized_strrt_goal_hold_regression: active-time "
                         "pair conflict at timestep "
                      << t << "\n";
            return false;
        }
    }

    return true;
}

bool testStrrtMinimumGoalTimeInvalidGoal() {
    constexpr double kVMax = 1.0;
    auto vector_space = std::make_shared<ob::RealVectorStateSpace>(1);
    ob::RealVectorBounds bounds(1);
    bounds.setLow(-1.0);
    bounds.setHigh(1.0);
    vector_space->setBounds(bounds);

    auto space = std::make_shared<ob::SpaceTimeStateSpace>(vector_space, kVMax);
    space->setTimeBounds(0.0, 5.0);

    auto si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker([](const ob::State *) { return true; });
    si->setMotionValidator(std::make_shared<SpaceTimeMotionValidator>(si, kVMax));

    og::SimpleSetup setup(si);
    ob::ScopedState<> start(space);
    start[0] = 0.0;
    start[1] = 0.0;

    ob::ScopedState<> goal(space);
    goal[0] = 1.0;
    goal[1] = 0.0;
    setup.setStartAndGoalStates(start, goal);

    auto planner = std::make_shared<og::STRRTstar>(si);
    planner->setRange(kVMax);
    planner->setMinimumGoalTime(6.0);
    setup.setPlanner(planner);

    const auto status = setup.solve(0.1);
    if (status != ob::PlannerStatus::INVALID_GOAL) {
        std::cerr << "prioritized_strrt_goal_hold_regression: expected "
                     "STRRT INVALID_GOAL, got "
                  << status.asString() << "\n";
        return false;
    }
    return true;
}

bool testBatchInflationMath() {
    const auto no_delay = comotion::detail::computeStrrtBatchInflation(
        3.0, 5.0, 512, 2.0, 2.0, 64);
    if (!expectEq("no-delay virtual expansions", no_delay.virtual_expansions, 0))
        return false;
    if (!expectEq("no-delay inflated batch", no_delay.inflated_batch_size, 512))
        return false;

    const auto exact_threshold = comotion::detail::computeStrrtBatchInflation(
        3.0, 6.0, 512, 2.0, 2.0, 64);
    if (!expectEq("threshold virtual expansions",
                  exact_threshold.virtual_expansions, 0))
        return false;
    if (!expectEq("threshold inflated batch",
                  exact_threshold.inflated_batch_size, 512))
        return false;

    const auto one_skip = comotion::detail::computeStrrtBatchInflation(
        3.0, 7.0, 512, 2.0, 2.0, 64);
    if (!expectEq("one-skip virtual expansions", one_skip.virtual_expansions, 1))
        return false;
    if (!expectEq("one-skip inflated batch", one_skip.inflated_batch_size, 1024))
        return false;

    const auto multi_skip = comotion::detail::computeStrrtBatchInflation(
        3.0, 25.0, 512, 2.0, 2.0, 64);
    if (!expectEq("multi-skip virtual expansions",
                  multi_skip.virtual_expansions, 3))
        return false;
    if (!expectEq("multi-skip inflated batch",
                  multi_skip.inflated_batch_size, 4096))
        return false;

    const auto capped = comotion::detail::computeStrrtBatchInflation(
        3.0, 500.0, 100, 2.0, 2.0, 4);
    if (!expectEq("capped inflated batch", capped.inflated_batch_size, 400))
        return false;

    return true;
}

std::shared_ptr<comotion::MultiRobotProblem> makeDelayedArrivalBatchProblem() {
    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(32);
    problem->setVmax(2.0);
    problem->addRobot(makeSphereRobot(), {-9.0, 0.0, 0.0}, {9.0, 0.0, 0.0});
    problem->addRobot(makeSphereRobot(), {1.5, 3.0, 0.0}, {1.5, 0.0, 0.0});
    return problem;
}

bool validateDelayedArrivalSolution(
    const comotion::MultiRobotProblem &problem,
    const std::vector<comotion::Path> &paths) {
    if (!expectEq("delayed-arrival solution path count", paths.size(), 2))
        return false;
    if (!expectTrue("delayed-arrival lower-priority path has timesteps",
                    paths[1].has_timesteps()))
        return false;

    const auto constraint = problem.collisionChecker().computeGoalHoldConstraint(
        *problem.robot(1).model, problem.robot(1).goal,
        *problem.robot(0).model, paths[0]);
    if (!expectFalse("delayed-arrival goal should be unblockable",
                     constraint.permanently_blocked)) {
        return false;
    }

    const std::size_t arrival_ts = paths[1].arrival_timestep();
    if (arrival_ts < constraint.min_safe_arrival_timestep) {
        std::cerr << "prioritized_strrt_goal_hold_regression: delayed-arrival "
                     "arrival "
                  << arrival_ts << " earlier than safe threshold "
                  << constraint.min_safe_arrival_timestep << "\n";
        return false;
    }

    for (std::size_t t = 0; t < paths[0].size() && t < paths[1].size(); ++t) {
        if (problem.collisionChecker().isValidPair(
                *problem.robot(0).model, paths[0][t],
                *problem.robot(1).model, paths[1][t])) {
            continue;
        }
        std::cerr << "prioritized_strrt_goal_hold_regression: delayed-arrival "
                     "unexpected conflict at timestep "
                  << t << "\n";
        return false;
    }

    return true;
}

bool testPrioritizedStrrtUnboundedNoInflationKeepsBaseBatch() {
    auto problem = makeDelayedArrivalBatchProblem();

    comotion::PrioritizedSTRRT planner;
    planner.setProblem(problem);
    planner.setEqualizePaths(false);
    planner.setPriorityOrder({0, 1});
    planner.setUseUnboundedTime(true);
    planner.setInflateInitialBatchFromMinGoalTime(false);
    planner.setStrrtInitialBatchSize(64);

    const auto status = planner.solve(15.0);
    if (status != ob::PlannerStatus::EXACT_SOLUTION) {
        std::cerr << "prioritized_strrt_goal_hold_regression: expected exact "
                     "unbounded no-inflation solution, got "
                  << status.asString() << "\n";
        return false;
    }

    return validateDelayedArrivalSolution(*problem, planner.getSolutionPaths());
}

bool testPrioritizedStrrtUnboundedInflatesBatch() {
    auto problem = makeDelayedArrivalBatchProblem();

    comotion::PrioritizedSTRRT planner;
    planner.setProblem(problem);
    planner.setEqualizePaths(false);
    planner.setPriorityOrder({0, 1});
    planner.setUseUnboundedTime(true);
    planner.setInflateInitialBatchFromMinGoalTime(true);
    planner.setStrrtInitialBatchSize(64);

    const auto status = planner.solve(15.0);
    if (status != ob::PlannerStatus::EXACT_SOLUTION) {
        std::cerr << "prioritized_strrt_goal_hold_regression: expected exact "
                     "unbounded inflated solution, got "
                  << status.asString() << "\n";
        return false;
    }

    return validateDelayedArrivalSolution(*problem, planner.getSolutionPaths());
}

} // namespace

int main() {
    if (!testGoalHoldConstraintSphere())
        return 1;
    if (!testPrioritizedStrrtDelaysArrival())
        return 1;
    if (!testPrioritizedStrrtSolutionMetricsUsePreEqualizationCosts())
        return 1;
    if (!testPrioritizedStrrtSeededShuffleStats())
        return 1;
    if (!testPrioritizedStrrtBudgetPolicies())
        return 1;
    if (!testPrioritizedStrrtPermanentBlock())
        return 1;
    if (!testPrioritizedStrrtCanDisableGoalPersistence())
        return 1;
    if (!testStrrtMinimumGoalTimeInvalidGoal())
        return 1;
    if (!testBatchInflationMath())
        return 1;
    if (!testPrioritizedStrrtUnboundedNoInflationKeepsBaseBatch())
        return 1;
    if (!testPrioritizedStrrtUnboundedInflatesBatch())
        return 1;

    std::cout << "prioritized_strrt_goal_hold_regression: OK\n";
    return 0;
}
