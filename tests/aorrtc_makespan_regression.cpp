#include "comotion/planning/AOARC.h"
#include "comotion/planning/AORRTCUtils.h"
#include "comotion/planning/MakespanCompositeStateSpace.h"
#include "comotion/planning/MakespanInformedSampler.h"
#include "comotion/planning/MultiRobotProblem.h"
#include "comotion/planning/PlanningRng.h"
#include "comotion/robot/FlyingSphere.h"

#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace ob = ompl::base;

namespace {

std::shared_ptr<comotion::FlyingSphere> makeSphere(double radius = 0.1) {
    return std::make_shared<comotion::FlyingSphere>(
        radius, std::vector<double>{-5.0, -5.0, -5.0},
        std::vector<double>{5.0, 5.0, 5.0});
}

bool expectTrue(const std::string &label, bool value) {
    if (!value)
        std::cerr << "aorrtc_makespan_regression: " << label << "\n";
    return value;
}

bool expectNear(const std::string &label, double actual, double expected,
                double tolerance = 1e-9) {
    if (std::abs(actual - expected) > tolerance) {
        std::cerr << "aorrtc_makespan_regression: " << label << " expected "
                  << expected << " got " << actual << "\n";
        return false;
    }
    return true;
}

void setRealVectorState(ob::State *state, const std::vector<double> &values) {
    auto *rv = state->as<ob::RealVectorStateSpace::StateType>();
    for (std::size_t i = 0; i < values.size(); ++i)
        rv->values[i] = values[i];
}

double makespanHeuristic(const comotion::MakespanCompositeStateSpace &space,
                         const ob::State *start, const ob::State *state,
                         const ob::State *goal) {
    return space.distance(start, state) + space.distance(state, goal);
}

std::shared_ptr<comotion::MultiRobotProblem> makeParallelSphereProblem() {
    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(16);
    problem->setVmax(1.0);
    problem->addRobot(makeSphere(), {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
    problem->addRobot(makeSphere(), {0.0, 2.0, 0.0}, {1.0, 2.0, 0.0});
    return problem;
}

bool testMakespanCompositeDistance() {
    comotion::MakespanCompositeStateSpace space({2, 3});
    ob::RealVectorBounds bounds(5);
    bounds.setLow(-10.0);
    bounds.setHigh(10.0);
    space.setBounds(bounds);

    ob::State *a = space.allocState();
    ob::State *b = space.allocState();
    auto *av = a->as<ob::RealVectorStateSpace::StateType>();
    auto *bv = b->as<ob::RealVectorStateSpace::StateType>();
    av->values[0] = 0.0;
    av->values[1] = 0.0;
    av->values[2] = 0.0;
    av->values[3] = 0.0;
    av->values[4] = 0.0;
    bv->values[0] = 3.0;
    bv->values[1] = 4.0;
    bv->values[2] = 1.0;
    bv->values[3] = 2.0;
    bv->values[4] = 2.0;

    const double distance = space.distance(a, b);
    space.freeState(a);
    space.freeState(b);
    return expectNear("makespan distance uses max block L2", distance, 5.0);
}

bool testMakespanCompositeRejectsNonfiniteStates() {
    comotion::MakespanCompositeStateSpace space({2, 3});
    ob::RealVectorBounds bounds(5);
    bounds.setLow(-10.0);
    bounds.setHigh(10.0);
    space.setBounds(bounds);

    ob::State *a = space.allocState();
    ob::State *b = space.allocState();
    auto *av = a->as<ob::RealVectorStateSpace::StateType>();
    auto *bv = b->as<ob::RealVectorStateSpace::StateType>();
    for (unsigned int i = 0; i < 5; ++i) {
        av->values[i] = 0.0;
        bv->values[i] = 1.0;
    }
    av->values[2] = std::numeric_limits<double>::quiet_NaN();

    const bool in_bounds = space.satisfiesBounds(a);
    const double distance = space.distance(a, b);
    space.freeState(a);
    space.freeState(b);
    return expectTrue("makespan space rejects NaN bounds", !in_bounds) &&
           expectTrue("makespan distance returns infinity for NaN state",
                      std::isinf(distance));
}

bool testMakespanDirectSamplerProducesInformedSamples() {
    comotion::seedOmplGlobalFromUserPlanningSeed(21);
    auto space = std::make_shared<comotion::MakespanCompositeStateSpace>(
        std::vector<unsigned int>{2, 2});
    ob::RealVectorBounds bounds(4);
    bounds.setLow(-20.0);
    bounds.setHigh(20.0);
    space->setBounds(bounds);
    auto si = std::make_shared<ob::SpaceInformation>(space);
    si->setup();

    auto pdef = std::make_shared<ob::ProblemDefinition>(si);
    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);
    setRealVectorState(start.get(), {0.0, 0.0, 0.0, 0.0});
    setRealVectorState(goal.get(), {4.0, 0.0, 0.0, 4.0});
    pdef->addStartState(start);
    pdef->setGoalState(goal);
    auto objective =
        std::make_shared<comotion::aorrtc::MakespanPathLengthObjective>(si);
    pdef->setOptimizationObjective(objective);
    auto sampler = objective->allocInformedStateSampler(pdef, 1000);

    ob::State *sample = space->allocState();
    bool ok = true;
    for (int i = 0; i < 200; ++i) {
        const bool sampled = sampler->sampleUniform(sample, ob::Cost(6.0));
        if (!expectTrue("makespan direct sampler returns a sample", sampled)) {
            ok = false;
            break;
        }
        const double heuristic =
            makespanHeuristic(*space, start.get(), sample, goal.get());
        if (!expectTrue("makespan direct sample is in bounds",
                        space->satisfiesBounds(sample)) ||
            !expectTrue("makespan direct sample satisfies informed bound",
                        heuristic < 6.0)) {
            ok = false;
            break;
        }
    }
    space->freeState(sample);
    return ok;
}

bool testMakespanDirectSamplerHandlesFixedDimensions() {
    comotion::seedOmplGlobalFromUserPlanningSeed(23);
    auto space = std::make_shared<comotion::MakespanCompositeStateSpace>(
        std::vector<unsigned int>{3, 3});
    ob::RealVectorBounds bounds(6);
    bounds.setLow(-20.0);
    bounds.setHigh(20.0);
    bounds.setLow(2, 0.0);
    bounds.setHigh(2, 0.0);
    bounds.setLow(5, 0.0);
    bounds.setHigh(5, 0.0);
    space->setBounds(bounds);
    auto si = std::make_shared<ob::SpaceInformation>(space);
    si->setup();

    auto pdef = std::make_shared<ob::ProblemDefinition>(si);
    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);
    setRealVectorState(start.get(), {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    setRealVectorState(goal.get(), {4.0, 0.0, 0.0, 0.0, 4.0, 0.0});
    pdef->addStartState(start);
    pdef->setGoalState(goal);
    auto objective =
        std::make_shared<comotion::aorrtc::MakespanPathLengthObjective>(si);
    pdef->setOptimizationObjective(objective);
    auto sampler = objective->allocInformedStateSampler(pdef, 1000);

    ob::State *sample = space->allocState();
    bool ok = true;
    for (int i = 0; i < 200; ++i) {
        const bool sampled = sampler->sampleUniform(sample, ob::Cost(6.0));
        const auto *rv = sample->as<ob::RealVectorStateSpace::StateType>();
        const double heuristic =
            makespanHeuristic(*space, start.get(), sample, goal.get());
        if (!expectTrue("makespan direct sampler handles fixed dimensions",
                        sampled) ||
            !expectTrue("fixed-dimension sample remains in bounds",
                        space->satisfiesBounds(sample)) ||
            !expectNear("fixed z dimension for robot 0", rv->values[2], 0.0) ||
            !expectNear("fixed z dimension for robot 1", rv->values[5], 0.0) ||
            !expectTrue("fixed-dimension sample satisfies informed bound",
                        heuristic < 6.0)) {
            ok = false;
            break;
        }
    }
    space->freeState(sample);
    return ok;
}

bool testMakespanDirectSamplerRejectsImpossibleTightBound() {
    comotion::seedOmplGlobalFromUserPlanningSeed(22);
    auto space = std::make_shared<comotion::MakespanCompositeStateSpace>(
        std::vector<unsigned int>{2, 2});
    ob::RealVectorBounds bounds(4);
    bounds.setLow(-20.0);
    bounds.setHigh(20.0);
    space->setBounds(bounds);
    auto si = std::make_shared<ob::SpaceInformation>(space);
    si->setup();

    auto pdef = std::make_shared<ob::ProblemDefinition>(si);
    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);
    setRealVectorState(start.get(), {0.0, 0.0, 0.0, 0.0});
    setRealVectorState(goal.get(), {4.0, 0.0, 0.0, 4.0});
    pdef->addStartState(start);
    pdef->setGoalState(goal);
    auto objective =
        std::make_shared<comotion::aorrtc::MakespanPathLengthObjective>(si);
    pdef->setOptimizationObjective(objective);
    auto sampler = objective->allocInformedStateSampler(pdef, 1000);

    ob::State *sample = space->allocState();
    const bool sampled = sampler->sampleUniform(sample, ob::Cost(3.9));
    space->freeState(sample);
    return expectTrue("makespan direct sampler rejects impossible bound",
                      !sampled);
}

bool testPerRobotInformedDoesNotImplyCompositeInformed() {
    comotion::MakespanCompositeStateSpace space({1, 1});
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-20.0);
    bounds.setHigh(20.0);
    space.setBounds(bounds);

    ob::State *start = space.allocState();
    ob::State *goal = space.allocState();
    ob::State *candidate = space.allocState();
    setRealVectorState(start, {0.0, 0.0});
    setRealVectorState(goal, {10.0, 10.0});
    setRealVectorState(candidate, {9.0, 1.0});

    const double robot0 =
        std::abs(9.0 - 0.0) + std::abs(10.0 - 9.0);
    const double robot1 =
        std::abs(1.0 - 0.0) + std::abs(10.0 - 1.0);
    const double composite = makespanHeuristic(space, start, candidate, goal);
    space.freeState(start);
    space.freeState(goal);
    space.freeState(candidate);

    return expectTrue("individual robot lenses can accept adversarial sample",
                      robot0 <= 10.0 && robot1 <= 10.0) &&
           expectTrue("composite makespan heuristic rejects adversarial sample",
                      composite > 10.0);
}

bool testBoundedSingleRobotAOXRRTConnect() {
    comotion::seedOmplGlobalFromUserPlanningSeed(11);
    auto problem = makeParallelSphereProblem();

    comotion::aorrtc::SolveOptions loose;
    loose.planning_seed = 11;
    loose.cost_bound_timesteps = 32;
    loose.max_internal_samples = 2000;
    loose.max_internal_vertices = 2000;
    auto loose_result =
        comotion::aorrtc::solveSingleRobotBounded(*problem, 0, 1.0, loose);
    if (!expectTrue("loose single-robot bound returns exact solution",
                    loose_result.status == ob::PlannerStatus::EXACT_SOLUTION))
        return false;

    comotion::aorrtc::SolveOptions tight;
    tight.planning_seed = 12;
    tight.cost_bound_timesteps = 2;
    tight.max_internal_samples = 200;
    tight.max_internal_vertices = 200;
    auto tight_result =
        comotion::aorrtc::solveSingleRobotBounded(*problem, 0, 0.2, tight);
    return expectTrue("tight single-robot bound rejects exact solution",
                      tight_result.status != ob::PlannerStatus::EXACT_SOLUTION);
}

bool testBoundedCompositeAOXRRTConnect() {
    comotion::seedOmplGlobalFromUserPlanningSeed(12);
    auto problem = makeParallelSphereProblem();

    comotion::aorrtc::SolveOptions loose;
    loose.planning_seed = 13;
    loose.cost_bound_timesteps = 32;
    loose.max_internal_samples = 3000;
    loose.max_internal_vertices = 3000;
    auto loose_result =
        comotion::aorrtc::solveCompositeBounded(*problem, {}, 1.0, loose);
    if (!expectTrue("loose composite bound returns exact solution",
                    loose_result.status == ob::PlannerStatus::EXACT_SOLUTION))
        return false;
    if (!expectTrue("composite result splits into two robot paths",
                    loose_result.paths.size() == 2))
        return false;

    comotion::aorrtc::SolveOptions tight;
    tight.planning_seed = 14;
    tight.cost_bound_timesteps = 2;
    tight.max_internal_samples = 200;
    tight.max_internal_vertices = 200;
    auto tight_result =
        comotion::aorrtc::solveCompositeBounded(*problem, {}, 0.2, tight);
    return expectTrue("tight composite bound rejects exact solution",
                      tight_result.status != ob::PlannerStatus::EXACT_SOLUTION);
}

bool testAOARCSmokeRecordsFirstSolution() {
    comotion::seedOmplGlobalFromUserPlanningSeed(13);
    auto problem = makeParallelSphereProblem();

    comotion::AOARC planner;
    planner.setProblem(problem);
    planner.setInitialWindow(4);
    planner.setExpansionStep(4);
    planner.setLocalCompositeRrtMaxSamples(200);
    const auto status = planner.solve(0.5);
    if (!expectTrue("AOARC returns exact solution",
                    status == ob::PlannerStatus::EXACT_SOLUTION))
        return false;

    const auto paths = planner.getSolutionPaths();
    if (!expectTrue("AOARC returns one path per robot", paths.size() == 2))
        return false;

    comotion::CompositePathValidationOptions options;
    options.check_environment = true;
    const auto conflict = problem->collisionChecker().findFirstCompositePathConflict(
        paths, problem->robotModelPtrs(), options);
    if (!expectTrue("AOARC output is conflict-free", !conflict.has_value()))
        return false;

    const auto stats = planner.plannerStatsJson();
    return expectTrue("AOARC stats include first solution event",
                      stats.contains("solution_events") &&
                          !stats["solution_events"].empty() &&
                          stats["solution_events"][0]["kind"] == "first_solution");
}

} // namespace

int main() {
    bool ok = true;
    ok = testMakespanCompositeDistance() && ok;
    ok = testMakespanCompositeRejectsNonfiniteStates() && ok;
    ok = testMakespanDirectSamplerProducesInformedSamples() && ok;
    ok = testMakespanDirectSamplerHandlesFixedDimensions() && ok;
    ok = testMakespanDirectSamplerRejectsImpossibleTightBound() && ok;
    ok = testPerRobotInformedDoesNotImplyCompositeInformed() && ok;
    ok = testBoundedSingleRobotAOXRRTConnect() && ok;
    ok = testBoundedCompositeAOXRRTConnect() && ok;
    ok = testAOARCSmokeRecordsFirstSolution() && ok;
    return ok ? 0 : 1;
}
