#pragma once

#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/Goal.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <cstdint>
#include <utility>

namespace comotion::detail {

class SeededRRTConnect : public ompl::geometric::RRTConnect {
public:
    SeededRRTConnect(const ompl::base::SpaceInformationPtr &si,
                     std::uint_fast32_t seed)
        : ompl::geometric::RRTConnect(si) {
        rng_.setLocalSeed(seed);
    }
};

class SeededRealVectorStateSampler
    : public ompl::base::RealVectorStateSampler {
public:
    SeededRealVectorStateSampler(const ompl::base::StateSpace *space,
                                 std::uint_fast32_t seed)
        : ompl::base::RealVectorStateSampler(space) {
        rng_.setLocalSeed(seed);
    }
};

class SeededPathSimplifier : public ompl::geometric::PathSimplifier {
public:
    SeededPathSimplifier(
        ompl::base::SpaceInformationPtr si, std::uint_fast32_t seed,
        const ompl::base::GoalPtr &goal = ompl::base::GoalPtr(),
        const ompl::base::OptimizationObjectivePtr &objective = nullptr)
        : ompl::geometric::PathSimplifier(std::move(si), goal, objective) {
        rng_.setLocalSeed(seed);
    }
};

} // namespace comotion::detail
