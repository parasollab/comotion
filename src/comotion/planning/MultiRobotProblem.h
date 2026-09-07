#pragma once

#include "comotion/robot/RobotModel.h"
#include "comotion/collision/CollisionChecker.h"
#include "comotion/planning/Path.h"

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>

#include <map>
#include <memory>
#include <vector>

namespace comotion {

struct RobotInstance {
    std::shared_ptr<RobotModel> model;
    std::vector<double> start;
    std::vector<double> goal;
};

class MultiRobotProblem {
public:
    MultiRobotProblem() = default;

    /// Use the same backend as the parent problem when cloning (e.g. ARC subproblems).
    explicit MultiRobotProblem(CollisionChecker::Backend collision_backend);

    void addRobot(std::shared_ptr<RobotModel> model,
                  const std::vector<double> &start,
                  const std::vector<double> &goal);

    void setObstacles(const std::vector<ObstacleSphere> &obstacles);
    void setCylinderObstacles(const std::vector<ObstacleCylinder> &cylinders);
    void setFixedRobots(std::vector<CollisionChecker::FixedRobot> robots) {
        cc_.setFixedRobots(std::move(robots));
    }

    int numRobots() const { return static_cast<int>(robots_.size()); }
    const RobotInstance &robot(int i) const { return robots_[i]; }
    const std::vector<RobotInstance> &robots() const { return robots_; }

    CollisionChecker &collisionChecker() { return cc_; }
    const CollisionChecker &collisionChecker() const { return cc_; }

    std::vector<const RobotModel *> robotModelPtrs() const;

    // Create an OMPL state space for a single robot
    std::shared_ptr<ompl::base::RealVectorStateSpace>
    createStateSpace(int robot_idx) const;

    // Create a composite state space for a subset of robots
    std::shared_ptr<ompl::base::RealVectorStateSpace>
    createCompositeStateSpace(const std::vector<int> &robot_indices) const;

    // Create SpaceInformation for a single robot (with obstacle + self-collision checking)
    std::shared_ptr<ompl::base::SpaceInformation>
    createSpaceInfo(int robot_idx) const;

    // Create SpaceInformation for composite space (obstacle + inter-robot checking)
    std::shared_ptr<ompl::base::SpaceInformation>
    createCompositeSpaceInfo(const std::vector<int> &robot_indices) const;

    // Create composite SpaceInformation whose distance/extent are the maximum
    // per-robot block motion. C-space bound overrides are preserved.
    std::shared_ptr<ompl::base::SpaceInformation>
    createMakespanCompositeSpaceInfo(
        const std::vector<int> &robot_indices) const;

    size_t resolution() const { return resolution_; }
    void setResolution(size_t r) { resolution_ = r; }

    double vmax() const { return vmax_; }
    void setVmax(double v) { vmax_ = v; }

    // Optional c-space bounds for subproblem sampling (clamped to joint limits)
    void setCspaceBoundsForRobot(int robot_idx,
                                 const std::vector<double> &lo,
                                 const std::vector<double> &hi);
    void clearCspaceBounds();

private:
    std::vector<RobotInstance> robots_;
    CollisionChecker cc_;
    size_t resolution_ = 128;
    double vmax_ = 1.0;
    std::map<int, std::pair<std::vector<double>, std::vector<double>>>
        cspace_bounds_override_;
};

} // namespace comotion
