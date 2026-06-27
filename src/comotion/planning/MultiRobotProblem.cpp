#include "comotion/planning/MultiRobotProblem.h"
#include "comotion/planning/MakespanCompositeStateSpace.h"
#include "comotion/collision/detail/ValidationUtils.h"

#include <ompl/base/MotionValidator.h>
#include <ompl/base/StateValidityChecker.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

#if COMOTION_HAVE_VAMP
#include <ompl/vamp/VampMotionValidator.h>
#include <ompl/vamp/VampStateSpace.h>
#include <ompl/vamp/VampStateValidityChecker.h>

#include <vamp/collision/environment.hh>
#include <vamp/collision/factory.hh>
#include <vamp/robots/panda.hh>
#include <vamp/robots/planar3.hh>
#include <vamp/robots/sphere.hh>
#include <vamp/robots/ur5.hh>
#endif

namespace comotion {

namespace {

ompl::base::RealVectorBounds computeBoundsForRobot(
    const RobotInstance &robot, int robot_idx,
    const std::map<int, std::pair<std::vector<double>, std::vector<double>>>
        &overrides) {
    const int ndof = robot.model->numJoints();
    ompl::base::RealVectorBounds bounds(ndof);
    auto it = overrides.find(robot_idx);
    if (it != overrides.end() &&
        static_cast<int>(it->second.first.size()) == ndof &&
        static_cast<int>(it->second.second.size()) == ndof) {
        for (int i = 0; i < ndof; ++i) {
            double lo = std::max(it->second.first[i], robot.model->jointLower(i));
            double hi = std::min(it->second.second[i], robot.model->jointUpper(i));
            if (lo > hi)
                lo = hi = 0.5 * (lo + hi);
            bounds.setLow(i, lo);
            bounds.setHigh(i, hi);
        }
    } else {
        for (int i = 0; i < ndof; ++i) {
            bounds.setLow(i, robot.model->jointLower(i));
            bounds.setHigh(i, robot.model->jointUpper(i));
        }
    }
    return bounds;
}

std::vector<double> extractConfigFromState(const ompl::base::State *state,
                                           int offset, int ndof) {
    const auto *rv =
        state->as<ompl::base::RealVectorStateSpace::StateType>();
    std::vector<double> config(static_cast<std::size_t>(ndof));
    for (int i = 0; i < ndof; ++i)
        config[static_cast<std::size_t>(i)] = rv->values[offset + i];
    return config;
}

class CompositeMotionValidator final : public ompl::base::MotionValidator {
public:
    struct RobotInfo {
        const RobotModel *model = nullptr;
        int offset = 0;
        int ndof = 0;
    };

    CompositeMotionValidator(
        const ompl::base::SpaceInformationPtr &si,
        std::shared_ptr<std::vector<RobotInfo>> infos,
        const CollisionChecker *checker, std::size_t resolution, double vmax)
        : ompl::base::MotionValidator(si),
          infos_(std::move(infos)),
          checker_(checker), resolution_(resolution), vmax_(vmax) {}

    bool checkMotion(const ompl::base::State *s1,
                     const ompl::base::State *s2) const override {
        CompositePathValidationOptions options;
        options.check_environment = true;
        const int spatial_checks = std::max(
            1, static_cast<int>(si_->getStateSpace()->validSegmentCount(s1, s2)));
        double max_dist = 0.0;
        if (vmax_ > 0.0 && resolution_ > 0) {
            for (const auto &info : *infos_) {
                const auto from =
                    extractConfigFromState(s1, info.offset, info.ndof);
                const auto to =
                    extractConfigFromState(s2, info.offset, info.ndof);
                double dist_sq = 0.0;
                for (std::size_t d = 0; d < from.size(); ++d) {
                    const double diff = to[d] - from[d];
                    dist_sq += diff * diff;
                }
                max_dist = std::max(max_dist, std::sqrt(dist_sq));
            }
        }
        options.discrete_num_checks_hint =
            detail::resolutionAwareMotionCheckCount(
                spatial_checks, max_dist, resolution_, vmax_);

        std::vector<const RobotModel *> robots;
        std::vector<std::vector<double>> from;
        std::vector<std::vector<double>> to;
        robots.reserve(infos_->size());
        from.reserve(infos_->size());
        to.reserve(infos_->size());
        for (const auto &info : *infos_) {
            robots.push_back(info.model);
            from.push_back(extractConfigFromState(s1, info.offset, info.ndof));
            to.push_back(extractConfigFromState(s2, info.offset, info.ndof));
        }
        return checker_->isCompositeMotionValid(robots, from, to, options);
    }

    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                     std::pair<ompl::base::State *, double> &lastValid) const override {
        lastValid.first = nullptr;
        lastValid.second = 0.0;
        return checkMotion(s1, s2);
    }

private:
    std::shared_ptr<std::vector<RobotInfo>> infos_;
    const CollisionChecker *checker_;
    std::size_t resolution_ = 1;
    double vmax_ = 1.0;
};

#if COMOTION_HAVE_VAMP

template <typename Robot>
class OwnedVampStateValidityChecker final
    : public ompl::vamp::VampStateValidityChecker<Robot> {
public:
    using Environment =
        typename ompl::vamp::VampStateValidityChecker<Robot>::Environment;

    OwnedVampStateValidityChecker(const ompl::base::SpaceInformationPtr &si,
                                  std::shared_ptr<Environment> env)
        : ompl::vamp::VampStateValidityChecker<Robot>(si, *env),
          env_(std::move(env)) {}

private:
    std::shared_ptr<Environment> env_;
};

template <typename Robot>
class OwnedVampMotionValidator final
    : public ompl::vamp::VampMotionValidator<Robot> {
public:
    using Environment =
        typename ompl::vamp::VampMotionValidator<Robot>::Environment;

    OwnedVampMotionValidator(const ompl::base::SpaceInformationPtr &si,
                             std::shared_ptr<Environment> env)
        : ompl::vamp::VampMotionValidator<Robot>(si, *env),
          env_(std::move(env)) {}

private:
    std::shared_ptr<Environment> env_;
};

void configureVampSphereRobot(const RobotModel &robot,
                              const ompl::base::RealVectorBounds &bounds) {
    std::array<float, 3> lows{};
    std::array<float, 3> highs{};
    for (int i = 0; i < 3; ++i) {
        lows[static_cast<std::size_t>(i)] = static_cast<float>(bounds.low[i]);
        highs[static_cast<std::size_t>(i)] = static_cast<float>(bounds.high[i]);
    }

    float radius = 0.0f;
    for (const auto &link : robot.links()) {
        if (!link.collision_spheres.empty()) {
            radius = static_cast<float>(link.collision_spheres.front().radius);
            break;
        }
    }
    if (radius <= 0.0f) {
        throw std::runtime_error(
            "MultiRobotProblem: VAMP sphere robot requires a collision sphere.");
    }

    vamp::robots::Sphere::set_lows(lows);
    vamp::robots::Sphere::set_highs(highs);
    vamp::robots::Sphere::set_radius(radius);
}

bool appendVampCylinderObstacle(vamp::collision::Environment<float> &environment,
                                const ObstacleCylinder &cylinder,
                                const Eigen::Affine3f &robot_from_world,
                                std::size_t index) {
    constexpr float kAxisTol = 1e-6f;
    if (cylinder.radius <= 0.0 || cylinder.half_height <= 0.0)
        return false;

    Eigen::Vector3f local_axis =
        robot_from_world.linear() * cylinder.axis.cast<float>();
    const float axis_norm = local_axis.norm();
    if (axis_norm <= kAxisTol)
        return false;

    local_axis /= axis_norm;

    const Eigen::Vector3f local_center =
        robot_from_world * cylinder.center.cast<float>();
    const Eigen::Vector3f half_axis =
        local_axis * static_cast<float>(cylinder.half_height);

    auto obstacle = vamp::collision::factory::cylinder::endpoints::eigen(
        local_center + half_axis, local_center - half_axis,
        static_cast<float>(cylinder.radius));
    obstacle.name = "obstacle_cylinder_" + std::to_string(index);
    environment.cylinders.emplace_back(obstacle);
    return true;
}

void appendVampAttachment(vamp::collision::Environment<float> &environment,
                          const AttachedBody &attachment) {
    Eigen::Isometry3f tf = Eigen::Isometry3f::Identity();
    tf.linear() = attachment.link_from_attachment.linear().cast<float>();
    tf.translation() =
        attachment.link_from_attachment.translation().cast<float>();
    environment.attachments.emplace(tf);
    environment.attachments->spheres.reserve(attachment.spheres.size());
    for (const auto &sphere : attachment.spheres) {
        environment.attachments->spheres.emplace_back(
            vamp::collision::factory::sphere::eigen(
                sphere.center.cast<float>(), static_cast<float>(sphere.radius)));
    }
}

template <typename Environment>
std::shared_ptr<Environment> buildVampEnvironment(const RobotModel &robot,
                                                  const CollisionChecker &cc) {
    vamp::collision::Environment<float> env_float;
    const Eigen::Affine3f robot_from_world =
        robot.getBaseTransform().cast<float>().inverse();
    for (std::size_t i = 0; i < cc.obstacles().size(); ++i) {
        const auto &sphere = cc.obstacles()[i];
        const Eigen::Vector3f local_center =
            robot_from_world * sphere.center.cast<float>();
        env_float.spheres.emplace_back(
            vamp::collision::factory::sphere::eigen(
                local_center, static_cast<float>(sphere.radius)));
        env_float.spheres.back().name = "obstacle_sphere_" + std::to_string(i);
    }
    for (std::size_t i = 0; i < cc.cylinders().size(); ++i) {
        appendVampCylinderObstacle(env_float, cc.cylinders()[i],
                                   robot_from_world, i);
    }
    if (robot.hasAttachment())
        appendVampAttachment(env_float, *robot.attachment());
    env_float.sort();
    return std::make_shared<Environment>(env_float);
}

template <typename Robot>
std::shared_ptr<ompl::base::SpaceInformation> createVampSpaceInfo(
    const RobotInstance &robot_inst, const ompl::base::RealVectorBounds &bounds,
    const CollisionChecker &checker) {
    auto space = std::make_shared<ompl::vamp::VampStateSpace<Robot>>();
    space->setBounds(bounds);
    auto si = std::make_shared<ompl::base::SpaceInformation>(space);
    auto env =
        buildVampEnvironment<typename ompl::vamp::VampStateValidityChecker<Robot>::Environment>(
            *robot_inst.model, checker);
    const RobotModel *robot = robot_inst.model.get();
    const int ndof = robot->numJoints();
    const CollisionChecker *checker_ptr = &checker;
    // Use CoMotion's collision adapter for point validity. OMPL's VAMP point
    // checker is unstable for zero-width dimensions in the 2D sphere setup.
    si->setStateValidityChecker(
        [robot, checker_ptr, ndof](const ompl::base::State *state) -> bool {
            return checker_ptr->isValidSingleFull(
                *robot, extractConfigFromState(state, 0, ndof));
        });
    si->setMotionValidator(
        std::make_shared<OwnedVampMotionValidator<Robot>>(si, env));
    si->setup();
    return si;
}

#endif

} // namespace

MultiRobotProblem::MultiRobotProblem(CollisionChecker::Backend collision_backend)
    : cc_(collision_backend) {}

void MultiRobotProblem::addRobot(std::shared_ptr<RobotModel> model,
                                  const std::vector<double> &start,
                                  const std::vector<double> &goal) {
    robots_.push_back({std::move(model), start, goal});
}

void MultiRobotProblem::setObstacles(
    const std::vector<ObstacleSphere> &obstacles) {
    cc_.setObstacles(obstacles);
}

void MultiRobotProblem::setCylinderObstacles(
    const std::vector<ObstacleCylinder> &cylinders) {
    cc_.setCylinderObstacles(cylinders);
}

std::vector<const RobotModel *> MultiRobotProblem::robotModelPtrs() const {
    std::vector<const RobotModel *> ptrs;
    ptrs.reserve(robots_.size());
    for (auto &r : robots_)
        ptrs.push_back(r.model.get());
    return ptrs;
}

void MultiRobotProblem::setCspaceBoundsForRobot(int robot_idx,
                                                const std::vector<double> &lo,
                                                const std::vector<double> &hi) {
    cspace_bounds_override_[robot_idx] = {lo, hi};
}

void MultiRobotProblem::clearCspaceBounds() {
    cspace_bounds_override_.clear();
}

std::shared_ptr<ompl::base::RealVectorStateSpace>
MultiRobotProblem::createStateSpace(int robot_idx) const {
    auto &r = robots_[robot_idx];
    int ndof = r.model->numJoints();
    auto space = std::make_shared<ompl::base::RealVectorStateSpace>(ndof);
    ompl::base::RealVectorBounds bounds =
        computeBoundsForRobot(r, robot_idx, cspace_bounds_override_);
    space->setBounds(bounds);
    return space;
}

std::shared_ptr<ompl::base::RealVectorStateSpace>
MultiRobotProblem::createCompositeStateSpace(
    const std::vector<int> &robot_indices) const {
    int total_dof = 0;
    for (int idx : robot_indices)
        total_dof += robots_[idx].model->numJoints();

    auto space = std::make_shared<ompl::base::RealVectorStateSpace>(total_dof);
    ompl::base::RealVectorBounds bounds(total_dof);
    int offset = 0;
    for (int idx : robot_indices) {
        auto &r = robots_[idx];
        int ndof = r.model->numJoints();
        auto it = cspace_bounds_override_.find(idx);
        if (it != cspace_bounds_override_.end() &&
            static_cast<int>(it->second.first.size()) == ndof &&
            static_cast<int>(it->second.second.size()) == ndof) {
            for (int i = 0; i < ndof; ++i) {
                double lo = std::max(it->second.first[i], r.model->jointLower(i));
                double hi = std::min(it->second.second[i], r.model->jointUpper(i));
                if (lo > hi)
                    lo = hi = 0.5 * (lo + hi);
                bounds.setLow(offset + i, lo);
                bounds.setHigh(offset + i, hi);
            }
        } else {
            for (int i = 0; i < ndof; ++i) {
                bounds.setLow(offset + i, r.model->jointLower(i));
                bounds.setHigh(offset + i, r.model->jointUpper(i));
            }
        }
        offset += ndof;
    }
    space->setBounds(bounds);
    return space;
}

std::shared_ptr<ompl::base::SpaceInformation>
MultiRobotProblem::createSpaceInfo(int robot_idx) const {
    const auto &robot_inst = robots_[robot_idx];
    const RobotModel *robot = robot_inst.model.get();
    const ompl::base::RealVectorBounds bounds =
        computeBoundsForRobot(robot_inst, robot_idx, cspace_bounds_override_);

#if COMOTION_HAVE_VAMP
    if (cc_.backend() == CollisionChecker::Backend::Vamp &&
        !robot->hasAttachment()) {
        switch (robot->robotFamily()) {
        case RobotModel::RobotFamily::Sphere:
            configureVampSphereRobot(*robot, bounds);
            return createVampSpaceInfo<vamp::robots::Sphere>(robot_inst,
                                                             bounds, cc_);
        case RobotModel::RobotFamily::Panda:
            return createVampSpaceInfo<vamp::robots::Panda>(robot_inst, bounds,
                                                            cc_);
        case RobotModel::RobotFamily::UR5:
            return createVampSpaceInfo<vamp::robots::UR5>(robot_inst, bounds,
                                                          cc_);
        case RobotModel::RobotFamily::Planar3:
            return createVampSpaceInfo<vamp::robots::Planar3>(robot_inst,
                                                              bounds, cc_);
        case RobotModel::RobotFamily::Unknown:
            throw std::runtime_error(
                "MultiRobotProblem: VAMP backend requires a supported robot "
                "family for single-robot OMPL planning.");
        }
    }
#endif

    auto space = createStateSpace(robot_idx);
    auto si = std::make_shared<ompl::base::SpaceInformation>(space);
    const CollisionChecker *checker = &cc_;
    int ndof = robot->numJoints();

    si->setStateValidityChecker(
        [robot, checker, ndof](const ompl::base::State *state) -> bool {
            const auto *rv =
                state->as<ompl::base::RealVectorStateSpace::StateType>();
            std::vector<double> config(static_cast<std::size_t>(ndof));
            for (int i = 0; i < ndof; ++i)
                config[static_cast<std::size_t>(i)] = rv->values[i];
            return checker->isValidSingleFull(*robot, config);
        });

    si->setup();
    return si;
}

std::shared_ptr<ompl::base::SpaceInformation>
MultiRobotProblem::createCompositeSpaceInfo(
    const std::vector<int> &robot_indices) const {
    auto space = createCompositeStateSpace(robot_indices);
    auto si = std::make_shared<ompl::base::SpaceInformation>(space);

    // Capture robot pointers and DOF offsets
    auto infos = std::make_shared<std::vector<CompositeMotionValidator::RobotInfo>>();
    int offset = 0;
    for (int idx : robot_indices) {
        int ndof = robots_[idx].model->numJoints();
        infos->push_back({robots_[idx].model.get(), offset, ndof});
        offset += ndof;
    }

    const CollisionChecker *checker = &cc_;

    si->setStateValidityChecker(
        [infos, checker](const ompl::base::State *state) -> bool {
            auto *rv = state->as<ompl::base::RealVectorStateSpace::StateType>();

            std::vector<const RobotModel *> robots;
            std::vector<std::vector<double>> configs;

            for (auto &ri : *infos) {
                std::vector<double> cfg(ri.ndof);
                for (int i = 0; i < ri.ndof; ++i)
                    cfg[i] = rv->values[ri.offset + i];
                robots.push_back(ri.model);
                configs.push_back(std::move(cfg));
            }
            return checker->isValidComposite(robots, configs);
        });

    // Use 1% of single-robot diagonal for motion validation (denser than default 1% of composite).
    // Pairwise clearance is governed by robot-scale motion, not full composite extent.
    if (!robot_indices.empty()) {
        auto single_space = createStateSpace(robot_indices[0]);
        double single_extent = single_space->getMaximumExtent();
        double composite_extent = space->getMaximumExtent();
        if (composite_extent > 1e-10) {
            double fraction = 0.01 * single_extent / composite_extent;
            si->setStateValidityCheckingResolution(fraction);
        }
    }

    si->setMotionValidator(
        std::make_shared<CompositeMotionValidator>(si, infos, checker,
                                                   resolution_, vmax_));
    si->setup();
    return si;
}

std::shared_ptr<ompl::base::SpaceInformation>
MultiRobotProblem::createMakespanCompositeSpaceInfo(
    const std::vector<int> &robot_indices) const {
    auto default_space = createCompositeStateSpace(robot_indices);

    std::vector<unsigned int> block_dims;
    block_dims.reserve(robot_indices.size());
    for (const int idx : robot_indices)
        block_dims.push_back(
            static_cast<unsigned int>(robots_[idx].model->numJoints()));

    auto space = std::make_shared<MakespanCompositeStateSpace>(block_dims);
    space->setBounds(default_space->getBounds());
    auto si = std::make_shared<ompl::base::SpaceInformation>(space);

    auto infos =
        std::make_shared<std::vector<CompositeMotionValidator::RobotInfo>>();
    int offset = 0;
    for (int idx : robot_indices) {
        int ndof = robots_[idx].model->numJoints();
        infos->push_back({robots_[idx].model.get(), offset, ndof});
        offset += ndof;
    }

    const CollisionChecker *checker = &cc_;
    si->setStateValidityChecker(
        [infos, checker](const ompl::base::State *state) -> bool {
            std::vector<const RobotModel *> robots;
            std::vector<std::vector<double>> configs;
            robots.reserve(infos->size());
            configs.reserve(infos->size());

            for (auto &ri : *infos) {
                robots.push_back(ri.model);
                configs.push_back(
                    extractConfigFromState(state, ri.offset, ri.ndof));
            }
            return checker->isValidComposite(robots, configs);
        });

    if (!robot_indices.empty()) {
        auto single_space = createStateSpace(robot_indices[0]);
        const double single_extent = single_space->getMaximumExtent();
        const double composite_extent = space->getMaximumExtent();
        if (composite_extent > 1e-10) {
            const double fraction = 0.01 * single_extent / composite_extent;
            si->setStateValidityCheckingResolution(fraction);
        }
    }

    si->setMotionValidator(
        std::make_shared<CompositeMotionValidator>(si, infos, checker,
                                                   resolution_, vmax_));
    si->setup();
    return si;
}

} // namespace comotion
