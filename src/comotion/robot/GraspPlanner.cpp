#include "comotion/robot/GraspPlanner.h"

#include "comotion/robot/IKSolver.h"

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace comotion {

std::string defaultEndEffectorLinkName(const RobotModel &robot) {
    if (robot.linkIndex("panda_grasptarget") >= 0)
        return "panda_grasptarget";
    if (robot.linkIndex("panda_hand") >= 0)
        return "panda_hand";
    if (robot.linkIndex("gripper_link") >= 0)
        return "gripper_link";
    if (robot.linkIndex("robotiq85_base_link") >= 0)
        return "robotiq85_base_link";
    if (robot.linkIndex("link_z") >= 0)
        return "link_z";
    if (robot.links().empty())
        return {};
    return robot.links().back().name;
}

void appendExistingLink(const RobotModel &robot,
                        std::vector<std::string> &links,
                        const std::string &name) {
    if (name.empty() || robot.linkIndex(name) < 0)
        return;
    if (std::find(links.begin(), links.end(), name) == links.end())
        links.push_back(name);
}

std::vector<std::string> defaultAttachmentTouchLinks(
    const RobotModel &robot,
    const std::string &ee_link_name) {
    std::vector<std::string> links;
    appendExistingLink(robot, links, ee_link_name);
    appendExistingLink(robot, links, "panda_grasptarget");
    appendExistingLink(robot, links, "panda_hand");
    appendExistingLink(robot, links, "panda_leftfinger");
    appendExistingLink(robot, links, "panda_rightfinger");
    appendExistingLink(robot, links, "panda_link8");
    appendExistingLink(robot, links, "panda_link7");
    appendExistingLink(robot, links, "panda_link6");
    appendExistingLink(robot, links, "gripper_link");
    appendExistingLink(robot, links, "robotiq85_base_link");
    appendExistingLink(robot, links, "wrist_3_link");
    appendExistingLink(robot, links, "link_z");
    return links;
}

AttachedBody makeSphericalAttachment(const std::string &object_name,
                                     const std::string &ee_link_name,
                                     const Eigen::Vector3d &object_center_from_ee,
                                     double object_radius) {
    Eigen::Affine3d link_from_object =
        Eigen::Affine3d(Eigen::Translation3d(object_center_from_ee));
    return makeSphericalAttachment(object_name, ee_link_name, link_from_object,
                                   object_radius);
}

AttachedBody makeSphericalAttachment(const std::string &object_name,
                                     const std::string &ee_link_name,
                                     const Eigen::Affine3d &link_from_object,
                                     double object_radius) {
    if (object_radius <= 0.0)
        throw std::invalid_argument(
            "makeSphericalAttachment requires a positive object radius");

    AttachedBody attachment;
    attachment.name = object_name;
    attachment.link_name = ee_link_name;
    attachment.link_from_attachment = link_from_object;
    attachment.spheres.push_back(AttachmentSphere{Eigen::Vector3d::Zero(),
                                                  object_radius});
    attachment.ignored_self_collision_links.push_back(ee_link_name);
    return attachment;
}

SphericalGraspResult computeSphericalObjectGrasp(
    const RobotModel &robot,
    const std::string &object_name,
    const Eigen::Affine3d &world_from_object,
    double object_radius,
    std::mt19937 &rng,
    const SphericalGraspOptions &options) {
    SphericalGraspOptions local = options;
    if (local.ee_link_names.empty()) {
        const auto ee = defaultEndEffectorLinkName(robot);
        if (!ee.empty())
            local.ee_link_names.push_back(ee);
    }
    if (local.object_center_from_ee.empty()) {
        local.object_center_from_ee.push_back(Eigen::Vector3d::Zero());
    }
    if (local.link_from_object_targets.empty()) {
        for (const auto &center_from_ee : local.object_center_from_ee) {
            local.link_from_object_targets.emplace_back(
                Eigen::Translation3d(center_from_ee));
        }
    }

    IKSolver solver(robot);
    SphericalGraspResult best;
    best.residual = std::numeric_limits<double>::infinity();

    for (const auto &ee_link : local.ee_link_names) {
        if (robot.linkIndex(ee_link) < 0)
            continue;
        for (const auto &link_from_object : local.link_from_object_targets) {
            const Eigen::Affine3d world_from_ee =
                world_from_object * link_from_object.inverse();
            IKRequest request;
            request.target_position = world_from_ee.translation();
            request.target_orientation =
                Eigen::Quaterniond(world_from_ee.linear()).normalized();
            request.ee_link_name = ee_link;
            request.seed_config = local.seed_config;
            request.position_tolerance = local.position_tolerance;
            request.orientation_tolerance = local.orientation_tolerance;
            request.orientation_weight = local.orientation_weight;
            request.max_iterations = local.max_iterations;
            request.step_scale = local.step_scale;
            request.damping = local.damping;

            IKResult result;
            if (request.seed_config.empty()) {
                result = solver.solveWithRestarts(request, rng, local.restarts);
            } else {
                result = solver.solve(request);
                if (!result.success && local.restarts > 0) {
                    IKRequest restart_request = request;
                    restart_request.seed_config.clear();
                    const auto restarted =
                        solver.solveWithRestarts(restart_request, rng,
                                                 local.restarts);
                    if (restarted.residual < result.residual)
                        result = restarted;
                }
            }

            if (result.residual >= best.residual)
                continue;

            best.success = result.success;
            best.ee_link_name = ee_link;
            best.config = result.config;
            best.attachment =
                makeSphericalAttachment(object_name, ee_link, link_from_object,
                                        object_radius);
            best.attachment.ignored_self_collision_links =
                local.attachment_touch_links.empty()
                    ? defaultAttachmentTouchLinks(robot, ee_link)
                    : local.attachment_touch_links;
            appendExistingLink(robot, best.attachment.ignored_self_collision_links,
                               ee_link);
            best.ee_target = request.target_position;
            best.world_from_ee_target = world_from_ee;
            best.residual = result.residual;
            best.position_residual = result.position_residual;
            best.orientation_residual = result.orientation_residual;
            if (result.success)
                return best;
        }
    }

    best.success = false;
    return best;
}

SphericalGraspResult computeSphericalObjectGrasp(
    const RobotModel &robot,
    const std::string &object_name,
    const Eigen::Vector3d &object_center,
    double object_radius,
    std::mt19937 &rng,
    const SphericalGraspOptions &options) {
    Eigen::Affine3d world_from_object = Eigen::Affine3d::Identity();
    world_from_object.translation() = object_center;
    return computeSphericalObjectGrasp(robot, object_name, world_from_object,
                                       object_radius, rng, options);
}

} // namespace comotion
