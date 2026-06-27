#pragma once

#include "comotion/robot/RobotModel.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

namespace comotion {

struct SphericalGraspOptions {
    std::vector<std::string> ee_link_names;
    std::vector<Eigen::Vector3d> object_center_from_ee;
    std::vector<Eigen::Affine3d> link_from_object_targets;
    std::vector<std::string> attachment_touch_links;
    std::vector<double> seed_config;
    int restarts = 80;
    int max_iterations = 250;
    double position_tolerance = 2e-3;
    double orientation_tolerance = 5e-2;
    double orientation_weight = 0.15;
    double step_scale = 0.45;
    double damping = 0.05;
};

struct SphericalGraspResult {
    bool success = false;
    std::string ee_link_name;
    std::vector<double> config;
    AttachedBody attachment;
    Eigen::Vector3d ee_target = Eigen::Vector3d::Zero();
    Eigen::Affine3d world_from_ee_target = Eigen::Affine3d::Identity();
    double residual = 0.0;
    double position_residual = 0.0;
    double orientation_residual = 0.0;
};

std::string defaultEndEffectorLinkName(const RobotModel &robot);

AttachedBody makeSphericalAttachment(const std::string &object_name,
                                     const std::string &ee_link_name,
                                     const Eigen::Vector3d &object_center_from_ee,
                                     double object_radius);

AttachedBody makeSphericalAttachment(const std::string &object_name,
                                     const std::string &ee_link_name,
                                     const Eigen::Affine3d &link_from_object,
                                     double object_radius);

SphericalGraspResult computeSphericalObjectGrasp(
    const RobotModel &robot,
    const std::string &object_name,
    const Eigen::Affine3d &world_from_object,
    double object_radius,
    std::mt19937 &rng,
    const SphericalGraspOptions &options = {});

SphericalGraspResult computeSphericalObjectGrasp(
    const RobotModel &robot,
    const std::string &object_name,
    const Eigen::Vector3d &object_center,
    double object_radius,
    std::mt19937 &rng,
    const SphericalGraspOptions &options = {});

} // namespace comotion
