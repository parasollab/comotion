#pragma once

#include "comotion/robot/RobotModel.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <limits>
#include <optional>
#include <random>
#include <string>
#include <vector>

namespace comotion {

struct IKRequest {
    Eigen::Vector3d target_position;
    std::optional<Eigen::Quaterniond> target_orientation;
    std::string ee_link_name;
    std::vector<double> seed_config;  // empty → random seed
    double position_tolerance = 1e-3;
    double orientation_tolerance = 5e-2;
    double orientation_weight = 0.15;
    int max_iterations = 200;
    double step_scale = 0.5;
    double damping = 0.05;
};

struct IKResult {
    bool success = false;
    std::vector<double> config;
    double residual = std::numeric_limits<double>::infinity();
    double position_residual = std::numeric_limits<double>::infinity();
    double orientation_residual = 0.0;
};

class IKSolver {
public:
    explicit IKSolver(const RobotModel &robot);

    IKResult solve(const IKRequest &req) const;

    // Solve with multiple random restarts, returning the first success.
    IKResult solveWithRestarts(const IKRequest &req,
                               std::mt19937 &rng,
                               int num_restarts = 50) const;

private:
    Eigen::Affine3d eePose(const std::vector<double> &config,
                           int ee_link_idx) const;

    Eigen::VectorXd poseError(const std::vector<double> &config,
                              int ee_link_idx,
                              const IKRequest &req,
                              double *position_residual = nullptr,
                              double *orientation_residual = nullptr) const;

    // Error-space Jacobian via finite differences.
    Eigen::MatrixXd errorJacobian(const std::vector<double> &config,
                                  int ee_link_idx,
                                  const IKRequest &req,
                                  const Eigen::VectorXd &error,
                                  double eps = 1e-6) const;

    std::vector<double> randomConfig(std::mt19937 &rng) const;
    void clampToLimits(std::vector<double> &config) const;

    const RobotModel &robot_;
};

} // namespace comotion
