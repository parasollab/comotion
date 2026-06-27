#include "comotion/robot/IKSolver.h"
#include <Eigen/Dense>
#include <cmath>

namespace comotion {

IKSolver::IKSolver(const RobotModel &robot) : robot_(robot) {}

Eigen::Affine3d IKSolver::eePose(const std::vector<double> &config,
                                  int ee_link_idx) const {
    auto transforms = robot_.getLinkTransforms(config);
    return transforms[ee_link_idx];
}

namespace {

Eigen::Vector3d orientationError(const Eigen::Quaterniond &target,
                                 const Eigen::Quaterniond &current) {
    Eigen::Quaterniond delta = target.normalized() * current.normalized().inverse();
    if (delta.w() < 0.0)
        delta.coeffs() *= -1.0;
    Eigen::AngleAxisd angle_axis(delta);
    return angle_axis.axis() * angle_axis.angle();
}

} // namespace

Eigen::VectorXd IKSolver::poseError(const std::vector<double> &config,
                                    int ee_link_idx,
                                    const IKRequest &req,
                                    double *position_residual,
                                    double *orientation_residual) const {
    const auto pose = eePose(config, ee_link_idx);
    const Eigen::Vector3d position_error =
        req.target_position - pose.translation();

    const bool with_orientation = req.target_orientation.has_value();
    Eigen::VectorXd error(with_orientation ? 6 : 3);
    error.head<3>() = position_error;

    double orientation_norm = 0.0;
    if (with_orientation) {
        const Eigen::Quaterniond current(pose.linear());
        const Eigen::Vector3d raw_orientation_error =
            orientationError(*req.target_orientation, current);
        orientation_norm = raw_orientation_error.norm();
        error.tail<3>() = req.orientation_weight * raw_orientation_error;
    }

    if (position_residual)
        *position_residual = position_error.norm();
    if (orientation_residual)
        *orientation_residual = orientation_norm;
    return error;
}

Eigen::MatrixXd IKSolver::errorJacobian(const std::vector<double> &config,
                                        int ee_link_idx,
                                        const IKRequest &req,
                                        const Eigen::VectorXd &error,
                                        double eps) const {
    int n = robot_.numJoints();
    Eigen::MatrixXd J(error.size(), n);

    std::vector<double> perturbed = config;
    for (int i = 0; i < n; ++i) {
        double orig = perturbed[i];
        perturbed[i] = orig + eps;
        Eigen::VectorXd e1 = poseError(perturbed, ee_link_idx, req);
        J.col(i) = (e1 - error) / eps;
        perturbed[i] = orig;
    }
    return J;
}

std::vector<double> IKSolver::randomConfig(std::mt19937 &rng) const {
    int n = robot_.numJoints();
    std::vector<double> cfg(n);
    for (int i = 0; i < n; ++i)
        cfg[i] = std::uniform_real_distribution<double>(
                     robot_.jointLower(i), robot_.jointUpper(i))(rng);
    return cfg;
}

void IKSolver::clampToLimits(std::vector<double> &config) const {
    int n = robot_.numJoints();
    for (int i = 0; i < n; ++i)
        config[i] = std::clamp(config[i], robot_.jointLower(i),
                               robot_.jointUpper(i));
}

IKResult IKSolver::solve(const IKRequest &req) const {
    int ee_idx = robot_.linkIndex(req.ee_link_name);
    if (ee_idx < 0)
        return {};

    int n = robot_.numJoints();
    std::vector<double> q = req.seed_config;
    if (static_cast<int>(q.size()) != n)
        return {};
    clampToLimits(q);

    for (int iter = 0; iter < req.max_iterations; ++iter) {
        double position_residual = 0.0;
        double orientation_residual = 0.0;
        Eigen::VectorXd err =
            poseError(q, ee_idx, req, &position_residual,
                      &orientation_residual);
        double residual = err.norm();

        if (position_residual < req.position_tolerance &&
            (!req.target_orientation.has_value() ||
             orientation_residual < req.orientation_tolerance)) {
            IKResult res;
            res.success = true;
            res.config = q;
            res.residual = residual;
            res.position_residual = position_residual;
            res.orientation_residual = orientation_residual;
            return res;
        }

        Eigen::MatrixXd J = errorJacobian(q, ee_idx, req, err);

        // Damped least-squares in error space: e(q + dq) ≈ e(q) + J dq.
        Eigen::MatrixXd JJt = J * J.transpose();
        JJt.diagonal().array() += req.damping * req.damping;
        Eigen::VectorXd v = JJt.ldlt().solve(err);
        Eigen::VectorXd dq = -J.transpose() * v;

        for (int i = 0; i < n; ++i)
            q[i] += req.step_scale * dq(i);
        clampToLimits(q);
    }

    // Did not converge — return best effort
    double position_residual = 0.0;
    double orientation_residual = 0.0;
    Eigen::VectorXd err =
        poseError(q, ee_idx, req, &position_residual, &orientation_residual);
    IKResult res;
    res.success = false;
    res.config = q;
    res.residual = err.norm();
    res.position_residual = position_residual;
    res.orientation_residual = orientation_residual;
    return res;
}

IKResult IKSolver::solveWithRestarts(const IKRequest &req,
                                      std::mt19937 &rng,
                                      int num_restarts) const {
    IKRequest local = req;
    IKResult best;

    for (int r = 0; r < num_restarts; ++r) {
        local.seed_config = randomConfig(rng);
        IKResult res = solve(local);
        if (res.success)
            return res;
        if (res.residual < best.residual)
            best = res;
    }
    return best;
}

} // namespace comotion
