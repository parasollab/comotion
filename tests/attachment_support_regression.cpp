#include "comotion/collision/CollisionChecker.h"
#include "comotion/robot/FlyingSphere.h"
#include "comotion/robot/GraspPlanner.h"

#include <Eigen/Core>

#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace {

bool expectEqual(const std::string &label, bool actual, bool expected) {
    if (actual == expected)
        return true;
    std::cerr << "attachment_support_regression: " << label << " expected "
              << expected << " got " << actual << "\n";
    return false;
}

comotion::AttachedBody payload(const std::string &name,
                               const Eigen::Vector3d &center,
                               double radius) {
    return comotion::makeSphericalAttachment(name, "link_z", center, radius);
}

} // namespace

int main() {
    auto carrier = std::make_shared<comotion::FlyingSphere>(0.10, -5.0, 5.0);
    carrier->setAttachment(payload("box", Eigen::Vector3d(0.30, 0.0, 0.0), 0.10));

    comotion::CollisionChecker sphere_backend(
        comotion::CollisionChecker::Backend::Spheres);
    comotion::CollisionChecker vamp_backend(
        comotion::CollisionChecker::Backend::Vamp);
    const comotion::ObstacleSphere obstacle{
        Eigen::Vector3d(0.30, 0.0, 0.0), 0.08};
    sphere_backend.setObstacles({obstacle});
    vamp_backend.setObstacles({obstacle});

    if (!expectEqual("sphere backend sees attached obstacle collision",
                     sphere_backend.isValidSingle(*carrier, {0.0, 0.0, 0.0}),
                     false)) {
        return 1;
    }
    if (!expectEqual("vamp backend sees attached obstacle collision",
                     vamp_backend.isValidSingle(*carrier, {0.0, 0.0, 0.0}),
                     false)) {
        return 1;
    }

    auto other = std::make_shared<comotion::FlyingSphere>(0.10, -5.0, 5.0);
    if (!expectEqual("sphere backend sees attached pair collision",
                     sphere_backend.isValidPair(*carrier, {0.0, 0.0, 0.0},
                                                *other, {0.30, 0.0, 0.0}),
                     false)) {
        return 1;
    }
    if (!expectEqual("vamp backend sees attached pair collision",
                     vamp_backend.isValidPair(*carrier, {0.0, 0.0, 0.0},
                                              *other, {0.30, 0.0, 0.0}),
                     false)) {
        return 1;
    }

    std::mt19937 rng(17);
    auto grasp = comotion::computeSphericalObjectGrasp(
        *other, "box", Eigen::Vector3d(0.4, 0.2, 0.1), 0.05, rng);
    if (!expectEqual("flying sphere grasp solves position IK", grasp.success,
                     true)) {
        return 1;
    }
    if (!expectEqual("grasp attachment has payload sphere",
                     grasp.attachment.spheres.size() == 1, true)) {
        return 1;
    }

    return 0;
}
