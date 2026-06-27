#pragma once

#include <Eigen/Geometry>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <utility>

namespace comotion {

struct CollisionSphere {
    Eigen::Vector3d center;
    double radius;
    int link_index;
};

struct CollisionBox {
    Eigen::Affine3d origin = Eigen::Affine3d::Identity();
    Eigen::Vector3d size = Eigen::Vector3d::Ones();
    int link_index = -1;
};

struct CollisionCylinder {
    Eigen::Affine3d origin = Eigen::Affine3d::Identity();
    double radius = 0.0;
    double length = 0.0;
    int link_index = -1;
};

// Triangle mesh collision entry from URDF (path resolved at load time).
struct CollisionMesh {
    Eigen::Affine3d origin = Eigen::Affine3d::Identity();
    std::string resolved_path;
    Eigen::Vector3d scale = Eigen::Vector3d::Ones();
};

struct JointInfo {
    std::string name;
    std::string type; // "revolute" or "fixed"
    std::string parent_link;
    std::string child_link;
    Eigen::Affine3d origin;
    Eigen::Vector3d axis;
    double lower_limit = 0.0;
    double upper_limit = 0.0;
    double velocity_limit = 0.0;
};

struct LinkInfo {
    std::string name;
    int index;
    int parent_joint_index = -1;
    std::vector<CollisionSphere> collision_spheres;
    std::vector<CollisionBox> collision_boxes;
    std::vector<CollisionCylinder> collision_cylinders;
    std::vector<CollisionMesh> collision_meshes;
};

class RobotModel {
public:
    enum class RobotFamily { Unknown, Sphere, Panda, UR5, Planar3 };

    RobotModel() = default;

    void loadURDF(const std::string &urdf_path);
    void loadSRDF(const std::string &srdf_path);

    void setBaseTransform(const Eigen::Affine3d &T);
    const Eigen::Affine3d &getBaseTransform() const { return base_transform_; }
    RobotFamily robotFamily() const;
    void setRobotFamily(RobotFamily family) { robot_family_ = family; }

    int numJoints() const { return static_cast<int>(active_joints_.size()); }
    double jointLower(int i) const { return active_joints_[i]->lower_limit; }
    double jointUpper(int i) const { return active_joints_[i]->upper_limit; }
    double jointVelocityLimit(int i) const { return active_joints_[i]->velocity_limit; }
    std::vector<std::string> activeJointNames() const;

    // FK: compute world-frame collision sphere positions for a given config
    std::vector<CollisionSphere> getCollisionSpheres(
        const std::vector<double> &config) const;

    // FK: compute link transforms for a given config
    std::vector<Eigen::Affine3d> getLinkTransforms(
        const std::vector<double> &config) const;

    bool isSelfCollisionDisabled(const std::string &link_a,
                                 const std::string &link_b) const;

    const std::vector<LinkInfo> &links() const { return links_; }
    const std::vector<JointInfo> &joints() const { return joints_; }

    int linkIndex(const std::string &name) const;

protected:
    static Eigen::Affine3d parseOrigin(const std::string &xyz_str,
                                        const std::string &rpy_str);
    static Eigen::Vector3d parseVec3(const std::string &s);

    void buildKinematicChain();

    std::vector<LinkInfo> links_;
    std::vector<JointInfo> joints_;
    std::vector<const JointInfo *> active_joints_;
    Eigen::Affine3d base_transform_ = Eigen::Affine3d::Identity();
    RobotFamily robot_family_ = RobotFamily::Unknown;

    std::map<std::string, int> link_name_to_index_;
    // Parent traversal: link_index -> (joint_index, parent_link_index)
    std::vector<std::pair<int, int>> link_parent_chain_;

    std::set<std::pair<std::string, std::string>> disabled_collisions_;
};

} // namespace comotion
