#include "comotion/robot/RobotModel.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <filesystem>
#include <array>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <map>

namespace comotion {

namespace pt = boost::property_tree;

static std::string resolveMeshFilename(const std::string &filename,
                                        const std::string &urdf_dir) {
    namespace fs = std::filesystem;
    if (filename.empty())
        return {};
    if (filename.rfind("package://", 0) == 0) {
        std::string rest = filename.substr(std::string("package://").size());
        fs::path combined = fs::path(urdf_dir) / rest;
        return combined.lexically_normal().string();
    }
    if (filename.rfind("file://", 0) == 0) {
        std::string rest = filename.substr(std::string("file://").size());
        fs::path p(rest);
        if (p.is_relative())
            p = fs::path(urdf_dir) / p;
        return p.lexically_normal().string();
    }
    fs::path p(filename);
    if (p.is_relative())
        p = fs::path(urdf_dir) / p;
    return p.lexically_normal().string();
}

Eigen::Vector3d RobotModel::parseVec3(const std::string &s) {
    std::istringstream ss(s);
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    ss >> v.x() >> v.y() >> v.z();
    return v;
}

Eigen::Affine3d RobotModel::parseOrigin(const std::string &xyz_str,
                                         const std::string &rpy_str) {
    Eigen::Affine3d T = Eigen::Affine3d::Identity();
    if (!xyz_str.empty()) {
        Eigen::Vector3d pos = parseVec3(xyz_str);
        T.translation() = pos;
    }
    if (!rpy_str.empty()) {
        Eigen::Vector3d rpy = parseVec3(rpy_str);
        T.linear() = (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()))
                          .toRotationMatrix();
    }
    return T;
}

void RobotModel::loadURDF(const std::string &urdf_path) {
    pt::ptree tree;
    pt::read_xml(urdf_path, tree);

    links_.clear();
    joints_.clear();
    link_name_to_index_.clear();

    std::string urdf_dir =
        std::filesystem::path(urdf_path).parent_path().string();

    auto &robot = tree.get_child("robot");

    // Parse links
    for (auto &node : robot) {
        if (node.first != "link") continue;

        LinkInfo link;
        link.name = node.second.get<std::string>("<xmlattr>.name");
        link.index = static_cast<int>(links_.size());
        link_name_to_index_[link.name] = link.index;

        for (auto &child : node.second) {
            if (child.first != "collision") continue;

            Eigen::Affine3d collision_origin = Eigen::Affine3d::Identity();
            auto origin_opt = child.second.get_child_optional("origin");
            if (origin_opt) {
                std::string xyz =
                    origin_opt->get<std::string>("<xmlattr>.xyz", "0 0 0");
                std::string rpy =
                    origin_opt->get<std::string>("<xmlattr>.rpy", "0 0 0");
                collision_origin = parseOrigin(xyz, rpy);
            }

            auto geom_sphere = child.second.get_child_optional("geometry.sphere");
            if (geom_sphere) {
                CollisionSphere sphere;
                sphere.radius = geom_sphere->get<double>("<xmlattr>.radius");
                sphere.link_index = link.index;
                sphere.center = collision_origin.translation();
                link.collision_spheres.push_back(sphere);
                continue;
            }

            auto geom_box = child.second.get_child_optional("geometry.box");
            if (geom_box) {
                CollisionBox box;
                box.origin = collision_origin;
                box.size =
                    parseVec3(geom_box->get<std::string>("<xmlattr>.size"));
                box.link_index = link.index;
                link.collision_boxes.push_back(std::move(box));
                continue;
            }

            auto geom_cylinder =
                child.second.get_child_optional("geometry.cylinder");
            if (geom_cylinder) {
                CollisionCylinder cylinder;
                cylinder.origin = collision_origin;
                cylinder.radius =
                    geom_cylinder->get<double>("<xmlattr>.radius");
                cylinder.length =
                    geom_cylinder->get<double>("<xmlattr>.length");
                cylinder.link_index = link.index;
                link.collision_cylinders.push_back(std::move(cylinder));
                continue;
            }

            auto geom_mesh = child.second.get_child_optional("geometry.mesh");
            if (geom_mesh) {
                std::string fname =
                    geom_mesh->get<std::string>("<xmlattr>.filename");
                CollisionMesh cm;
                cm.origin = collision_origin;
                cm.resolved_path = resolveMeshFilename(fname, urdf_dir);
                std::string scale_str =
                    geom_mesh->get<std::string>("<xmlattr>.scale", "1 1 1");
                cm.scale = parseVec3(scale_str);
                if (cm.scale.x() == 0.0 && cm.scale.y() == 0.0 &&
                    cm.scale.z() == 0.0)
                    cm.scale = Eigen::Vector3d::Ones();
                link.collision_meshes.push_back(std::move(cm));
            }
        }

        links_.push_back(link);
    }

    // Parse joints
    for (auto &node : robot) {
        if (node.first != "joint") continue;

        JointInfo joint;
        joint.name = node.second.get<std::string>("<xmlattr>.name");
        joint.type = node.second.get<std::string>("<xmlattr>.type");
        joint.parent_link = node.second.get<std::string>("parent.<xmlattr>.link");
        joint.child_link = node.second.get<std::string>("child.<xmlattr>.link");

        auto origin_opt = node.second.get_child_optional("origin");
        if (origin_opt) {
            std::string xyz = origin_opt->get<std::string>("<xmlattr>.xyz", "0 0 0");
            std::string rpy = origin_opt->get<std::string>("<xmlattr>.rpy", "0 0 0");
            joint.origin = parseOrigin(xyz, rpy);
        }

        auto axis_opt = node.second.get_child_optional("axis");
        if (axis_opt) {
            std::string xyz = axis_opt->get<std::string>("<xmlattr>.xyz", "0 0 1");
            joint.axis = parseVec3(xyz);
        } else {
            joint.axis = Eigen::Vector3d::UnitZ();
        }

        auto limit_opt = node.second.get_child_optional("limit");
        if (limit_opt) {
            joint.lower_limit = limit_opt->get<double>("<xmlattr>.lower", 0.0);
            joint.upper_limit = limit_opt->get<double>("<xmlattr>.upper", 0.0);
            joint.velocity_limit = limit_opt->get<double>("<xmlattr>.velocity", 0.0);
        }

        joints_.push_back(joint);
    }

    buildKinematicChain();
}

void RobotModel::buildKinematicChain() {
    active_joints_.clear();
    link_parent_chain_.resize(links_.size(), std::make_pair(-1, -1));

    for (int j = 0; j < static_cast<int>(joints_.size()); ++j) {
        auto &joint = joints_[j];
        auto child_it = link_name_to_index_.find(joint.child_link);
        auto parent_it = link_name_to_index_.find(joint.parent_link);
        if (child_it == link_name_to_index_.end() ||
            parent_it == link_name_to_index_.end())
            continue;

        int child_idx = child_it->second;
        links_[child_idx].parent_joint_index = j;
        link_parent_chain_[child_idx] = std::make_pair(j, parent_it->second);

        if (joint.type == "revolute" || joint.type == "prismatic") {
            active_joints_.push_back(&joints_[j]);
        }
    }
}

void RobotModel::loadSRDF(const std::string &srdf_path) {
    pt::ptree tree;
    pt::read_xml(srdf_path, tree);

    disabled_collisions_.clear();

    auto &robot = tree.get_child("robot");
    for (auto &node : robot) {
        if (node.first != "disable_collisions") continue;
        std::string l1 = node.second.get<std::string>("<xmlattr>.link1");
        std::string l2 = node.second.get<std::string>("<xmlattr>.link2");
        if (l1 > l2) std::swap(l1, l2);
        disabled_collisions_.insert({l1, l2});
    }
}

void RobotModel::setBaseTransform(const Eigen::Affine3d &T) {
    base_transform_ = T;
}

RobotModel::RobotFamily RobotModel::robotFamily() const {
    if (robot_family_ != RobotFamily::Unknown)
        return robot_family_;

    if (active_joints_.size() == 3) {
        bool all_prismatic = true;
        for (const JointInfo *joint : active_joints_) {
            if (!joint || joint->type != "prismatic") {
                all_prismatic = false;
                break;
            }
        }
        if (all_prismatic)
            return RobotFamily::Sphere;
    }

    static const std::array<const char *, 7> panda_joint_names = {{
        "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
        "panda_joint5", "panda_joint6", "panda_joint7",
    }};
    if (active_joints_.size() == panda_joint_names.size()) {
        bool panda = true;
        for (std::size_t i = 0; i < panda_joint_names.size(); ++i) {
            if (!active_joints_[i] ||
                active_joints_[i]->name != panda_joint_names[i]) {
                panda = false;
                break;
            }
        }
        if (panda)
            return RobotFamily::Panda;
    }

    static const std::array<const char *, 6> ur5_joint_names = {{
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
    }};
    if (active_joints_.size() == ur5_joint_names.size()) {
        bool ur5 = true;
        for (std::size_t i = 0; i < ur5_joint_names.size(); ++i) {
            if (!active_joints_[i] ||
                active_joints_[i]->name != ur5_joint_names[i]) {
                ur5 = false;
                break;
            }
        }
        if (ur5)
            return RobotFamily::UR5;
    }

    static const std::array<const char *, 3> planar3_joint_names = {{
        "planar3_joint1", "planar3_joint2", "planar3_joint3",
    }};
    if (active_joints_.size() == planar3_joint_names.size()) {
        bool planar3 = true;
        for (std::size_t i = 0; i < planar3_joint_names.size(); ++i) {
            if (!active_joints_[i] ||
                active_joints_[i]->name != planar3_joint_names[i]) {
                planar3 = false;
                break;
            }
        }
        if (planar3)
            return RobotFamily::Planar3;
    }

    return RobotFamily::Unknown;
}

std::vector<std::string> RobotModel::activeJointNames() const {
    std::vector<std::string> names;
    names.reserve(active_joints_.size());
    for (const JointInfo *joint : active_joints_)
        names.push_back(joint ? joint->name : std::string{});
    return names;
}

int RobotModel::linkIndex(const std::string &name) const {
    auto it = link_name_to_index_.find(name);
    if (it != link_name_to_index_.end()) return it->second;
    return -1;
}

bool RobotModel::isSelfCollisionDisabled(const std::string &link_a,
                                          const std::string &link_b) const {
    std::string l1 = link_a, l2 = link_b;
    if (l1 > l2) std::swap(l1, l2);
    return disabled_collisions_.count({l1, l2}) > 0;
}

std::vector<Eigen::Affine3d> RobotModel::getLinkTransforms(
    const std::vector<double> &config) const {

    std::vector<Eigen::Affine3d> transforms(links_.size(),
                                             Eigen::Affine3d::Identity());

    // Map active joint name -> config value
    std::map<std::string, double> joint_values;
    for (int i = 0; i < static_cast<int>(active_joints_.size()) &&
                    i < static_cast<int>(config.size()); ++i) {
        joint_values[active_joints_[i]->name] = config[i];
    }

    // Compute transforms following parent chain
    // We need a topological ordering. Since URDF is a tree, we can use
    // the fact that parent links always appear before children.
    for (int li = 0; li < static_cast<int>(links_.size()); ++li) {
        auto [joint_idx, parent_link_idx] = link_parent_chain_[li];
        if (joint_idx < 0) {
            transforms[li] = base_transform_;
            continue;
        }

        const auto &joint = joints_[joint_idx];
        Eigen::Affine3d joint_transform = joint.origin;

        auto it = joint_values.find(joint.name);
        double q = (it != joint_values.end()) ? it->second : 0.0;

        if (joint.type == "revolute") {
            joint_transform =
                joint_transform *
                Eigen::Affine3d(Eigen::AngleAxisd(q, joint.axis));
        } else if (joint.type == "prismatic") {
            joint_transform =
                joint_transform *
                Eigen::Translation3d(q * joint.axis);
        }

        transforms[li] = transforms[parent_link_idx] * joint_transform;
    }

    return transforms;
}

std::vector<CollisionSphere> RobotModel::getCollisionSpheres(
    const std::vector<double> &config) const {

    auto transforms = getLinkTransforms(config);

    std::vector<CollisionSphere> world_spheres;
    for (auto &link : links_) {
        if (link.collision_spheres.empty()) continue;
        const auto &T = transforms[link.index];
        for (auto &local_sphere : link.collision_spheres) {
            CollisionSphere ws;
            ws.center = T * local_sphere.center;
            ws.radius = local_sphere.radius;
            ws.link_index = local_sphere.link_index;
            world_spheres.push_back(ws);
        }
    }
    return world_spheres;
}

} // namespace comotion
