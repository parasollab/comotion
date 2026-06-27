#include "comotion/collision/CollisionChecker.h"
#include "comotion/robot/RobotModel.h"

#include <Eigen/Geometry>

#include <array>
#include <cstring>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;

const std::array<Eigen::Vector3d, 8> kCubeVerts = {{
    {-0.5, -0.5, -0.5},
    {0.5, -0.5, -0.5},
    {0.5, 0.5, -0.5},
    {-0.5, 0.5, -0.5},
    {-0.5, -0.5, 0.5},
    {0.5, -0.5, 0.5},
    {0.5, 0.5, 0.5},
    {-0.5, 0.5, 0.5},
}};

const std::array<std::array<int, 3>, 12> kCubeTris = {{
    {{0, 1, 2}},
    {{0, 2, 3}},
    {{4, 6, 5}},
    {{4, 7, 6}},
    {{0, 4, 5}},
    {{0, 5, 1}},
    {{1, 5, 6}},
    {{1, 6, 2}},
    {{2, 6, 7}},
    {{2, 7, 3}},
    {{3, 7, 4}},
    {{3, 4, 0}},
}};

bool expectEqual(const std::string &label, bool actual, bool expected) {
    if (actual != expected) {
        std::cerr << "fcl_urdf_geometry_regression: " << label
                  << " expected " << expected << " got " << actual << "\n";
        return false;
    }
    return true;
}

bool expectTrue(const std::string &label, bool actual) {
    return expectEqual(label, actual, true);
}

void writeText(const fs::path &path, const std::string &text) {
    std::ofstream out(path);
    if (!out)
        throw std::runtime_error("cannot write " + path.string());
    out << text;
}

std::string urdfWithGeometry(const std::string &geometry_xml) {
    return "<?xml version=\"1.0\"?>\n"
           "<robot name=\"fcl_fixture\">\n"
           "  <link name=\"base\">\n"
           "    <collision>\n"
           "      <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n"
           "      <geometry>\n" +
           geometry_xml +
           "      </geometry>\n"
           "    </collision>\n"
           "  </link>\n"
           "</robot>\n";
}

void writeObjCube(const fs::path &path) {
    std::string text;
    for (const auto &v : kCubeVerts) {
        text += "v " + std::to_string(v.x()) + " " +
                std::to_string(v.y()) + " " + std::to_string(v.z()) + "\n";
    }
    text += "f 1 2 3 4\n";
    text += "f 5 8 7 6\n";
    text += "f 1 5 6 2\n";
    text += "f 2 6 7 3\n";
    text += "f 3 7 8 4\n";
    text += "f 4 8 5 1\n";
    writeText(path, text);
}

void writeAsciiStlCube(const fs::path &path) {
    std::string text = "solid cube\n";
    for (const auto &tri : kCubeTris) {
        text += "  facet normal 0 0 0\n";
        text += "    outer loop\n";
        for (int idx : tri) {
            const auto &v = kCubeVerts[static_cast<std::size_t>(idx)];
            text += "      vertex " + std::to_string(v.x()) + " " +
                    std::to_string(v.y()) + " " + std::to_string(v.z()) +
                    "\n";
        }
        text += "    endloop\n";
        text += "  endfacet\n";
    }
    text += "endsolid cube\n";
    writeText(path, text);
}

void writeLeU32(std::ofstream &out, std::uint32_t value) {
    const char bytes[4] = {
        static_cast<char>(value & 0xffu),
        static_cast<char>((value >> 8) & 0xffu),
        static_cast<char>((value >> 16) & 0xffu),
        static_cast<char>((value >> 24) & 0xffu),
    };
    out.write(bytes, sizeof(bytes));
}

void writeLeU16(std::ofstream &out, std::uint16_t value) {
    const char bytes[2] = {
        static_cast<char>(value & 0xffu),
        static_cast<char>((value >> 8) & 0xffu),
    };
    out.write(bytes, sizeof(bytes));
}

void writeLeFloat(std::ofstream &out, float value) {
    static_assert(sizeof(value) == sizeof(std::uint32_t),
                  "unexpected float size");
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    writeLeU32(out, bits);
}

void writeBinaryStlCube(const fs::path &path) {
    std::ofstream out(path, std::ios::binary);
    if (!out)
        throw std::runtime_error("cannot write " + path.string());

    std::array<char, 80> header = {};
    out.write(header.data(), static_cast<std::streamsize>(header.size()));
    writeLeU32(out, static_cast<std::uint32_t>(kCubeTris.size()));
    for (const auto &tri : kCubeTris) {
        writeLeFloat(out, 0.0f);
        writeLeFloat(out, 0.0f);
        writeLeFloat(out, 0.0f);
        for (int idx : tri) {
            const auto &v = kCubeVerts[static_cast<std::size_t>(idx)];
            writeLeFloat(out, static_cast<float>(v.x()));
            writeLeFloat(out, static_cast<float>(v.y()));
            writeLeFloat(out, static_cast<float>(v.z()));
        }
        writeLeU16(out, 0);
    }
}

void writeDaeCube(const fs::path &path) {
    std::string positions;
    for (const auto &v : kCubeVerts) {
        positions += std::to_string(v.x()) + " " + std::to_string(v.y()) +
                     " " + std::to_string(v.z()) + " ";
    }

    std::string indices;
    for (const auto &tri : kCubeTris) {
        indices += std::to_string(tri[0]) + " " + std::to_string(tri[1]) +
                   " " + std::to_string(tri[2]) + " ";
    }

    writeText(path,
              "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n"
              "<COLLADA xmlns=\"http://www.collada.org/2005/11/"
              "COLLADASchema\" version=\"1.4.1\">\n"
              "  <asset><unit name=\"meter\" meter=\"1\"/></asset>\n"
              "  <library_geometries>\n"
              "    <geometry id=\"cube-geometry\">\n"
              "      <mesh>\n"
              "        <source id=\"cube-positions\">\n"
              "          <float_array id=\"cube-positions-array\" count=\"24\">" +
                  positions +
                  "</float_array>\n"
              "          <technique_common>\n"
              "            <accessor source=\"#cube-positions-array\" count=\"8\" "
              "stride=\"3\"/>\n"
              "          </technique_common>\n"
              "        </source>\n"
              "        <vertices id=\"cube-vertices\">\n"
              "          <input semantic=\"POSITION\" source=\"#cube-positions\"/>\n"
              "        </vertices>\n"
              "        <triangles count=\"12\">\n"
              "          <input semantic=\"VERTEX\" source=\"#cube-vertices\" "
              "offset=\"0\"/>\n"
              "          <p>" +
                  indices +
                  "</p>\n"
              "        </triangles>\n"
              "      </mesh>\n"
              "    </geometry>\n"
              "  </library_geometries>\n"
              "</COLLADA>\n");
}

bool expectFclCollisionAt(const fs::path &urdf_path,
                          const Eigen::Vector3d &probe_center,
                          double probe_radius) {
    comotion::RobotModel robot;
    robot.loadURDF(urdf_path.string());

    comotion::CollisionChecker checker(comotion::CollisionChecker::Backend::Fcl);
    checker.setObstacles({comotion::ObstacleSphere{probe_center, probe_radius}});
    bool ok = expectEqual(urdf_path.filename().string() + " colliding probe",
                          checker.isValidSingle(robot, {}), false);

    checker.setObstacles(
        {comotion::ObstacleSphere{Eigen::Vector3d(3.0, 0.0, 0.0), 0.05}});
    ok = expectEqual(urdf_path.filename().string() + " far probe",
                     checker.isValidSingle(robot, {}), true) &&
         ok;
    return ok;
}

bool runPrimitiveTests(const fs::path &dir) {
    const fs::path box_urdf = dir / "box.urdf";
    writeText(box_urdf, urdfWithGeometry(
                            "        <box size=\"1 1 1\"/>\n"));

    comotion::RobotModel box_robot;
    box_robot.loadURDF(box_urdf.string());
    bool ok = expectTrue("box primitive parsed",
                         box_robot.links().size() == 1 &&
                             box_robot.links()[0].collision_boxes.size() == 1);
    ok = expectFclCollisionAt(box_urdf, Eigen::Vector3d(0.49, 0.0, 0.0),
                              0.08) &&
         ok;

    const fs::path cylinder_urdf = dir / "cylinder.urdf";
    writeText(cylinder_urdf,
              urdfWithGeometry(
                  "        <cylinder radius=\"0.25\" length=\"1.0\"/>\n"));

    comotion::RobotModel cylinder_robot;
    cylinder_robot.loadURDF(cylinder_urdf.string());
    ok = expectTrue("cylinder primitive parsed",
                    cylinder_robot.links().size() == 1 &&
                        cylinder_robot.links()[0].collision_cylinders.size() ==
                            1) &&
         ok;
    ok = expectFclCollisionAt(cylinder_urdf, Eigen::Vector3d(0.24, 0.0, 0.0),
                              0.08) &&
         ok;
    return ok;
}

bool runMeshFixtureTests(const fs::path &dir) {
    writeObjCube(dir / "cube.obj");
    writeAsciiStlCube(dir / "cube_ascii.stl");
    writeBinaryStlCube(dir / "cube_binary.stl");
    writeBinaryStlCube(dir / "cube_upper.STL");
    writeDaeCube(dir / "cube.dae");

    const std::vector<std::string> mesh_files = {
        "cube.obj", "cube_ascii.stl", "cube_binary.stl", "cube_upper.STL",
        "cube.dae"};

    bool ok = true;
    for (const std::string &mesh_file : mesh_files) {
        const fs::path urdf_path = dir / (mesh_file + ".urdf");
        writeText(urdf_path, urdfWithGeometry("        <mesh filename=\"" +
                                             mesh_file +
                                             "\" scale=\"1 1 1\"/>\n"));
        ok = expectFclCollisionAt(urdf_path, Eigen::Vector3d(0.49, 0.0, 0.0),
                                  0.08) &&
             ok;
    }
    return ok;
}

bool runExternalUr5Test() {
    const fs::path urdf =
        "external/como-ompl/external/vamp/resources/ur5/ur5.urdf";
    if (!fs::exists(urdf)) {
        std::cerr << "fcl_urdf_geometry_regression: missing " << urdf
                  << "\n";
        return false;
    }

    comotion::RobotModel robot;
    robot.loadURDF(urdf.string());
    bool ok = expectTrue("UR5 external box parsed",
                         !robot.links().empty() &&
                             [&robot]() {
                                 for (const auto &link : robot.links()) {
                                     if (!link.collision_boxes.empty())
                                         return true;
                                 }
                                 return false;
                             }());

    comotion::CollisionChecker checker(comotion::CollisionChecker::Backend::Fcl);
    ok = expectEqual("UR5 external STL/box geometry builds",
                     checker.isValidSingle(
                         robot, std::vector<double>(
                                    static_cast<std::size_t>(robot.numJoints()),
                                    0.0)),
                     true) &&
         ok;
    return ok;
}

bool runExternalDaeTest(const fs::path &dir) {
    const fs::path dae =
        "external/como-ompl/external/vamp/resources/fetch/meshes/"
        "torso_fixed_link.dae";
    if (!fs::exists(dae)) {
        std::cerr << "fcl_urdf_geometry_regression: missing " << dae << "\n";
        return false;
    }

    const fs::path urdf_path = dir / "external_fetch_dae.urdf";
    writeText(urdf_path,
              urdfWithGeometry("        <mesh filename=\"file://" +
                               fs::absolute(dae).string() + "\"/>\n"));

    comotion::RobotModel robot;
    robot.loadURDF(urdf_path.string());
    comotion::CollisionChecker checker(comotion::CollisionChecker::Backend::Fcl);
    return expectEqual("Fetch external DAE geometry builds",
                       checker.isValidSingle(robot, {}), true);
}

} // namespace

int main() {
    try {
        const fs::path dir = fs::temp_directory_path() /
                             "comotion_fcl_urdf_geometry_regression";
        fs::create_directories(dir);

        bool ok = true;
        ok = runPrimitiveTests(dir) && ok;
        ok = runMeshFixtureTests(dir) && ok;
        ok = runExternalDaeTest(dir) && ok;
        ok = runExternalUr5Test() && ok;
        return ok ? 0 : 1;
    } catch (const std::exception &e) {
        std::cerr << "fcl_urdf_geometry_regression: " << e.what() << "\n";
        return 1;
    }
}
