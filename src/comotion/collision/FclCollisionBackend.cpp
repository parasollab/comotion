#include "comotion/collision/detail/CollisionBackend.h"
#include "comotion/collision/detail/ValidationUtils.h"
#include "comotion/planning/detail/PosixProcess.h"

#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/cylinder.h>
#include <fcl/geometry/shape/sphere.h>
#include <fcl/math/bv/OBBRSS.h>
#include <fcl/math/triangle.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_object.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <chrono>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

#if !defined(_WIN32)
#include <signal.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace comotion {
namespace detail {

namespace {

using Clock = std::chrono::steady_clock;

#if !defined(_WIN32)
bool readExact(int fd, void *buffer, std::size_t bytes) {
    auto *out = static_cast<char *>(buffer);
    std::size_t read_total = 0;
    while (read_total < bytes) {
        const ssize_t got = ::read(fd, out + read_total, bytes - read_total);
        if (got == 0)
            return false;
        if (got < 0) {
            if (errno == EINTR)
                continue;
            return false;
        }
        read_total += static_cast<std::size_t>(got);
    }
    return true;
}

bool writeExact(int fd, const void *buffer, std::size_t bytes) {
    const auto *in = static_cast<const char *>(buffer);
    std::size_t written_total = 0;
    while (written_total < bytes) {
        const ssize_t written =
            ::write(fd, in + written_total, bytes - written_total);
        if (written < 0) {
            if (errno == EINTR)
                continue;
            return false;
        }
        if (written == 0)
            return false;
        written_total += static_cast<std::size_t>(written);
    }
    return true;
}

template <typename T>
bool readValue(int fd, T &value) {
    return readExact(fd, &value, sizeof(T));
}

template <typename T>
bool writeValue(int fd, const T &value) {
    return writeExact(fd, &value, sizeof(T));
}
#endif

double processCpuSeconds() {
#if defined(CLOCK_PROCESS_CPUTIME_ID)
    timespec ts {};
    if (::clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts) == 0) {
        return static_cast<double>(ts.tv_sec) +
               static_cast<double>(ts.tv_nsec) * 1e-9;
    }
#endif
    return static_cast<double>(std::clock()) /
           static_cast<double>(CLOCKS_PER_SEC);
}

double elapsedProcessCpuSeconds(double start) {
    const double elapsed = processCpuSeconds() - start;
    return elapsed < 0.0 ? 0.0 : elapsed;
}

double elapsedWallSeconds(const Clock::time_point &start) {
    return std::chrono::duration<double>(Clock::now() - start).count();
}

using S = double;
using Vector3 = fcl::Vector3<S>;
using Transform3 = fcl::Transform3<S>;
using CollisionGeometryPtr = std::shared_ptr<fcl::CollisionGeometry<S>>;
using BVHModel = fcl::BVHModel<fcl::OBBRSS<S>>;
namespace bpt = boost::property_tree;

static Transform3 eigenAffineToFcl(const Eigen::Affine3d &T) {
    Transform3 out = Transform3::Identity();
    out.linear() = T.rotation();
    out.translation() =
        Vector3(T.translation().x(), T.translation().y(), T.translation().z());
    return out;
}

static std::string meshExtensionLower(const std::string &path) {
    std::string cleaned = path;
    const std::size_t suffix = cleaned.find_first_of("?#");
    if (suffix != std::string::npos)
        cleaned.resize(suffix);

    std::string ext = std::filesystem::path(cleaned).extension().string();
    for (char &c : ext)
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return ext;
}

static std::string stripUriFragment(const std::string &source) {
    if (!source.empty() && source[0] == '#')
        return source.substr(1);
    return source;
}

static std::vector<double> parseDoubleList(const std::string &text) {
    std::vector<double> values;
    std::istringstream ss(text);
    double value = 0.0;
    while (ss >> value)
        values.push_back(value);
    return values;
}

static std::vector<int> parseIntList(const std::string &text) {
    std::vector<int> values;
    std::istringstream ss(text);
    int value = 0;
    while (ss >> value)
        values.push_back(value);
    return values;
}

static bool loadObjMesh(const std::string &path,
                        std::vector<Vector3> &out_vertices,
                        std::vector<fcl::Triangle> &out_tris) {
    std::ifstream in(path);
    if (!in)
        throw std::runtime_error("FCL backend: cannot open mesh: " + path);

    std::vector<Vector3> raw_v;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#')
            continue;
        std::istringstream ls(line);
        std::string tag;
        ls >> tag;
        if (tag == "v") {
            double x, y, z;
            if (!(ls >> x >> y >> z))
                continue;
            raw_v.emplace_back(x, y, z);
        }
    }

    in.clear();
    in.seekg(0);
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#')
            continue;
        std::istringstream ls(line);
        std::string tag;
        ls >> tag;
        if (tag != "f")
            continue;

        std::vector<int> corner;
        std::string tok;
        while (ls >> tok) {
            std::string first = tok;
            auto slash = first.find('/');
            if (slash != std::string::npos)
                first = first.substr(0, slash);
            int idx = std::stoi(first);
            if (idx < 0)
                idx = static_cast<int>(raw_v.size()) + idx + 1;
            corner.push_back(idx - 1);
        }
        if (corner.size() < 3)
            continue;
        for (size_t k = 1; k + 1 < corner.size(); ++k) {
            int i0 = corner[0];
            int i1 = corner[k];
            int i2 = corner[k + 1];
            if (i0 < 0 || i1 < 0 || i2 < 0 ||
                i0 >= static_cast<int>(raw_v.size()) ||
                i1 >= static_cast<int>(raw_v.size()) ||
                i2 >= static_cast<int>(raw_v.size()))
                continue;
            out_tris.emplace_back(static_cast<std::size_t>(i0),
                                  static_cast<std::size_t>(i1),
                                  static_cast<std::size_t>(i2));
        }
    }
    out_vertices = std::move(raw_v);
    return !out_tris.empty();
}

static std::uint32_t readLeU32(const char *bytes) {
    return (static_cast<std::uint32_t>(
                static_cast<unsigned char>(bytes[0]))
            << 0) |
           (static_cast<std::uint32_t>(
                static_cast<unsigned char>(bytes[1]))
            << 8) |
           (static_cast<std::uint32_t>(
                static_cast<unsigned char>(bytes[2]))
            << 16) |
           (static_cast<std::uint32_t>(
                static_cast<unsigned char>(bytes[3]))
            << 24);
}

static float readLeFloat(const char *bytes) {
    const std::uint32_t bits = readLeU32(bytes);
    float value = 0.0f;
    static_assert(sizeof(value) == sizeof(bits), "unexpected float size");
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

static bool detectBinaryStl(const std::string &path, std::uint32_t &tri_count) {
    std::ifstream in(path, std::ios::binary | std::ios::ate);
    if (!in)
        throw std::runtime_error("FCL backend: cannot open mesh: " + path);

    const std::streamoff file_size = in.tellg();
    if (file_size < 84)
        return false;

    in.seekg(80, std::ios::beg);
    char count_buf[4] = {};
    if (!in.read(count_buf, sizeof(count_buf)))
        return false;

    tri_count = readLeU32(count_buf);
    const std::uint64_t expected_size =
        84ULL + 50ULL * static_cast<std::uint64_t>(tri_count);
    return expected_size == static_cast<std::uint64_t>(file_size);
}

static bool loadBinaryStlMesh(const std::string &path,
                              std::uint32_t tri_count,
                              std::vector<Vector3> &out_vertices,
                              std::vector<fcl::Triangle> &out_tris) {
    std::ifstream in(path, std::ios::binary);
    if (!in)
        throw std::runtime_error("FCL backend: cannot open mesh: " + path);

    in.seekg(84, std::ios::beg);
    for (std::uint32_t i = 0; i < tri_count; ++i) {
        char record[50] = {};
        if (!in.read(record, sizeof(record)))
            throw std::runtime_error("FCL backend: truncated binary STL: " + path);

        const std::size_t base = out_vertices.size();
        for (int v = 0; v < 3; ++v) {
            const char *p = record + 12 + v * 12;
            out_vertices.emplace_back(readLeFloat(p + 0),
                                      readLeFloat(p + 4),
                                      readLeFloat(p + 8));
        }
        out_tris.emplace_back(base, base + 1, base + 2);
    }
    return !out_tris.empty();
}

static bool loadAsciiStlMesh(const std::string &path,
                             std::vector<Vector3> &out_vertices,
                             std::vector<fcl::Triangle> &out_tris) {
    std::ifstream in(path);
    if (!in)
        throw std::runtime_error("FCL backend: cannot open mesh: " + path);

    std::array<std::size_t, 3> pending = {};
    int pending_count = 0;
    std::string line;
    while (std::getline(in, line)) {
        std::istringstream ls(line);
        std::string tag;
        ls >> tag;
        if (tag != "vertex")
            continue;

        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        if (!(ls >> x >> y >> z))
            continue;

        pending[static_cast<std::size_t>(pending_count++)] =
            out_vertices.size();
        out_vertices.emplace_back(x, y, z);
        if (pending_count == 3) {
            out_tris.emplace_back(pending[0], pending[1], pending[2]);
            pending_count = 0;
        }
    }
    return !out_tris.empty();
}

static bool loadStlMesh(const std::string &path,
                        std::vector<Vector3> &out_vertices,
                        std::vector<fcl::Triangle> &out_tris) {
    std::uint32_t tri_count = 0;
    if (detectBinaryStl(path, tri_count))
        return loadBinaryStlMesh(path, tri_count, out_vertices, out_tris);
    return loadAsciiStlMesh(path, out_vertices, out_tris);
}

struct DaeSource {
    std::vector<double> values;
    int stride = 3;
};

struct DaeInput {
    std::string semantic;
    std::string source;
    int offset = 0;
};

static std::vector<DaeInput> daeInputs(const bpt::ptree &primitive) {
    std::vector<DaeInput> inputs;
    for (const auto &child : primitive) {
        if (child.first != "input")
            continue;
        DaeInput input;
        input.semantic =
            child.second.get<std::string>("<xmlattr>.semantic", "");
        input.source = stripUriFragment(
            child.second.get<std::string>("<xmlattr>.source", ""));
        input.offset = child.second.get<int>("<xmlattr>.offset", 0);
        inputs.push_back(std::move(input));
    }
    return inputs;
}

static bool resolveDaePositionInput(
    const std::vector<DaeInput> &inputs,
    const std::map<std::string, DaeSource> &sources,
    const std::map<std::string, std::string> &vertices_to_position,
    int &position_offset,
    const DaeSource *&position_source,
    int &input_stride) {

    input_stride = 0;
    position_offset = -1;
    position_source = nullptr;

    for (const auto &input : inputs) {
        input_stride = std::max(input_stride, input.offset + 1);
        if (input.semantic != "VERTEX" && input.semantic != "POSITION")
            continue;

        std::string source_id = input.source;
        if (input.semantic == "VERTEX") {
            auto vit = vertices_to_position.find(source_id);
            if (vit == vertices_to_position.end())
                continue;
            source_id = vit->second;
        }

        auto sit = sources.find(source_id);
        if (sit == sources.end())
            continue;

        position_offset = input.offset;
        position_source = &sit->second;
    }

    return position_offset >= 0 && position_source != nullptr &&
           input_stride > 0;
}

static bool daeIndexValid(const DaeSource &source, int index) {
    if (index < 0 || source.stride < 3)
        return false;
    const std::size_t base =
        static_cast<std::size_t>(index) * static_cast<std::size_t>(source.stride);
    return base + 2 < source.values.size();
}

static Vector3 daePosition(const DaeSource &source, int index, double unit) {
    const std::size_t base =
        static_cast<std::size_t>(index) * static_cast<std::size_t>(source.stride);
    return Vector3(unit * source.values[base + 0],
                   unit * source.values[base + 1],
                   unit * source.values[base + 2]);
}

static void addDaeTriangle(const DaeSource &source, double unit, int i0, int i1,
                           int i2, std::vector<Vector3> &out_vertices,
                           std::vector<fcl::Triangle> &out_tris) {
    if (!daeIndexValid(source, i0) || !daeIndexValid(source, i1) ||
        !daeIndexValid(source, i2))
        return;

    const std::size_t base = out_vertices.size();
    out_vertices.push_back(daePosition(source, i0, unit));
    out_vertices.push_back(daePosition(source, i1, unit));
    out_vertices.push_back(daePosition(source, i2, unit));
    out_tris.emplace_back(base, base + 1, base + 2);
}

static void addDaePolygonFan(const DaeSource &source, double unit,
                             const std::vector<int> &polygon,
                             std::vector<Vector3> &out_vertices,
                             std::vector<fcl::Triangle> &out_tris) {
    if (polygon.size() < 3)
        return;
    for (std::size_t k = 1; k + 1 < polygon.size(); ++k)
        addDaeTriangle(source, unit, polygon[0], polygon[k], polygon[k + 1],
                       out_vertices, out_tris);
}

static void loadDaeTrianglePrimitive(
    const bpt::ptree &primitive,
    const std::map<std::string, DaeSource> &sources,
    const std::map<std::string, std::string> &vertices_to_position,
    double unit, std::vector<Vector3> &out_vertices,
    std::vector<fcl::Triangle> &out_tris) {

    const auto inputs = daeInputs(primitive);
    int position_offset = -1;
    int input_stride = 0;
    const DaeSource *position_source = nullptr;
    if (!resolveDaePositionInput(inputs, sources, vertices_to_position,
                                 position_offset, position_source,
                                 input_stride))
        return;

    auto p_opt = primitive.get_child_optional("p");
    if (!p_opt)
        return;

    const std::vector<int> p = parseIntList(p_opt->data());
    const std::size_t vertices =
        p.size() / static_cast<std::size_t>(input_stride);
    for (std::size_t v = 0; v + 2 < vertices; v += 3) {
        const int i0 =
            p[v * static_cast<std::size_t>(input_stride) + position_offset];
        const int i1 =
            p[(v + 1) * static_cast<std::size_t>(input_stride) +
              position_offset];
        const int i2 =
            p[(v + 2) * static_cast<std::size_t>(input_stride) +
              position_offset];
        addDaeTriangle(*position_source, unit, i0, i1, i2, out_vertices,
                       out_tris);
    }
}

static void loadDaePolylistPrimitive(
    const bpt::ptree &primitive,
    const std::map<std::string, DaeSource> &sources,
    const std::map<std::string, std::string> &vertices_to_position,
    double unit, std::vector<Vector3> &out_vertices,
    std::vector<fcl::Triangle> &out_tris) {

    const auto inputs = daeInputs(primitive);
    int position_offset = -1;
    int input_stride = 0;
    const DaeSource *position_source = nullptr;
    if (!resolveDaePositionInput(inputs, sources, vertices_to_position,
                                 position_offset, position_source,
                                 input_stride))
        return;

    auto vcount_opt = primitive.get_child_optional("vcount");
    auto p_opt = primitive.get_child_optional("p");
    if (!vcount_opt || !p_opt)
        return;

    const std::vector<int> vcounts = parseIntList(vcount_opt->data());
    const std::vector<int> p = parseIntList(p_opt->data());
    std::size_t cursor = 0;
    for (int vcount : vcounts) {
        if (vcount < 0)
            return;

        std::vector<int> polygon;
        polygon.reserve(static_cast<std::size_t>(vcount));
        for (int i = 0; i < vcount; ++i) {
            const std::size_t pos =
                cursor + static_cast<std::size_t>(position_offset);
            if (pos >= p.size())
                return;
            polygon.push_back(p[pos]);
            cursor += static_cast<std::size_t>(input_stride);
        }
        addDaePolygonFan(*position_source, unit, polygon, out_vertices,
                         out_tris);
    }
}

static void loadDaePolygonsPrimitive(
    const bpt::ptree &primitive,
    const std::map<std::string, DaeSource> &sources,
    const std::map<std::string, std::string> &vertices_to_position,
    double unit, std::vector<Vector3> &out_vertices,
    std::vector<fcl::Triangle> &out_tris) {

    const auto inputs = daeInputs(primitive);
    int position_offset = -1;
    int input_stride = 0;
    const DaeSource *position_source = nullptr;
    if (!resolveDaePositionInput(inputs, sources, vertices_to_position,
                                 position_offset, position_source,
                                 input_stride))
        return;

    for (const auto &child : primitive) {
        if (child.first != "p")
            continue;

        const std::vector<int> p = parseIntList(child.second.data());
        const std::size_t vertices =
            p.size() / static_cast<std::size_t>(input_stride);
        std::vector<int> polygon;
        polygon.reserve(vertices);
        for (std::size_t v = 0; v < vertices; ++v) {
            polygon.push_back(
                p[v * static_cast<std::size_t>(input_stride) +
                  position_offset]);
        }
        addDaePolygonFan(*position_source, unit, polygon, out_vertices,
                         out_tris);
    }
}

static void loadDaeMeshNode(const bpt::ptree &mesh, double unit,
                            std::vector<Vector3> &out_vertices,
                            std::vector<fcl::Triangle> &out_tris) {
    std::map<std::string, DaeSource> sources;
    std::map<std::string, std::string> vertices_to_position;

    for (const auto &child : mesh) {
        if (child.first != "source")
            continue;

        const std::string id =
            child.second.get<std::string>("<xmlattr>.id", "");
        auto floats_opt = child.second.get_child_optional("float_array");
        if (id.empty() || !floats_opt)
            continue;

        DaeSource source;
        source.values = parseDoubleList(floats_opt->data());
        if (auto accessor_opt =
                child.second.get_child_optional("technique_common.accessor")) {
            source.stride =
                accessor_opt->get<int>("<xmlattr>.stride", source.stride);
        }
        if (!source.values.empty())
            sources[id] = std::move(source);
    }

    for (const auto &child : mesh) {
        if (child.first != "vertices")
            continue;

        const std::string id =
            child.second.get<std::string>("<xmlattr>.id", "");
        if (id.empty())
            continue;

        for (const auto &input_node : child.second) {
            if (input_node.first != "input")
                continue;
            if (input_node.second.get<std::string>("<xmlattr>.semantic", "") !=
                "POSITION")
                continue;

            vertices_to_position[id] = stripUriFragment(
                input_node.second.get<std::string>("<xmlattr>.source", ""));
        }
    }

    for (const auto &child : mesh) {
        if (child.first == "triangles") {
            loadDaeTrianglePrimitive(child.second, sources,
                                     vertices_to_position, unit, out_vertices,
                                     out_tris);
        } else if (child.first == "polylist") {
            loadDaePolylistPrimitive(child.second, sources,
                                     vertices_to_position, unit, out_vertices,
                                     out_tris);
        } else if (child.first == "polygons") {
            loadDaePolygonsPrimitive(child.second, sources,
                                     vertices_to_position, unit, out_vertices,
                                     out_tris);
        }
    }
}

static bool loadDaeMesh(const std::string &path,
                        std::vector<Vector3> &out_vertices,
                        std::vector<fcl::Triangle> &out_tris) {
    bpt::ptree tree;
    try {
        bpt::read_xml(path, tree, bpt::xml_parser::trim_whitespace);
    } catch (const bpt::xml_parser_error &e) {
        throw std::runtime_error("FCL backend: cannot parse Collada mesh " +
                                 path + ": " + e.what());
    }

    const bpt::ptree *collada = &tree;
    if (auto collada_opt = tree.get_child_optional("COLLADA"))
        collada = &collada_opt.get();

    const double unit =
        collada->get<double>("asset.unit.<xmlattr>.meter", 1.0);
    auto geoms_opt = collada->get_child_optional("library_geometries");
    if (!geoms_opt)
        return false;

    for (const auto &geometry : geoms_opt.get()) {
        if (geometry.first != "geometry")
            continue;
        auto mesh_opt = geometry.second.get_child_optional("mesh");
        if (!mesh_opt)
            continue;
        loadDaeMeshNode(mesh_opt.get(), unit, out_vertices, out_tris);
    }
    return !out_tris.empty();
}

static bool loadMesh(const std::string &path,
                     std::vector<Vector3> &out_vertices,
                     std::vector<fcl::Triangle> &out_tris) {
    out_vertices.clear();
    out_tris.clear();

    const std::string ext = meshExtensionLower(path);
    if (ext == ".obj")
        return loadObjMesh(path, out_vertices, out_tris);
    if (ext == ".stl")
        return loadStlMesh(path, out_vertices, out_tris);
    if (ext == ".dae")
        return loadDaeMesh(path, out_vertices, out_tris);

    throw std::runtime_error("FCL backend: unsupported mesh extension for " +
                             path +
                             " (supported: .obj, .stl, .dae)");
}

static CollisionGeometryPtr makeMeshGeometry(const CollisionMesh &cm) {
    std::vector<Vector3> verts;
    std::vector<fcl::Triangle> tris;
    if (!loadMesh(cm.resolved_path, verts, tris))
        throw std::runtime_error("FCL backend: no triangles in mesh: " +
                                 cm.resolved_path);

    Eigen::Matrix3d R = cm.origin.linear();
    Eigen::Vector3d t = cm.origin.translation();
    for (auto &v : verts) {
        Eigen::Vector3d p(v[0], v[1], v[2]);
        p = p.cwiseProduct(Eigen::Vector3d(cm.scale.x(), cm.scale.y(),
                                           cm.scale.z()));
        Eigen::Vector3d q = R * p + t;
        v = Vector3(q.x(), q.y(), q.z());
    }

    auto model = std::make_shared<BVHModel>();
    model->beginModel(static_cast<int>(tris.size()),
                      static_cast<int>(verts.size()));
    for (const auto &v : verts)
        model->addVertex(v);
    for (const auto &tri : tris)
        model->addTriangle(verts[tri[0]], verts[tri[1]], verts[tri[2]]);
    if (model->endModel() != fcl::BVH_OK)
        throw std::runtime_error("FCL backend: BVH build failed for " +
                                 cm.resolved_path);
    return model;
}

struct GeomPrim {
    CollisionGeometryPtr geom;
    Transform3 tf_link;
    int link_index = -1;
};

struct RobotPrimCache {
    std::vector<GeomPrim> prims;
};

static std::vector<GeomPrim> buildPrimsForRobot(const RobotModel &robot) {
    std::vector<GeomPrim> prims;
    for (const auto &link : robot.links()) {
        for (const auto &sph : link.collision_spheres) {
            GeomPrim g;
            g.geom = std::make_shared<fcl::Sphere<S>>(sph.radius);
            g.tf_link = Transform3(fcl::Translation3<S>(
                Vector3(sph.center.x(), sph.center.y(), sph.center.z())));
            g.link_index = link.index;
            prims.push_back(std::move(g));
        }
        for (const auto &box : link.collision_boxes) {
            GeomPrim g;
            g.geom = std::make_shared<fcl::Box<S>>(
                box.size.x(), box.size.y(), box.size.z());
            g.tf_link = eigenAffineToFcl(box.origin);
            g.link_index = link.index;
            prims.push_back(std::move(g));
        }
        for (const auto &cylinder : link.collision_cylinders) {
            GeomPrim g;
            g.geom = std::make_shared<fcl::Cylinder<S>>(cylinder.radius,
                                                        cylinder.length);
            g.tf_link = eigenAffineToFcl(cylinder.origin);
            g.link_index = link.index;
            prims.push_back(std::move(g));
        }
        for (const auto &mesh : link.collision_meshes) {
            GeomPrim g;
            g.geom = makeMeshGeometry(mesh);
            g.tf_link = Transform3::Identity();
            g.link_index = link.index;
            prims.push_back(std::move(g));
        }
    }
    return prims;
}

static bool fclPairCollide(const CollisionGeometryPtr &ga, const Transform3 &Ta,
                           const CollisionGeometryPtr &gb, const Transform3 &Tb) {
    fcl::CollisionObject<S> oa(ga, Ta);
    fcl::CollisionObject<S> ob(gb, Tb);
    fcl::CollisionRequest<S> req(1, false);
    fcl::CollisionResult<S> res;
    return fcl::collide(&oa, &ob, req, res) > 0;
}

static Transform3 cylinderWorldPose(const ObstacleCylinder &cyl) {
    Eigen::Vector3d ez(0, 0, 1);
    Eigen::Matrix3d Rmat = Eigen::Matrix3d::Identity();
    Eigen::Vector3d ax = cyl.axis.normalized();
    if (ax.dot(ez) < -1.0 + 1e-9) {
        Rmat = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
    } else if (ax.dot(ez) < 1.0 - 1e-9) {
        Eigen::Quaterniond q =
            Eigen::Quaterniond::FromTwoVectors(ez, ax);
        Rmat = q.toRotationMatrix();
    }
    Transform3 T = Transform3::Identity();
    T.linear() = Rmat;
    T.translation() = Vector3(cyl.center.x(), cyl.center.y(), cyl.center.z());
    return T;
}

class FclBackendImpl final : public CollisionBackend {
public:
    std::unique_ptr<CollisionBackend> clone() const override {
        auto n = std::make_unique<FclBackendImpl>();
        n->env_spheres_ = env_spheres_;
        n->env_cylinders_ = env_cylinders_;
        n->rebuildEnvironmentObjects();
        return n;
    }

    void onEnvironmentChanged(
        const std::vector<ObstacleSphere> &spheres,
        const std::vector<ObstacleCylinder> &cylinders) override {
        env_spheres_ = spheres;
        env_cylinders_ = cylinders;
        rebuildEnvironmentObjects();
    }

    ValidationWorkStats lastValidationWorkStats() const override {
        return last_work_stats_;
    }

    void rebuildEnvironmentObjects() {
        env_geoms_.clear();
        env_transforms_.clear();
        for (const auto &o : env_spheres_) {
            auto g = std::make_shared<fcl::Sphere<S>>(o.radius);
            Transform3 T = Transform3::Identity();
            T.translation() =
                Vector3(o.center.x(), o.center.y(), o.center.z());
            env_geoms_.push_back(std::move(g));
            env_transforms_.push_back(T);
        }
        for (const auto &c : env_cylinders_) {
            double lz = 2.0 * c.half_height;
            auto g = std::make_shared<fcl::Cylinder<S>>(c.radius, lz);
            env_geoms_.push_back(std::move(g));
            env_transforms_.push_back(cylinderWorldPose(c));
        }
    }

    const RobotPrimCache &robotCache(const RobotModel &robot) const {
        const RobotModel *key = &robot;
        auto it = robot_cache_.find(key);
        if (it != robot_cache_.end())
            return it->second;
        RobotPrimCache entry;
        entry.prims = buildPrimsForRobot(robot);
        auto ins = robot_cache_.emplace(key, std::move(entry));
        return ins.first->second;
    }

    bool isValidSingle(const RobotModel &robot,
                       const std::vector<double> &config,
                       const std::vector<ObstacleSphere> &,
                       const std::vector<ObstacleCylinder> &) const override {
        const auto &cache = robotCache(robot);
        if (cache.prims.empty())
            return true;

        auto link_tf = robot.getLinkTransforms(config);
        for (const auto &prim : cache.prims) {
            Transform3 world =
                eigenAffineToFcl(link_tf[static_cast<size_t>(prim.link_index)]) *
                prim.tf_link;
            for (size_t e = 0; e < env_geoms_.size(); ++e) {
                if (fclPairCollide(prim.geom, world, env_geoms_[e],
                                   env_transforms_[e]))
                    return false;
            }
        }
        return true;
    }

    bool isSelfCollisionFree(
        const RobotModel &robot,
        const std::vector<double> &config) const override {
        const auto &cache = robotCache(robot);
        const auto &prims = cache.prims;
        if (prims.size() < 2)
            return true;

        auto link_tf = robot.getLinkTransforms(config);
        for (size_t i = 0; i < prims.size(); ++i) {
            Transform3 Twi =
                eigenAffineToFcl(
                    link_tf[static_cast<size_t>(prims[i].link_index)]) *
                prims[i].tf_link;
            for (size_t j = i + 1; j < prims.size(); ++j) {
                if (prims[i].link_index == prims[j].link_index)
                    continue;
                const auto &li = robot.links()[static_cast<size_t>(prims[i].link_index)];
                const auto &lj = robot.links()[static_cast<size_t>(prims[j].link_index)];
                if (robot.isSelfCollisionDisabled(li.name, lj.name))
                    continue;
                Transform3 Twj =
                    eigenAffineToFcl(
                        link_tf[static_cast<size_t>(prims[j].link_index)]) *
                    prims[j].tf_link;
                if (fclPairCollide(prims[i].geom, Twi, prims[j].geom, Twj))
                    return false;
            }
        }
        return true;
    }

    bool isValidPair(const RobotModel &robot_a,
                     const std::vector<double> &config_a,
                     const RobotModel &robot_b,
                     const std::vector<double> &config_b) const override {
        const auto &ca = robotCache(robot_a);
        const auto &cb = robotCache(robot_b);
        if (ca.prims.empty() || cb.prims.empty())
            return true;

        auto tfa = robot_a.getLinkTransforms(config_a);
        auto tfb = robot_b.getLinkTransforms(config_b);
        for (const auto &pa : ca.prims) {
            Transform3 Twa =
                eigenAffineToFcl(tfa[static_cast<size_t>(pa.link_index)]) *
                pa.tf_link;
            for (const auto &pb : cb.prims) {
                Transform3 Twb =
                    eigenAffineToFcl(tfb[static_cast<size_t>(pb.link_index)]) *
                    pb.tf_link;
                if (fclPairCollide(pa.geom, Twa, pb.geom, Twb))
                    return false;
            }
        }
        return true;
    }

    bool isValidSingleExhaustive(
        const RobotModel &robot, const std::vector<double> &config) const {
        const auto &cache = robotCache(robot);
        if (cache.prims.empty())
            return true;

        bool valid = true;
        const auto link_tf = robot.getLinkTransforms(config);
        for (const auto &prim : cache.prims) {
            const Transform3 world =
                eigenAffineToFcl(
                    link_tf[static_cast<std::size_t>(prim.link_index)]) *
                prim.tf_link;
            for (std::size_t e = 0; e < env_geoms_.size(); ++e) {
                if (fclPairCollide(prim.geom, world, env_geoms_[e],
                                   env_transforms_[e])) {
                    valid = false;
                }
            }
        }
        return valid;
    }

    bool isSelfCollisionFreeExhaustive(
        const RobotModel &robot, const std::vector<double> &config) const {
        const auto &prims = robotCache(robot).prims;
        if (prims.size() < 2)
            return true;

        bool valid = true;
        const auto link_tf = robot.getLinkTransforms(config);
        for (std::size_t i = 0; i < prims.size(); ++i) {
            const Transform3 world_i =
                eigenAffineToFcl(
                    link_tf[static_cast<std::size_t>(prims[i].link_index)]) *
                prims[i].tf_link;
            for (std::size_t j = i + 1; j < prims.size(); ++j) {
                if (prims[i].link_index == prims[j].link_index)
                    continue;
                const auto &link_i =
                    robot.links()[static_cast<std::size_t>(prims[i].link_index)];
                const auto &link_j =
                    robot.links()[static_cast<std::size_t>(prims[j].link_index)];
                if (robot.isSelfCollisionDisabled(link_i.name, link_j.name))
                    continue;
                const Transform3 world_j =
                    eigenAffineToFcl(
                        link_tf[static_cast<std::size_t>(prims[j].link_index)]) *
                    prims[j].tf_link;
                if (fclPairCollide(prims[i].geom, world_i, prims[j].geom,
                                   world_j)) {
                    valid = false;
                }
            }
        }
        return valid;
    }

    bool isValidPairExhaustive(
        const RobotModel &robot_a, const std::vector<double> &config_a,
        const RobotModel &robot_b,
        const std::vector<double> &config_b) const {
        const auto &prims_a = robotCache(robot_a).prims;
        const auto &prims_b = robotCache(robot_b).prims;
        if (prims_a.empty() || prims_b.empty())
            return true;

        bool valid = true;
        const auto transforms_a = robot_a.getLinkTransforms(config_a);
        const auto transforms_b = robot_b.getLinkTransforms(config_b);
        for (const auto &prim_a : prims_a) {
            const Transform3 world_a =
                eigenAffineToFcl(
                    transforms_a[static_cast<std::size_t>(prim_a.link_index)]) *
                prim_a.tf_link;
            for (const auto &prim_b : prims_b) {
                const Transform3 world_b =
                    eigenAffineToFcl(transforms_b[static_cast<std::size_t>(
                                         prim_b.link_index)]) *
                    prim_b.tf_link;
                if (fclPairCollide(prim_a.geom, world_a, prim_b.geom,
                                   world_b)) {
                    valid = false;
                }
            }
        }
        return valid;
    }

    bool isMotionValid(const RobotModel &robot,
                       const std::vector<double> &from,
                       const std::vector<double> &to, int num_checks,
                       const std::vector<ObstacleSphere> &obstacles,
                       const std::vector<ObstacleCylinder> &cylinders) const override {
        const int steps = std::max(1, num_checks);
        int dim = static_cast<int>(from.size());
        std::vector<double> interp(dim);
        for (int step = 0; step <= steps; ++step) {
            double t = static_cast<double>(step) / steps;
            for (int d = 0; d < dim; ++d)
                interp[d] = from[d] + t * (to[d] - from[d]);
            if (!isValidSingle(robot, interp, obstacles, cylinders) ||
                !isSelfCollisionFree(robot, interp))
                return false;
        }
        return true;
    }

    bool isRobotPathValid(const RobotModel &robot, const Path &path,
                          const std::vector<ObstacleSphere> &obstacles,
                          const std::vector<ObstacleCylinder> &cylinders) const override {
        for (const auto &config : path) {
            if (!isValidSingle(robot, config, obstacles, cylinders) ||
                !isSelfCollisionFree(robot, config)) {
                return false;
            }
        }
        return true;
    }

    bool isPairPathValid(const RobotModel &robot_a, const Path &path_a,
                         const RobotModel &robot_b, const Path &path_b,
                         std::size_t t_begin, std::size_t t_end) const override {
        return !findFirstPairPathConflict(robot_a, path_a, robot_b, path_b,
                                          t_begin, t_end).has_value();
    }

    std::optional<PairPathConflict> findFirstPairPathConflict(
        const RobotModel &robot_a, const Path &path_a,
        const RobotModel &robot_b, const Path &path_b,
        std::size_t t_begin, std::size_t t_end) const override {
        if (path_a.empty() || path_b.empty())
            return std::nullopt;

        const std::size_t max_t = std::max(path_a.size(), path_b.size());
        const std::size_t end = std::min(max_t, t_end);
        for (std::size_t t = t_begin; t < end; ++t) {
            const auto &config_a = configAt(path_a, t);
            const auto &config_b = configAt(path_b, t);
            if (!isValidPair(robot_a, config_a, robot_b, config_b)) {
                return PairPathConflict{t, 0.0, ConflictKind::Vertex,
                                        config_a, config_b};
            }
        }
        return std::nullopt;
    }

    GoalHoldConstraint computeGoalHoldConstraint(
        const RobotModel &goal_robot,
        const std::vector<double> &goal_config,
        const RobotModel &prior_robot,
        const Path &prior_path) const override {
        if (prior_path.empty())
            return {};

        for (std::size_t t = prior_path.size(); t-- > 0;) {
            if (isValidPair(goal_robot, goal_config, prior_robot, prior_path[t]))
                continue;

            if (t + 1 == prior_path.size())
                return GoalHoldConstraint{0, true};
            return GoalHoldConstraint{t + 1, false};
        }

        return {};
    }

    bool isCompositeMotionValid(
        const std::vector<const RobotModel *> &robots,
        const std::vector<std::vector<double>> &from,
        const std::vector<std::vector<double>> &to,
        const CompositePathValidationOptions &options,
        const std::vector<ObstacleSphere> &obstacles,
        const std::vector<ObstacleCylinder> &cylinders) const override {
        last_work_stats_ = {};
        if (robots.size() != from.size() || robots.size() != to.size())
            return false;

        const int num_checks =
            options.discrete_num_checks_hint > 0 ? options.discrete_num_checks_hint : 10;
        const std::size_t timestep_count =
            static_cast<std::size_t>(num_checks) + 1;
        const std::size_t pair_count =
            robots.size() * (robots.size() - 1) / 2;
        last_work_stats_.motion_timesteps_possible = timestep_count;
        if (options.check_environment) {
            last_work_stats_.robot_state_checks_possible =
                timestep_count * robots.size();
        }
        last_work_stats_.robot_pair_checks_possible =
            timestep_count * pair_count;
        std::vector<bool> timestep_checked(timestep_count, false);
        const auto mark_timestep = [&](std::size_t step) {
            if (!timestep_checked[step]) {
                timestep_checked[step] = true;
                ++last_work_stats_.motion_timesteps_checked;
            }
        };
        if (options.exhaustive) {
            bool valid = true;
            std::vector<std::vector<double>> interp(from.size());
            if (options.check_environment) {
                for (std::size_t i = 0; i < robots.size(); ++i) {
                    for (int step = 0; step <= num_checks; ++step) {
                        const double alpha =
                            static_cast<double>(step) / static_cast<double>(num_checks);
                        interpolateConfigInto(from[i], to[i], alpha, interp[i]);
                        mark_timestep(static_cast<std::size_t>(step));
                        ++last_work_stats_.robot_state_checks_completed;
                        const bool environment_valid =
                            isValidSingleExhaustive(*robots[i], interp[i]);
                        const bool self_valid =
                            isSelfCollisionFreeExhaustive(*robots[i],
                                                          interp[i]);
                        if (!environment_valid || !self_valid) {
                            valid = false;
                        }
                    }
                }
            }

            for (int step = 0; step <= num_checks; ++step) {
                const double alpha =
                    static_cast<double>(step) / static_cast<double>(num_checks);
                for (std::size_t i = 0; i < from.size(); ++i)
                    interpolateConfigInto(from[i], to[i], alpha, interp[i]);

                for (std::size_t i = 0; i < robots.size(); ++i) {
                    for (std::size_t j = i + 1; j < robots.size(); ++j) {
                        mark_timestep(static_cast<std::size_t>(step));
                        ++last_work_stats_.robot_pair_checks_completed;
                        if (!isValidPairExhaustive(
                                *robots[i], interp[i], *robots[j],
                                interp[j])) {
                            valid = false;
                        }
                    }
                }
            }
            return valid;
        }

        if (options.check_environment) {
            for (std::size_t i = 0; i < robots.size(); ++i) {
                std::vector<double> config;
                for (int step = 0; step <= num_checks; ++step) {
                    const double alpha = static_cast<double>(step) /
                                         static_cast<double>(num_checks);
                    interpolateConfigInto(from[i], to[i], alpha, config);
                    mark_timestep(static_cast<std::size_t>(step));
                    ++last_work_stats_.robot_state_checks_completed;
                    if (!isValidSingle(*robots[i], config, obstacles,
                                       cylinders)) {
                        return false;
                    }
                    if (!isSelfCollisionFree(*robots[i], config))
                        return false;
                }
            }
        }

        std::vector<std::vector<double>> interp(from.size());
        for (int step = 0; step <= num_checks; ++step) {
            const double alpha =
                static_cast<double>(step) / static_cast<double>(num_checks);
            for (std::size_t i = 0; i < from.size(); ++i)
                interpolateConfigInto(from[i], to[i], alpha, interp[i]);

            for (std::size_t i = 0; i < robots.size(); ++i) {
                for (std::size_t j = i + 1; j < robots.size(); ++j) {
                    mark_timestep(static_cast<std::size_t>(step));
                    ++last_work_stats_.robot_pair_checks_completed;
                    if (!isValidPair(*robots[i], interp[i], *robots[j], interp[j])) {
                        return false;
                    }
                }
            }
        }
        return true;
    }

    std::optional<CompositeConflict> findFirstCompositeMotionConflict(
        const std::vector<const RobotModel *> &robots,
        const std::vector<std::vector<double>> &from,
        const std::vector<std::vector<double>> &to,
        const CompositePathValidationOptions &options,
        const std::vector<ObstacleSphere> &obstacles,
        const std::vector<ObstacleCylinder> &cylinders) const override {
        if (robots.size() != from.size() || robots.size() != to.size())
            return std::nullopt;

        const int num_checks =
            options.discrete_num_checks_hint > 0 ? options.discrete_num_checks_hint : 10;
        std::vector<std::vector<double>> interp(from.size());
        for (int step = 0; step <= num_checks; ++step) {
            const double alpha =
                static_cast<double>(step) / static_cast<double>(num_checks);
            for (std::size_t i = 0; i < from.size(); ++i)
                interpolateConfigInto(from[i], to[i], alpha, interp[i]);

            if (options.check_environment) {
                for (std::size_t i = 0; i < robots.size(); ++i) {
                    if (!isValidSingle(*robots[i], interp[i], obstacles, cylinders)) {
                        return CompositeConflict{ConflictScope::Environment,
                                                 static_cast<int>(i), -1,
                                                 static_cast<std::size_t>(step),
                                                 0.0, ConflictKind::Vertex,
                                                 interp[i], {}};
                    }
                    if (!isSelfCollisionFree(*robots[i], interp[i])) {
                        return CompositeConflict{ConflictScope::Self,
                                                 static_cast<int>(i), -1,
                                                 static_cast<std::size_t>(step),
                                                 0.0, ConflictKind::Vertex,
                                                 interp[i], {}};
                    }
                }
            }

            for (std::size_t i = 0; i < robots.size(); ++i) {
                for (std::size_t j = i + 1; j < robots.size(); ++j) {
                    if (!isValidPair(*robots[i], interp[i], *robots[j], interp[j])) {
                        return CompositeConflict{ConflictScope::InterRobot,
                                                 static_cast<int>(i),
                                                 static_cast<int>(j),
                                                 static_cast<std::size_t>(step),
                                                 0.0, ConflictKind::Vertex,
                                                 interp[i], interp[j]};
                    }
                }
            }
        }
        return std::nullopt;
    }

    bool validateCompositePaths(
        const std::vector<Path> &paths,
        const std::vector<const RobotModel *> &robots,
        const CompositePathValidationOptions &options,
        const std::vector<ObstacleSphere> &obstacles,
        const std::vector<ObstacleCylinder> &cylinders) const override {
        if (paths.size() != robots.size())
            return false;

        const auto effective_starts = effectivePathStarts(paths.size(), options);
        const auto effective_pair_starts =
            effectivePairStarts(paths.size(), options, effective_starts);
        const std::size_t max_t = maxPathLength(paths);
        const std::size_t end = std::min(max_t, options.t_end);
        if (options.exhaustive) {
            bool valid = true;
            if (options.check_environment) {
                for (std::size_t i = 0; i < paths.size(); ++i) {
                    if (effective_starts[i] >= end)
                        continue;
                    for (std::size_t t = effective_starts[i]; t < end; ++t) {
                        const auto &config = configAt(paths[i], t);
                        if (!isValidSingle(*robots[i], config, obstacles,
                                           cylinders) ||
                            !isSelfCollisionFree(*robots[i], config)) {
                            valid = false;
                        }
                    }
                }
            }

            for (std::size_t i = 0; i < paths.size(); ++i) {
                for (std::size_t j = i + 1; j < paths.size(); ++j) {
                    const std::size_t pair_begin = effective_pair_starts[
                        pairFrontierIndex(i, j, paths.size())];
                    if (pair_begin >= end)
                        continue;
                    for (std::size_t t = pair_begin; t < end; ++t) {
                        const auto &config_i = configAt(paths[i], t);
                        const auto &config_j = configAt(paths[j], t);
                        if (!isValidPair(*robots[i], config_i, *robots[j],
                                         config_j)) {
                            valid = false;
                        }
                    }
                }
            }
            return valid;
        }

        if (options.check_environment) {
            for (std::size_t i = 0; i < paths.size(); ++i) {
                if (effective_starts[i] >= end)
                    continue;
                for (std::size_t t = effective_starts[i]; t < end; ++t) {
                    const auto &config = configAt(paths[i], t);
                    if (!isValidSingle(*robots[i], config, obstacles, cylinders) ||
                        !isSelfCollisionFree(*robots[i], config)) {
                        return false;
                    }
                }
            }
        }

        for (std::size_t i = 0; i < paths.size(); ++i) {
            for (std::size_t j = i + 1; j < paths.size(); ++j) {
                const std::size_t pair_begin = effective_pair_starts[
                    pairFrontierIndex(i, j, paths.size())];
                if (pair_begin >= end)
                    continue;
                if (!isPairPathValid(*robots[i], paths[i], *robots[j], paths[j],
                                     pair_begin, end)) {
                    return false;
                }
            }
        }
        return true;
    }

    std::optional<CompositeConflict> findFirstCompositePathConflict(
        const std::vector<Path> &paths,
        const std::vector<const RobotModel *> &robots,
        const CompositePathValidationOptions &options,
        const std::vector<ObstacleSphere> &obstacles,
        const std::vector<ObstacleCylinder> &cylinders,
        std::vector<std::size_t> *next_t_begin_by_robot_out) const override {
        if (paths.size() != robots.size())
            return std::nullopt;

        const auto effective_starts = effectivePathStarts(paths.size(), options);
        const auto effective_pair_starts =
            effectivePairStarts(paths.size(), options, effective_starts);
        initializeNextPathStarts(next_t_begin_by_robot_out, effective_starts);
        const std::size_t max_t = maxPathLength(paths);
        const std::size_t end = std::min(max_t, options.t_end);
        if (effective_starts.empty()) {
            return std::nullopt;
        }

        const std::size_t global_begin =
            *std::min_element(effective_starts.begin(), effective_starts.end());
        ActiveRobotSchedule schedule(effective_starts);
        std::vector<std::size_t> active;
        for (std::size_t t = global_begin; t < end; ++t) {
            schedule.activateThrough(t, active);
            if (options.check_environment) {
                for (const std::size_t robot : active) {
                    const auto &config = configAt(paths[robot], t);
                    if (!isValidSingle(*robots[robot], config, obstacles, cylinders)) {
                        return CompositeConflict{ConflictScope::Environment,
                                                 static_cast<int>(robot), -1, t, 0.0,
                                                 ConflictKind::Vertex, config, {}};
                    }
                    if (!isSelfCollisionFree(*robots[robot], config)) {
                        return CompositeConflict{ConflictScope::Self,
                                                 static_cast<int>(robot), -1, t, 0.0,
                                                 ConflictKind::Vertex, config, {}};
                    }
                }
            }

            for (std::size_t ai = 0; ai < active.size(); ++ai) {
                const std::size_t i = active[ai];
                for (std::size_t aj = ai + 1; aj < active.size(); ++aj) {
                    const std::size_t j = active[aj];
                    if (effective_pair_starts[pairFrontierIndex(
                            i, j, paths.size())] > t)
                        continue;
                    const auto &config_i = configAt(paths[i], t);
                    const auto &config_j = configAt(paths[j], t);
                    if (!isValidPair(*robots[i], config_i, *robots[j], config_j)) {
                        return CompositeConflict{ConflictScope::InterRobot,
                                                 static_cast<int>(i),
                                                 static_cast<int>(j), t, 0.0,
                                                 ConflictKind::Vertex, config_i,
                                                 config_j};
                    }
                }
            }
            assignActivePathStarts(next_t_begin_by_robot_out, active, t + 1);
        }
        return std::nullopt;
    }

    std::vector<CompositeConflict> findInterRobotPathConflictsCompositeScan(
        const std::vector<Path> &paths,
        const std::vector<const RobotModel *> &robots,
        const CompositePathValidationOptions &options,
        std::size_t max_conflicts, bool unique,
        const InterRobotConflictCallback &on_conflict,
        const std::vector<ObstacleSphere> &,
        const std::vector<ObstacleCylinder> &,
        std::vector<std::size_t> *next_t_begin_by_robot_out,
        std::vector<std::size_t> *next_t_begin_by_pair_out) const override {
        if (unique && options.conflict_find_parallel_workers > 1 &&
            options.conflict_find_parallel_horizon > 0) {
            return findInterRobotPathConflictsCompositeScanParallel(
                paths, robots, options, max_conflicts, on_conflict,
                next_t_begin_by_robot_out, next_t_begin_by_pair_out);
        }

        std::vector<CompositeConflict> out;
        if (paths.size() != robots.size())
            return out;

        const auto effective_starts = effectivePathStarts(paths.size(), options);
        const auto effective_pair_starts =
            effectivePairStarts(paths.size(), options, effective_starts);
        initializeNextPathStarts(next_t_begin_by_robot_out, effective_starts);
        initializeNextPairStarts(next_t_begin_by_pair_out,
                                 effective_pair_starts);
        const std::size_t max_t = maxPathLength(paths);
        const std::size_t end = std::min(max_t, options.t_end);
        if (effective_pair_starts.empty()) {
            return out;
        }

        const std::size_t global_begin =
            *std::min_element(effective_pair_starts.begin(),
                              effective_pair_starts.end());
        if (global_begin >= end) {
            return out;
        }
        const bool optimistic_unique =
            unique &&
            options.inter_robot_conflict_batch_mode ==
                InterRobotConflictBatchMode::OptimisticIndependent;
        std::vector<char> robot_used(paths.size(), 0);
        std::vector<char> pair_has_accepted(effective_pair_starts.size(), 0);
        std::vector<AcceptedInterRobotConflictClaim> accepted_claims;

        for (std::size_t t = global_begin; t < end; ++t) {
            for (std::size_t i = 0; i < paths.size(); ++i) {
                for (std::size_t j = i + 1; j < paths.size(); ++j) {
                    const std::size_t pair_index =
                        pairFrontierIndex(i, j, paths.size());
                    if (effective_pair_starts[pair_index] > t)
                        continue;
                    if (optimistic_unique && (robot_used[i] || robot_used[j]))
                        continue;
                    const auto &config_i = configAt(paths[i], t);
                    const auto &config_j = configAt(paths[j], t);
                    if (!isValidPair(*robots[i], config_i, *robots[j],
                                     config_j)) {
                        const CompositeConflict conflict{
                            ConflictScope::InterRobot, static_cast<int>(i),
                            static_cast<int>(j), t, 0.0, ConflictKind::Vertex,
                            config_i, config_j};
                        if (acceptInterRobotConflictCandidate(
                                conflict, on_conflict, unique, robot_used, out,
                                accepted_claims,
                                options.inter_robot_conflict_batch_mode)) {
                            if (optimistic_unique && next_t_begin_by_pair_out &&
                                !pair_has_accepted[pair_index]) {
                                (*next_t_begin_by_pair_out)[pair_index] = t;
                                pair_has_accepted[pair_index] = 1;
                            }
                            if (max_conflicts > 0 &&
                                out.size() >= max_conflicts) {
                                if (unique) {
                                    assignUniqueConflictFrontier(
                                        next_t_begin_by_robot_out,
                                        effective_starts, accepted_claims);
                                }
                                return out;
                            }
                        }
                    } else if (optimistic_unique && next_t_begin_by_pair_out &&
                               !pair_has_accepted[pair_index]) {
                        (*next_t_begin_by_pair_out)[pair_index] = t + 1;
                    }
                }
            }
        }
        if (unique) {
            assignUniqueConflictFrontier(next_t_begin_by_robot_out,
                                         effective_starts, accepted_claims);
        }
        return out;
    }

private:
    struct ConflictCandidate {
        CompositeConflict conflict;
        std::size_t pair_index = 0;
    };

    struct WorkPair {
        std::size_t robot_i = 0;
        std::size_t robot_j = 0;
        std::size_t pair_index = 0;
    };

    std::vector<CompositeConflict>
    findInterRobotPathConflictsCompositeScanParallel(
        const std::vector<Path> &paths,
        const std::vector<const RobotModel *> &robots,
        const CompositePathValidationOptions &options,
        std::size_t max_conflicts,
        const InterRobotConflictCallback &on_conflict,
        std::vector<std::size_t> *next_t_begin_by_robot_out,
        std::vector<std::size_t> *next_t_begin_by_pair_out) const {
#if defined(_WIN32)
        (void)paths;
        (void)robots;
        (void)options;
        (void)max_conflicts;
        (void)on_conflict;
        (void)next_t_begin_by_robot_out;
        (void)next_t_begin_by_pair_out;
        throw std::runtime_error(
            "Process-parallel FCL conflict finding requires POSIX fork support");
#else
        enum class CommandKind : std::uint64_t { Scan = 1, Stop = 2 };
        struct ProcessScanCommand {
            std::uint64_t kind = 0;
            std::uint64_t segment_begin = 0;
            std::uint64_t segment_end = 0;
            std::uint64_t claimed_robot_count = 0;
        };
        struct RawConflictCandidate {
            std::uint64_t pair_index = 0;
            std::uint64_t robot_i = 0;
            std::uint64_t robot_j = 0;
            std::uint64_t timestep = 0;
        };
        struct ProcessScanResultHeader {
            std::uint64_t candidate_count = 0;
            double build_worker_wall_seconds = 0.0;
            double build_worker_cpu_seconds = 0.0;
            double collision_worker_wall_seconds = 0.0;
            double collision_worker_cpu_seconds = 0.0;
        };
        struct ProcessWorker {
            pid_t pid = -1;
            int fd = -1;
        };
        struct SigpipeIgnoreGuard {
            struct sigaction old_action {};
            bool active = false;
            SigpipeIgnoreGuard() {
                struct sigaction ignore_action {};
                ignore_action.sa_handler = SIG_IGN;
                sigemptyset(&ignore_action.sa_mask);
                ignore_action.sa_flags = 0;
                active = ::sigaction(SIGPIPE, &ignore_action, &old_action) == 0;
            }
            ~SigpipeIgnoreGuard() {
                if (active)
                    (void)::sigaction(SIGPIPE, &old_action, nullptr);
            }
        } sigpipe_guard;

        std::vector<CompositeConflict> out;
        if (paths.size() != robots.size())
            return out;

        const auto effective_starts = effectivePathStarts(paths.size(), options);
        const auto effective_pair_starts =
            effectivePairStarts(paths.size(), options, effective_starts);
        initializeNextPathStarts(next_t_begin_by_robot_out, effective_starts);
        initializeNextPairStarts(next_t_begin_by_pair_out,
                                 effective_pair_starts);
        const std::size_t max_t = maxPathLength(paths);
        const std::size_t end = std::min(max_t, options.t_end);
        if (effective_pair_starts.empty()) {
            return out;
        }

        const std::size_t global_begin =
            *std::min_element(effective_pair_starts.begin(),
                              effective_pair_starts.end());
        if (global_begin >= end) {
            return out;
        }

        const std::size_t worker_count =
            std::max<std::size_t>(1, options.conflict_find_parallel_workers);
        const std::size_t horizon =
            std::max<std::size_t>(1, options.conflict_find_parallel_horizon);
        const bool optimistic_unique =
            options.inter_robot_conflict_batch_mode ==
            InterRobotConflictBatchMode::OptimisticIndependent;
        std::vector<std::vector<WorkPair>> worker_pairs(worker_count);

        // Distribute the pair frontier uniformly in round-robin order.
        for (std::size_t i = 0; i < paths.size(); ++i) {
            for (std::size_t j = i + 1; j < paths.size(); ++j) {
                const std::size_t pair_index =
                    pairFrontierIndex(i, j, paths.size());
                worker_pairs[pair_index % worker_count].push_back(
                    WorkPair{i, j, pair_index});
            }
        }

        std::vector<char> robot_used(paths.size(), 0);
        std::vector<char> pair_has_accepted(effective_pair_starts.size(), 0);
        std::vector<AcceptedInterRobotConflictClaim> accepted_claims;
        std::vector<ProcessWorker> workers;
        workers.reserve(worker_count);

        const auto closeWorkerFd = [](ProcessWorker &worker) {
            if (worker.fd >= 0) {
                ::close(worker.fd);
                worker.fd = -1;
            }
        };

        const auto workerMain = [&](int fd, std::size_t worker_index) -> int {
            while (true) {
                ProcessScanCommand command;
                if (!readValue(fd, command))
                    return 2;
                if (command.kind ==
                    static_cast<std::uint64_t>(CommandKind::Stop)) {
                    return 0;
                }
                if (command.kind !=
                        static_cast<std::uint64_t>(CommandKind::Scan) ||
                    command.claimed_robot_count != paths.size()) {
                    return 2;
                }

                std::vector<char> claimed_robots(paths.size(), 0);
                if (!claimed_robots.empty() &&
                    !readExact(fd, claimed_robots.data(),
                               claimed_robots.size())) {
                    return 2;
                }

                std::vector<RawConflictCandidate> raw_candidates;
                ProcessScanResultHeader result;
                double build_worker_wall_seconds = 0.0;
                double build_worker_cpu_seconds = 0.0;
                double collision_worker_wall_seconds = 0.0;
                double collision_worker_cpu_seconds = 0.0;
                const std::size_t segment_begin =
                    static_cast<std::size_t>(command.segment_begin);
                const std::size_t segment_end =
                    static_cast<std::size_t>(command.segment_end);
                const auto pairValidAtT = [&](const WorkPair &pair,
                                              std::size_t timestep) {
                    const auto build_wall_start = Clock::now();
                    const double build_cpu_start = processCpuSeconds();
                    const auto &cache_a = robotCache(*robots[pair.robot_i]);
                    const auto &cache_b = robotCache(*robots[pair.robot_j]);
                    const auto config_a = configAt(paths[pair.robot_i], timestep);
                    const auto config_b = configAt(paths[pair.robot_j], timestep);
                    if (cache_a.prims.empty() || cache_b.prims.empty()) {
                        build_worker_wall_seconds +=
                            elapsedWallSeconds(build_wall_start);
                        build_worker_cpu_seconds +=
                            elapsedProcessCpuSeconds(build_cpu_start);
                        return true;
                    }
                    auto transforms_a =
                        robots[pair.robot_i]->getLinkTransforms(config_a);
                    auto transforms_b =
                        robots[pair.robot_j]->getLinkTransforms(config_b);
                    build_worker_wall_seconds +=
                        elapsedWallSeconds(build_wall_start);
                    build_worker_cpu_seconds +=
                        elapsedProcessCpuSeconds(build_cpu_start);

                    const auto collision_wall_start = Clock::now();
                    const double collision_cpu_start = processCpuSeconds();
                    for (const auto &prim_a : cache_a.prims) {
                        Transform3 world_a =
                            eigenAffineToFcl(
                                transforms_a[static_cast<std::size_t>(
                                    prim_a.link_index)]) *
                            prim_a.tf_link;
                        for (const auto &prim_b : cache_b.prims) {
                            Transform3 world_b =
                                eigenAffineToFcl(
                                    transforms_b[static_cast<std::size_t>(
                                        prim_b.link_index)]) *
                                prim_b.tf_link;
                            if (fclPairCollide(prim_a.geom, world_a, prim_b.geom,
                                               world_b)) {
                                collision_worker_wall_seconds +=
                                    elapsedWallSeconds(collision_wall_start);
                                collision_worker_cpu_seconds +=
                                    elapsedProcessCpuSeconds(
                                        collision_cpu_start);
                                return false;
                            }
                        }
                    }
                    collision_worker_wall_seconds +=
                        elapsedWallSeconds(collision_wall_start);
                    collision_worker_cpu_seconds +=
                        elapsedProcessCpuSeconds(collision_cpu_start);
                    return true;
                };
                for (std::size_t t = segment_begin; t < segment_end; ++t) {
                    for (const auto &pair : worker_pairs[worker_index]) {
                        if (effective_pair_starts[pair.pair_index] > t) {
                            continue;
                        }
                        if (optimistic_unique &&
                            (claimed_robots[pair.robot_i] ||
                             claimed_robots[pair.robot_j])) {
                            continue;
                        }
                        if (!pairValidAtT(pair, t)) {
                            raw_candidates.push_back(RawConflictCandidate{
                                static_cast<std::uint64_t>(pair.pair_index),
                                static_cast<std::uint64_t>(pair.robot_i),
                                static_cast<std::uint64_t>(pair.robot_j),
                                static_cast<std::uint64_t>(t)});
                        }
                    }
                }
                result.candidate_count = raw_candidates.size();
                result.build_worker_wall_seconds = build_worker_wall_seconds;
                result.build_worker_cpu_seconds = build_worker_cpu_seconds;
                result.collision_worker_wall_seconds =
                    collision_worker_wall_seconds;
                result.collision_worker_cpu_seconds =
                    collision_worker_cpu_seconds;
                if (!writeValue(fd, result))
                    return 2;
                if (!raw_candidates.empty() &&
                    !writeExact(fd, raw_candidates.data(),
                                raw_candidates.size() *
                                    sizeof(RawConflictCandidate))) {
                    return 2;
                }
            }
        };

        auto shutdownWorkers = [&](bool terminate) {
            for (auto &worker : workers) {
                if (worker.pid <= 0)
                    continue;
                if (terminate) {
                    ::kill(worker.pid, SIGTERM);
                } else if (worker.fd >= 0) {
                    const ProcessScanCommand stop{
                        static_cast<std::uint64_t>(CommandKind::Stop), 0, 0, 0};
                    (void)writeValue(worker.fd, stop);
                }
                closeWorkerFd(worker);
            }
            for (auto &worker : workers) {
                if (worker.pid <= 0)
                    continue;
                int status = 0;
                while (::waitpid(worker.pid, &status, 0) < 0) {
                    if (errno == EINTR)
                        continue;
                    break;
                }
                worker.pid = -1;
            }
        };

        for (std::size_t worker = 0; worker < worker_count; ++worker) {
            std::array<int, 2> fds{{-1, -1}};
            if (::socketpair(AF_UNIX, SOCK_STREAM, 0, fds.data()) != 0) {
                shutdownWorkers(true);
                throw std::runtime_error(
                    "Process-parallel FCL conflict finder socketpair failed");
            }
            const pid_t parent_pid = ::getpid();
            const pid_t pid = ::fork();
            if (pid < 0) {
                ::close(fds[0]);
                ::close(fds[1]);
                shutdownWorkers(true);
                throw std::runtime_error(
                    "Process-parallel FCL conflict finder fork failed");
            }
            if (pid == 0) {
                if (!comotion::detail::armParentDeathSignal(parent_pid))
                    _exit(3);
                ::close(fds[0]);
                const int exit_code = workerMain(fds[1], worker);
                ::close(fds[1]);
                _exit(exit_code);
            }
            ::close(fds[1]);
            workers.push_back(ProcessWorker{pid, fds[0]});
        }

        try {
            for (std::size_t segment_begin = global_begin; segment_begin < end;
                 segment_begin += std::min(horizon, end - segment_begin)) {
                const std::size_t segment_end =
                    segment_begin + std::min(horizon, end - segment_begin);
                const auto segment_robot_used = robot_used;

                const ProcessScanCommand command{
                    static_cast<std::uint64_t>(CommandKind::Scan),
                    static_cast<std::uint64_t>(segment_begin),
                    static_cast<std::uint64_t>(segment_end),
                    static_cast<std::uint64_t>(segment_robot_used.size())};
                for (auto &worker : workers) {
                    const bool ok =
                        writeValue(worker.fd, command) &&
                        (segment_robot_used.empty() ||
                         writeExact(worker.fd, segment_robot_used.data(),
                                    segment_robot_used.size()));
                    if (!ok) {
                        std::ostringstream msg;
                        msg << "Process-parallel FCL conflict finder command "
                               "write failed";
                        if (errno != 0)
                            msg << ": " << std::strerror(errno);
                        int status = 0;
                        const pid_t waited =
                            ::waitpid(worker.pid, &status, WNOHANG);
                        if (waited == worker.pid) {
                            worker.pid = -1;
                            msg << " (worker exited";
                            if (WIFEXITED(status)) {
                                msg << " status=" << WEXITSTATUS(status);
                            } else if (WIFSIGNALED(status)) {
                                msg << " signal=" << WTERMSIG(status);
                            }
                            msg << ")";
                        }
                        throw std::runtime_error(msg.str());
                    }
                }

                std::vector<ConflictCandidate> candidates;

                for (std::size_t worker_index = 0;
                     worker_index < workers.size(); ++worker_index) {
                    auto &worker = workers[worker_index];
                    ProcessScanResultHeader result;
                    if (!readValue(worker.fd, result)) {
                        throw std::runtime_error(
                            "Process-parallel FCL conflict finder result read "
                            "failed");
                    }
                    if (options.conflict_find_timing_instrumentation) {
                        // Accumulate per-worker timings for planner metrics.
                        options.conflict_find_timing_instrumentation
                            ->recordWorkerResult(
                                worker_index,
                                result.build_worker_wall_seconds,
                                result.build_worker_cpu_seconds,
                                result.collision_worker_wall_seconds,
                                result.collision_worker_cpu_seconds);
                    }
                    std::vector<RawConflictCandidate> raw_candidates(
                        static_cast<std::size_t>(result.candidate_count));
                    if (!raw_candidates.empty() &&
                        !readExact(worker.fd, raw_candidates.data(),
                                   raw_candidates.size() *
                                       sizeof(RawConflictCandidate))) {
                        throw std::runtime_error(
                            "Process-parallel FCL conflict finder candidate "
                            "read failed");
                    }

                    candidates.reserve(candidates.size() +
                                       raw_candidates.size());
                    for (const auto &raw : raw_candidates) {
                        const auto robot_i =
                            static_cast<std::size_t>(raw.robot_i);
                        const auto robot_j =
                            static_cast<std::size_t>(raw.robot_j);
                        const auto t = static_cast<std::size_t>(raw.timestep);
                        candidates.push_back(ConflictCandidate{
                            CompositeConflict{
                                ConflictScope::InterRobot,
                                static_cast<int>(robot_i),
                                static_cast<int>(robot_j), t, 0.0,
                                ConflictKind::Vertex,
                                configAt(paths[robot_i], t),
                                configAt(paths[robot_j], t)},
                            static_cast<std::size_t>(raw.pair_index)});
                    }
                }

                std::vector<std::size_t> earliest_candidate_by_pair(
                    effective_pair_starts.size(),
                    std::numeric_limits<std::size_t>::max());
                std::vector<std::size_t> earliest_candidate_by_robot(
                    paths.size(), std::numeric_limits<std::size_t>::max());
                for (const auto &candidate : candidates) {
                    auto &earliest =
                        earliest_candidate_by_pair[candidate.pair_index];
                    earliest = std::min(earliest,
                                        candidate.conflict.timestep);
                    const auto robot_i =
                        static_cast<std::size_t>(candidate.conflict.robot_i);
                    const auto robot_j =
                        static_cast<std::size_t>(candidate.conflict.robot_j);
                    earliest_candidate_by_robot[robot_i] = std::min(
                        earliest_candidate_by_robot[robot_i],
                        candidate.conflict.timestep);
                    earliest_candidate_by_robot[robot_j] = std::min(
                        earliest_candidate_by_robot[robot_j],
                        candidate.conflict.timestep);
                }

                std::sort(candidates.begin(), candidates.end(),
                          [](const ConflictCandidate &lhs,
                             const ConflictCandidate &rhs) {
                              if (lhs.conflict.timestep !=
                                  rhs.conflict.timestep) {
                                  return lhs.conflict.timestep <
                                         rhs.conflict.timestep;
                              }
                              if (lhs.conflict.robot_i !=
                                  rhs.conflict.robot_i) {
                                  return lhs.conflict.robot_i <
                                         rhs.conflict.robot_i;
                              }
                              return lhs.conflict.robot_j <
                                     rhs.conflict.robot_j;
                          });

                for (const auto &candidate : candidates) {
                    const auto &conflict = candidate.conflict;
                    if (optimistic_unique &&
                        (robot_used[static_cast<std::size_t>(
                             conflict.robot_i)] ||
                         robot_used[static_cast<std::size_t>(
                             conflict.robot_j)])) {
                        continue;
                    }
                    if (acceptInterRobotConflictCandidate(
                            conflict, on_conflict, true, robot_used, out,
                            accepted_claims,
                            options.inter_robot_conflict_batch_mode)) {
                        if (optimistic_unique && next_t_begin_by_pair_out &&
                            !pair_has_accepted[candidate.pair_index]) {
                            (*next_t_begin_by_pair_out)
                                [candidate.pair_index] = conflict.timestep;
                            pair_has_accepted[candidate.pair_index] = 1;
                        }
                        if (max_conflicts > 0 &&
                            out.size() >= max_conflicts) {
                            break;
                        }
                    }
                }

                std::vector<std::size_t> first_claim_by_robot(
                    paths.size(), std::numeric_limits<std::size_t>::max());
                for (const auto &claim : accepted_claims) {
                    for (const int robot : claim.robots) {
                        const auto robot_index =
                            static_cast<std::size_t>(robot);
                        if (robot_index < first_claim_by_robot.size()) {
                            first_claim_by_robot[robot_index] =
                                std::min(first_claim_by_robot[robot_index],
                                         claim.timestep);
                        }
                    }
                }

                if (optimistic_unique && next_t_begin_by_pair_out) {
                    for (const auto &pairs : worker_pairs) {
                        for (const auto &pair : pairs) {
                            if (pair_has_accepted[pair.pair_index])
                                continue;
                            if (segment_robot_used[pair.robot_i] ||
                                segment_robot_used[pair.robot_j]) {
                                continue;
                            }
                            if (effective_pair_starts[pair.pair_index] >=
                                segment_end) {
                                continue;
                            }

                            std::size_t checked_end = segment_end;
                            checked_end = std::min(
                                checked_end,
                                first_claim_by_robot[pair.robot_i]);
                            checked_end = std::min(
                                checked_end,
                                first_claim_by_robot[pair.robot_j]);
                            checked_end = std::min(
                                checked_end,
                                earliest_candidate_by_robot[pair.robot_i]);
                            checked_end = std::min(
                                checked_end,
                                earliest_candidate_by_robot[pair.robot_j]);
                            const std::size_t earliest_candidate =
                                earliest_candidate_by_pair[pair.pair_index];
                            if (earliest_candidate !=
                                std::numeric_limits<std::size_t>::max()) {
                                checked_end =
                                    std::min(checked_end, earliest_candidate);
                            }
                            if (checked_end >
                                (*next_t_begin_by_pair_out)
                                    [pair.pair_index]) {
                                (*next_t_begin_by_pair_out)
                                    [pair.pair_index] = checked_end;
                            }
                        }
                    }
                }

                if (max_conflicts > 0 && out.size() >= max_conflicts)
                    break;
            }
        } catch (...) {
            shutdownWorkers(true);
            throw;
        }

        assignUniqueConflictFrontier(next_t_begin_by_robot_out,
                                     effective_starts, accepted_claims);
        shutdownWorkers(false);
        return out;
#endif
    }

    std::vector<ObstacleSphere> env_spheres_;
    std::vector<ObstacleCylinder> env_cylinders_;
    std::vector<CollisionGeometryPtr> env_geoms_;
    std::vector<Transform3> env_transforms_;
    mutable std::unordered_map<const RobotModel *, RobotPrimCache> robot_cache_;
    mutable ValidationWorkStats last_work_stats_;
};

} // namespace

std::unique_ptr<CollisionBackend> makeFclBackend() {
    return std::make_unique<FclBackendImpl>();
}

} // namespace detail
} // namespace comotion
