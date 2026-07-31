#include "comotion/collision/detail/CollisionBackend.h"
#include "comotion/collision/detail/VampPackingUtils.h"
#include "comotion/collision/detail/ValidationUtils.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>

#if COMOTION_HAVE_VAMP
#include <vamp/collision/environment.hh>
#include <vamp/collision/factory.hh>
#include <vamp/collision/sphere_sphere.hh>
#include <vamp/planning/validate.hh>
#include <vamp/robots/panda.hh>
#include <vamp/robots/planar3.hh>
#include <vamp/robots/sphere.hh>
#include <vamp/robots/ur5.hh>
#include <vamp/vector.hh>
#endif

#if COMOTION_HAVE_VAMP && !defined(_WIN32)
#include <poll.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace comotion {
namespace detail {

#if COMOTION_HAVE_VAMP
namespace {

using Clock = std::chrono::steady_clock;

constexpr std::size_t kRake = detail::kVampPackingWidth;
using EnvironmentFloat = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<kRake>>;
using BatchPack = detail::VampPackingLayout;

struct PackedSphereBlock {
    vamp::FloatVector<kRake> x;
    vamp::FloatVector<kRake> y;
    vamp::FloatVector<kRake> z;
    vamp::FloatVector<kRake> r;
};

struct PackedRobotSpheres {
    std::vector<PackedSphereBlock> spheres;
};

#if !defined(_WIN32)
bool readExact(int fd, void *buffer, std::size_t bytes) {
    auto *out = static_cast<char *>(buffer);
    std::size_t read_total = 0;
    while (read_total < bytes) {
        const ssize_t got =
            ::read(fd, out + read_total, bytes - read_total);
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

std::vector<BatchPack> makeBatchPacks(std::size_t begin, std::size_t end,
                                      VampBatchPacking packing) {
    return detail::makeVampPackingLayouts(begin, end, packing);
}

BatchPack filterPackAtOrAfter(const BatchPack &pack, std::size_t begin) {
    BatchPack filtered;
    for (std::size_t lane = 0; lane < pack.lanes; ++lane) {
        const std::size_t timestep = pack.timesteps[lane];
        if (timestep < begin)
            continue;
        filtered.timesteps[filtered.lanes++] = timestep;
    }
    return filtered;
}

std::string robotFamilyName(RobotModel::RobotFamily family) {
    switch (family) {
    case RobotModel::RobotFamily::Sphere:
        return "Sphere";
    case RobotModel::RobotFamily::Panda:
        return "Panda";
    case RobotModel::RobotFamily::UR5:
        return "UR5";
    case RobotModel::RobotFamily::Planar3:
        return "Planar3";
    case RobotModel::RobotFamily::Unknown:
        break;
    }
    return "Unknown";
}

[[noreturn]] void throwUnsupportedRobotFamily(const RobotModel &robot,
                                              const char *context) {
    throw std::runtime_error(std::string("VAMP backend: ") + context +
                             " requires a supported robot family, got " +
                             robotFamilyName(robot.robotFamily()));
}

struct SphereRobotSettings {
    std::array<float, 3> lows{};
    std::array<float, 3> highs{};
    float radius = 0.0f;
};

SphereRobotSettings extractSphereRobotSettings(const RobotModel &robot) {
    if (robot.numJoints() != 3)
        throw std::runtime_error(
            "VAMP backend: Sphere family requires exactly 3 active joints.");

    SphereRobotSettings settings;
    for (int i = 0; i < 3; ++i) {
        settings.lows[static_cast<std::size_t>(i)] =
            static_cast<float>(robot.jointLower(i));
        settings.highs[static_cast<std::size_t>(i)] =
            static_cast<float>(robot.jointUpper(i));
    }

    for (const auto &link : robot.links()) {
        if (!link.collision_spheres.empty()) {
            settings.radius =
                static_cast<float>(link.collision_spheres.front().radius);
            return settings;
        }
    }

    throw std::runtime_error(
        "VAMP backend: Sphere family robot must expose a collision sphere.");
}

void configureSphereRobot(const RobotModel &robot) {
    const auto settings = extractSphereRobotSettings(robot);
    vamp::robots::Sphere::set_lows(settings.lows);
    vamp::robots::Sphere::set_highs(settings.highs);
    vamp::robots::Sphere::set_radius(settings.radius);
}

void ensureCompatibleSphereRobots(const RobotModel &robot_a,
                                  const RobotModel &robot_b) {
    const auto a = extractSphereRobotSettings(robot_a);
    const auto b = extractSphereRobotSettings(robot_b);
    constexpr float kTol = 1e-5f;
    for (std::size_t i = 0; i < 3; ++i) {
        if (std::abs(a.lows[i] - b.lows[i]) > kTol ||
            std::abs(a.highs[i] - b.highs[i]) > kTol) {
            throw std::runtime_error(
                "VAMP backend: mixed Sphere robot bounds are not supported in "
                "the same pairwise SIMD query.");
        }
    }
    if (std::abs(a.radius - b.radius) > kTol) {
        throw std::runtime_error(
            "VAMP backend: mixed Sphere robot radii are not supported in the "
            "same pairwise SIMD query.");
    }
    configureSphereRobot(robot_a);
}

bool appendCylinderObstacle(EnvironmentFloat &environment,
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

template <typename Robot>
typename Robot::Configuration toConfiguration(const std::vector<double> &config) {
    if (config.size() < Robot::dimension) {
        throw std::runtime_error("VAMP backend: configuration dimension mismatch.");
    }

    alignas(vamp::FloatVectorAlignment)
        std::array<float, Robot::Configuration::num_scalars_rounded> buffer{};
    for (std::size_t i = 0; i < Robot::dimension; ++i)
        buffer[i] = static_cast<float>(config[i]);
    return typename Robot::Configuration(buffer.data());
}

template <typename Robot>
typename Robot::template ConfigurationBlock<kRake>
makeConfigurationBlock(
    const std::array<const std::vector<double> *, kRake> &configs,
    std::size_t lanes) {
    typename Robot::template ConfigurationBlock<kRake> block;
    if (lanes == 0)
        return block;

    for (std::size_t dim = 0; dim < Robot::dimension; ++dim) {
        std::array<float, kRake> lane_values{};
        for (std::size_t lane = 0; lane < kRake; ++lane) {
            const auto *config = configs[lane < lanes ? lane : 0];
            if (!config || config->size() < Robot::dimension) {
                throw std::runtime_error(
                    "VAMP backend: configuration dimension mismatch.");
            }
            lane_values[lane] = static_cast<float>((*config)[dim]);
        }
        using Row = std::decay_t<decltype(block[0])>;
        block[dim] = Row(lane_values);
    }

    return block;
}

template <typename Robot>
void applyBaseTransform(
    const Eigen::Affine3f &transform,
    typename Robot::template Spheres<kRake> &spheres) {
    const auto rotation = transform.linear();
    const auto translation = transform.translation();

    const float r00 = rotation(0, 0);
    const float r01 = rotation(0, 1);
    const float r02 = rotation(0, 2);
    const float r10 = rotation(1, 0);
    const float r11 = rotation(1, 1);
    const float r12 = rotation(1, 2);
    const float r20 = rotation(2, 0);
    const float r21 = rotation(2, 1);
    const float r22 = rotation(2, 2);
    const float tx = translation.x();
    const float ty = translation.y();
    const float tz = translation.z();

    for (std::size_t i = 0; i < Robot::n_spheres; ++i) {
        const auto x = spheres.x[i];
        const auto y = spheres.y[i];
        const auto z = spheres.z[i];

        spheres.x[i] = r00 * x + r01 * y + r02 * z + tx;
        spheres.y[i] = r10 * x + r11 * y + r12 * z + ty;
        spheres.z[i] = r20 * x + r21 * y + r22 * z + tz;
    }
}

template <typename VecT>
void markCollidingLanes(const VecT &distances,
                        std::size_t lanes,
                        std::array<bool, kRake> &lane_collision) {
    std::array<float, VecT::num_scalars_rounded> buffer{};
    distances.to_array_unaligned(buffer.data());
    for (std::size_t lane = 0; lane < lanes; ++lane) {
        if (buffer[lane] < 0.0f)
            lane_collision[lane] = true;
    }
}

struct ConfigPointerPack {
    std::array<std::vector<double>, kRake> sampled;
    std::array<const std::vector<double> *, kRake> pointers{};
    std::size_t lanes = 0;

    ConfigPointerPack() = default;
    ConfigPointerPack(const ConfigPointerPack &) = delete;
    ConfigPointerPack &operator=(const ConfigPointerPack &) = delete;

    ConfigPointerPack(ConfigPointerPack &&other) noexcept
        : sampled(std::move(other.sampled)), lanes(other.lanes) {
        bindPointers();
    }

    ConfigPointerPack &operator=(ConfigPointerPack &&other) noexcept {
        if (this == &other)
            return *this;
        sampled = std::move(other.sampled);
        lanes = other.lanes;
        bindPointers();
        return *this;
    }

    void bindPointers() {
        pointers.fill(nullptr);
        for (std::size_t lane = 0; lane < lanes; ++lane)
            pointers[lane] = &sampled[lane];
    }
};

ConfigPointerPack configPointersForPack(const Path &path,
                                        const BatchPack &pack) {
    ConfigPointerPack configs;
    configs.lanes = pack.lanes;
    for (std::size_t lane = 0; lane < pack.lanes; ++lane)
        configAt(path, pack.timesteps[lane], configs.sampled[lane]);
    configs.bindPointers();
    return configs;
}

template <typename RobotA, typename RobotB>
std::optional<std::size_t> firstCollidingLane(
    const RobotModel &robot_a,
    const std::array<const std::vector<double> *, kRake> &configs_a,
    const RobotModel &robot_b,
    const std::array<const std::vector<double> *, kRake> &configs_b,
    std::size_t lanes) {
    typename RobotA::template Spheres<kRake> spheres_a;
    typename RobotB::template Spheres<kRake> spheres_b;

    auto block_a = makeConfigurationBlock<RobotA>(configs_a, lanes);
    auto block_b = makeConfigurationBlock<RobotB>(configs_b, lanes);

    RobotA::template sphere_fk<kRake>(block_a, spheres_a);
    RobotB::template sphere_fk<kRake>(block_b, spheres_b);

    applyBaseTransform<RobotA>(robot_a.getBaseTransform().cast<float>(), spheres_a);
    applyBaseTransform<RobotB>(robot_b.getBaseTransform().cast<float>(), spheres_b);

    std::array<bool, kRake> lane_collision{};

    for (std::size_t i = 0; i < RobotA::n_spheres; ++i) {
        for (std::size_t j = 0; j < RobotB::n_spheres; ++j) {
            auto distances = vamp::collision::sphere_sphere_sql2(
                spheres_a.x[i], spheres_a.y[i], spheres_a.z[i], spheres_a.r[i],
                spheres_b.x[j], spheres_b.y[j], spheres_b.z[j], spheres_b.r[j]);
            markCollidingLanes(distances, lanes, lane_collision);
        }
    }

    for (std::size_t lane = 0; lane < lanes; ++lane) {
        if (lane_collision[lane]) {
            return lane;
        }
    }
    return std::nullopt;
}

template <typename RobotA, typename RobotB>
std::optional<std::size_t> earliestCollidingTimestepInPack(
    const RobotModel &robot_a,
    const std::array<const std::vector<double> *, kRake> &configs_a,
    const RobotModel &robot_b,
    const std::array<const std::vector<double> *, kRake> &configs_b,
    const BatchPack &pack) {
    typename RobotA::template Spheres<kRake> spheres_a;
    typename RobotB::template Spheres<kRake> spheres_b;

    auto block_a = makeConfigurationBlock<RobotA>(configs_a, pack.lanes);
    auto block_b = makeConfigurationBlock<RobotB>(configs_b, pack.lanes);

    RobotA::template sphere_fk<kRake>(block_a, spheres_a);
    RobotB::template sphere_fk<kRake>(block_b, spheres_b);

    applyBaseTransform<RobotA>(robot_a.getBaseTransform().cast<float>(), spheres_a);
    applyBaseTransform<RobotB>(robot_b.getBaseTransform().cast<float>(), spheres_b);

    std::array<bool, kRake> lane_collision{};
    for (std::size_t i = 0; i < RobotA::n_spheres; ++i) {
        for (std::size_t j = 0; j < RobotB::n_spheres; ++j) {
            auto distances = vamp::collision::sphere_sphere_sql2(
                spheres_a.x[i], spheres_a.y[i], spheres_a.z[i], spheres_a.r[i],
                spheres_b.x[j], spheres_b.y[j], spheres_b.z[j], spheres_b.r[j]);
            markCollidingLanes(distances, pack.lanes, lane_collision);
        }
    }

    std::optional<std::size_t> earliest;
    for (std::size_t lane = 0; lane < pack.lanes; ++lane) {
        if (!lane_collision[lane])
            continue;
        const std::size_t timestep = pack.timesteps[lane];
        earliest = earliest ? std::min(*earliest, timestep) : timestep;
    }
    return earliest;
}

template <typename Robot>
PackedRobotSpheres buildPackedRobotSpheresForPack(const RobotModel &robot,
                                                  const Path &path,
                                                  const BatchPack &pack) {
    if constexpr (std::is_same_v<Robot, vamp::robots::Sphere>)
        configureSphereRobot(robot);

    const auto configs = configPointersForPack(path, pack);
    auto block = makeConfigurationBlock<Robot>(configs.pointers, pack.lanes);
    typename Robot::template Spheres<kRake> spheres;
    Robot::template sphere_fk<kRake>(block, spheres);
    applyBaseTransform<Robot>(robot.getBaseTransform().cast<float>(), spheres);

    PackedRobotSpheres packed;
    packed.spheres.reserve(Robot::n_spheres);
    for (std::size_t i = 0; i < Robot::n_spheres; ++i) {
        packed.spheres.push_back(
            PackedSphereBlock{spheres.x[i], spheres.y[i], spheres.z[i],
                              spheres.r[i]});
    }
    return packed;
}

PackedRobotSpheres buildPackedRobotSpheresForPack(const RobotModel &robot,
                                                  const Path &path,
                                                  const BatchPack &pack) {
    if (path.empty() || pack.lanes == 0)
        return {};

    switch (robot.robotFamily()) {
    case RobotModel::RobotFamily::Sphere:
        return buildPackedRobotSpheresForPack<vamp::robots::Sphere>(
            robot, path, pack);
    case RobotModel::RobotFamily::Panda:
        return buildPackedRobotSpheresForPack<vamp::robots::Panda>(
            robot, path, pack);
    case RobotModel::RobotFamily::UR5:
        return buildPackedRobotSpheresForPack<vamp::robots::UR5>(
            robot, path, pack);
    case RobotModel::RobotFamily::Planar3:
        return buildPackedRobotSpheresForPack<vamp::robots::Planar3>(
            robot, path, pack);
    case RobotModel::RobotFamily::Unknown:
        throwUnsupportedRobotFamily(robot, "buildPackedRobotSpheresForPack");
    }
    return {};
}

std::optional<std::size_t> earliestPackedSphereConflictTimestep(
    const PackedRobotSpheres &spheres_a, const PackedRobotSpheres &spheres_b,
    const BatchPack &pack, std::size_t scan_begin = 0) {
    std::array<bool, kRake> lane_collision{};
    for (const auto &sa : spheres_a.spheres) {
        for (const auto &sb : spheres_b.spheres) {
            auto distances = vamp::collision::sphere_sphere_sql2(
                sa.x, sa.y, sa.z, sa.r, sb.x, sb.y, sb.z, sb.r);
            markCollidingLanes(distances, pack.lanes, lane_collision);
        }
    }

    std::optional<std::size_t> earliest;
    for (std::size_t lane = 0; lane < pack.lanes; ++lane) {
        if (!lane_collision[lane])
            continue;
        const std::size_t timestep = pack.timesteps[lane];
        if (timestep < scan_begin)
            continue;
        earliest = earliest ? std::min(*earliest, timestep) : timestep;
    }
    return earliest;
}

template <typename Robot>
bool isStateBlockValid(const RobotModel &robot,
                       const std::array<const std::vector<double> *, kRake> &configs,
                       std::size_t lanes,
                       const EnvironmentVector &environment) {
    if constexpr (std::is_same_v<Robot, vamp::robots::Sphere>)
        configureSphereRobot(robot);
    if (lanes == 0)
        return true;
    auto block = makeConfigurationBlock<Robot>(configs, lanes);
    return Robot::template fkcc<kRake>(environment, block);
}

template <typename Robot>
bool isRobotPathValidImpl(const RobotModel &robot, const Path &path,
                          const EnvironmentVector &environment,
                          VampBatchPacking packing) {
    if constexpr (std::is_same_v<Robot, vamp::robots::Sphere>)
        configureSphereRobot(robot);
    if (path.empty())
        return true;

    auto packs = makeBatchPacks(0, pathTimestepCount(path), packing);
    for (const auto &pack : packs) {
        const auto configs = configPointersForPack(path, pack);
        if (!isStateBlockValid<Robot>(robot, configs.pointers, pack.lanes,
                                      environment))
            return false;
    }
    return true;
}

template <typename Robot>
bool isRobotBatchValidImpl(const RobotModel &robot, const Path &path,
                           const EnvironmentVector &environment,
                           const std::vector<BatchPack> &packs) {
    if constexpr (std::is_same_v<Robot, vamp::robots::Sphere>)
        configureSphereRobot(robot);
    if (path.empty())
        return true;

    for (const auto &pack : packs) {
        const auto configs = configPointersForPack(path, pack);
        if (!isStateBlockValid<Robot>(robot, configs.pointers, pack.lanes,
                                      environment))
            return false;
    }
    return true;
}

template <typename Robot>
bool isRobotPackValidImpl(const RobotModel &robot, const Path &path,
                          const EnvironmentVector &environment,
                          const BatchPack &pack) {
    if constexpr (std::is_same_v<Robot, vamp::robots::Sphere>)
        configureSphereRobot(robot);
    if (path.empty() || pack.lanes == 0)
        return true;

    const auto configs = configPointersForPack(path, pack);
    return isStateBlockValid<Robot>(robot, configs.pointers, pack.lanes,
                                    environment);
}

template <typename Robot>
bool isSelfCollisionFreeImpl(const RobotModel &robot,
                             const std::vector<double> &config) {
    const EnvironmentVector empty_environment;
    std::array<const std::vector<double> *, kRake> configs{};
    configs[0] = &config;
    return isStateBlockValid<Robot>(robot, configs, 1, empty_environment);
}

template <typename Robot>
bool isValidSingleImpl(const RobotModel &robot,
                       const std::vector<double> &config,
                       const EnvironmentVector &environment) {
    std::array<const std::vector<double> *, kRake> configs{};
    configs[0] = &config;
    return isStateBlockValid<Robot>(robot, configs, 1, environment);
}

template <typename Robot>
bool isMotionValidImpl(const RobotModel &robot,
                       const std::vector<double> &from,
                       const std::vector<double> &to,
                       const EnvironmentVector &environment) {
    if constexpr (std::is_same_v<Robot, vamp::robots::Sphere>)
        configureSphereRobot(robot);
    return vamp::planning::validate_motion<Robot, kRake, Robot::resolution>(
        toConfiguration<Robot>(from), toConfiguration<Robot>(to), environment);
}

template <typename RobotA, typename RobotB>
bool isPairPathValidRaked(const RobotModel &robot_a, const Path &path_a,
                          const RobotModel &robot_b, const Path &path_b,
                          std::size_t t_begin, std::size_t t_end,
                          VampBatchPacking packing) {
    if (path_a.empty() || path_b.empty())
        return true;

    const std::size_t max_t =
        std::max(pathTimestepCount(path_a), pathTimestepCount(path_b));
    const std::size_t end = std::min(max_t, t_end);
    if (t_begin >= end)
        return true;

    const auto packs = makeBatchPacks(t_begin, end, packing);
    for (const auto &pack : packs) {
        const auto configs_a = configPointersForPack(path_a, pack);
        const auto configs_b = configPointersForPack(path_b, pack);
        if (firstCollidingLane<RobotA, RobotB>(
                robot_a, configs_a.pointers, robot_b, configs_b.pointers,
                pack.lanes)) {
            return false;
        }
    }

    return true;
}

template <typename RobotA, typename RobotB>
bool isPairPackValidRaked(const RobotModel &robot_a, const Path &path_a,
                          const RobotModel &robot_b, const Path &path_b,
                          const BatchPack &pack) {
    if (path_a.empty() || path_b.empty() || pack.lanes == 0)
        return true;

    const auto configs_a = configPointersForPack(path_a, pack);
    const auto configs_b = configPointersForPack(path_b, pack);
    return !firstCollidingLane<RobotA, RobotB>(
                robot_a, configs_a.pointers, robot_b, configs_b.pointers,
                pack.lanes)
                .has_value();
}

template <typename RobotA, typename RobotB>
std::optional<std::size_t> findFirstPairPackConflictTimestep(
    const RobotModel &robot_a, const Path &path_a, const RobotModel &robot_b,
    const Path &path_b, const BatchPack &pack) {
    if (path_a.empty() || path_b.empty() || pack.lanes == 0)
        return std::nullopt;

    const auto configs_a = configPointersForPack(path_a, pack);
    const auto configs_b = configPointersForPack(path_b, pack);
    return earliestCollidingTimestepInPack<RobotA, RobotB>(
        robot_a, configs_a.pointers, robot_b, configs_b.pointers, pack);
}

template <typename GoalRobot, typename PriorRobot>
GoalHoldConstraint computeGoalHoldConstraintRaked(
    const RobotModel &goal_robot,
    const std::vector<double> &goal_config,
    const RobotModel &prior_robot,
    const Path &prior_path) {
    if (prior_path.empty())
        return {};

    std::array<const std::vector<double> *, kRake> goal_configs{};
    goal_configs.fill(&goal_config);

    const std::size_t horizon = pathTimestepCount(prior_path);
    for (std::size_t remaining = horizon; remaining > 0;) {
        const std::size_t lanes = std::min<std::size_t>(kRake, remaining);
        std::array<const std::vector<double> *, kRake> prior_configs{};
        std::array<std::vector<double>, kRake> prior_samples{};
        for (std::size_t lane = 0; lane < lanes; ++lane) {
            const std::size_t timestep = remaining - 1 - lane;
            configAt(prior_path, timestep, prior_samples[lane]);
            prior_configs[lane] = &prior_samples[lane];
        }

        auto lane = firstCollidingLane<GoalRobot, PriorRobot>(
            goal_robot, goal_configs, prior_robot, prior_configs, lanes);
        if (lane) {
            const std::size_t conflict_timestep = remaining - 1 - *lane;
            if (conflict_timestep + 1 == horizon)
                return GoalHoldConstraint{0, true};
            return GoalHoldConstraint{conflict_timestep + 1, false};
        }

        remaining -= lanes;
    }

    return {};
}

struct SphereEnvironmentCache {
    std::size_t revision = 0;
    std::array<float, 16> base_transform{};
    bool valid = false;
    EnvironmentFloat env_float;
    EnvironmentVector env_vector;
};

std::array<float, 16> baseTransformKey(const RobotModel &robot) {
    std::array<float, 16> key{};
    const auto matrix = robot.getBaseTransform().matrix().cast<float>();
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            key[static_cast<std::size_t>(row * 4 + col)] = matrix(row, col);
        }
    }
    return key;
}

} // namespace

class VampCollisionBackend final : public CollisionBackend {
public:
    std::unique_ptr<CollisionBackend> clone() const override {
        auto copy = std::make_unique<VampCollisionBackend>();
        copy->world_spheres_ = world_spheres_;
        copy->world_cylinders_ = world_cylinders_;
        copy->environment_revision_ = environment_revision_;
        copy->strategy_ = strategy_;
        return copy;
    }

    void setVampValidationStrategy(
        const VampValidationStrategy &strategy) override {
        strategy_ = strategy;
    }

    VampValidationStrategy vampValidationStrategy() const override {
        return strategy_;
    }

    void onEnvironmentChanged(
        const std::vector<ObstacleSphere> &spheres,
        const std::vector<ObstacleCylinder> &cylinders) override {
        world_spheres_ = spheres;
        world_cylinders_ = cylinders;
        ++environment_revision_;
        sphere_env_cache_.clear();
    }

    bool isValidSingle(const RobotModel &robot,
                       const std::vector<double> &config,
                       const std::vector<ObstacleSphere> &,
                       const std::vector<ObstacleCylinder> &) const override {
        switch (robot.robotFamily()) {
        case RobotModel::RobotFamily::Sphere:
            return isValidSingleImpl<vamp::robots::Sphere>(
                robot, config, environmentForRobot(robot));
        case RobotModel::RobotFamily::Panda:
            return isValidSingleImpl<vamp::robots::Panda>(
                robot, config, environmentForRobot(robot));
        case RobotModel::RobotFamily::UR5:
            return isValidSingleImpl<vamp::robots::UR5>(
                robot, config, environmentForRobot(robot));
        case RobotModel::RobotFamily::Planar3:
            return isValidSingleImpl<vamp::robots::Planar3>(
                robot, config, environmentForRobot(robot));
        case RobotModel::RobotFamily::Unknown:
            throwUnsupportedRobotFamily(robot, "isValidSingle");
        }
        return false;
    }

    bool isSelfCollisionFree(
        const RobotModel &robot,
        const std::vector<double> &config) const override {
        switch (robot.robotFamily()) {
        case RobotModel::RobotFamily::Sphere:
            return isSelfCollisionFreeImpl<vamp::robots::Sphere>(robot, config);
        case RobotModel::RobotFamily::Panda:
            return isSelfCollisionFreeImpl<vamp::robots::Panda>(robot, config);
        case RobotModel::RobotFamily::UR5:
            return isSelfCollisionFreeImpl<vamp::robots::UR5>(robot, config);
        case RobotModel::RobotFamily::Planar3:
            return isSelfCollisionFreeImpl<vamp::robots::Planar3>(robot, config);
        case RobotModel::RobotFamily::Unknown:
            throwUnsupportedRobotFamily(robot, "isSelfCollisionFree");
        }
        return false;
    }

    bool isValidPair(const RobotModel &robot_a,
                     const std::vector<double> &config_a,
                     const RobotModel &robot_b,
                     const std::vector<double> &config_b) const override {
        Path path_a;
        Path path_b;
        path_a.push_back(config_a);
        path_b.push_back(config_b);
        return isPairPathValid(robot_a, path_a, robot_b, path_b, 0, 1);
    }

    bool isMotionValid(const RobotModel &robot,
                       const std::vector<double> &from,
                       const std::vector<double> &to, int,
                       const std::vector<ObstacleSphere> &,
                       const std::vector<ObstacleCylinder> &) const override {
        switch (robot.robotFamily()) {
        case RobotModel::RobotFamily::Sphere:
            return isMotionValidImpl<vamp::robots::Sphere>(
                robot, from, to, environmentForRobot(robot));
        case RobotModel::RobotFamily::Panda:
            return isMotionValidImpl<vamp::robots::Panda>(
                robot, from, to, environmentForRobot(robot));
        case RobotModel::RobotFamily::UR5:
            return isMotionValidImpl<vamp::robots::UR5>(
                robot, from, to, environmentForRobot(robot));
        case RobotModel::RobotFamily::Planar3:
            return isMotionValidImpl<vamp::robots::Planar3>(
                robot, from, to, environmentForRobot(robot));
        case RobotModel::RobotFamily::Unknown:
            throwUnsupportedRobotFamily(robot, "isMotionValid");
        }
        return false;
    }

    bool isRobotPathValid(const RobotModel &robot, const Path &path,
                          const std::vector<ObstacleSphere> &,
                          const std::vector<ObstacleCylinder> &) const override {
        switch (robot.robotFamily()) {
        case RobotModel::RobotFamily::Sphere:
            return isRobotPathValidImpl<vamp::robots::Sphere>(
                robot, path, environmentForRobot(robot), strategy_.packing);
        case RobotModel::RobotFamily::Panda:
            return isRobotPathValidImpl<vamp::robots::Panda>(
                robot, path, environmentForRobot(robot), strategy_.packing);
        case RobotModel::RobotFamily::UR5:
            return isRobotPathValidImpl<vamp::robots::UR5>(
                robot, path, environmentForRobot(robot), strategy_.packing);
        case RobotModel::RobotFamily::Planar3:
            return isRobotPathValidImpl<vamp::robots::Planar3>(
                robot, path, environmentForRobot(robot), strategy_.packing);
        case RobotModel::RobotFamily::Unknown:
            throwUnsupportedRobotFamily(robot, "isRobotPathValid");
        }
        return false;
    }

    bool isPairPathValid(const RobotModel &robot_a, const Path &path_a,
                         const RobotModel &robot_b, const Path &path_b,
                         std::size_t t_begin, std::size_t t_end) const override {
        if (path_a.empty() || path_b.empty())
            return true;

        if (robot_a.robotFamily() == RobotModel::RobotFamily::Sphere &&
            robot_b.robotFamily() == RobotModel::RobotFamily::Sphere) {
            ensureCompatibleSphereRobots(robot_a, robot_b);
        } else if (robot_a.robotFamily() == RobotModel::RobotFamily::Sphere) {
            configureSphereRobot(robot_a);
        } else if (robot_b.robotFamily() == RobotModel::RobotFamily::Sphere) {
            configureSphereRobot(robot_b);
        }

        switch (robot_a.robotFamily()) {
        case RobotModel::RobotFamily::Sphere:
            switch (robot_b.robotFamily()) {
            case RobotModel::RobotFamily::Sphere:
                return isPairPathValidRaked<vamp::robots::Sphere,
                                            vamp::robots::Sphere>(
                    robot_a, path_a, robot_b, path_b, t_begin, t_end,
                    strategy_.packing);
            case RobotModel::RobotFamily::Panda:
                return isPairPathValidRaked<vamp::robots::Sphere,
                                            vamp::robots::Panda>(
                    robot_a, path_a, robot_b, path_b, t_begin, t_end,
                    strategy_.packing);
            case RobotModel::RobotFamily::UR5:
                return isPairPathValidRaked<vamp::robots::Sphere,
                                            vamp::robots::UR5>(
                    robot_a, path_a, robot_b, path_b, t_begin, t_end,
                    strategy_.packing);
            case RobotModel::RobotFamily::Planar3:
                return isPairPathValidRaked<vamp::robots::Sphere,
                                            vamp::robots::Planar3>(
                    robot_a, path_a, robot_b, path_b, t_begin, t_end,
                    strategy_.packing);
            case RobotModel::RobotFamily::Unknown:
                throwUnsupportedRobotFamily(robot_b, "isPairPathValid");
            }
            break;
        case RobotModel::RobotFamily::Panda:
            switch (robot_b.robotFamily()) {
            case RobotModel::RobotFamily::Sphere:
                return isPairPathValidRaked<vamp::robots::Panda,
                                            vamp::robots::Sphere>(
                    robot_a, path_a, robot_b, path_b, t_begin, t_end,
                    strategy_.packing);
            case RobotModel::RobotFamily::Panda:
                return isPairPathValidRaked<vamp::robots::Panda,
                                            vamp::robots::Panda>(
                    robot_a, path_a, robot_b, path_b, t_begin, t_end,
                    strategy_.packing);
            case RobotModel::RobotFamily::UR5:
                return isPairPathValidRaked<vamp::robots::Panda,
                                            vamp::robots::UR5>(
                    robot_a, path_a, robot_b, path_b, t_begin, t_end,
                    strategy_.packing);
            case RobotModel::RobotFamily::Planar3:
                return isPairPathValidRaked<vamp::robots::Panda,
                                            vamp::robots::Planar3>(
                    robot_a, path_a, robot_b, path_b, t_begin, t_end,
                    strategy_.packing);
            case RobotModel::RobotFamily::Unknown:
                throwUnsupportedRobotFamily(robot_b, "isPairPathValid");
            }
            break;
        case RobotModel::RobotFamily::UR5:
            switch (robot_b.robotFamily()) {
            case RobotModel::RobotFamily::Sphere:
                return isPairPathValidRaked<vamp::robots::UR5,
                                            vamp::robots::Sphere>(
                    robot_a, path_a, robot_b, path_b, t_begin, t_end,
                    strategy_.packing);
            case RobotModel::RobotFamily::Panda:
                return isPairPathValidRaked<vamp::robots::UR5,
                                            vamp::robots::Panda>(
                    robot_a, path_a, robot_b, path_b, t_begin, t_end,
                    strategy_.packing);
            case RobotModel::RobotFamily::UR5:
                return isPairPathValidRaked<vamp::robots::UR5,
                                            vamp::robots::UR5>(
                    robot_a, path_a, robot_b, path_b, t_begin, t_end,
                    strategy_.packing);
            case RobotModel::RobotFamily::Planar3:
                return isPairPathValidRaked<vamp::robots::UR5,
                                            vamp::robots::Planar3>(
                    robot_a, path_a, robot_b, path_b, t_begin, t_end,
                    strategy_.packing);
            case RobotModel::RobotFamily::Unknown:
                throwUnsupportedRobotFamily(robot_b, "isPairPathValid");
            }
            break;
        case RobotModel::RobotFamily::Planar3:
            switch (robot_b.robotFamily()) {
            case RobotModel::RobotFamily::Sphere:
                return isPairPathValidRaked<vamp::robots::Planar3,
                                            vamp::robots::Sphere>(
                    robot_a, path_a, robot_b, path_b, t_begin, t_end,
                    strategy_.packing);
            case RobotModel::RobotFamily::Panda:
                return isPairPathValidRaked<vamp::robots::Planar3,
                                            vamp::robots::Panda>(
                    robot_a, path_a, robot_b, path_b, t_begin, t_end,
                    strategy_.packing);
            case RobotModel::RobotFamily::UR5:
                return isPairPathValidRaked<vamp::robots::Planar3,
                                            vamp::robots::UR5>(
                    robot_a, path_a, robot_b, path_b, t_begin, t_end,
                    strategy_.packing);
            case RobotModel::RobotFamily::Planar3:
                return isPairPathValidRaked<vamp::robots::Planar3,
                                            vamp::robots::Planar3>(
                    robot_a, path_a, robot_b, path_b, t_begin, t_end,
                    strategy_.packing);
            case RobotModel::RobotFamily::Unknown:
                throwUnsupportedRobotFamily(robot_b, "isPairPathValid");
            }
            break;
        case RobotModel::RobotFamily::Unknown:
            throwUnsupportedRobotFamily(robot_a, "isPairPathValid");
        }
        return false;
    }

    std::optional<PairPathConflict> findFirstPairPathConflict(
        const RobotModel &robot_a, const Path &path_a,
        const RobotModel &robot_b, const Path &path_b,
        std::size_t t_begin, std::size_t t_end) const override {
        if (path_a.empty() || path_b.empty())
            return std::nullopt;

        const std::size_t max_t =
            std::max(pathTimestepCount(path_a), pathTimestepCount(path_b));
        const std::size_t end = std::min(max_t, t_end);
        if (t_begin >= end)
            return std::nullopt;

        if (robot_a.robotFamily() == RobotModel::RobotFamily::Sphere &&
            robot_b.robotFamily() == RobotModel::RobotFamily::Sphere) {
            ensureCompatibleSphereRobots(robot_a, robot_b);
        } else if (robot_a.robotFamily() == RobotModel::RobotFamily::Sphere) {
            configureSphereRobot(robot_a);
        } else if (robot_b.robotFamily() == RobotModel::RobotFamily::Sphere) {
            configureSphereRobot(robot_b);
        }

        const auto packs = makeBatchPacks(t_begin, end, VampBatchPacking::Linear);
        for (const auto &pack : packs) {
            const auto configs_a = configPointersForPack(path_a, pack);
            const auto configs_b = configPointersForPack(path_b, pack);

            std::optional<std::size_t> pack_timestep;
            switch (robot_a.robotFamily()) {
            case RobotModel::RobotFamily::Sphere:
                switch (robot_b.robotFamily()) {
                case RobotModel::RobotFamily::Sphere:
                    pack_timestep =
                        earliestCollidingTimestepInPack<vamp::robots::Sphere,
                                                        vamp::robots::Sphere>(
                            robot_a, configs_a.pointers, robot_b,
                            configs_b.pointers, pack);
                    break;
                case RobotModel::RobotFamily::Panda:
                    pack_timestep =
                        earliestCollidingTimestepInPack<vamp::robots::Sphere,
                                                        vamp::robots::Panda>(
                            robot_a, configs_a.pointers, robot_b,
                            configs_b.pointers, pack);
                    break;
                case RobotModel::RobotFamily::UR5:
                    pack_timestep =
                        earliestCollidingTimestepInPack<vamp::robots::Sphere,
                                                        vamp::robots::UR5>(
                            robot_a, configs_a.pointers, robot_b,
                            configs_b.pointers, pack);
                    break;
                case RobotModel::RobotFamily::Planar3:
                    pack_timestep =
                        earliestCollidingTimestepInPack<vamp::robots::Sphere,
                                                        vamp::robots::Planar3>(
                            robot_a, configs_a.pointers, robot_b,
                            configs_b.pointers, pack);
                    break;
                case RobotModel::RobotFamily::Unknown:
                    throwUnsupportedRobotFamily(robot_b, "findFirstPairPathConflict");
                }
                break;
            case RobotModel::RobotFamily::Panda:
                switch (robot_b.robotFamily()) {
                case RobotModel::RobotFamily::Sphere:
                    pack_timestep =
                        earliestCollidingTimestepInPack<vamp::robots::Panda,
                                                        vamp::robots::Sphere>(
                            robot_a, configs_a.pointers, robot_b,
                            configs_b.pointers, pack);
                    break;
                case RobotModel::RobotFamily::Panda:
                    pack_timestep =
                        earliestCollidingTimestepInPack<vamp::robots::Panda,
                                                        vamp::robots::Panda>(
                            robot_a, configs_a.pointers, robot_b,
                            configs_b.pointers, pack);
                    break;
                case RobotModel::RobotFamily::UR5:
                    pack_timestep =
                        earliestCollidingTimestepInPack<vamp::robots::Panda,
                                                        vamp::robots::UR5>(
                            robot_a, configs_a.pointers, robot_b,
                            configs_b.pointers, pack);
                    break;
                case RobotModel::RobotFamily::Planar3:
                    pack_timestep =
                        earliestCollidingTimestepInPack<vamp::robots::Panda,
                                                        vamp::robots::Planar3>(
                            robot_a, configs_a.pointers, robot_b,
                            configs_b.pointers, pack);
                    break;
                case RobotModel::RobotFamily::Unknown:
                    throwUnsupportedRobotFamily(robot_b, "findFirstPairPathConflict");
                }
                break;
            case RobotModel::RobotFamily::UR5:
                switch (robot_b.robotFamily()) {
                case RobotModel::RobotFamily::Sphere:
                    pack_timestep =
                        earliestCollidingTimestepInPack<vamp::robots::UR5,
                                                        vamp::robots::Sphere>(
                            robot_a, configs_a.pointers, robot_b,
                            configs_b.pointers, pack);
                    break;
                case RobotModel::RobotFamily::Panda:
                    pack_timestep =
                        earliestCollidingTimestepInPack<vamp::robots::UR5,
                                                        vamp::robots::Panda>(
                            robot_a, configs_a.pointers, robot_b,
                            configs_b.pointers, pack);
                    break;
                case RobotModel::RobotFamily::UR5:
                    pack_timestep =
                        earliestCollidingTimestepInPack<vamp::robots::UR5,
                                                        vamp::robots::UR5>(
                            robot_a, configs_a.pointers, robot_b,
                            configs_b.pointers, pack);
                    break;
                case RobotModel::RobotFamily::Planar3:
                    pack_timestep =
                        earliestCollidingTimestepInPack<vamp::robots::UR5,
                                                        vamp::robots::Planar3>(
                            robot_a, configs_a.pointers, robot_b,
                            configs_b.pointers, pack);
                    break;
                case RobotModel::RobotFamily::Unknown:
                    throwUnsupportedRobotFamily(robot_b, "findFirstPairPathConflict");
                }
                break;
            case RobotModel::RobotFamily::Planar3:
                switch (robot_b.robotFamily()) {
                case RobotModel::RobotFamily::Sphere:
                    pack_timestep =
                        earliestCollidingTimestepInPack<vamp::robots::Planar3,
                                                        vamp::robots::Sphere>(
                            robot_a, configs_a.pointers, robot_b,
                            configs_b.pointers, pack);
                    break;
                case RobotModel::RobotFamily::Panda:
                    pack_timestep =
                        earliestCollidingTimestepInPack<vamp::robots::Planar3,
                                                        vamp::robots::Panda>(
                            robot_a, configs_a.pointers, robot_b,
                            configs_b.pointers, pack);
                    break;
                case RobotModel::RobotFamily::UR5:
                    pack_timestep =
                        earliestCollidingTimestepInPack<vamp::robots::Planar3,
                                                        vamp::robots::UR5>(
                            robot_a, configs_a.pointers, robot_b,
                            configs_b.pointers, pack);
                    break;
                case RobotModel::RobotFamily::Planar3:
                    pack_timestep =
                        earliestCollidingTimestepInPack<vamp::robots::Planar3,
                                                        vamp::robots::Planar3>(
                            robot_a, configs_a.pointers, robot_b,
                            configs_b.pointers, pack);
                    break;
                case RobotModel::RobotFamily::Unknown:
                    throwUnsupportedRobotFamily(robot_b, "findFirstPairPathConflict");
                }
                break;
            case RobotModel::RobotFamily::Unknown:
                throwUnsupportedRobotFamily(robot_a, "findFirstPairPathConflict");
            }

            if (pack_timestep) {
                return PairPathConflict{
                    *pack_timestep,
                    0.0,
                    ConflictKind::Vertex,
                    configAt(path_a, *pack_timestep),
                    configAt(path_b, *pack_timestep),
                };
            }
        }

        return std::nullopt;
    }

    std::optional<PairPathConflict> findFirstPairPackConflict(
        const RobotModel &robot_a, const Path &path_a,
        const RobotModel &robot_b, const Path &path_b,
        const BatchPack &pack) const {
        if (path_a.empty() || path_b.empty() || pack.lanes == 0)
            return std::nullopt;

        if (robot_a.robotFamily() == RobotModel::RobotFamily::Sphere &&
            robot_b.robotFamily() == RobotModel::RobotFamily::Sphere) {
            ensureCompatibleSphereRobots(robot_a, robot_b);
        } else if (robot_a.robotFamily() == RobotModel::RobotFamily::Sphere) {
            configureSphereRobot(robot_a);
        } else if (robot_b.robotFamily() == RobotModel::RobotFamily::Sphere) {
            configureSphereRobot(robot_b);
        }

        std::optional<std::size_t> timestep;
        switch (robot_a.robotFamily()) {
        case RobotModel::RobotFamily::Sphere:
            switch (robot_b.robotFamily()) {
            case RobotModel::RobotFamily::Sphere:
                timestep =
                    findFirstPairPackConflictTimestep<vamp::robots::Sphere,
                                                      vamp::robots::Sphere>(
                        robot_a, path_a, robot_b, path_b, pack);
                break;
            case RobotModel::RobotFamily::Panda:
                timestep =
                    findFirstPairPackConflictTimestep<vamp::robots::Sphere,
                                                      vamp::robots::Panda>(
                        robot_a, path_a, robot_b, path_b, pack);
                break;
            case RobotModel::RobotFamily::UR5:
                timestep =
                    findFirstPairPackConflictTimestep<vamp::robots::Sphere,
                                                      vamp::robots::UR5>(
                        robot_a, path_a, robot_b, path_b, pack);
                break;
            case RobotModel::RobotFamily::Planar3:
                timestep =
                    findFirstPairPackConflictTimestep<vamp::robots::Sphere,
                                                      vamp::robots::Planar3>(
                        robot_a, path_a, robot_b, path_b, pack);
                break;
            case RobotModel::RobotFamily::Unknown:
                throwUnsupportedRobotFamily(robot_b, "findFirstPairPackConflict");
            }
            break;
        case RobotModel::RobotFamily::Panda:
            switch (robot_b.robotFamily()) {
            case RobotModel::RobotFamily::Sphere:
                timestep =
                    findFirstPairPackConflictTimestep<vamp::robots::Panda,
                                                      vamp::robots::Sphere>(
                        robot_a, path_a, robot_b, path_b, pack);
                break;
            case RobotModel::RobotFamily::Panda:
                timestep =
                    findFirstPairPackConflictTimestep<vamp::robots::Panda,
                                                      vamp::robots::Panda>(
                        robot_a, path_a, robot_b, path_b, pack);
                break;
            case RobotModel::RobotFamily::UR5:
                timestep =
                    findFirstPairPackConflictTimestep<vamp::robots::Panda,
                                                      vamp::robots::UR5>(
                        robot_a, path_a, robot_b, path_b, pack);
                break;
            case RobotModel::RobotFamily::Planar3:
                timestep =
                    findFirstPairPackConflictTimestep<vamp::robots::Panda,
                                                      vamp::robots::Planar3>(
                        robot_a, path_a, robot_b, path_b, pack);
                break;
            case RobotModel::RobotFamily::Unknown:
                throwUnsupportedRobotFamily(robot_b, "findFirstPairPackConflict");
            }
            break;
        case RobotModel::RobotFamily::UR5:
            switch (robot_b.robotFamily()) {
            case RobotModel::RobotFamily::Sphere:
                timestep =
                    findFirstPairPackConflictTimestep<vamp::robots::UR5,
                                                      vamp::robots::Sphere>(
                        robot_a, path_a, robot_b, path_b, pack);
                break;
            case RobotModel::RobotFamily::Panda:
                timestep =
                    findFirstPairPackConflictTimestep<vamp::robots::UR5,
                                                      vamp::robots::Panda>(
                        robot_a, path_a, robot_b, path_b, pack);
                break;
            case RobotModel::RobotFamily::UR5:
                timestep =
                    findFirstPairPackConflictTimestep<vamp::robots::UR5,
                                                      vamp::robots::UR5>(
                        robot_a, path_a, robot_b, path_b, pack);
                break;
            case RobotModel::RobotFamily::Planar3:
                timestep =
                    findFirstPairPackConflictTimestep<vamp::robots::UR5,
                                                      vamp::robots::Planar3>(
                        robot_a, path_a, robot_b, path_b, pack);
                break;
            case RobotModel::RobotFamily::Unknown:
                throwUnsupportedRobotFamily(robot_b, "findFirstPairPackConflict");
            }
            break;
        case RobotModel::RobotFamily::Planar3:
            switch (robot_b.robotFamily()) {
            case RobotModel::RobotFamily::Sphere:
                timestep =
                    findFirstPairPackConflictTimestep<vamp::robots::Planar3,
                                                      vamp::robots::Sphere>(
                        robot_a, path_a, robot_b, path_b, pack);
                break;
            case RobotModel::RobotFamily::Panda:
                timestep =
                    findFirstPairPackConflictTimestep<vamp::robots::Planar3,
                                                      vamp::robots::Panda>(
                        robot_a, path_a, robot_b, path_b, pack);
                break;
            case RobotModel::RobotFamily::UR5:
                timestep =
                    findFirstPairPackConflictTimestep<vamp::robots::Planar3,
                                                      vamp::robots::UR5>(
                        robot_a, path_a, robot_b, path_b, pack);
                break;
            case RobotModel::RobotFamily::Planar3:
                timestep =
                    findFirstPairPackConflictTimestep<vamp::robots::Planar3,
                                                      vamp::robots::Planar3>(
                        robot_a, path_a, robot_b, path_b, pack);
                break;
            case RobotModel::RobotFamily::Unknown:
                throwUnsupportedRobotFamily(robot_b, "findFirstPairPackConflict");
            }
            break;
        case RobotModel::RobotFamily::Unknown:
            throwUnsupportedRobotFamily(robot_a, "findFirstPairPackConflict");
        }

        if (!timestep)
            return std::nullopt;
        return PairPathConflict{*timestep,
                                0.0,
                                ConflictKind::Vertex,
                                configAt(path_a, *timestep),
                                configAt(path_b, *timestep)};
    }

    GoalHoldConstraint computeGoalHoldConstraint(
        const RobotModel &goal_robot,
        const std::vector<double> &goal_config,
        const RobotModel &prior_robot,
        const Path &prior_path) const override {
        if (prior_path.empty())
            return {};

        if (goal_robot.robotFamily() == RobotModel::RobotFamily::Sphere &&
            prior_robot.robotFamily() == RobotModel::RobotFamily::Sphere) {
            ensureCompatibleSphereRobots(goal_robot, prior_robot);
        } else if (goal_robot.robotFamily() == RobotModel::RobotFamily::Sphere) {
            configureSphereRobot(goal_robot);
        } else if (prior_robot.robotFamily() == RobotModel::RobotFamily::Sphere) {
            configureSphereRobot(prior_robot);
        }

        switch (goal_robot.robotFamily()) {
        case RobotModel::RobotFamily::Sphere:
            switch (prior_robot.robotFamily()) {
            case RobotModel::RobotFamily::Sphere:
                return computeGoalHoldConstraintRaked<vamp::robots::Sphere,
                                                      vamp::robots::Sphere>(
                    goal_robot, goal_config, prior_robot, prior_path);
            case RobotModel::RobotFamily::Panda:
                return computeGoalHoldConstraintRaked<vamp::robots::Sphere,
                                                      vamp::robots::Panda>(
                    goal_robot, goal_config, prior_robot, prior_path);
            case RobotModel::RobotFamily::UR5:
                return computeGoalHoldConstraintRaked<vamp::robots::Sphere,
                                                      vamp::robots::UR5>(
                    goal_robot, goal_config, prior_robot, prior_path);
            case RobotModel::RobotFamily::Planar3:
                return computeGoalHoldConstraintRaked<vamp::robots::Sphere,
                                                      vamp::robots::Planar3>(
                    goal_robot, goal_config, prior_robot, prior_path);
            case RobotModel::RobotFamily::Unknown:
                throwUnsupportedRobotFamily(prior_robot, "computeGoalHoldConstraint");
            }
            break;
        case RobotModel::RobotFamily::Panda:
            switch (prior_robot.robotFamily()) {
            case RobotModel::RobotFamily::Sphere:
                return computeGoalHoldConstraintRaked<vamp::robots::Panda,
                                                      vamp::robots::Sphere>(
                    goal_robot, goal_config, prior_robot, prior_path);
            case RobotModel::RobotFamily::Panda:
                return computeGoalHoldConstraintRaked<vamp::robots::Panda,
                                                      vamp::robots::Panda>(
                    goal_robot, goal_config, prior_robot, prior_path);
            case RobotModel::RobotFamily::UR5:
                return computeGoalHoldConstraintRaked<vamp::robots::Panda,
                                                      vamp::robots::UR5>(
                    goal_robot, goal_config, prior_robot, prior_path);
            case RobotModel::RobotFamily::Planar3:
                return computeGoalHoldConstraintRaked<vamp::robots::Panda,
                                                      vamp::robots::Planar3>(
                    goal_robot, goal_config, prior_robot, prior_path);
            case RobotModel::RobotFamily::Unknown:
                throwUnsupportedRobotFamily(prior_robot, "computeGoalHoldConstraint");
            }
            break;
        case RobotModel::RobotFamily::UR5:
            switch (prior_robot.robotFamily()) {
            case RobotModel::RobotFamily::Sphere:
                return computeGoalHoldConstraintRaked<vamp::robots::UR5,
                                                      vamp::robots::Sphere>(
                    goal_robot, goal_config, prior_robot, prior_path);
            case RobotModel::RobotFamily::Panda:
                return computeGoalHoldConstraintRaked<vamp::robots::UR5,
                                                      vamp::robots::Panda>(
                    goal_robot, goal_config, prior_robot, prior_path);
            case RobotModel::RobotFamily::UR5:
                return computeGoalHoldConstraintRaked<vamp::robots::UR5,
                                                      vamp::robots::UR5>(
                    goal_robot, goal_config, prior_robot, prior_path);
            case RobotModel::RobotFamily::Planar3:
                return computeGoalHoldConstraintRaked<vamp::robots::UR5,
                                                      vamp::robots::Planar3>(
                    goal_robot, goal_config, prior_robot, prior_path);
            case RobotModel::RobotFamily::Unknown:
                throwUnsupportedRobotFamily(prior_robot, "computeGoalHoldConstraint");
            }
            break;
        case RobotModel::RobotFamily::Planar3:
            switch (prior_robot.robotFamily()) {
            case RobotModel::RobotFamily::Sphere:
                return computeGoalHoldConstraintRaked<vamp::robots::Planar3,
                                                      vamp::robots::Sphere>(
                    goal_robot, goal_config, prior_robot, prior_path);
            case RobotModel::RobotFamily::Panda:
                return computeGoalHoldConstraintRaked<vamp::robots::Planar3,
                                                      vamp::robots::Panda>(
                    goal_robot, goal_config, prior_robot, prior_path);
            case RobotModel::RobotFamily::UR5:
                return computeGoalHoldConstraintRaked<vamp::robots::Planar3,
                                                      vamp::robots::UR5>(
                    goal_robot, goal_config, prior_robot, prior_path);
            case RobotModel::RobotFamily::Planar3:
                return computeGoalHoldConstraintRaked<vamp::robots::Planar3,
                                                      vamp::robots::Planar3>(
                    goal_robot, goal_config, prior_robot, prior_path);
            case RobotModel::RobotFamily::Unknown:
                throwUnsupportedRobotFamily(prior_robot, "computeGoalHoldConstraint");
            }
            break;
        case RobotModel::RobotFamily::Unknown:
            throwUnsupportedRobotFamily(goal_robot, "computeGoalHoldConstraint");
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
        if (robots.size() != from.size() || robots.size() != to.size())
            return false;

        const int num_checks =
            options.discrete_num_checks_hint > 0 ? options.discrete_num_checks_hint : 10;
        auto paths = makeCompositeMotionPaths(from, to, num_checks);
        return validateCompositePaths(paths, robots, options, obstacles, cylinders);
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
        auto paths = makeCompositeMotionPaths(from, to, num_checks);
        CompositePathValidationOptions path_options = options;
        path_options.t_begin = 0;
        path_options.t_end = std::numeric_limits<std::size_t>::max();
        return findFirstCompositePathConflict(paths, robots, path_options, obstacles,
                                              cylinders, nullptr);
    }

    bool validateCompositePaths(
        const std::vector<Path> &paths,
        const std::vector<const RobotModel *> &robots,
        const CompositePathValidationOptions &options,
        const std::vector<ObstacleSphere> &obstacles,
        const std::vector<ObstacleCylinder> &cylinders) const override {
        (void)obstacles;
        (void)cylinders;
        if (paths.size() != robots.size())
            return false;

        const auto effective_starts = effectivePathStarts(paths.size(), options);
        const auto effective_pair_starts =
            effectivePairStarts(paths.size(), options, effective_starts);
        const std::size_t max_t = maxPathLength(paths);
        const std::size_t end = std::min(max_t, options.t_end);
        std::size_t scan_begin = end;
        if (options.check_environment && !effective_starts.empty()) {
            scan_begin =
                *std::min_element(effective_starts.begin(),
                                  effective_starts.end());
        }
        if (!effective_pair_starts.empty()) {
            scan_begin = std::min(
                scan_begin, *std::min_element(effective_pair_starts.begin(),
                                              effective_pair_starts.end()));
        }
        if (scan_begin >= end)
            return true;
        const auto packs =
            makeBatchPacks(scan_begin, end, strategy_.packing);

        if (strategy_.ordering == VampBatchOrdering::Hierarchical) {
            if (options.check_environment) {
                for (const auto &pack : packs) {
                    for (std::size_t i = 0; i < paths.size(); ++i) {
                        const auto robot_pack =
                            filterPackAtOrAfter(pack, effective_starts[i]);
                        if (robot_pack.lanes == 0)
                            continue;
                        if (!isRobotPackValid(*robots[i], paths[i],
                                              robot_pack)) {
                            return false;
                        }
                    }
                }
            }

            for (const auto &pack : packs) {
                for (std::size_t i = 0; i < paths.size(); ++i) {
                    for (std::size_t j = i + 1; j < paths.size(); ++j) {
                        const auto pair_pack = filterPackAtOrAfter(
                            pack, effective_pair_starts[pairFrontierIndex(
                                      i, j, paths.size())]);
                        if (pair_pack.lanes == 0)
                            continue;
                        if (!isPairPackValid(*robots[i], paths[i], *robots[j],
                                             paths[j], pair_pack)) {
                            return false;
                        }
                    }
                }
            }
            return true;
        }

        for (const auto &pack : packs) {
            if (options.check_environment) {
                for (std::size_t i = 0; i < paths.size(); ++i) {
                    const auto robot_pack =
                        filterPackAtOrAfter(pack, effective_starts[i]);
                    if (robot_pack.lanes == 0)
                        continue;
                    if (!isRobotPackValid(*robots[i], paths[i], robot_pack)) {
                        return false;
                    }
                }
            }

            for (std::size_t i = 0; i < paths.size(); ++i) {
                for (std::size_t j = i + 1; j < paths.size(); ++j) {
                    const auto pair_pack = filterPackAtOrAfter(
                        pack, effective_pair_starts[pairFrontierIndex(
                                  i, j, paths.size())]);
                    if (pair_pack.lanes == 0)
                        continue;
                    if (!isPairPackValid(*robots[i], paths[i], *robots[j],
                                         paths[j], pair_pack)) {
                        return false;
                    }
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
        (void)obstacles;
        (void)cylinders;
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
        std::size_t conflict_scan_begin =
            *std::min_element(effective_starts.begin(), effective_starts.end());
        if (!options.check_environment && !effective_pair_starts.empty()) {
            conflict_scan_begin =
                *std::min_element(effective_pair_starts.begin(),
                                  effective_pair_starts.end());
        } else if (!effective_pair_starts.empty()) {
            conflict_scan_begin = std::min(
                conflict_scan_begin,
                *std::min_element(effective_pair_starts.begin(),
                                  effective_pair_starts.end()));
        }
        if (conflict_scan_begin >= end) {
            return std::nullopt;
        }

        const auto packs =
            makeBatchPacks(conflict_scan_begin, end, VampBatchPacking::Linear);

        if (strategy_.ordering == VampBatchOrdering::Hierarchical) {
            if (options.check_environment) {
                for (const auto &pack : packs) {
                    auto conflict = findPackEnvironmentConflict(paths, robots, pack);
                    if (conflict) {
                        return conflict;
                    }
                }
            }

            for (const auto &pack : packs) {
                auto conflict = findPackInterRobotConflict(
                    paths, robots, pack, &effective_pair_starts);
                if (conflict) {
                    return conflict;
                }
            }
            return std::nullopt;
        }

        for (const auto &pack : packs) {
            std::optional<CompositeConflict> best;
            if (options.check_environment) {
                auto conflict = findPackEnvironmentConflict(paths, robots, pack);
                if (conflict)
                    best = std::move(conflict);
            }

            auto conflict = findPackInterRobotConflict(
                paths, robots, pack, &effective_pair_starts);
            if (conflict &&
                (!best || conflict->timestep < best->timestep)) {
                best = std::move(conflict);
            }
            if (best) {
                return best;
            }
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
        const auto packs =
            makeBatchPacks(global_begin, end, VampBatchPacking::Linear);

        std::vector<char> robot_used(paths.size(), 0);
        std::vector<char> pair_has_accepted(effective_pair_starts.size(), 0);
        std::vector<AcceptedInterRobotConflictClaim> accepted_claims;
        struct Candidate {
            CompositeConflict conflict;
            std::size_t pair_index = 0;
        };

        for (const auto &pack : packs) {
            std::vector<Candidate> candidates;

            for (std::size_t i = 0; i < paths.size(); ++i) {
                if (optimistic_unique && robot_used[i] != 0)
                    continue;
                for (std::size_t j = i + 1; j < paths.size(); ++j) {
                    if (optimistic_unique && robot_used[j] != 0)
                        continue;
                    const std::size_t pair_index =
                        pairFrontierIndex(i, j, paths.size());
                    std::size_t pair_begin = effective_pair_starts[pair_index];
                    const std::size_t pack_last =
                        pack.timesteps[pack.lanes - 1];
                    if (pair_begin > pack_last)
                        continue;
                    bool pair_had_candidate = false;
                    while (pair_begin <= pack_last) {
                        const BatchPack pair_pack =
                            filterPackAtOrAfter(pack, pair_begin);
                        if (pair_pack.lanes == 0)
                            break;
                        auto conflict = findFirstPairPackConflict(
                            *robots[i], paths[i], *robots[j], paths[j],
                            pair_pack);
                        if (!conflict)
                            break;
                        pair_had_candidate = true;
                        candidates.push_back(Candidate{CompositeConflict{
                            ConflictScope::InterRobot, static_cast<int>(i),
                            static_cast<int>(j), conflict->timestep,
                            conflict->alpha, conflict->kind,
                            conflict->config_a, conflict->config_b},
                            pair_index});
                        if (optimistic_unique || max_conflicts == 1)
                            break;
                        pair_begin = conflict->timestep + 1;
                    }
                    if (optimistic_unique && !pair_had_candidate &&
                        next_t_begin_by_pair_out &&
                        !pair_has_accepted[pair_index]) {
                        (*next_t_begin_by_pair_out)[pair_index] =
                            std::min(end, pack_last + 1);
                    }
                }
            }

            std::sort(candidates.begin(), candidates.end(),
                      [](const Candidate &lhs, const Candidate &rhs) {
                          if (lhs.conflict.timestep != rhs.conflict.timestep)
                              return lhs.conflict.timestep <
                                     rhs.conflict.timestep;
                          if (lhs.conflict.robot_i != rhs.conflict.robot_i)
                              return lhs.conflict.robot_i <
                                     rhs.conflict.robot_i;
                          return lhs.conflict.robot_j < rhs.conflict.robot_j;
                      });

            for (const auto &candidate : candidates) {
                const auto &conflict = candidate.conflict;
                if (optimistic_unique &&
                    (robot_used[static_cast<std::size_t>(conflict.robot_i)] ||
                     robot_used[static_cast<std::size_t>(conflict.robot_j)])) {
                    continue;
                }
                if (acceptInterRobotConflictCandidate(
                        conflict, on_conflict, unique, robot_used, out,
                        accepted_claims,
                        options.inter_robot_conflict_batch_mode)) {
                    if (optimistic_unique && next_t_begin_by_pair_out &&
                        !pair_has_accepted[candidate.pair_index]) {
                        (*next_t_begin_by_pair_out)[candidate.pair_index] =
                            conflict.timestep;
                        pair_has_accepted[candidate.pair_index] = 1;
                    }
                    if (max_conflicts > 0 && out.size() >= max_conflicts) {
                        if (unique) {
                            assignUniqueConflictFrontier(
                                next_t_begin_by_robot_out, effective_starts,
                                accepted_claims);
                        } else if (next_t_begin_by_robot_out) {
                            const std::size_t next_t = conflict.timestep + 1;
                            for (std::size_t robot = 0; robot < paths.size();
                                 ++robot) {
                                if (effective_starts[robot] <= conflict.timestep) {
                                    (*next_t_begin_by_robot_out)[robot] = next_t;
                                }
                            }
                        }
                        return out;
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

    std::optional<std::size_t> findFirstPairPackConflictTimestepNoStats(
        const RobotModel &robot_a, const Path &path_a,
        const RobotModel &robot_b, const Path &path_b,
        const BatchPack &pack) const {
        if (path_a.empty() || path_b.empty() || pack.lanes == 0)
            return std::nullopt;

        if (robot_a.robotFamily() == RobotModel::RobotFamily::Sphere &&
            robot_b.robotFamily() == RobotModel::RobotFamily::Sphere) {
            ensureCompatibleSphereRobots(robot_a, robot_b);
        } else if (robot_a.robotFamily() == RobotModel::RobotFamily::Sphere) {
            configureSphereRobot(robot_a);
        } else if (robot_b.robotFamily() == RobotModel::RobotFamily::Sphere) {
            configureSphereRobot(robot_b);
        }

        switch (robot_a.robotFamily()) {
        case RobotModel::RobotFamily::Sphere:
            switch (robot_b.robotFamily()) {
            case RobotModel::RobotFamily::Sphere:
                return findFirstPairPackConflictTimestep<vamp::robots::Sphere,
                                                         vamp::robots::Sphere>(
                    robot_a, path_a, robot_b, path_b, pack);
            case RobotModel::RobotFamily::Panda:
                return findFirstPairPackConflictTimestep<vamp::robots::Sphere,
                                                         vamp::robots::Panda>(
                    robot_a, path_a, robot_b, path_b, pack);
            case RobotModel::RobotFamily::UR5:
                return findFirstPairPackConflictTimestep<vamp::robots::Sphere,
                                                         vamp::robots::UR5>(
                    robot_a, path_a, robot_b, path_b, pack);
            case RobotModel::RobotFamily::Planar3:
                return findFirstPairPackConflictTimestep<vamp::robots::Sphere,
                                                         vamp::robots::Planar3>(
                    robot_a, path_a, robot_b, path_b, pack);
            case RobotModel::RobotFamily::Unknown:
                throwUnsupportedRobotFamily(
                    robot_b, "findFirstPairPackConflictTimestepNoStats");
            }
            break;
        case RobotModel::RobotFamily::Panda:
            switch (robot_b.robotFamily()) {
            case RobotModel::RobotFamily::Sphere:
                return findFirstPairPackConflictTimestep<vamp::robots::Panda,
                                                         vamp::robots::Sphere>(
                    robot_a, path_a, robot_b, path_b, pack);
            case RobotModel::RobotFamily::Panda:
                return findFirstPairPackConflictTimestep<vamp::robots::Panda,
                                                         vamp::robots::Panda>(
                    robot_a, path_a, robot_b, path_b, pack);
            case RobotModel::RobotFamily::UR5:
                return findFirstPairPackConflictTimestep<vamp::robots::Panda,
                                                         vamp::robots::UR5>(
                    robot_a, path_a, robot_b, path_b, pack);
            case RobotModel::RobotFamily::Planar3:
                return findFirstPairPackConflictTimestep<vamp::robots::Panda,
                                                         vamp::robots::Planar3>(
                    robot_a, path_a, robot_b, path_b, pack);
            case RobotModel::RobotFamily::Unknown:
                throwUnsupportedRobotFamily(
                    robot_b, "findFirstPairPackConflictTimestepNoStats");
            }
            break;
        case RobotModel::RobotFamily::UR5:
            switch (robot_b.robotFamily()) {
            case RobotModel::RobotFamily::Sphere:
                return findFirstPairPackConflictTimestep<vamp::robots::UR5,
                                                         vamp::robots::Sphere>(
                    robot_a, path_a, robot_b, path_b, pack);
            case RobotModel::RobotFamily::Panda:
                return findFirstPairPackConflictTimestep<vamp::robots::UR5,
                                                         vamp::robots::Panda>(
                    robot_a, path_a, robot_b, path_b, pack);
            case RobotModel::RobotFamily::UR5:
                return findFirstPairPackConflictTimestep<vamp::robots::UR5,
                                                         vamp::robots::UR5>(
                    robot_a, path_a, robot_b, path_b, pack);
            case RobotModel::RobotFamily::Planar3:
                return findFirstPairPackConflictTimestep<vamp::robots::UR5,
                                                         vamp::robots::Planar3>(
                    robot_a, path_a, robot_b, path_b, pack);
            case RobotModel::RobotFamily::Unknown:
                throwUnsupportedRobotFamily(
                    robot_b, "findFirstPairPackConflictTimestepNoStats");
            }
            break;
        case RobotModel::RobotFamily::Planar3:
            switch (robot_b.robotFamily()) {
            case RobotModel::RobotFamily::Sphere:
                return findFirstPairPackConflictTimestep<vamp::robots::Planar3,
                                                         vamp::robots::Sphere>(
                    robot_a, path_a, robot_b, path_b, pack);
            case RobotModel::RobotFamily::Panda:
                return findFirstPairPackConflictTimestep<vamp::robots::Planar3,
                                                         vamp::robots::Panda>(
                    robot_a, path_a, robot_b, path_b, pack);
            case RobotModel::RobotFamily::UR5:
                return findFirstPairPackConflictTimestep<vamp::robots::Planar3,
                                                         vamp::robots::UR5>(
                    robot_a, path_a, robot_b, path_b, pack);
            case RobotModel::RobotFamily::Planar3:
                return findFirstPairPackConflictTimestep<vamp::robots::Planar3,
                                                         vamp::robots::Planar3>(
                    robot_a, path_a, robot_b, path_b, pack);
            case RobotModel::RobotFamily::Unknown:
                throwUnsupportedRobotFamily(
                    robot_b, "findFirstPairPackConflictTimestepNoStats");
            }
            break;
        case RobotModel::RobotFamily::Unknown:
            throwUnsupportedRobotFamily(
                robot_a, "findFirstPairPackConflictTimestepNoStats");
        }
        return std::nullopt;
    }

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
            "Process-parallel VAMP conflict finding requires POSIX fork support");
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
        constexpr std::size_t kInvalidRobotSlot =
            std::numeric_limits<std::size_t>::max();
        std::vector<std::vector<WorkPair>> worker_pairs(worker_count);
        std::vector<std::vector<std::size_t>> worker_robots(worker_count);
        std::vector<std::vector<std::size_t>> worker_robot_slots(
            worker_count,
            std::vector<std::size_t>(paths.size(), kInvalidRobotSlot));

        for (std::size_t worker = 0; worker < worker_count; ++worker) {
            worker_robots[worker].reserve(paths.size());
            for (std::size_t robot = 0; robot < paths.size(); ++robot) {
                worker_robot_slots[worker][robot] =
                    worker_robots[worker].size();
                worker_robots[worker].push_back(robot);
            }
        }
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

                struct EligiblePair {
                    const WorkPair *pair = nullptr;
                    std::size_t scan_begin = 0;
                };
                std::vector<EligiblePair> eligible_pairs;
                eligible_pairs.reserve(worker_pairs[worker_index].size());

                for (const auto &pair : worker_pairs[worker_index]) {
                    if (optimistic_unique &&
                        (claimed_robots[pair.robot_i] ||
                         claimed_robots[pair.robot_j])) {
                        continue;
                    }

                    const std::size_t pair_start =
                        effective_pair_starts[pair.pair_index];
                    if (pair_start >= segment_end) {
                        continue;
                    }
                    const std::size_t scan_begin =
                        std::max(pair_start, segment_begin);
                    eligible_pairs.push_back(EligiblePair{&pair, scan_begin});
                }

                std::vector<char> pair_has_candidate(
                    effective_pair_starts.size(), 0);
                const auto &local_robots = worker_robots[worker_index];
                const auto pack_build_wall_start = Clock::now();
                const double pack_build_cpu_start = processCpuSeconds();
                const auto packs = makeBatchPacks(
                    segment_begin, segment_end, VampBatchPacking::Linear);
                build_worker_wall_seconds +=
                    elapsedWallSeconds(pack_build_wall_start);
                build_worker_cpu_seconds +=
                    elapsedProcessCpuSeconds(pack_build_cpu_start);
                for (const auto &pack : packs) {
                    std::vector<PackedRobotSpheres> packed_cache(
                        local_robots.size());
                    std::vector<char> packed_cached(local_robots.size(), 0);
                    const auto packedFor = [&](std::size_t robot)
                        -> const PackedRobotSpheres & {
                        const std::size_t local_slot =
                            worker_robot_slots[worker_index][robot];
                        if (local_slot == kInvalidRobotSlot) {
                            throw std::runtime_error(
                                "VAMP conflict worker missing assigned robot "
                                "intermediate slot");
                        }
                        if (!packed_cached[local_slot]) {
                            const auto build_wall_start = Clock::now();
                            const double build_cpu_start = processCpuSeconds();
                            packed_cache[local_slot] =
                                buildPackedRobotSpheresForPack(
                                    *robots[robot], paths[robot], pack);
                            build_worker_wall_seconds +=
                                elapsedWallSeconds(build_wall_start);
                            build_worker_cpu_seconds +=
                                elapsedProcessCpuSeconds(build_cpu_start);
                            packed_cached[local_slot] = 1;
                        }
                        return packed_cache[local_slot];
                    };

                    const std::size_t pack_last = pack.timesteps[pack.lanes - 1];
                    for (const auto &eligible : eligible_pairs) {
                        const WorkPair &pair = *eligible.pair;
                        if (optimistic_unique &&
                            pair_has_candidate[pair.pair_index])
                            continue;
                        if (eligible.scan_begin > pack_last)
                            continue;

                        const auto &spheres_i = packedFor(pair.robot_i);
                        const auto &spheres_j = packedFor(pair.robot_j);
                        std::size_t scan_begin = eligible.scan_begin;
                        while (scan_begin <= pack_last) {
                            const auto collision_wall_start = Clock::now();
                            const double collision_cpu_start =
                                processCpuSeconds();
                            const auto timestep =
                                earliestPackedSphereConflictTimestep(
                                    spheres_i, spheres_j, pack, scan_begin);
                            collision_worker_wall_seconds +=
                                elapsedWallSeconds(collision_wall_start);
                            collision_worker_cpu_seconds +=
                                elapsedProcessCpuSeconds(collision_cpu_start);
                            if (!timestep)
                                break;
                            raw_candidates.push_back(RawConflictCandidate{
                                static_cast<std::uint64_t>(pair.pair_index),
                                static_cast<std::uint64_t>(pair.robot_i),
                                static_cast<std::uint64_t>(pair.robot_j),
                                static_cast<std::uint64_t>(*timestep)});
                            pair_has_candidate[pair.pair_index] = 1;
                            if (optimistic_unique)
                                break;
                            scan_begin = *timestep + 1;
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
                    "Process-parallel VAMP conflict finder socketpair failed");
            }
            const pid_t pid = ::fork();
            if (pid < 0) {
                ::close(fds[0]);
                ::close(fds[1]);
                shutdownWorkers(true);
                throw std::runtime_error(
                    "Process-parallel VAMP conflict finder fork failed");
            }
            if (pid == 0) {
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
                if (options.stop_requested && options.stop_requested()) {
                    shutdownWorkers(true);
                    return out;
                }
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
                        msg << "Process-parallel VAMP conflict finder command "
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
                std::vector<char> worker_done(workers.size(), 0);
                std::size_t remaining_workers = workers.size();
                while (remaining_workers > 0) {
                    if (options.stop_requested && options.stop_requested()) {
                        shutdownWorkers(true);
                        return out;
                    }

                    std::vector<pollfd> poll_fds;
                    std::vector<std::size_t> poll_workers;
                    poll_fds.reserve(remaining_workers);
                    poll_workers.reserve(remaining_workers);
                    for (std::size_t worker_index = 0;
                         worker_index < workers.size(); ++worker_index) {
                        if (worker_done[worker_index] ||
                            workers[worker_index].fd < 0) {
                            continue;
                        }
                        pollfd pfd {};
                        pfd.fd = workers[worker_index].fd;
                        pfd.events = POLLIN | POLLHUP | POLLERR;
                        poll_fds.push_back(pfd);
                        poll_workers.push_back(worker_index);
                    }

                    int poll_rc = -1;
                    do {
                        poll_rc = ::poll(
                            poll_fds.data(),
                            static_cast<nfds_t>(poll_fds.size()), 50);
                    } while (poll_rc < 0 && errno == EINTR);
                    if (poll_rc < 0) {
                        throw std::runtime_error(
                            "Process-parallel VAMP conflict finder poll "
                            "failed");
                    }
                    if (poll_rc == 0)
                        continue;

                    for (std::size_t i = 0; i < poll_fds.size(); ++i) {
                        if (poll_fds[i].revents == 0)
                            continue;
                        const std::size_t worker_index = poll_workers[i];
                        auto &worker = workers[worker_index];
                        ProcessScanResultHeader result;
                        if (!readValue(worker.fd, result)) {
                            throw std::runtime_error(
                                "Process-parallel VAMP conflict finder result "
                                "read failed");
                        }
                        if (options.temporary_conflict_find_instrumentation) {
                            // TEMP(ablation): remove this accumulation once
                            // the conflict-detection timing table is no longer
                            // needed for reproduction work.
                            options.temporary_conflict_find_instrumentation
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
                                "Process-parallel VAMP conflict finder "
                                "candidate read failed");
                        }

                        worker_done[worker_index] = 1;
                        if (remaining_workers > 0)
                            --remaining_workers;

                        candidates.reserve(candidates.size() +
                                           raw_candidates.size());
                        for (const auto &raw : raw_candidates) {
                            const auto robot_i =
                                static_cast<std::size_t>(raw.robot_i);
                            const auto robot_j =
                                static_cast<std::size_t>(raw.robot_j);
                            const auto t =
                                static_cast<std::size_t>(raw.timestep);
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

    bool isRobotBatchValid(const RobotModel &robot, const Path &path,
                           const std::vector<BatchPack> &packs) const {
        switch (robot.robotFamily()) {
        case RobotModel::RobotFamily::Sphere:
            return isRobotBatchValidImpl<vamp::robots::Sphere>(
                robot, path, environmentForRobot(robot), packs);
        case RobotModel::RobotFamily::Panda:
            return isRobotBatchValidImpl<vamp::robots::Panda>(
                robot, path, environmentForRobot(robot), packs);
        case RobotModel::RobotFamily::UR5:
            return isRobotBatchValidImpl<vamp::robots::UR5>(
                robot, path, environmentForRobot(robot), packs);
        case RobotModel::RobotFamily::Planar3:
            return isRobotBatchValidImpl<vamp::robots::Planar3>(
                robot, path, environmentForRobot(robot), packs);
        case RobotModel::RobotFamily::Unknown:
            throwUnsupportedRobotFamily(robot, "isRobotBatchValid");
        }
        return false;
    }

    bool isRobotPackValid(const RobotModel &robot, const Path &path,
                          const BatchPack &pack) const {
        switch (robot.robotFamily()) {
        case RobotModel::RobotFamily::Sphere:
            return isRobotPackValidImpl<vamp::robots::Sphere>(
                robot, path, environmentForRobot(robot), pack);
        case RobotModel::RobotFamily::Panda:
            return isRobotPackValidImpl<vamp::robots::Panda>(
                robot, path, environmentForRobot(robot), pack);
        case RobotModel::RobotFamily::UR5:
            return isRobotPackValidImpl<vamp::robots::UR5>(
                robot, path, environmentForRobot(robot), pack);
        case RobotModel::RobotFamily::Planar3:
            return isRobotPackValidImpl<vamp::robots::Planar3>(
                robot, path, environmentForRobot(robot), pack);
        case RobotModel::RobotFamily::Unknown:
            throwUnsupportedRobotFamily(robot, "isRobotPackValid");
        }
        return false;
    }

    bool isPairPackValid(const RobotModel &robot_a, const Path &path_a,
                         const RobotModel &robot_b, const Path &path_b,
                         const BatchPack &pack) const {
        switch (robot_a.robotFamily()) {
        case RobotModel::RobotFamily::Sphere:
            switch (robot_b.robotFamily()) {
            case RobotModel::RobotFamily::Sphere:
                return isPairPackValidRaked<vamp::robots::Sphere,
                                            vamp::robots::Sphere>(
                    robot_a, path_a, robot_b, path_b, pack);
            case RobotModel::RobotFamily::Panda:
                return isPairPackValidRaked<vamp::robots::Sphere,
                                            vamp::robots::Panda>(
                    robot_a, path_a, robot_b, path_b, pack);
            case RobotModel::RobotFamily::UR5:
                return isPairPackValidRaked<vamp::robots::Sphere,
                                            vamp::robots::UR5>(
                    robot_a, path_a, robot_b, path_b, pack);
            case RobotModel::RobotFamily::Planar3:
                return isPairPackValidRaked<vamp::robots::Sphere,
                                            vamp::robots::Planar3>(
                    robot_a, path_a, robot_b, path_b, pack);
            case RobotModel::RobotFamily::Unknown:
                throwUnsupportedRobotFamily(robot_b, "isPairPackValid");
            }
            break;
        case RobotModel::RobotFamily::Panda:
            switch (robot_b.robotFamily()) {
            case RobotModel::RobotFamily::Sphere:
                return isPairPackValidRaked<vamp::robots::Panda,
                                            vamp::robots::Sphere>(
                    robot_a, path_a, robot_b, path_b, pack);
            case RobotModel::RobotFamily::Panda:
                return isPairPackValidRaked<vamp::robots::Panda,
                                            vamp::robots::Panda>(
                    robot_a, path_a, robot_b, path_b, pack);
            case RobotModel::RobotFamily::UR5:
                return isPairPackValidRaked<vamp::robots::Panda,
                                            vamp::robots::UR5>(
                    robot_a, path_a, robot_b, path_b, pack);
            case RobotModel::RobotFamily::Planar3:
                return isPairPackValidRaked<vamp::robots::Panda,
                                            vamp::robots::Planar3>(
                    robot_a, path_a, robot_b, path_b, pack);
            case RobotModel::RobotFamily::Unknown:
                throwUnsupportedRobotFamily(robot_b, "isPairPackValid");
            }
            break;
        case RobotModel::RobotFamily::UR5:
            switch (robot_b.robotFamily()) {
            case RobotModel::RobotFamily::Sphere:
                return isPairPackValidRaked<vamp::robots::UR5,
                                            vamp::robots::Sphere>(
                    robot_a, path_a, robot_b, path_b, pack);
            case RobotModel::RobotFamily::Panda:
                return isPairPackValidRaked<vamp::robots::UR5,
                                            vamp::robots::Panda>(
                    robot_a, path_a, robot_b, path_b, pack);
            case RobotModel::RobotFamily::UR5:
                return isPairPackValidRaked<vamp::robots::UR5,
                                            vamp::robots::UR5>(
                    robot_a, path_a, robot_b, path_b, pack);
            case RobotModel::RobotFamily::Planar3:
                return isPairPackValidRaked<vamp::robots::UR5,
                                            vamp::robots::Planar3>(
                    robot_a, path_a, robot_b, path_b, pack);
            case RobotModel::RobotFamily::Unknown:
                throwUnsupportedRobotFamily(robot_b, "isPairPackValid");
            }
            break;
        case RobotModel::RobotFamily::Planar3:
            switch (robot_b.robotFamily()) {
            case RobotModel::RobotFamily::Sphere:
                return isPairPackValidRaked<vamp::robots::Planar3,
                                            vamp::robots::Sphere>(
                    robot_a, path_a, robot_b, path_b, pack);
            case RobotModel::RobotFamily::Panda:
                return isPairPackValidRaked<vamp::robots::Planar3,
                                            vamp::robots::Panda>(
                    robot_a, path_a, robot_b, path_b, pack);
            case RobotModel::RobotFamily::UR5:
                return isPairPackValidRaked<vamp::robots::Planar3,
                                            vamp::robots::UR5>(
                    robot_a, path_a, robot_b, path_b, pack);
            case RobotModel::RobotFamily::Planar3:
                return isPairPackValidRaked<vamp::robots::Planar3,
                                            vamp::robots::Planar3>(
                    robot_a, path_a, robot_b, path_b, pack);
            case RobotModel::RobotFamily::Unknown:
                throwUnsupportedRobotFamily(robot_b, "isPairPackValid");
            }
            break;
        case RobotModel::RobotFamily::Unknown:
            throwUnsupportedRobotFamily(robot_a, "isPairPackValid");
        }
        return false;
    }

    std::optional<CompositeConflict> findPackEnvironmentConflict(
        const std::vector<Path> &paths,
        const std::vector<const RobotModel *> &robots,
        const BatchPack &pack) const {
        std::optional<CompositeConflict> best;
        for (std::size_t i = 0; i < paths.size(); ++i) {
            for (std::size_t lane = 0; lane < pack.lanes; ++lane) {
                const std::size_t t = pack.timesteps[lane];
                const auto &config = configAt(paths[i], t);
                std::optional<CompositeConflict> candidate;
                if (!isValidSingle(*robots[i], config, world_spheres_,
                                   world_cylinders_)) {
                    candidate = CompositeConflict{
                        ConflictScope::Environment, static_cast<int>(i), -1, t, 0.0,
                        ConflictKind::Vertex, config, {}};
                } else if (!isSelfCollisionFree(*robots[i], config)) {
                    candidate = CompositeConflict{
                        ConflictScope::Self, static_cast<int>(i), -1, t, 0.0,
                        ConflictKind::Vertex, config, {}};
                }

                if (candidate &&
                    (!best || candidate->timestep < best->timestep)) {
                    best = std::move(candidate);
                }
            }
        }
        return best;
    }

    std::optional<CompositeConflict> findBatchEnvironmentConflict(
        const std::vector<Path> &paths,
        const std::vector<const RobotModel *> &robots,
        std::size_t batch_begin, std::size_t batch_end) const {
        for (std::size_t t = batch_begin; t < batch_end; ++t) {
            for (std::size_t i = 0; i < paths.size(); ++i) {
                const auto &config = configAt(paths[i], t);
                if (!isValidSingle(*robots[i], config, world_spheres_,
                                   world_cylinders_)) {
                    return CompositeConflict{ConflictScope::Environment,
                                             static_cast<int>(i), -1, t, 0.0,
                                             ConflictKind::Vertex, config, {}};
                }
                if (!isSelfCollisionFree(*robots[i], config)) {
                    return CompositeConflict{ConflictScope::Self,
                                             static_cast<int>(i), -1, t, 0.0,
                                             ConflictKind::Vertex, config, {}};
                }
            }
        }
        return std::nullopt;
    }

    std::optional<CompositeConflict> findPackInterRobotConflict(
        const std::vector<Path> &paths,
        const std::vector<const RobotModel *> &robots,
        const BatchPack &pack,
        const std::vector<std::size_t> *effective_pair_starts = nullptr,
        const std::vector<char> *robot_used = nullptr) const {
        std::optional<CompositeConflict> best;
        for (std::size_t i = 0; i < paths.size(); ++i) {
            if (robot_used && (*robot_used)[i] != 0)
                continue;
            for (std::size_t j = i + 1; j < paths.size(); ++j) {
                if (robot_used && (*robot_used)[j] != 0)
                    continue;
                const std::size_t pair_begin =
                    effective_pair_starts
                        ? (*effective_pair_starts)
                              [pairFrontierIndex(i, j, paths.size())]
                        : 0;
                const BatchPack pair_pack =
                    pair_begin > 0 ? filterPackAtOrAfter(pack, pair_begin)
                                   : pack;
                if (pair_pack.lanes == 0)
                    continue;
                auto conflict = findFirstPairPackConflict(
                    *robots[i], paths[i], *robots[j], paths[j], pair_pack);
                if (!conflict)
                    continue;

                CompositeConflict candidate{ConflictScope::InterRobot,
                                            static_cast<int>(i),
                                            static_cast<int>(j),
                                            conflict->timestep,
                                            conflict->alpha,
                                            conflict->kind,
                                            conflict->config_a,
                                            conflict->config_b};
                if (!best || candidate.timestep < best->timestep)
                    best = std::move(candidate);
            }
        }
        return best;
    }

    const EnvironmentVector &environmentForRobot(const RobotModel &robot) const {
        auto key = baseTransformKey(robot);
        auto &cache = sphere_env_cache_[&robot];
        const bool cache_hit = cache.valid && cache.revision == environment_revision_ &&
                               cache.base_transform == key;
        if (!cache_hit) {
            cache.revision = environment_revision_;
            cache.base_transform = key;
            cache.env_float = buildEnvironmentForRobot(robot);
            cache.env_float.sort();
            cache.env_vector = EnvironmentVector(cache.env_float);
            cache.valid = true;
        }
        return cache.env_vector;
    }

    EnvironmentFloat buildEnvironmentForRobot(const RobotModel &robot) const {
        EnvironmentFloat environment;
        const Eigen::Affine3f robot_from_world =
            robot.getBaseTransform().cast<float>().inverse();
        for (std::size_t i = 0; i < world_spheres_.size(); ++i) {
            const auto &sphere = world_spheres_[i];
            const Eigen::Vector3f local_center =
                robot_from_world * sphere.center.cast<float>();
            environment.spheres.emplace_back(
                vamp::collision::factory::sphere::eigen(
                    local_center, static_cast<float>(sphere.radius)));
            environment.spheres.back().name = "obstacle_sphere_" + std::to_string(i);
        }
        for (std::size_t i = 0; i < world_cylinders_.size(); ++i) {
            appendCylinderObstacle(environment, world_cylinders_[i],
                                   robot_from_world, i);
        }
        return environment;
    }

    std::vector<ObstacleSphere> world_spheres_;
    std::vector<ObstacleCylinder> world_cylinders_;
    std::size_t environment_revision_ = 0;
    VampValidationStrategy strategy_{};
    mutable std::unordered_map<const RobotModel *, SphereEnvironmentCache>
        sphere_env_cache_;
};

#else

class VampCollisionBackend final : public CollisionBackend {
public:
    std::unique_ptr<CollisionBackend> clone() const override {
        return std::make_unique<VampCollisionBackend>();
    }

    void onEnvironmentChanged(const std::vector<ObstacleSphere> &,
                              const std::vector<ObstacleCylinder> &) override {}

    bool isValidSingle(const RobotModel &, const std::vector<double> &,
                       const std::vector<ObstacleSphere> &,
                       const std::vector<ObstacleCylinder> &) const override {
        throw std::runtime_error("VAMP backend requested but COMOTION_HAVE_VAMP=0.");
    }

    bool isSelfCollisionFree(const RobotModel &,
                             const std::vector<double> &) const override {
        throw std::runtime_error("VAMP backend requested but COMOTION_HAVE_VAMP=0.");
    }

    bool isValidPair(const RobotModel &, const std::vector<double> &,
                     const RobotModel &, const std::vector<double> &) const override {
        throw std::runtime_error("VAMP backend requested but COMOTION_HAVE_VAMP=0.");
    }

    bool isMotionValid(const RobotModel &, const std::vector<double> &,
                       const std::vector<double> &, int,
                       const std::vector<ObstacleSphere> &,
                       const std::vector<ObstacleCylinder> &) const override {
        throw std::runtime_error("VAMP backend requested but COMOTION_HAVE_VAMP=0.");
    }

    bool isRobotPathValid(const RobotModel &, const Path &,
                          const std::vector<ObstacleSphere> &,
                          const std::vector<ObstacleCylinder> &) const override {
        throw std::runtime_error("VAMP backend requested but COMOTION_HAVE_VAMP=0.");
    }

    bool isPairPathValid(const RobotModel &, const Path &,
                         const RobotModel &, const Path &,
                         std::size_t, std::size_t) const override {
        throw std::runtime_error("VAMP backend requested but COMOTION_HAVE_VAMP=0.");
    }

    std::optional<PairPathConflict> findFirstPairPathConflict(
        const RobotModel &, const Path &,
        const RobotModel &, const Path &,
        std::size_t, std::size_t) const override {
        throw std::runtime_error("VAMP backend requested but COMOTION_HAVE_VAMP=0.");
    }

    GoalHoldConstraint computeGoalHoldConstraint(
        const RobotModel &, const std::vector<double> &,
        const RobotModel &, const Path &) const override {
        throw std::runtime_error("VAMP backend requested but COMOTION_HAVE_VAMP=0.");
    }

    bool isCompositeMotionValid(
        const std::vector<const RobotModel *> &,
        const std::vector<std::vector<double>> &,
        const std::vector<std::vector<double>> &,
        const CompositePathValidationOptions &,
        const std::vector<ObstacleSphere> &,
        const std::vector<ObstacleCylinder> &) const override {
        throw std::runtime_error("VAMP backend requested but COMOTION_HAVE_VAMP=0.");
    }

    std::optional<CompositeConflict> findFirstCompositeMotionConflict(
        const std::vector<const RobotModel *> &,
        const std::vector<std::vector<double>> &,
        const std::vector<std::vector<double>> &,
        const CompositePathValidationOptions &,
        const std::vector<ObstacleSphere> &,
        const std::vector<ObstacleCylinder> &) const override {
        throw std::runtime_error("VAMP backend requested but COMOTION_HAVE_VAMP=0.");
    }

    bool validateCompositePaths(
        const std::vector<Path> &,
        const std::vector<const RobotModel *> &,
        const CompositePathValidationOptions &,
        const std::vector<ObstacleSphere> &,
        const std::vector<ObstacleCylinder> &) const override {
        throw std::runtime_error("VAMP backend requested but COMOTION_HAVE_VAMP=0.");
    }

    std::optional<CompositeConflict> findFirstCompositePathConflict(
        const std::vector<Path> &,
        const std::vector<const RobotModel *> &,
        const CompositePathValidationOptions &,
        const std::vector<ObstacleSphere> &,
        const std::vector<ObstacleCylinder> &,
        std::vector<std::size_t> *) const override {
        throw std::runtime_error("VAMP backend requested but COMOTION_HAVE_VAMP=0.");
    }

    std::vector<CompositeConflict> findInterRobotPathConflictsCompositeScan(
        const std::vector<Path> &,
        const std::vector<const RobotModel *> &,
        const CompositePathValidationOptions &,
        std::size_t, bool,
        const InterRobotConflictCallback &,
        const std::vector<ObstacleSphere> &,
        const std::vector<ObstacleCylinder> &,
        std::vector<std::size_t> *,
        std::vector<std::size_t> *) const override {
        throw std::runtime_error("VAMP backend requested but COMOTION_HAVE_VAMP=0.");
    }
};

#endif

std::unique_ptr<CollisionBackend> makeVampBackend() {
    return std::make_unique<VampCollisionBackend>();
}

} // namespace detail
} // namespace comotion
