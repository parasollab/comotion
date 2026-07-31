#include "comotion/collision/detail/CollisionBackend.h"
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

class SphereCollisionBackend final : public CollisionBackend {
public:
    std::unique_ptr<CollisionBackend> clone() const override {
        return std::make_unique<SphereCollisionBackend>();
    }

    void onEnvironmentChanged(const std::vector<ObstacleSphere> &,
                              const std::vector<ObstacleCylinder> &) override {}

    static bool spheresCollide(const Eigen::Vector3d &c1, double r1,
                               const Eigen::Vector3d &c2, double r2) {
        double dist_sq = (c1 - c2).squaredNorm();
        double radii_sum = r1 + r2;
        return dist_sq < radii_sum * radii_sum;
    }

    static bool sphereCylinderCollide(const Eigen::Vector3d &sphere_center,
                                      double sphere_radius,
                                      const ObstacleCylinder &cyl) {
        Eigen::Vector3d v = sphere_center - cyl.center;
        double t = v.dot(cyl.axis);
        double radial_dist = (v - t * cyl.axis).norm();
        double axial_excess = std::max(0.0, std::abs(t) - cyl.half_height);
        double radial_excess = std::max(0.0, radial_dist - cyl.radius);
        double dist_sq =
            axial_excess * axial_excess + radial_excess * radial_excess;
        return dist_sq < sphere_radius * sphere_radius;
    }

    bool isValidSingle(const RobotModel &robot,
                       const std::vector<double> &config,
                       const std::vector<ObstacleSphere> &obstacles,
                       const std::vector<ObstacleCylinder> &cylinders) const override {
        auto spheres = robot.getCollisionSpheres(config);
        for (auto &rs : spheres) {
            for (auto &obs : obstacles) {
                if (spheresCollide(rs.center, rs.radius, obs.center, obs.radius))
                    return false;
            }
            for (auto &cyl : cylinders) {
                if (sphereCylinderCollide(rs.center, rs.radius, cyl))
                    return false;
            }
        }
        return true;
    }

    bool isSelfCollisionFree(
        const RobotModel &robot,
        const std::vector<double> &config) const override {
        auto spheres = robot.getCollisionSpheres(config);
        for (size_t i = 0; i < spheres.size(); ++i) {
            for (size_t j = i + 1; j < spheres.size(); ++j) {
                if (spheres[i].link_index == spheres[j].link_index)
                    continue;

                const auto &li = robot.links()[spheres[i].link_index];
                const auto &lj = robot.links()[spheres[j].link_index];
                if (robot.isSelfCollisionDisabled(li.name, lj.name))
                    continue;

                if (spheresCollide(spheres[i].center, spheres[i].radius,
                                   spheres[j].center, spheres[j].radius))
                    return false;
            }
        }
        return true;
    }

    bool isValidPair(const RobotModel &robot_a,
                     const std::vector<double> &config_a,
                     const RobotModel &robot_b,
                     const std::vector<double> &config_b) const override {
        auto spheres_a = robot_a.getCollisionSpheres(config_a);
        auto spheres_b = robot_b.getCollisionSpheres(config_b);
        return areSphereSetsPairValid(spheres_a, spheres_b);
    }

    static bool areSphereSetsPairValid(
        const std::vector<CollisionSphere> &spheres_a,
        const std::vector<CollisionSphere> &spheres_b) {
        for (auto &sa : spheres_a) {
            for (auto &sb : spheres_b) {
                if (spheresCollide(sa.center, sa.radius, sb.center, sb.radius))
                    return false;
            }
        }
        return true;
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
        if (robots.size() != from.size() || robots.size() != to.size())
            return false;

        const int num_checks =
            options.discrete_num_checks_hint > 0 ? options.discrete_num_checks_hint : 10;
        if (options.check_environment) {
            for (std::size_t i = 0; i < robots.size(); ++i) {
                if (!isMotionValid(*robots[i], from[i], to[i], num_checks, obstacles,
                                   cylinders)) {
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
            std::vector<std::vector<CollisionSphere>> sphere_cache(paths.size());
            std::vector<char> sphere_cached(paths.size(), 0);
            const auto spheresFor = [&](std::size_t robot)
                -> const std::vector<CollisionSphere> & {
                if (!sphere_cached[robot]) {
                    sphere_cache[robot] =
                        robots[robot]->getCollisionSpheres(
                            configAt(paths[robot], t));
                    sphere_cached[robot] = 1;
                }
                return sphere_cache[robot];
            };

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
                    if (!areSphereSetsPairValid(spheresFor(i), spheresFor(j))) {
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
            "Process-parallel conflict finding requires POSIX fork support");
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
                for (std::size_t t = segment_begin; t < segment_end; ++t) {
                    const auto &local_robots = worker_robots[worker_index];
                    std::vector<std::vector<CollisionSphere>> sphere_cache(
                        local_robots.size());
                    std::vector<char> sphere_cached(local_robots.size(), 0);

                    const auto spheresFor = [&](std::size_t robot)
                        -> const std::vector<CollisionSphere> & {
                        const std::size_t local_slot =
                            worker_robot_slots[worker_index][robot];
                        if (!sphere_cached[local_slot]) {
                            const auto build_wall_start = Clock::now();
                            const double build_cpu_start = processCpuSeconds();
                            sphere_cache[local_slot] =
                                robots[robot]->getCollisionSpheres(
                                    configAt(paths[robot], t));
                            build_worker_wall_seconds +=
                                elapsedWallSeconds(build_wall_start);
                            build_worker_cpu_seconds +=
                                elapsedProcessCpuSeconds(build_cpu_start);
                            sphere_cached[local_slot] = 1;
                        }
                        return sphere_cache[local_slot];
                    };

                    const auto pairSetsValid =
                        [&](const std::vector<CollisionSphere> &spheres_a,
                            const std::vector<CollisionSphere> &spheres_b) {
                            for (const auto &sa : spheres_a) {
                                for (const auto &sb : spheres_b) {
                                    if (spheresCollide(sa.center, sa.radius,
                                                       sb.center, sb.radius)) {
                                        return false;
                                    }
                                }
                            }
                            return true;
                        };

                    for (const auto &pair : worker_pairs[worker_index]) {
                        if (effective_pair_starts[pair.pair_index] > t) {
                            continue;
                        }
                        if (optimistic_unique &&
                            (claimed_robots[pair.robot_i] ||
                             claimed_robots[pair.robot_j])) {
                            continue;
                        }
                        const auto collision_wall_start = Clock::now();
                        const double collision_cpu_start = processCpuSeconds();
                        const bool pair_valid =
                            pairSetsValid(spheresFor(pair.robot_i),
                                          spheresFor(pair.robot_j));
                        collision_worker_wall_seconds +=
                            elapsedWallSeconds(collision_wall_start);
                        collision_worker_cpu_seconds +=
                            elapsedProcessCpuSeconds(collision_cpu_start);
                        if (!pair_valid) {
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
                    "Process-parallel conflict finder socketpair failed");
            }
            const pid_t pid = ::fork();
            if (pid < 0) {
                ::close(fds[0]);
                ::close(fds[1]);
                shutdownWorkers(true);
                throw std::runtime_error(
                    "Process-parallel conflict finder fork failed");
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
                        msg << "Process-parallel conflict finder command "
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
                            "Process-parallel conflict finder result read "
                            "failed");
                    }
                    if (options.temporary_conflict_find_instrumentation) {
                        // TEMP(ablation): remove this accumulation once the
                        // conflict-detection timing table has been reproduced.
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
                            "Process-parallel conflict finder candidate read "
                            "failed");
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
};

} // namespace

std::unique_ptr<CollisionBackend> makeSphereBackend() {
    return std::make_unique<SphereCollisionBackend>();
}

} // namespace detail
} // namespace comotion
