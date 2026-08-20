#include "comotion/planning/CompositeRRT.h"
#include "comotion/planning/MultiRobotProblem.h"
#include "comotion/planning/PlanningRng.h"
#include "comotion/planning/PlanningSeed.h"
#include "comotion/robot/FlyingSphere.h"

#include <ompl/util/RandomNumbers.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <vector>

#if !defined(_WIN32)
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace ob = ompl::base;

namespace {

struct ChildResult {
    int completed = 0;
    int exact = 0;
    std::uint64_t path_fingerprint = 0;
    std::uint64_t state_sampler_seed = 0;
    std::uint64_t rrt_connect_seed = 0;
    std::uint64_t path_simplifier_seed = 0;
    std::uint64_t iterations = 0;
    std::uint64_t first_solution_iteration = 0;
};

bool expectTrue(const std::string &label, bool condition) {
    if (!condition)
        std::cerr << "composite_rrt_seed_regression: " << label << "\n";
    return condition;
}

void hashCombine(std::uint64_t &hash, std::uint64_t value) {
    hash ^= value + 0x9e3779b97f4a7c15ULL + (hash << 6u) + (hash >> 2u);
}

std::uint64_t fingerprintPaths(const std::vector<comotion::Path> &paths) {
    std::uint64_t hash = 0x6a09e667f3bcc909ULL;
    hashCombine(hash, paths.size());
    for (const auto &path : paths) {
        hashCombine(hash, path.size());
        hashCombine(hash, path.arrival_timestep());
        hashCombine(hash, path.has_explicit_timesteps() ? 1u : 0u);
        for (std::size_t waypoint = 0; waypoint < path.size(); ++waypoint) {
            hashCombine(hash, path.timestep_at(waypoint));
            hashCombine(hash, path[waypoint].size());
            for (const double value : path[waypoint]) {
                std::uint64_t bits = 0;
                static_assert(sizeof(bits) == sizeof(value));
                std::memcpy(&bits, &value, sizeof(bits));
                hashCombine(hash, bits);
            }
        }
    }
    return hash;
}

std::shared_ptr<comotion::MultiRobotProblem> makeProblem() {
    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(16);
    problem->setVmax(1.0);
    auto robot = std::make_shared<comotion::FlyingSphere>(
        0.1, std::vector<double>{-5.0, -5.0, -5.0},
        std::vector<double>{5.0, 5.0, 5.0});
    problem->addRobot(robot, {-4.0, 0.0, 0.0}, {4.0, 0.0, 0.0});
    return problem;
}

ChildResult solveWithSeed(std::uint32_t seed) {
    ChildResult result;
    try {
        comotion::CompositeRRT planner;
        planner.setProblem(makeProblem());
        planner.setPlanningSeed(seed);
        planner.setSimplifySolution(false);
        planner.setRange(0.5);
        planner.setMaxRrtConnectIterations(1);
        planner.setContinueAfterSolutionUntilIterationCap(true);

        const auto status = planner.solve(5.0);
        const auto &stats = planner.plannerStatsJson();
        result.completed = 1;
        result.exact = status == ob::PlannerStatus::EXACT_SOLUTION ? 1 : 0;
        result.path_fingerprint = fingerprintPaths(planner.getSolutionPaths());
        result.state_sampler_seed =
            stats.at("state_sampler_seed").get<std::uint64_t>();
        result.rrt_connect_seed =
            stats.at("rrt_connect_seed").get<std::uint64_t>();
        result.path_simplifier_seed =
            stats.at("path_simplifier_seed").get<std::uint64_t>();
        result.iterations =
            stats.at("rrt_connect_iterations").get<std::uint64_t>();
        result.first_solution_iteration =
            stats.at("rrt_connect_first_solution_iteration")
                .get<std::uint64_t>();
    } catch (...) {
        result.completed = 0;
    }
    return result;
}

#if !defined(_WIN32)
bool writeAll(int fd, const void *data, std::size_t size) {
    const auto *bytes = static_cast<const char *>(data);
    std::size_t written = 0;
    while (written < size) {
        const auto count = ::write(fd, bytes + written, size - written);
        if (count < 0 && errno == EINTR)
            continue;
        if (count <= 0)
            return false;
        written += static_cast<std::size_t>(count);
    }
    return true;
}

bool readAll(int fd, void *data, std::size_t size) {
    auto *bytes = static_cast<char *>(data);
    std::size_t read = 0;
    while (read < size) {
        const auto count = ::read(fd, bytes + read, size - read);
        if (count < 0 && errno == EINTR)
            continue;
        if (count <= 0)
            return false;
        read += static_cast<std::size_t>(count);
    }
    return true;
}

bool solveInFreshChild(std::uint32_t seed, ChildResult &result) {
    int pipe_fds[2] = {-1, -1};
    if (::pipe(pipe_fds) != 0)
        return false;

    const pid_t pid = ::fork();
    if (pid < 0) {
        ::close(pipe_fds[0]);
        ::close(pipe_fds[1]);
        return false;
    }
    if (pid == 0) {
        ::close(pipe_fds[0]);
        const ChildResult child_result = solveWithSeed(seed);
        const bool wrote =
            writeAll(pipe_fds[1], &child_result, sizeof(child_result));
        ::close(pipe_fds[1]);
        _exit(wrote ? 0 : 2);
    }

    ::close(pipe_fds[1]);
    const bool read_ok = readAll(pipe_fds[0], &result, sizeof(result));
    ::close(pipe_fds[0]);
    int status = 0;
    while (::waitpid(pid, &status, 0) < 0) {
        if (errno != EINTR)
            return false;
    }
    return read_ok && WIFEXITED(status) && WEXITSTATUS(status) == 0;
}
#endif

bool testForkedSeedDiversity() {
#if defined(_WIN32)
    return true;
#else
    constexpr std::uint32_t kSeedA = 101;
    constexpr std::uint32_t kSeedB = 202;

    // All children inherit this exact OMPL global generator state. The assigned
    // local planning seed must therefore be what differentiates their searches.
    comotion::seedOmplGlobalFromUserPlanningSeed(77);
    ompl::RNG prime_global_generator;
    (void)prime_global_generator;

    ChildResult a1;
    ChildResult a2;
    ChildResult b;
    if (!expectTrue("first seed-A child completed",
                    solveInFreshChild(kSeedA, a1)) ||
        !expectTrue("second seed-A child completed",
                    solveInFreshChild(kSeedA, a2)) ||
        !expectTrue("seed-B child completed", solveInFreshChild(kSeedB, b))) {
        return false;
    }

    if (!expectTrue("all children returned exact solutions",
                    a1.completed && a2.completed && b.completed && a1.exact &&
                        a2.exact && b.exact) ||
        !expectTrue("fixed run performs one iteration",
                    a1.iterations == 1 && a2.iterations == 1 &&
                        b.iterations == 1) ||
        !expectTrue("first solution occurs at iteration one",
                    a1.first_solution_iteration == 1 &&
                        a2.first_solution_iteration == 1 &&
                        b.first_solution_iteration == 1) ||
        !expectTrue("same planning seed reproduces the same path",
                    a1.path_fingerprint == a2.path_fingerprint) ||
        !expectTrue("different planning seeds diversify the path",
                    a1.path_fingerprint != b.path_fingerprint)) {
        return false;
    }

    if (!expectTrue(
            "seed-A sampler telemetry matches derivation",
            a1.state_sampler_seed ==
                comotion::compositeRrtStateSamplerSeed(kSeedA)) ||
        !expectTrue(
            "seed-A planner telemetry matches derivation",
            a1.rrt_connect_seed == comotion::compositeRrtPlannerSeed(kSeedA)) ||
        !expectTrue(
            "seed-A simplifier telemetry matches derivation",
            a1.path_simplifier_seed ==
                comotion::compositeRrtPathSimplifierSeed(kSeedA)) ||
        !expectTrue(
            "seed-B sampler telemetry matches derivation",
            b.state_sampler_seed ==
                comotion::compositeRrtStateSamplerSeed(kSeedB)) ||
        !expectTrue("different seeds have different sampler streams",
                    a1.state_sampler_seed != b.state_sampler_seed)) {
        return false;
    }

    return true;
#endif
}

bool testRepairAttemptSeedHierarchy() {
    constexpr std::uint32_t kParentSeed = 19;
    std::set<std::uint32_t> attempt_roots;
    std::set<std::uint_fast32_t> sampler_seeds;
    for (std::uint64_t repair = 0; repair < 8; ++repair) {
        for (std::uint64_t attempt = 0; attempt < 16; ++attempt) {
            const auto root = comotion::arcRepairAttemptPlanningSeed(
                kParentSeed, repair, attempt);
            const auto composite =
                comotion::arcRepairCompositePlanningSeed(root);
            attempt_roots.insert(root);
            sampler_seeds.insert(
                comotion::compositeRrtStateSamplerSeed(composite));
            if (!expectTrue("solver domains are separated",
                            comotion::arcRepairPrioritizedPlanningSeed(root) !=
                                composite)) {
                return false;
            }
        }
    }
    return expectTrue("128 attempts have unique root seeds",
                      attempt_roots.size() == 128) &&
           expectTrue("128 attempts have unique effective sampler seeds",
                      sampler_seeds.size() == 128);
}

bool testParallelArcRepairSeedHierarchy() {
    constexpr std::uint32_t kParentSeed = 23;
    std::set<std::uint32_t> seeds;
    for (std::uint64_t batch = 0; batch < 8; ++batch) {
        for (std::uint64_t task = 0; task < 32; ++task) {
            for (std::uint64_t attempt = 0; attempt < 8; ++attempt) {
                const auto seed =
                    comotion::parallelArcRepairAttemptPlanningSeed(
                        kParentSeed, batch, task, attempt);
                seeds.insert(seed);
                if (!expectTrue(
                        "P-ARC repair seed is deterministic",
                        seed == comotion::parallelArcRepairAttemptPlanningSeed(
                                    kParentSeed, batch, task, attempt))) {
                    return false;
                }
            }
        }
    }
    if (!expectTrue("2048 P-ARC attempts have unique seeds",
                    seeds.size() == 2048)) {
        return false;
    }

    // The former additive salt mapped these two distinct tuples to the same
    // integer before hashing: task + 1 was identical to attempt + 64.
    if (!expectTrue(
            "historically colliding P-ARC tuples are separated",
            comotion::parallelArcRepairAttemptPlanningSeed(kParentSeed, 0, 1,
                                                           0) !=
                comotion::parallelArcRepairAttemptPlanningSeed(kParentSeed, 0,
                                                               0, 64))) {
        return false;
    }

    return expectTrue(
        "full-width P-ARC identifiers do not truncate",
        comotion::parallelArcRepairAttemptPlanningSeed(
            kParentSeed, (std::uint64_t{1} << 40), 3, 5) !=
            comotion::parallelArcRepairAttemptPlanningSeed(kParentSeed, 0, 3,
                                                           5));
}

} // namespace

int main() {
    if (!testForkedSeedDiversity())
        return 1;
    if (!testRepairAttemptSeedHierarchy())
        return 1;
    if (!testParallelArcRepairSeedHierarchy())
        return 1;
    std::cout << "composite_rrt_seed_regression: OK\n";
    return 0;
}
