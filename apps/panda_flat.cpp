#include "benchmark_app_common.hpp"
#include "panda_flat_builtin_tasks.hpp"

#include "cage_scene_json.hpp"

#include "comotion/collision/CollisionChecker.h"
#include "comotion/planning/MultiRobotPlanner.h"
#include "comotion/planning/MultiRobotProblem.h"
#include "comotion/planning/OrParallelPlanner.h"
#include "comotion/planning/Path.h"
#include "comotion/planning/PlanningRng.h"
#include "comotion/robot/IKSolver.h"
#include "comotion/robot/RobotModel.h"

#include <Eigen/Geometry>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

using json = nlohmann::json;
namespace builtin = comotion::benchmark_apps::panda_flat_builtin;
namespace common = comotion::benchmark_apps::common;

namespace {

bool g_app_verbose = false;
std::filesystem::path g_executable_dir;

using common::PlannerBlueprint;
using common::TrialMetrics;
using common::backendName;
using common::parseArcLocalSolverMode;
using common::parseCollisionBackend;
using common::requireValue;
using common::writeJson;

struct AppOptions {
    int num_robots = 0;
    int task_index = 0;
    int num_tasks = 5;
    std::string algorithm = "arc";
    comotion::CollisionChecker::Backend collision_backend =
        comotion::CollisionChecker::Backend::Vamp;
    double time_limit = 60.0;
    std::uint32_t seed = 0;
    std::size_t resolution = 128;
    std::string output_dir = "benchmarks/results/panda_flat";
    bool output_paths = false;
    std::optional<std::string> metrics_json_path;
    bool exit_nonzero_without_exact_solution = false;

    std::optional<std::string> task_file;
    std::optional<std::string> generate_tasks_json;
    bool generate_only = false;
    std::uint32_t task_generation_seed = 0;
    bool task_generation_seed_explicit = false;
    std::optional<int> max_ik_attempts;
    std::optional<int> max_restarts;
    std::optional<double> reachable_radius;
    std::optional<std::string> ee_link;

    std::string urdf_rel = "panda/panda_spherized.urdf";
    std::string srdf_rel = "panda/panda.srdf";
    bool urdf_explicit = false;
    bool srdf_explicit = false;

    unsigned int strrt_initial_batch_size = 4096;
    double strrt_initial_time_factor = 4.0;
    double strrt_time_bound_factor_increase = 2.0;
    bool strrt_shuffle_priority_order = false;
    bool strrt_return_first_solution = true;
    std::string strrt_rewiring = "off";
    int drrt_roadmap_size = 200;
    int drrt_iterations_per_batch = 8;
    std::string drrt_cost_metric = "sum_of_costs";
    std::string drrt_tensor_search = "drrt";
    bool drrt_exclude_roadmap_build_time = false;
    double composite_rrt_range = 0.0;
    bool composite_rrt_simplify_solution = false;
    bool composite_rrt_use_makespan_metric = false;
    std::size_t composite_aorrtc_max_internal_samples = 10000;
    std::size_t composite_aorrtc_max_internal_vertices = 10000;
    unsigned int cooperative_rrt_worker_threads = 2;
    int arc_initial_window = 1000;
    int arc_expansion_step = 1000;
    unsigned int arc_local_composite_max_samples = 500000;
    bool arc_local_composite_use_makespan_metric = false;
    bool arc_simplify_initial_solutions = true;
    bool arc_simplify_conflict_solutions = false;
    std::string arc_local_solvers = "both";
    unsigned int arc_local_prioritized_max_iterations = 5;
    std::uint64_t ao_arc_local_bound_epsilon_timesteps = 1;
    unsigned int or_parallel_worker_processes = 1;
    unsigned int parallel_arc_worker_processes = 2;
    bool parallel_arc_parallel_initial_plans = true;
    bool parallel_arc_initial_solution_or = false;
    bool parallel_arc_repair_duplicate_attempts = true;
    std::string parallel_arc_strategy = "synchronous";
    std::string parallel_arc_conflict_strategy = "greedy";
    std::string parallel_arc_conflict_find_mode = "segment_parallel";
    std::string parallel_arc_conflict_batch_mode = "optimistic";
    std::size_t parallel_arc_conflict_find_horizon = 400;
    int stcbs_max_ct_nodes = 5000;
    int stcbs_max_samples = 75000;
};

struct FlatLayout {
    int rows = 0;
    int cols = 0;
    double spacing = 0.75;
    double base_radius = 0.0;
};

struct TaskConfigs {
    std::vector<std::vector<double>> configs;
    std::vector<Eigen::Vector3d> targets;
};

struct GeneratedScenario {
    int num_robots = 0;
    int task_index = 0;
    int task_count = 0;
    std::string task_source;
    std::vector<std::vector<double>> starts;
    std::vector<std::vector<double>> goals;
    json start_targets = json::array();
    json goal_targets = json::array();
    json robot_bases = json::array();
    cage_scene::Affine3dVector base_transforms;
    json layout = json::object();
    json flat_base = json::object();
    std::vector<comotion::ObstacleSphere> sphere_obstacles;
    std::vector<comotion::ObstacleCylinder> cylinder_obstacles;
};

constexpr double kPi = 3.14159265358979323846;
constexpr double kGridSpacing = 0.75;
constexpr double kFlatBaseHalfHeight = 0.05;
constexpr double kFlatBaseCenterZ = -0.10;
constexpr double kFlatBaseMargin = 0.60;

void setExecutablePath(const char *argv0) {
    if (!argv0 || std::string(argv0).empty())
        return;
    std::error_code ec;
    const auto path = std::filesystem::weakly_canonical(
        std::filesystem::absolute(argv0), ec);
    g_executable_dir = (ec ? std::filesystem::absolute(argv0) : path).parent_path();
}

std::string getResourcePath(const std::string &relative) {
    const std::filesystem::path rel(relative);
    std::vector<std::filesystem::path> candidates;
    if (rel.is_absolute()) {
        candidates.push_back(rel);
    } else if (!g_executable_dir.empty()) {
        candidates.push_back(g_executable_dir / ".." / "share" / "comotion" /
                             "resources" / rel);
        candidates.push_back(g_executable_dir / ".." / ".." / "resources" / rel);
        candidates.push_back(g_executable_dir / ".." / "resources" / rel);
    }
    const char *prefixes[] = {"../resources/", "../../resources/",
                              "../../../resources/", "resources/"};
    for (const char *prefix : prefixes) {
        candidates.emplace_back(std::string(prefix) + relative);
    }
    for (const auto &candidate : candidates) {
        const auto normalized = candidate.lexically_normal();
        const std::string path = normalized.string();
        std::ifstream file(path);
        if (file.good())
            return path;
    }
    return std::string("resources/") + relative;
}

std::string resolveResourcePath(const std::string &path) {
    if (!path.empty() && path[0] == '/')
        return path;
    if (path.rfind("resources/", 0) == 0)
        return getResourcePath(path.substr(std::string("resources/").size()));
    return getResourcePath(path);
}

std::string toRepoRelativePath(const std::string &path) {
    std::string p = path;
    while (p.size() >= 3 && p.substr(0, 3) == "../")
        p = p.substr(3);
    const auto pos = p.find("/resources/");
    if (p.rfind("resources/", 0) == 0)
        return p;
    if (pos != std::string::npos)
        return p.substr(pos + 1);
    return p;
}

Eigen::Vector3d vector3FromJson(const json &value) {
    return Eigen::Vector3d(value.at(0).get<double>(), value.at(1).get<double>(),
                           value.at(2).get<double>());
}

json basePoseJson(const Eigen::Affine3d &transform) {
    Eigen::Quaterniond q(transform.rotation());
    return {
        {"position",
         {transform.translation().x(), transform.translation().y(),
          transform.translation().z()}},
        {"quaternion_xyzw", {q.x(), q.y(), q.z(), q.w()}},
    };
}

json sphereObstaclesJson(const std::vector<comotion::ObstacleSphere> &spheres) {
    json obstacles = json::array();
    for (std::size_t i = 0; i < spheres.size(); ++i) {
        const auto &sphere = spheres[i];
        obstacles.push_back({
            {"id", "sphere_" + std::to_string(i)},
            {"type", "sphere"},
            {"pose",
             {{"position",
               {sphere.center.x(), sphere.center.y(), sphere.center.z()}},
              {"quaternion_xyzw", {0.0, 0.0, 0.0, 1.0}}}},
            {"geometry", {{"radius", sphere.radius}}},
        });
    }
    return obstacles;
}

json cylinderObstaclesJson(
    const std::vector<comotion::ObstacleCylinder> &cylinders) {
    json obstacles = json::array();
    for (std::size_t i = 0; i < cylinders.size(); ++i) {
        const auto &cylinder = cylinders[i];
        obstacles.push_back({
            {"id", "cyl_" + std::to_string(i)},
            {"type", "cylinder"},
            {"pose",
             {{"position",
               {cylinder.center.x(), cylinder.center.y(),
                cylinder.center.z()}},
              {"axis",
               {cylinder.axis.x(), cylinder.axis.y(), cylinder.axis.z()}}}},
            {"geometry",
             {{"radius", cylinder.radius},
              {"half_height", cylinder.half_height}}},
        });
    }
    return obstacles;
}

json obstaclesJson(const std::vector<comotion::ObstacleSphere> &spheres,
                   const std::vector<comotion::ObstacleCylinder> &cylinders) {
    json out = sphereObstaclesJson(spheres);
    for (const auto &obstacle : cylinderObstaclesJson(cylinders))
        out.push_back(obstacle);
    return out;
}

FlatLayout layoutForRobotCount(int num_robots) {
    FlatLayout layout;
    layout.spacing = kGridSpacing;
    switch (num_robots) {
    case 4:
        layout.rows = 2;
        layout.cols = 2;
        break;
    case 8:
        layout.rows = 2;
        layout.cols = 4;
        break;
    case 16:
        layout.rows = 4;
        layout.cols = 4;
        break;
    default:
        throw std::runtime_error(
            "Panda flat layouts support robot counts 4, 8, and 16");
    }

    const double max_x = 0.5 * static_cast<double>(layout.cols - 1) *
                         layout.spacing;
    const double max_y = 0.5 * static_cast<double>(layout.rows - 1) *
                         layout.spacing;
    layout.base_radius = std::sqrt(max_x * max_x + max_y * max_y) +
                         kFlatBaseMargin;
    return layout;
}

json layoutJson(const FlatLayout &layout) {
    return {
        {"rows", layout.rows},
        {"cols", layout.cols},
        {"spacing", layout.spacing},
        {"ordering", "row_major_lower_y_first"},
        {"row_orientation", "opposing_rows"},
    };
}

json flatBaseJson(const FlatLayout &layout) {
    return {
        {"type", "cylinder"},
        {"center", {0.0, 0.0, kFlatBaseCenterZ}},
        {"axis", {0, 0, 1}},
        {"radius", layout.base_radius},
        {"half_height", kFlatBaseHalfHeight},
    };
}

json flatBaseObstacleJson(const FlatLayout &layout) {
    json obstacle = flatBaseJson(layout);
    obstacle["id"] = "flat_base";
    return obstacle;
}

json robotBasesJson(const FlatLayout &layout) {
    json bases = json::array();
    for (int row = 0; row < layout.rows; ++row) {
        const double y =
            (static_cast<double>(row) - 0.5 * (layout.rows - 1)) *
            layout.spacing;
        const double yaw = y < 0.0 ? 0.5 * kPi : -0.5 * kPi;
        const Eigen::Quaterniond q(Eigen::AngleAxisd(
            yaw, Eigen::Vector3d::UnitZ()));
        for (int col = 0; col < layout.cols; ++col) {
            const double x =
                (static_cast<double>(col) - 0.5 * (layout.cols - 1)) *
                layout.spacing;
            bases.push_back({
                {"position", {x, y, 0.0}},
                {"quaternion_xyzw", {q.x(), q.y(), q.z(), q.w()}},
            });
        }
    }
    return bases;
}

void ensureFlatTemplate(json &doc, int num_robots) {
    const FlatLayout layout = layoutForRobotCount(num_robots);
    if (!doc.contains("layout"))
        doc["layout"] = layoutJson(layout);
    if (!doc.contains("flat_base"))
        doc["flat_base"] = flatBaseJson(layout);
    if (!doc.contains("robot_bases"))
        doc["robot_bases"] = robotBasesJson(layout);
    if (!doc.contains("obstacles") || !doc.at("obstacles").is_array() ||
        doc.at("obstacles").empty()) {
        doc["obstacles"] = json::array({flatBaseObstacleJson(layout)});
    }
    doc["num_robots"] = num_robots;
}

std::vector<comotion::ObstacleSphere> parseSphereObstacles(const json &doc) {
    std::vector<comotion::ObstacleSphere> spheres;
    if (!doc.contains("obstacles"))
        return spheres;
    for (const auto &obs : doc.at("obstacles")) {
        if (obs.at("type").get<std::string>() != "sphere")
            continue;
        comotion::ObstacleSphere sphere;
        sphere.center = vector3FromJson(obs.at("center"));
        sphere.radius = obs.at("radius").get<double>();
        spheres.push_back(sphere);
    }
    return spheres;
}

Eigen::Vector3d sampleFlatTarget(const Eigen::Affine3d &base_transform,
                                 double reach_radius, std::mt19937 &rng) {
    std::uniform_real_distribution<double> forward(0.25, 0.60);
    std::uniform_real_distribution<double> lateral(-0.25, 0.25);
    std::uniform_real_distribution<double> vertical(0.20, 0.55);

    for (int i = 0; i < 100000; ++i) {
        Eigen::Vector3d local(forward(rng), lateral(rng), vertical(rng));
        if (local.norm() <= reach_radius)
            return base_transform * local;
    }
    throw std::runtime_error(
        "Failed to sample a point in the Panda flat local target box");
}

TaskConfigs sampleConfigSet(
    const std::vector<std::shared_ptr<comotion::RobotModel>> &robots,
    const cage_scene::Affine3dVector &base_transforms,
    const std::vector<comotion::IKSolver> &ik_solvers,
    const comotion::CollisionChecker &checker, double reach_radius,
    const std::string &ee_link, std::mt19937 &rng, int max_ik_attempts,
    int max_restarts) {
    const int num_robots = static_cast<int>(robots.size());

    for (int restart = 0; restart < max_restarts; ++restart) {
        TaskConfigs configs;
        configs.configs.resize(static_cast<std::size_t>(num_robots));
        configs.targets.resize(static_cast<std::size_t>(num_robots));
        bool all_ok = true;

        for (int r = 0; r < num_robots; ++r) {
            bool robot_ok = false;
            for (int attempt = 0; attempt < max_ik_attempts; ++attempt) {
                Eigen::Vector3d target = sampleFlatTarget(
                    base_transforms[static_cast<std::size_t>(r)],
                    reach_radius, rng);

                comotion::IKRequest request;
                request.target_position = target;
                request.ee_link_name = ee_link;
                auto ik = ik_solvers[static_cast<std::size_t>(r)]
                              .solveWithRestarts(request, rng, 20);
                if (!ik.success)
                    continue;
                if (!checker.isValidSingleFull(*robots[static_cast<std::size_t>(r)],
                                               ik.config))
                    continue;

                bool pair_ok = true;
                for (int prev = 0; prev < r; ++prev) {
                    if (!checker.isValidPair(
                            *robots[static_cast<std::size_t>(prev)],
                            configs.configs[static_cast<std::size_t>(prev)],
                            *robots[static_cast<std::size_t>(r)], ik.config)) {
                        pair_ok = false;
                        break;
                    }
                }
                if (!pair_ok)
                    continue;

                configs.configs[static_cast<std::size_t>(r)] = ik.config;
                configs.targets[static_cast<std::size_t>(r)] = target;
                robot_ok = true;
                break;
            }
            if (!robot_ok) {
                all_ok = false;
                break;
            }
        }

        if (all_ok)
            return configs;
    }

    throw std::runtime_error(
        "Failed to sample a full collision-free Panda flat config set");
}

json loadJsonFile(const std::filesystem::path &path) {
    std::ifstream ifs(path);
    if (!ifs.good())
        throw std::runtime_error("Cannot open JSON file: " + path.string());
    json doc;
    ifs >> doc;
    return doc;
}

json builtinTaskDoc(int num_robots) {
    return json::parse(builtin::taskJsonForRobotCount(num_robots));
}

json generateTasksDoc(const AppOptions &options) {
    json doc = builtinTaskDoc(options.num_robots);
    ensureFlatTemplate(doc, options.num_robots);

    const std::string urdf_path =
        resolveResourcePath(doc.value("urdf_path", "panda/panda_spherized.urdf"));
    const std::string srdf_path =
        resolveResourcePath(doc.value("srdf_path", "panda/panda.srdf"));
    const std::string ee_link =
        options.ee_link.value_or(doc.value("ee_link", "panda_link8"));
    const double reach_radius =
        options.reachable_radius.value_or(doc.value("reachable_radius", 0.855));
    const int max_ik_attempts = options.max_ik_attempts.value_or(
        doc.value("max_ik_attempts", 400));
    const int max_restarts = options.max_restarts.value_or(
        doc.value("max_restarts", 400));

    cage_scene::Affine3dVector base_transforms =
        cage_scene::parseRobotBasesArray(doc.at("robot_bases"));
    if (static_cast<int>(base_transforms.size()) != options.num_robots)
        throw std::runtime_error("Built-in robot base count does not match --num-robots");

    comotion::CollisionChecker checker(options.collision_backend);
    checker.setCylinderObstacles(cage_scene::parseCylinderObstacles(doc));
    checker.setObstacles(parseSphereObstacles(doc));

    std::vector<std::shared_ptr<comotion::RobotModel>> robots;
    std::vector<comotion::IKSolver> ik_solvers;
    robots.reserve(base_transforms.size());
    ik_solvers.reserve(base_transforms.size());
    for (const auto &base : base_transforms) {
        auto robot = std::make_shared<comotion::RobotModel>();
        robot->loadURDF(urdf_path);
        robot->loadSRDF(srdf_path);
        robot->setBaseTransform(base);
        ik_solvers.emplace_back(*robot);
        robots.push_back(std::move(robot));
    }

    json tasks = json::array();
    std::mt19937 rng(options.task_generation_seed);
    for (int t = 0; t < options.num_tasks; ++t) {
        std::cout << "Sampling Panda flat task " << (t + 1) << "/"
                  << options.num_tasks << "..." << std::flush;
        TaskConfigs starts =
            sampleConfigSet(robots, base_transforms, ik_solvers, checker,
                            reach_radius, ee_link, rng, max_ik_attempts,
                            max_restarts);
        TaskConfigs goals =
            sampleConfigSet(robots, base_transforms, ik_solvers, checker,
                            reach_radius, ee_link, rng, max_ik_attempts,
                            max_restarts);

        json task;
        task["starts"] = starts.configs;
        task["goals"] = goals.configs;
        task["start_targets"] = json::array();
        task["goal_targets"] = json::array();
        for (int r = 0; r < options.num_robots; ++r) {
            const auto &st = starts.targets[static_cast<std::size_t>(r)];
            const auto &gt = goals.targets[static_cast<std::size_t>(r)];
            task["start_targets"].push_back({st.x(), st.y(), st.z()});
            task["goal_targets"].push_back({gt.x(), gt.y(), gt.z()});
        }
        tasks.push_back(std::move(task));
        std::cout << " done\n";
    }

    doc["num_robots"] = options.num_robots;
    doc["num_tasks"] = options.num_tasks;
    doc["ee_link"] = ee_link;
    doc["reachable_radius"] = reach_radius;
    doc["max_ik_attempts"] = max_ik_attempts;
    doc["max_restarts"] = max_restarts;
    doc["task_generation_seed"] = options.task_generation_seed;
    doc["tasks"] = std::move(tasks);
    return doc;
}

GeneratedScenario loadScenarioFromDoc(const json &doc, const AppOptions &options,
                                      const std::string &task_source) {
    const auto &tasks = doc.at("tasks");
    if (options.task_index < 0 ||
        options.task_index >= static_cast<int>(tasks.size())) {
        throw std::runtime_error(
            "--task-index " + std::to_string(options.task_index) +
            " out of range [0, " + std::to_string(tasks.size()) + ")");
    }

    const auto &task = tasks.at(static_cast<std::size_t>(options.task_index));
    GeneratedScenario generated;
    generated.num_robots = options.num_robots;
    generated.task_index = options.task_index;
    generated.task_count = static_cast<int>(tasks.size());
    generated.task_source = task_source;
    generated.starts = task.at("starts").get<std::vector<std::vector<double>>>();
    generated.goals = task.at("goals").get<std::vector<std::vector<double>>>();
    generated.start_targets = task.value("start_targets", json::array());
    generated.goal_targets = task.value("goal_targets", json::array());
    generated.robot_bases = doc.at("robot_bases");
    generated.base_transforms =
        cage_scene::parseRobotBasesArray(generated.robot_bases);
    generated.layout = doc.value("layout", json::object());
    generated.flat_base = doc.value("flat_base", json::object());
    generated.sphere_obstacles = parseSphereObstacles(doc);
    generated.cylinder_obstacles = cage_scene::parseCylinderObstacles(doc);

    if (generated.starts.size() != generated.goals.size())
        throw std::runtime_error("Panda flat task starts/goals length mismatch");
    if (static_cast<int>(generated.starts.size()) != options.num_robots) {
        throw std::runtime_error(
            "Panda flat task has " + std::to_string(generated.starts.size()) +
            " robots, expected " + std::to_string(options.num_robots));
    }
    if (static_cast<int>(generated.base_transforms.size()) != options.num_robots) {
        throw std::runtime_error(
            "Panda flat robot_bases length " +
            std::to_string(generated.base_transforms.size()) +
            " does not match --num-robots " +
            std::to_string(options.num_robots));
    }
    return generated;
}

json benchmarkContextJson(const GeneratedScenario &generated,
                          const AppOptions &options,
                          const std::string &visual_urdf,
                          const std::string &collision_urdf,
                          const std::string &srdf) {
    return json{
        {"suite", "panda_flat"},
        {"scenario", "flat"},
        {"num_robots", generated.num_robots},
        {"task_index", generated.task_index},
        {"task_count", generated.task_count},
        {"task_source", generated.task_source},
        {"seed", options.seed},
        {"time_limit_seconds", options.time_limit},
        {"resolution", options.resolution},
        {"visual_urdf_path", toRepoRelativePath(visual_urdf)},
        {"collision_urdf_path", toRepoRelativePath(collision_urdf)},
        {"srdf_path", toRepoRelativePath(srdf)},
        {"robot_bases", generated.robot_bases},
        {"starts", generated.starts},
        {"goals", generated.goals},
        {"start_targets", generated.start_targets},
        {"goal_targets", generated.goal_targets},
        {"layout", generated.layout},
        {"flat_base", generated.flat_base},
        {"obstacles",
         obstaclesJson(generated.sphere_obstacles,
                       generated.cylinder_obstacles)},
    };
}

std::string outputBasename(const GeneratedScenario &generated,
                           const AppOptions &options) {
    return "panda_flat_n" + std::to_string(generated.num_robots) + "_task" +
           std::to_string(generated.task_index) + "_seed" +
           std::to_string(options.seed);
}

std::vector<std::shared_ptr<comotion::RobotModel>>
loadRobots(const GeneratedScenario &generated, const std::string &collision_urdf,
           const std::string &srdf) {
    std::vector<std::shared_ptr<comotion::RobotModel>> robots;
    robots.reserve(generated.base_transforms.size());
    for (const auto &base : generated.base_transforms) {
        auto robot = std::make_shared<comotion::RobotModel>();
        robot->loadURDF(collision_urdf);
        robot->loadSRDF(srdf);
        robot->setBaseTransform(base);
        robots.push_back(std::move(robot));
    }
    return robots;
}

void writePathArtifacts(const TrialMetrics &metrics,
                        const GeneratedScenario &generated,
                        const std::shared_ptr<comotion::MultiRobotProblem> &problem,
                        const std::vector<comotion::Path> &paths,
                        const std::filesystem::path &output_dir,
                        const std::string &basename,
                        const std::string &visual_urdf,
                        const std::string &collision_urdf,
                        const std::string &srdf) {
    std::filesystem::create_directories(output_dir);

    auto robot_models = problem->robotModelPtrs();
    comotion::CompositePathValidationOptions validation_options;
    validation_options.check_environment = true;
    auto conflict = problem->collisionChecker().findFirstCompositePathConflict(
        paths, robot_models, validation_options);

    json out;
    out["schema_version"] = "1.0";
    out["solver"] = metrics.planner;
    out["collision_backend"] = metrics.collision_backend;
    out["planning_time_seconds"] = metrics.planning_time_seconds;
    out["solve_time_seconds"] = metrics.solve_time_seconds;
    out["compute_time_seconds"] = metrics.compute_time_seconds;
    out["validation_time_seconds"] = metrics.validation_time_seconds;
    out["sum_of_cost_timesteps"] = metrics.sum_of_cost_timesteps;
    out["makespan_timesteps"] = metrics.makespan_timesteps;
    out["verification_conflict"] = conflict
        ? json{{"scope", conflict->scope == comotion::ConflictScope::Environment
                              ? "environment"
                              : conflict->scope == comotion::ConflictScope::Self
                                    ? "self"
                                    : "inter_robot"},
               {"robot_i", conflict->robot_i},
               {"robot_j", conflict->robot_j},
               {"timestep", conflict->timestep}}
        : json(nullptr);
    out["benchmark"] = {
        {"context", metrics.benchmark_context},
        {"solution_summary", metrics.solution_summary},
    };
    out["robots"] = json::array();
    out["obstacles"] = obstaclesJson(problem->collisionChecker().obstacles(),
                                     problem->collisionChecker().cylinders());

    std::size_t timesteps = 0;
    double total_path_cost = 0.0;
    const auto &robots = problem->robots();
    for (std::size_t r = 0; r < paths.size(); ++r) {
        const auto &path = paths[r];
        timesteps = std::max(timesteps, path.size());
        total_path_cost += path.path_cost();

        json robot_json;
        robot_json["name"] = "panda_" + std::to_string(r);
        robot_json["robot_type"] = "panda";
        robot_json["urdf_path"] = toRepoRelativePath(visual_urdf);
        robot_json["visual_urdf_path"] = toRepoRelativePath(visual_urdf);
        robot_json["collision_urdf_path"] = toRepoRelativePath(collision_urdf);
        robot_json["srdf_path"] = toRepoRelativePath(srdf);
        robot_json["joint_names"] = robots[r].model->activeJointNames();
        robot_json["base_pose"] =
            basePoseJson(robots[r].model->getBaseTransform());
        robot_json["path_length"] = path.size();
        robot_json["path_cost"] = path.path_cost();
        robot_json["path"] = json::array();
        for (const auto &cfg : path)
            robot_json["path"].push_back(cfg);
        out["robots"].push_back(robot_json);

        const std::filesystem::path pth_path =
            output_dir / (basename + "_" + metrics.planner + "_robot" +
                          std::to_string(r) + ".pth");
        std::ofstream pth_ofs(pth_path);
        if (!pth_ofs.good())
            throw std::runtime_error("Unable to write path file " +
                                     pth_path.string());
        for (const auto &cfg : path) {
            for (std::size_t d = 0; d < cfg.size(); ++d) {
                if (d > 0)
                    pth_ofs << " ";
                pth_ofs << cfg[d];
            }
            pth_ofs << "\n";
        }
    }

    out["timesteps"] = timesteps;
    out["total_path_cost"] = total_path_cost;

    writeJson(out,
              output_dir / (basename + "_" + metrics.planner + "_result.json"),
              2);
}

TrialMetrics runPlanner(const GeneratedScenario &generated,
                        const AppOptions &options,
                        const json &benchmark_context,
                        const std::shared_ptr<comotion::MultiRobotProblem> &problem,
                        const std::shared_ptr<comotion::MultiRobotPlanner> &planner,
                        const std::string &planner_name,
                        const std::string &visual_urdf,
                        const std::string &collision_urdf,
                        const std::string &srdf) {
    comotion::seedOmplGlobalFromUserPlanningSeed(options.seed);
    planner->setPlanningSeed(options.seed);
    planner->setProblem(problem);

    if (g_app_verbose)
        std::cout << "Running " << planner_name << "\n";

    const auto cpu0 = common::processCpuUsageSnapshot();
    const auto t0 = std::chrono::steady_clock::now();
    const auto status = planner->solve(options.time_limit);
    const auto t1 = std::chrono::steady_clock::now();
    const auto cpu1 = common::processCpuUsageSnapshot();
    const double solve_time = std::chrono::duration<double>(t1 - t0).count();
    const double compute_time =
        common::elapsedProcessTreeCpuSeconds(cpu0, cpu1);

    TrialMetrics metrics;
    metrics.planner = planner_name;
    metrics.collision_backend = backendName(problem->collisionChecker().backend());
    metrics.planner_status = status.asString();
    metrics.success = (status == ompl::base::PlannerStatus::EXACT_SOLUTION);
    metrics.planning_time_seconds = solve_time;
    metrics.solve_time_seconds = solve_time;
    metrics.compute_time_seconds = compute_time;
    metrics.sum_of_cost_timesteps = planner->sumOfCostTimesteps()
                                        ? json(*planner->sumOfCostTimesteps())
                                        : json(nullptr);
    metrics.makespan_timesteps = planner->makespanTimesteps()
                                     ? json(*planner->makespanTimesteps())
                                     : json(nullptr);
    metrics.planner_stats = planner->plannerStatsJson();
    metrics.benchmark_context = benchmark_context;
    metrics.solution_summary = common::solutionSummaryJson(metrics);

    std::cout << "Status: " << status.asString() << "\n";
    std::cout << "Total planning time: " << solve_time << " seconds\n";
    std::cout << "Total compute time: " << compute_time << " seconds\n";

    const std::string basename = outputBasename(generated, options);
    if (options.metrics_json_path) {
        writeJson(metrics.toJson(), *options.metrics_json_path, 0);
    }

    if (options.output_paths) {
        if (metrics.success) {
            writePathArtifacts(metrics, generated, problem,
                               planner->getSolutionPaths(),
                               options.output_dir, basename, visual_urdf,
                               collision_urdf, srdf);
        } else if (g_app_verbose) {
            std::cout << "No exact solution; skipping path artifacts\n";
        }
    }

    return metrics;
}

void printUsage(const char *prog) {
    std::cout
        << "Usage: " << prog
        << " --num-robots <4|8|16> [--task-index <i>] [options]\n"
        << "  --num-robots <N>       Built-in/custom task robot count\n"
        << "  --task-index <i>       Zero-based task index (default: 0)\n"
        << "  --task-file <path>     Read custom/generated Panda flat task JSON\n"
        << "  --generate-tasks-json <path>\n"
        << "                         Generate IK tasks, save JSON, then run from it\n"
        << "  --generate-only        With --generate-tasks-json, write JSON and exit\n"
        << "  --num-tasks <n>        Number of generated tasks (default: 5)\n"
        << "  --task-generation-seed <n>  Seed for IK task generation (default: --seed)\n"
        << "  --max-ik-attempts <n>  Override generation attempts per robot\n"
        << "  --max-restarts <n>     Override generation full-set restarts\n"
        << "  --reachable-radius <r> Override generation reach radius\n"
        << "  --ee-link <name>       Override IK end-effector link\n"
        << "  --algorithm <name>     composite, composite_rrtstar, composite_prmstar, composite_aorrtc,\n"
        << "                         cooperative_composite, prioritized, drrt, drrt_star,\n"
        << "                         ao_drrt,\n"
        << "                         arc, parallel_arc, stcbs (default: arc)\n"
        << "  --collision-backend <b> sphere, fcl, vamp (default: vamp)\n"
        << "  --time-limit <sec>     Planning time limit (default: 60)\n"
        << "  --seed <n>             Planning seed (default: 0)\n"
        << "  --resolution <n>       Timesteps per second (default: 128)\n"
        << "  --metrics-json <path>  Write compact trial metrics JSON\n"
        << "  --output-paths         Write visualization result JSON and .pth files\n"
        << "  --output-dir <dir>     Output directory for path artifacts\n"
        << "      (default: benchmarks/results/panda_flat)\n"
        << "  --urdf <path>          Override planning URDF\n"
        << "  --srdf <path>          Override SRDF\n"
        << "  --strrt-initial-batch-size <n>  Initial STRRT batch size (default: 4096)\n"
        << "  --strrt-initial-time-factor <x> Initial STRRT time bound factor (default: 4)\n"
        << "  --strrt-time-bound-factor-increase <x> STRRT time bound growth factor (default: 2)\n"
        << "  --strrt-shuffle-priority-order  Shuffle PrioritizedSTRRT priority order from --seed\n"
        << "  --strrt-return-first-solution <0|1> Stop each STRRT solve at first solution (default: 1)\n"
        << "  --strrt-rewiring <off|radius|knearest> STRRT rewiring mode (default: off)\n"
        << "  --drrt-roadmap-size <n> Override dRRT* roadmap size (default: 200)\n"
        << "  --drrt-iterations-per-batch <n> Override dRRT* batch size (default: 8)\n"
        << "  --drrt-cost-metric <sum_of_costs|composite_l2|makespan>\n"
        << "  --drrt-tensor-search <drrt|astar|lazy_astar> (default: drrt)\n"
        << "  --drrt-exclude-roadmap-build-time\n"
        << "                         Give dRRT tensor search the full time limit after PRM* build\n"
        << "  --arc-initial-window <n>\n"
        << "  --arc-expansion-step <n>\n"
        << "  --arc-local-composite-max-samples <n>\n"
        << "  --arc-local-composite-use-makespan-metric\n"
        << "  --arc-simplify-initial-solutions / --no-arc-simplify-initial-solutions (default: on)\n"
        << "  --arc-simplify-conflict-solutions / --no-arc-simplify-conflict-solutions (default: off)\n"
        << "  --composite-rrt-use-makespan-metric\n"
        << "  --composite-rrt-simplify Run path simplification after Composite RRT-C succeeds\n"
        << "  --aorrtc-restart-effort <n> Set CompositeAORRTC sample/vertex caps\n"
        << "  --aorrtc-max-internal-samples <n>\n"
        << "  --aorrtc-max-internal-vertices <n>\n"
        << "  --arc-local-solvers <both|prioritized|composite> (default: both)\n"
        << "  --arc-local-prioritized-max-iterations <n> (default: 5; 0 disables cap)\n"
        << "  --ao-arc-local-bound-epsilon-timesteps <n> (default: 1; 0 disables)\n"
        << "  --cooperative-rrt-worker-threads <n>\n"
        << "  --or-parallel-worker-processes <n>\n"
        << "  --parallel-arc-worker-processes <n>\n"
        << "  --parallel-arc-parallel-initial-plans / --no-parallel-arc-parallel-initial-plans (default: on)\n"
        << "  --parallel-arc-initial-solution-or / --no-parallel-arc-initial-solution-or (default: off)\n"
        << "  --parallel-arc-repair-duplicate-attempts / --no-parallel-arc-repair-duplicate-attempts (default: on)\n"
        << "  --parallel-arc-strategy <synchronous|asynchronous>\n"
        << "  --parallel-arc-conflict-strategy <greedy|spatial_distribution>\n"
        << "  --parallel-arc-conflict-find-mode <sequential|segment_parallel>\n"
        << "  --parallel-arc-conflict-batch-mode <optimistic|independent_only>\n"
        << "  --parallel-arc-conflict-find-horizon <n>\n"
        << "  --exit-nonzero-without-exact-solution\n"
        << "  --verbose\n";
}

AppOptions parseArgs(int argc, char **argv) {
    AppOptions options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--num-robots") {
            options.num_robots = std::stoi(requireValue(i, argc, argv, arg));
        } else if (arg == "--task-index" || arg == "--task") {
            options.task_index = std::stoi(requireValue(i, argc, argv, arg));
        } else if (arg == "--num-tasks") {
            options.num_tasks = std::stoi(requireValue(i, argc, argv, arg));
        } else if (arg == "--task-file") {
            options.task_file = requireValue(i, argc, argv, arg);
        } else if (arg == "--generate-tasks-json") {
            options.generate_tasks_json = requireValue(i, argc, argv, arg);
        } else if (arg == "--generate-only") {
            options.generate_only = true;
        } else if (arg == "--task-generation-seed") {
            options.task_generation_seed = static_cast<std::uint32_t>(
                std::stoul(requireValue(i, argc, argv, arg)));
            options.task_generation_seed_explicit = true;
        } else if (arg == "--max-ik-attempts") {
            options.max_ik_attempts =
                std::stoi(requireValue(i, argc, argv, arg));
        } else if (arg == "--max-restarts") {
            options.max_restarts = std::stoi(requireValue(i, argc, argv, arg));
        } else if (arg == "--reachable-radius") {
            options.reachable_radius =
                std::stod(requireValue(i, argc, argv, arg));
        } else if (arg == "--ee-link") {
            options.ee_link = requireValue(i, argc, argv, arg);
        } else if (arg == "--algorithm") {
            options.algorithm = requireValue(i, argc, argv, arg);
        } else if (arg == "--collision-backend") {
            options.collision_backend =
                parseCollisionBackend(requireValue(i, argc, argv, arg));
        } else if (arg == "--time-limit") {
            options.time_limit = std::stod(requireValue(i, argc, argv, arg));
        } else if (arg == "--seed") {
            options.seed = static_cast<std::uint32_t>(
                std::stoul(requireValue(i, argc, argv, arg)));
        } else if (arg == "--resolution" || arg == "-r") {
            options.resolution = static_cast<std::size_t>(
                std::stoul(requireValue(i, argc, argv, arg)));
        } else if (arg == "--metrics-json") {
            options.metrics_json_path = requireValue(i, argc, argv, arg);
        } else if (arg == "--output-paths") {
            options.output_paths = true;
        } else if (arg == "--output-dir") {
            options.output_dir = requireValue(i, argc, argv, arg);
        } else if (arg == "--urdf") {
            options.urdf_rel = requireValue(i, argc, argv, arg);
            options.urdf_explicit = true;
        } else if (arg == "--srdf") {
            options.srdf_rel = requireValue(i, argc, argv, arg);
            options.srdf_explicit = true;
        } else if (arg == "--strrt-initial-batch-size") {
            options.strrt_initial_batch_size = static_cast<unsigned int>(
                std::stoul(requireValue(i, argc, argv, arg)));
        } else if (arg == "--strrt-initial-time-factor") {
            options.strrt_initial_time_factor =
                std::stod(requireValue(i, argc, argv, arg));
        } else if (arg == "--strrt-time-bound-factor-increase") {
            options.strrt_time_bound_factor_increase =
                std::stod(requireValue(i, argc, argv, arg));
        } else if (arg == "--strrt-shuffle-priority-order") {
            options.strrt_shuffle_priority_order = true;
        } else if (arg == "--strrt-return-first-solution") {
            options.strrt_return_first_solution =
                common::parseBoolValue(requireValue(i, argc, argv, arg));
        } else if (arg == "--strrt-rewiring") {
            options.strrt_rewiring = requireValue(i, argc, argv, arg);
        } else if (arg == "--drrt-roadmap-size") {
            options.drrt_roadmap_size =
                std::stoi(requireValue(i, argc, argv, arg));
        } else if (arg == "--drrt-iterations-per-batch") {
            options.drrt_iterations_per_batch =
                std::stoi(requireValue(i, argc, argv, arg));
        } else if (arg == "--drrt-cost-metric") {
            options.drrt_cost_metric = requireValue(i, argc, argv, arg);
        } else if (arg == "--drrt-tensor-search") {
            options.drrt_tensor_search = requireValue(i, argc, argv, arg);
        } else if (arg == "--drrt-exclude-roadmap-build-time") {
            options.drrt_exclude_roadmap_build_time = true;
        } else if (arg == "--composite-rrt-use-makespan-metric") {
            options.composite_rrt_use_makespan_metric = true;
        } else if (arg == "--composite-rrt-simplify") {
            options.composite_rrt_simplify_solution = true;
        } else if (arg == "--aorrtc-restart-effort") {
            const auto value = static_cast<std::size_t>(
                std::stoull(requireValue(i, argc, argv, arg)));
            options.composite_aorrtc_max_internal_samples = value;
            options.composite_aorrtc_max_internal_vertices = value;
        } else if (arg == "--aorrtc-max-internal-samples") {
            options.composite_aorrtc_max_internal_samples =
                static_cast<std::size_t>(
                    std::stoull(requireValue(i, argc, argv, arg)));
        } else if (arg == "--aorrtc-max-internal-vertices") {
            options.composite_aorrtc_max_internal_vertices =
                static_cast<std::size_t>(
                    std::stoull(requireValue(i, argc, argv, arg)));
        } else if (arg == "--cooperative-rrt-worker-threads") {
            options.cooperative_rrt_worker_threads =
                static_cast<unsigned int>(
                    std::stoul(requireValue(i, argc, argv, arg)));
        } else if (arg == "--arc-initial-window") {
            options.arc_initial_window =
                std::stoi(requireValue(i, argc, argv, arg));
        } else if (arg == "--arc-expansion-step") {
            options.arc_expansion_step =
                std::stoi(requireValue(i, argc, argv, arg));
        } else if (arg == "--arc-local-composite-max-samples") {
            options.arc_local_composite_max_samples = static_cast<unsigned int>(
                std::stoul(requireValue(i, argc, argv, arg)));
        } else if (arg == "--arc-local-composite-use-makespan-metric") {
            options.arc_local_composite_use_makespan_metric = true;
        } else if (arg == "--arc-simplify-initial-solutions") {
            options.arc_simplify_initial_solutions = true;
        } else if (arg == "--no-arc-simplify-initial-solutions") {
            options.arc_simplify_initial_solutions = false;
        } else if (arg == "--arc-simplify-conflict-solutions") {
            options.arc_simplify_conflict_solutions = true;
        } else if (arg == "--no-arc-simplify-conflict-solutions") {
            options.arc_simplify_conflict_solutions = false;
        } else if (arg == "--arc-local-solvers") {
            options.arc_local_solvers = requireValue(i, argc, argv, arg);
        } else if (arg == "--arc-local-prioritized-max-iterations") {
            options.arc_local_prioritized_max_iterations =
                static_cast<unsigned int>(
                    std::stoul(requireValue(i, argc, argv, arg)));
        } else if (arg == "--ao-arc-local-bound-epsilon-timesteps") {
            options.ao_arc_local_bound_epsilon_timesteps =
                static_cast<std::uint64_t>(
                    std::stoull(requireValue(i, argc, argv, arg)));
        } else if (arg == "--or-parallel-worker-processes") {
            options.or_parallel_worker_processes = static_cast<unsigned int>(
                std::stoul(requireValue(i, argc, argv, arg)));
        } else if (arg == "--parallel-arc-worker-processes") {
            options.parallel_arc_worker_processes = static_cast<unsigned int>(
                std::stoul(requireValue(i, argc, argv, arg)));
        } else if (arg == "--parallel-arc-parallel-initial-plans") {
            options.parallel_arc_parallel_initial_plans = true;
        } else if (arg == "--no-parallel-arc-parallel-initial-plans") {
            options.parallel_arc_parallel_initial_plans = false;
        } else if (arg == "--parallel-arc-initial-solution-or") {
            options.parallel_arc_initial_solution_or = true;
        } else if (arg == "--no-parallel-arc-initial-solution-or") {
            options.parallel_arc_initial_solution_or = false;
        } else if (arg == "--parallel-arc-repair-duplicate-attempts") {
            options.parallel_arc_repair_duplicate_attempts = true;
        } else if (arg == "--no-parallel-arc-repair-duplicate-attempts") {
            options.parallel_arc_repair_duplicate_attempts = false;
        } else if (arg == "--parallel-arc-strategy") {
            options.parallel_arc_strategy = requireValue(i, argc, argv, arg);
        } else if (arg == "--parallel-arc-conflict-strategy") {
            options.parallel_arc_conflict_strategy =
                requireValue(i, argc, argv, arg);
        } else if (arg == "--parallel-arc-conflict-find-mode") {
            options.parallel_arc_conflict_find_mode =
                requireValue(i, argc, argv, arg);
        } else if (arg == "--parallel-arc-conflict-batch-mode") {
            options.parallel_arc_conflict_batch_mode =
                requireValue(i, argc, argv, arg);
        } else if (arg == "--parallel-arc-conflict-find-horizon") {
            options.parallel_arc_conflict_find_horizon =
                static_cast<std::size_t>(
                    std::stoull(requireValue(i, argc, argv, arg)));
        } else if (arg == "--stcbs-max-ct-nodes") {
            options.stcbs_max_ct_nodes =
                std::stoi(requireValue(i, argc, argv, arg));
        } else if (arg == "--stcbs-max-samples") {
            options.stcbs_max_samples =
                std::stoi(requireValue(i, argc, argv, arg));
        } else if (arg == "--exit-nonzero-without-exact-solution") {
            options.exit_nonzero_without_exact_solution = true;
        } else if (arg == "--verbose" || arg == "-v") {
            g_app_verbose = true;
        } else if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            std::exit(0);
        } else {
            throw std::runtime_error("Unknown option: " + arg);
        }
    }

    if (options.num_robots == 0)
        throw std::runtime_error("--num-robots is required");
    if (options.num_robots != 4 && options.num_robots != 8 &&
        options.num_robots != 16)
        throw std::runtime_error("--num-robots must be one of 4, 8, or 16");
    if (options.task_index < 0)
        throw std::runtime_error("--task-index must be non-negative");
    if (options.num_tasks <= 0)
        throw std::runtime_error("--num-tasks must be at least 1");
    if (options.time_limit <= 0.0 && !options.generate_only)
        throw std::runtime_error("--time-limit must be positive");
    if (options.resolution == 0)
        throw std::runtime_error("--resolution must be at least 1");
    if (options.generate_only && !options.generate_tasks_json)
        throw std::runtime_error("--generate-only requires --generate-tasks-json");
    if (options.generate_tasks_json && options.task_file)
        throw std::runtime_error(
            "Use either --generate-tasks-json or --task-file, not both");
    if (options.max_ik_attempts && *options.max_ik_attempts <= 0)
        throw std::runtime_error("--max-ik-attempts must be positive");
    if (options.max_restarts && *options.max_restarts <= 0)
        throw std::runtime_error("--max-restarts must be positive");
    if (options.reachable_radius && *options.reachable_radius <= 0.0)
        throw std::runtime_error("--reachable-radius must be positive");
    if (options.strrt_initial_batch_size == 0)
        throw std::runtime_error("--strrt-initial-batch-size must be at least 1");
    if (options.strrt_initial_time_factor <= 1.0)
        throw std::runtime_error(
            "--strrt-initial-time-factor must be greater than 1.0");
    if (options.strrt_time_bound_factor_increase <= 1.0)
        throw std::runtime_error(
            "--strrt-time-bound-factor-increase must be greater than 1.0");
    (void)common::parseStrrtRewiring(options.strrt_rewiring);
    if (options.drrt_roadmap_size < 2)
        throw std::runtime_error("--drrt-roadmap-size must be at least 2");
    if (options.drrt_iterations_per_batch < 1)
        throw std::runtime_error("--drrt-iterations-per-batch must be at least 1");
    (void)common::parseDrrtCostMetric(options.drrt_cost_metric);
    (void)common::parseDrrtTensorSearchMode(options.drrt_tensor_search);
    if (options.composite_aorrtc_max_internal_samples == 0)
        throw std::runtime_error(
            "--aorrtc-max-internal-samples must be at least 1");
    if (options.composite_aorrtc_max_internal_vertices == 0)
        throw std::runtime_error(
            "--aorrtc-max-internal-vertices must be at least 1");
    if (options.cooperative_rrt_worker_threads == 0)
        throw std::runtime_error(
            "--cooperative-rrt-worker-threads must be at least 1");
    if (options.arc_initial_window < 1)
        throw std::runtime_error("--arc-initial-window must be at least 1");
    if (options.arc_expansion_step < 1)
        throw std::runtime_error("--arc-expansion-step must be at least 1");
    (void)parseArcLocalSolverMode(options.arc_local_solvers);
    if (options.or_parallel_worker_processes == 0)
        throw std::runtime_error("--or-parallel-worker-processes must be at least 1");
    if (options.parallel_arc_worker_processes == 0)
        throw std::runtime_error("--parallel-arc-worker-processes must be at least 1");
    if (options.parallel_arc_conflict_find_mode == "segment_parallel" &&
        options.parallel_arc_conflict_find_horizon == 0) {
        throw std::runtime_error(
            "--parallel-arc-conflict-find-horizon must be at least 1 for "
            "segment_parallel mode");
    }
    if (options.stcbs_max_ct_nodes < 1)
        throw std::runtime_error("--stcbs-max-ct-nodes must be at least 1");
    if (options.stcbs_max_samples < 1)
        throw std::runtime_error("--stcbs-max-samples must be at least 1");

    if (!options.task_generation_seed_explicit)
        options.task_generation_seed = options.seed;
    return options;
}

} // namespace

int main(int argc, char **argv) {
    try {
        setExecutablePath(argc > 0 ? argv[0] : nullptr);
        AppOptions options = parseArgs(argc, argv);

        json task_doc;
        std::string task_source;
        if (options.generate_tasks_json) {
            task_doc = generateTasksDoc(options);
            writeJson(task_doc, *options.generate_tasks_json, 2);
            std::cout << "Wrote " << options.num_tasks << " Panda flat task(s) to "
                      << *options.generate_tasks_json << "\n";
            task_source = *options.generate_tasks_json;
            if (options.generate_only)
                return 0;
        } else if (options.task_file) {
            task_doc = loadJsonFile(*options.task_file);
            task_source = *options.task_file;
        } else {
            task_doc = builtinTaskDoc(options.num_robots);
            task_source = "builtin";
        }
        ensureFlatTemplate(task_doc, options.num_robots);

        const GeneratedScenario generated =
            loadScenarioFromDoc(task_doc, options, task_source);

        const std::string collision_urdf = resolveResourcePath(options.urdf_rel);
        const std::string visual_urdf = collision_urdf;
        const std::string srdf = resolveResourcePath(options.srdf_rel);

        const auto robots = loadRobots(generated, collision_urdf, srdf);

        auto problem =
            std::make_shared<comotion::MultiRobotProblem>(options.collision_backend);
        for (int i = 0; i < generated.num_robots; ++i) {
            const auto index = static_cast<std::size_t>(i);
            problem->addRobot(robots[index], generated.starts[index],
                              generated.goals[index]);
        }
        problem->setObstacles(generated.sphere_obstacles);
        problem->setCylinderObstacles(generated.cylinder_obstacles);
        problem->setResolution(options.resolution);

        const json context =
            benchmarkContextJson(generated, options, visual_urdf,
                                 collision_urdf, srdf);

        PlannerBlueprint blueprint =
            common::makePlannerBlueprint(options, g_app_verbose);
        blueprint.prepare_problem(problem);

        std::shared_ptr<comotion::MultiRobotPlanner> planner;
        std::string planner_name = blueprint.planner_name;
        if (options.or_parallel_worker_processes > 1 &&
            blueprint.allow_outer_or_parallel) {
            auto or_planner = std::make_shared<comotion::OrParallelPlanner>();
            or_planner->setBasePlannerName(blueprint.planner_name);
            or_planner->setPlannerFactory(blueprint.factory);
            or_planner->setWorkerProcesses(options.or_parallel_worker_processes);
            planner = or_planner;
            planner_name = or_planner->name();
        } else {
            planner = blueprint.factory();
        }

        const TrialMetrics metrics =
            runPlanner(generated, options, context, problem, planner,
                       planner_name, visual_urdf, collision_urdf, srdf);
        if (options.exit_nonzero_without_exact_solution && !metrics.success)
            return 1;
        return 0;
    } catch (const std::exception &ex) {
        std::cerr << "error: " << ex.what() << "\n";
        return 1;
    }
}
