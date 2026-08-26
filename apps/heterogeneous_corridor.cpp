#include "benchmark_app_common.hpp"
#include "cage_scene_json.hpp"

#include "comotion/collision/CollisionChecker.h"
#include "comotion/planning/MultiRobotPlanner.h"
#include "comotion/planning/MultiRobotProblem.h"
#include "comotion/planning/OrParallelPlanner.h"
#include "comotion/planning/Path.h"
#include "comotion/planning/PlanningRng.h"
#include "comotion/planning/PrioritizedSTRRT.h"
#include "comotion/robot/FlyingSphere.h"
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
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

using json = nlohmann::json;
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
    int num_pandas = 0;
    int num_spheres = 0;
    int task_index = 0;
    std::string algorithm = "arc";
    comotion::CollisionChecker::Backend collision_backend =
        comotion::CollisionChecker::Backend::Vamp;
    std::string vamp_validation_strategy = "combined_rake";
    double time_limit = 60.0;
    std::uint32_t seed = 0;
    std::size_t resolution = 128;
    std::string output_dir = "benchmarks/results/heterogeneous_corridor";
    bool output_paths = false;
    bool track_arc_history = false;
    bool output_endpoint_paths = false;
    std::optional<std::string> metrics_json_path;
    bool exit_nonzero_without_exact_solution = false;

    std::optional<std::string> task_file;
    std::string urdf_rel = "panda/panda_spherized.urdf";
    bool urdf_explicit = false;
    std::string srdf_rel = "panda/panda.srdf";

    unsigned int strrt_initial_batch_size = 4096;
    double strrt_initial_time_factor = 4.0;
    double strrt_time_bound_factor_increase = 2.0;
    bool strrt_shuffle_priority_order = true;
    bool strrt_return_first_solution = true;
    std::string strrt_rewiring = "off";
    int drrt_roadmap_size = 200;
    int drrt_iterations_per_batch = 8;
    std::string drrt_cost_metric = "sum_of_costs";
    std::string drrt_tensor_search = "drrt";
    std::string drrt_local_connector = "prioritized";
    bool drrt_exclude_roadmap_build_time = false;
    double composite_rrt_range = 0.0;
    bool composite_rrt_simplify_solution = false;
    bool composite_rrt_use_makespan_metric = false;
    std::size_t composite_aorrtc_max_internal_samples = 10000;
    std::size_t composite_aorrtc_max_internal_vertices = 10000;
    unsigned int cooperative_rrt_worker_threads = 2;
    int arc_initial_window = 200;
    double arc_expansion_step = 200.0;
    std::string arc_expansion_policy = "linear";
    std::string arc_expansion_multipliers = "1,1,1,2,2,2,4,8";
    std::optional<std::string> arc_initial_valid_expansion_policy;
    std::optional<double> arc_initial_valid_expansion_step;
    std::optional<std::string> arc_initial_valid_expansion_multipliers;
    bool arc_initial_valid_expansion_symmetric = true;
    double arc_cspace_bound_margin = 2.0;
    double arc_min_cspace_bound_range = 2.0;
    unsigned int arc_simplification_max_shortcut_steps = 128;
    unsigned int arc_simplification_max_empty_steps = 32;
    unsigned int arc_simplification_max_smooth_steps = 1;
    unsigned int arc_simplification_max_passes = 1;
    bool arc_conflict_simplification_options_explicit = false;
    unsigned int arc_conflict_simplification_max_shortcut_steps = 128;
    unsigned int arc_conflict_simplification_max_empty_steps = 32;
    unsigned int arc_conflict_simplification_max_smooth_steps = 1;
    unsigned int arc_conflict_simplification_max_passes = 1;
    unsigned int arc_local_composite_max_samples = 500000;
    double arc_local_composite_range = 0.0;
    bool arc_local_composite_use_makespan_metric = false;
    bool arc_simplify_initial_solutions = true;
    bool arc_simplify_conflict_solutions = false;
    std::string arc_local_solvers = "composite";
    unsigned int arc_local_prioritized_max_iterations = 5;
    bool arc_local_prioritized_return_first_solution = true;
    std::string arc_local_prioritized_rewiring = "knearest";
    bool arc_local_prioritized_persist_at_goal = false;
    std::uint64_t ao_arc_local_bound_epsilon_timesteps = 1;
    unsigned int or_parallel_worker_processes = 1;
    unsigned int parallel_arc_worker_processes = 2;
    bool parallel_arc_parallel_initial_plans = true;
    bool parallel_arc_initial_solution_or = false;
    bool parallel_arc_repair_duplicate_attempts = true;
    std::string parallel_arc_strategy = "synchronous";
    std::string parallel_arc_conflict_strategy = "greedy";
    std::string parallel_arc_conflict_find_mode = "segment_parallel";
    std::string parallel_arc_conflict_find_assignment =
        "cyclic_cover_greedy";
    std::size_t parallel_arc_conflict_find_horizon = 400;
    int stcbs_max_ct_nodes = 5000;
    int stcbs_max_samples = 75000;
};

struct GeneratedScenario {
    int num_robots = 0;
    int num_pandas = 0;
    int num_spheres = 0;
    int task_index = 0;
    int task_count = 0;
    std::string task_source;
    std::vector<std::vector<double>> starts;
    std::vector<std::vector<double>> goals;
    std::vector<std::vector<double>> panda_starts;
    std::vector<std::vector<double>> panda_goals;
    std::vector<std::vector<double>> sphere_starts;
    std::vector<std::vector<double>> sphere_goals;
    json start_targets = json::array();
    json goal_targets = json::array();
    json robot_bases = json::array();
    json robot_definitions = json::array();
    cage_scene::Affine3dVector base_transforms;
    json layout = json::object();
    json flat_base = nullptr;
    std::vector<std::string> robot_names;
    std::vector<std::string> robot_types;
    std::vector<double> sphere_radii;
    std::vector<double> sphere_env_min;
    std::vector<double> sphere_env_max;
    std::vector<comotion::ObstacleSphere> sphere_obstacles;
    std::vector<comotion::ObstacleCylinder> cylinder_obstacles;
};

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
    for (const char *prefix : prefixes)
        candidates.emplace_back(std::string(prefix) + relative);
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

json loadJsonFile(const std::filesystem::path &path) {
    std::ifstream ifs(path);
    if (!ifs.good())
        throw std::runtime_error("Cannot open JSON file: " + path.string());
    json doc;
    ifs >> doc;
    return doc;
}

std::string defaultTaskResource(int num_pandas) {
    switch (num_pandas) {
    case 4:
        return "benchmarks/heterogeneous_four_pandas_eight_spheres_tasks.json";
    case 8:
        return "benchmarks/heterogeneous_eight_pandas_sixteen_spheres_tasks.json";
    case 16:
        return "benchmarks/heterogeneous_sixteen_pandas_thirtytwo_spheres_tasks.json";
    default:
        throw std::runtime_error(
            "Heterogeneous corridor supports 4, 8, or 16 Panda robots");
    }
}

GeneratedScenario loadScenarioFromDoc(const json &doc, const AppOptions &options,
                                      const std::string &task_source) {
    const auto &environment = doc.at("environment");
    auto env_min = environment.at("min").get<std::vector<double>>();
    auto env_max = environment.at("max").get<std::vector<double>>();
    if (env_min.size() < 3 || env_max.size() < 3)
        throw std::runtime_error(
            "environment min/max must each contain [x,y,z]");

    const auto &robot_defs = doc.at("robots");
    if (!robot_defs.is_array() || robot_defs.empty())
        throw std::runtime_error(
            "Heterogeneous task must contain a non-empty robots array");

    const auto &tasks = doc.at("tasks");
    if (options.task_index < 0 ||
        options.task_index >= static_cast<int>(tasks.size())) {
        throw std::runtime_error(
            "--task-index " + std::to_string(options.task_index) +
            " out of range [0, " + std::to_string(tasks.size()) + ")");
    }

    const auto &task = tasks.at(static_cast<std::size_t>(options.task_index));
    GeneratedScenario generated;
    generated.task_index = options.task_index;
    generated.task_count = static_cast<int>(tasks.size());
    generated.task_source = task_source;
    generated.starts =
        task.at("starts").get<std::vector<std::vector<double>>>();
    generated.goals =
        task.at("goals").get<std::vector<std::vector<double>>>();
    generated.robot_definitions = robot_defs;
    generated.layout = {
        {"source", "canonical_task_file"},
        {"ordering", "task_file_robot_order"},
    };

    if (generated.starts.size() != generated.goals.size())
        throw std::runtime_error("Heterogeneous task starts/goals length mismatch");
    if (generated.starts.size() != robot_defs.size())
        throw std::runtime_error(
            "Heterogeneous robots array must match starts/goals length");

    bool saw_sphere = false;
    double max_sphere_radius = 0.0;
    std::vector<std::size_t> sphere_indices;
    for (std::size_t index = 0; index < robot_defs.size(); ++index) {
        const auto &robot = robot_defs.at(index);
        const std::string type = robot.at("type").get<std::string>();
        generated.robot_types.push_back(type);
        generated.robot_names.push_back(
            robot.value("name", type + "_" + std::to_string(index)));

        if (type == "panda") {
            if (saw_sphere)
                throw std::runtime_error(
                    "Canonical heterogeneous tasks require Pandas before spheres");
            if (!robot.contains("base"))
                throw std::runtime_error("Panda robot is missing base");
            generated.robot_bases.push_back(robot.at("base"));
            generated.panda_starts.push_back(generated.starts[index]);
            generated.panda_goals.push_back(generated.goals[index]);
            if (generated.starts[index].size() != 7 ||
                generated.goals[index].size() != 7)
                throw std::runtime_error(
                    "Panda starts/goals must contain seven joint values");
            ++generated.num_pandas;
        } else if (type == "sphere") {
            saw_sphere = true;
            const double radius = robot.at("radius").get<double>();
            if (radius <= 0.0)
                throw std::runtime_error("Sphere radius must be positive");
            if (generated.starts[index].size() < 3 ||
                generated.goals[index].size() < 3)
                throw std::runtime_error(
                    "Sphere starts/goals must contain [x,y,z]");
            generated.sphere_radii.push_back(radius);
            generated.sphere_starts.push_back(generated.starts[index]);
            generated.sphere_goals.push_back(generated.goals[index]);
            sphere_indices.push_back(index);
            max_sphere_radius = std::max(max_sphere_radius, radius);
            ++generated.num_spheres;
        } else {
            throw std::runtime_error("Unknown heterogeneous robot type: " + type);
        }
    }
    generated.num_robots =
        static_cast<int>(generated.robot_types.size());
    generated.base_transforms =
        cage_scene::parseRobotBasesArray(generated.robot_bases);

    if (generated.num_pandas != options.num_pandas) {
        throw std::runtime_error(
            "Heterogeneous task has " + std::to_string(generated.num_pandas) +
            " robots, expected " + std::to_string(options.num_pandas));
    }
    if (generated.num_spheres != options.num_spheres) {
        throw std::runtime_error(
            "Heterogeneous task has " + std::to_string(generated.num_spheres) +
            " spheres, expected " + std::to_string(options.num_spheres));
    }

    if (!sphere_indices.empty()) {
        double min_x = std::numeric_limits<double>::infinity();
        double max_x = -std::numeric_limits<double>::infinity();
        double min_y = std::numeric_limits<double>::infinity();
        double max_y = -std::numeric_limits<double>::infinity();
        for (const std::size_t index : sphere_indices) {
            for (const auto &state :
                 {generated.starts[index], generated.goals[index]}) {
                min_x = std::min(min_x, state[0]);
                max_x = std::max(max_x, state[0]);
                min_y = std::min(min_y, state[1]);
                max_y = std::max(max_y, state[1]);
            }
        }
        const double buffer = 2.0 * max_sphere_radius;
        env_min[0] = min_x - buffer;
        env_min[1] = min_y - buffer;
        env_max[0] = max_x + buffer;
        env_max[1] = max_y + buffer;
    }
    generated.sphere_env_min = std::move(env_min);
    generated.sphere_env_max = std::move(env_max);
    generated.sphere_obstacles = parseSphereObstacles(doc);
    generated.cylinder_obstacles = cage_scene::parseCylinderObstacles(doc);
    return generated;
}

json benchmarkContextJson(const GeneratedScenario &generated,
                          const AppOptions &options,
                          const std::string &visual_urdf,
                          const std::string &collision_urdf,
                          const std::string &srdf) {
    return json{
        {"suite", "heterogeneous_corridor"},
        {"scenario", "panda_sphere_corridor"},
        {"num_robots", generated.num_robots},
        {"num_pandas", generated.num_pandas},
        {"num_spheres", generated.num_spheres},
        {"robot_order", "pandas_first_spheres_second"},
        {"task_index", generated.task_index},
        {"task_count", generated.task_count},
        {"task_source", generated.task_source},
        {"seed", options.seed},
        {"time_limit_seconds", options.time_limit},
        {"vamp_validation_strategy", options.vamp_validation_strategy},
        {"resolution", options.resolution},
        {"visual_urdf_path", toRepoRelativePath(visual_urdf)},
        {"collision_urdf_path", toRepoRelativePath(collision_urdf)},
        {"srdf_path", toRepoRelativePath(srdf)},
        {"robot_definitions", generated.robot_definitions},
        {"sphere_radii", generated.sphere_radii},
        {"sphere_environment",
         {{"min", generated.sphere_env_min}, {"max", generated.sphere_env_max}}},
        {"robot_bases", generated.robot_bases},
        {"panda_starts", generated.panda_starts},
        {"panda_goals", generated.panda_goals},
        {"sphere_starts", generated.sphere_starts},
        {"sphere_goals", generated.sphere_goals},
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
    return "heterogeneous_corridor_p" +
           std::to_string(generated.num_pandas) + "_s" +
           std::to_string(generated.num_spheres) + "_task" +
           std::to_string(generated.task_index) + "_seed" +
           std::to_string(options.seed);
}

std::vector<std::shared_ptr<comotion::RobotModel>>
loadRobots(const GeneratedScenario &generated, const std::string &collision_urdf,
           const std::string &srdf) {
    std::vector<std::shared_ptr<comotion::RobotModel>> robots;
    robots.reserve(static_cast<std::size_t>(generated.num_robots));
    for (const auto &base : generated.base_transforms) {
        auto robot = std::make_shared<comotion::RobotModel>();
        robot->loadURDF(collision_urdf);
        robot->loadSRDF(srdf);
        robot->setBaseTransform(base);
        robots.push_back(std::move(robot));
    }
    for (const double radius : generated.sphere_radii) {
        robots.push_back(std::make_shared<comotion::FlyingSphere>(
            radius, generated.sphere_env_min, generated.sphere_env_max));
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
                        const std::string &srdf,
                        const std::shared_ptr<comotion::MultiRobotPlanner> &planner = {},
                        bool output_paths = false,
                        bool track_arc_history = false) {
    std::filesystem::create_directories(output_dir);

    auto robot_models = problem->robotModelPtrs();
    const auto export_paths = common::densePathsForExport(
        paths, problem->resolution(), problem->vmax());
    comotion::CompositePathValidationOptions validation_options;
    validation_options.check_environment = true;
    auto conflict = problem->collisionChecker().findFirstCompositePathConflict(
        export_paths, robot_models, validation_options);

    json out;
    out["schema_version"] = "1.0";
    out["solver"] = metrics.planner;
    out["drrt_local_connector"] =
        common::drrtLocalConnectorSummary(metrics);
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
    for (std::size_t r = 0; r < export_paths.size(); ++r) {
        const auto &path = export_paths[r];
        timesteps = std::max(timesteps, path.size());
        total_path_cost += path.path_cost();

        json robot_json;
        if (static_cast<int>(r) < generated.num_pandas) {
            robot_json["name"] = generated.robot_names[r];
            robot_json["robot_type"] = "panda";
            robot_json["urdf_path"] = toRepoRelativePath(visual_urdf);
            robot_json["visual_urdf_path"] = toRepoRelativePath(visual_urdf);
            robot_json["collision_urdf_path"] = toRepoRelativePath(collision_urdf);
            robot_json["srdf_path"] = toRepoRelativePath(srdf);
            robot_json["base_pose"] =
                basePoseJson(robots[r].model->getBaseTransform());
        } else {
            robot_json["name"] = generated.robot_names[r];
            robot_json["robot_type"] = "sphere";
            robot_json["robot_radius"] =
                generated.sphere_radii[r -
                                       static_cast<std::size_t>(
                                           generated.num_pandas)];
            robot_json["base_pose"] = {
                {"position", {0.0, 0.0, 0.0}},
                {"quaternion_xyzw", {0.0, 0.0, 0.0, 1.0}},
            };
        }
        robot_json["joint_names"] = robots[r].model->activeJointNames();
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
    common::appendArcVisualization(out, planner, output_paths,
                                   track_arc_history, problem->resolution(),
                                   problem->vmax());

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
    common::enableArcHistoryTracking(
        planner, options.output_paths, options.track_arc_history);

    if (g_app_verbose)
        std::cout << "Running " << planner_name << "\n";

    common::resetValidationTimingForSolve();
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
    common::captureValidationTimingForSolve(metrics);
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
    std::cout << "Multi-robot validation time: "
              << metrics.validation_time_seconds << " seconds\n";

    const std::string basename = outputBasename(generated, options);
    if (options.metrics_json_path)
        writeJson(metrics.toJson(), *options.metrics_json_path, 0);

    if (options.output_paths) {
        const auto *history_paths = common::arcHistoryArtifactPaths(
            planner, options.output_paths, options.track_arc_history);
        if (metrics.success || history_paths) {
            const auto artifact_paths =
                metrics.success ? planner->getSolutionPaths() : *history_paths;
            writePathArtifacts(metrics, generated, problem,
                               artifact_paths,
                               options.output_dir, basename, visual_urdf,
                               collision_urdf, srdf, planner,
                               options.output_paths,
                               options.track_arc_history);
        } else if (g_app_verbose) {
            std::cout << "No complete path set; skipping path artifacts\n";
        }
    }

    return metrics;
}

void printUsage(const char *prog) {
    std::cout
        << "Usage: " << prog
        << " --num-pandas <4|8|16> [--num-spheres <N>] [--task-index <i>] [options]\n"
        << "  --num-pandas <N>       Number of Panda manipulators in the corridor\n"
        << "  --num-spheres <N>      Number of flying spheres (default: 2*num-pandas)\n"
        << "  --task-index <i>       Zero-based Panda task index (default: 0)\n"
        << "  --task-file <path>     Read an mr-ompl heterogeneous task JSON\n"
        << "  --algorithm <name>     composite, composite_rrtstar, composite_prmstar, composite_aorrtc,\n"
        << "                         cooperative_composite, prioritized, drrt, drrt_star,\n"
        << "                         ao_drrt, arc, ao_arc, parallel_arc, stcbs (default: arc)\n"
        << "  --collision-backend <b> sphere, fcl, vamp (default: vamp)\n"
        << "  --vamp-validation-strategy <s> combined_rake, combined_linear,\n"
        << "                         hierarchical_rake, hierarchical_linear\n"
        << "                         (default: combined_rake)\n"
        << "  --time-limit <sec>     Planning time limit (default: 60)\n"
        << "  --seed <n>             Planning seed (default: 0)\n"
        << "  --resolution <n>       Timesteps per second (default: 128)\n"
        << "  --metrics-json <path>  Write compact trial metrics JSON\n"
        << "  --output-paths         Write visualization result JSON and .pth files\n"
        << "  --track-arc-history    With --output-paths, embed ARC process history\n"
        << "  --output-endpoint-paths Write fake two-state start/goal paths and exit\n"
        << "  --output-dir <dir>     Output directory for path artifacts\n"
        << "      (default: benchmarks/results/heterogeneous_corridor)\n"
        << "  --urdf <path>          Override planning URDF\n"
        << "  --srdf <path>          Override SRDF\n"
        << "  --strrt-initial-batch-size <n>  Initial STRRT batch size (default: 4096)\n"
        << "  --strrt-initial-time-factor <x> Initial STRRT time bound factor (default: 4)\n"
        << "  --strrt-time-bound-factor-increase <x> STRRT time bound growth factor (default: 2)\n"
        << "  --strrt-shuffle-priority-order / --no-strrt-shuffle-priority-order\n"
        << "                         Shuffle Pandas first, then spheres, within each group (default: on)\n"
        << "  --strrt-return-first-solution <0|1> Stop each STRRT solve at first solution (default: 1)\n"
        << "  --strrt-rewiring <off|radius|knearest> STRRT rewiring mode (default: off)\n"
        << "  --drrt-roadmap-size <n> Override dRRT* roadmap size (default: 200)\n"
        << "  --drrt-iterations-per-batch <n> Override dRRT* batch size (default: 8)\n"
        << "  --drrt-cost-metric <sum_of_costs|composite_l2|makespan>\n"
        << "  --drrt-tensor-search <drrt|astar|lazy_astar> (default: drrt)\n"
        << "  --drrt-local-connector <prioritized|synchronized> (default: prioritized)\n"
        << "  --drrt-exclude-roadmap-build-time\n"
        << "  --arc-initial-window <n>\n"
        << "  --arc-expansion-step <x>\n"
        << "  --arc-expansion-policy <linear|logarithmic|exponential|multiplied> (baseline ARC only)\n"
        << "  --arc-expansion-multipliers <csv> (baseline ARC only; default: 1,1,1,2,2,2,4,8)\n"
        << "  --arc-initial-valid-expansion-policy <linear|logarithmic|exponential|multiplied> (baseline ARC only; default: main policy)\n"
        << "  --arc-initial-valid-expansion-step <x> (baseline ARC only; default: main step)\n"
        << "  --arc-initial-valid-expansion-multipliers <csv> (baseline ARC only; default: main multipliers)\n"
        << "  --arc-initial-valid-symmetric-expansion / --arc-initial-valid-asymmetric-expansion (baseline ARC only; default: symmetric)\n"
        << "  --arc-cspace-bound-margin <x> (default: 2)\n"
        << "  --arc-min-cspace-bound-range <x> (default: 2)\n"
        << "  --arc-simplification-max-shortcut-steps <n> (default: 128)\n"
        << "  --arc-simplification-max-empty-steps <n> (default: 32)\n"
        << "  --arc-simplification-max-smooth-steps <n> (default: 1)\n"
        << "  --arc-simplification-max-passes <n> (default: 1)\n"
        << "  --arc-conflict-simplification-max-shortcut-steps <n>\n"
        << "  --arc-conflict-simplification-max-empty-steps <n>\n"
        << "  --arc-conflict-simplification-max-smooth-steps <n>\n"
        << "  --arc-conflict-simplification-max-passes <n>\n"
        << "  --arc-local-composite-max-samples <n>\n"
        << "  --arc-local-composite-range <x> (default: automatic)\n"
        << "  --arc-local-composite-use-makespan-metric\n"
        << "  --arc-simplify-initial-solutions / --no-arc-simplify-initial-solutions (default: on)\n"
        << "  --arc-simplify-conflict-solutions / --no-arc-simplify-conflict-solutions (default: off)\n"
        << "  --composite-rrt-use-makespan-metric\n"
        << "  --composite-rrt-simplify Run path simplification after Composite RRT-C succeeds\n"
        << "  --aorrtc-restart-effort <n> Set CompositeAORRTC sample/vertex caps\n"
        << "  --aorrtc-max-internal-samples <n>\n"
        << "  --aorrtc-max-internal-vertices <n>\n"
        << "  --arc-local-solvers <both|prioritized|composite> (default: composite)\n"
        << "  --arc-local-prioritized-max-iterations <n> (default: 5; 0 disables cap)\n"
        << "  --arc-local-prioritized-return-first-solution <0|1> (default: 1)\n"
        << "  --arc-local-prioritized-rewiring <off|radius|knearest> (default: knearest)\n"
        << "  --arc-local-prioritized-persist-at-goal / --no-arc-local-prioritized-persist-at-goal\n"
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
        << "  --parallel-arc-conflict-find-assignment <auto|pair_cover|round_robin|balanced_pair_cover|pair_first_greedy|cyclic_cover_greedy> (default: cyclic_cover_greedy)\n"
        << "  --parallel-arc-conflict-find-horizon <n>\n"
        << "  --exit-nonzero-without-exact-solution\n"
        << "  --verbose\n";
}

AppOptions parseArgs(int argc, char **argv) {
    AppOptions options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--num-pandas") {
            options.num_pandas = std::stoi(requireValue(i, argc, argv, arg));
        } else if (arg == "--num-spheres") {
            options.num_spheres = std::stoi(requireValue(i, argc, argv, arg));
        } else if (arg == "--task-index" || arg == "--task") {
            options.task_index = std::stoi(requireValue(i, argc, argv, arg));
        } else if (arg == "--task-file") {
            options.task_file = requireValue(i, argc, argv, arg);
        } else if (arg == "--algorithm") {
            options.algorithm = requireValue(i, argc, argv, arg);
        } else if (arg == "--collision-backend") {
            options.collision_backend =
                parseCollisionBackend(requireValue(i, argc, argv, arg));
        } else if (arg == "--vamp-validation-strategy") {
            options.vamp_validation_strategy = requireValue(i, argc, argv, arg);
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
        } else if (arg == "--track-arc-history") {
            options.track_arc_history = true;
        } else if (arg == "--output-endpoint-paths" ||
                   arg == "--output-fake-paths") {
            options.output_endpoint_paths = true;
        } else if (arg == "--output-dir") {
            options.output_dir = requireValue(i, argc, argv, arg);
        } else if (arg == "--urdf") {
            options.urdf_rel = requireValue(i, argc, argv, arg);
            options.urdf_explicit = true;
        } else if (arg == "--srdf") {
            options.srdf_rel = requireValue(i, argc, argv, arg);
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
        } else if (arg == "--no-strrt-shuffle-priority-order") {
            options.strrt_shuffle_priority_order = false;
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
        } else if (arg == "--drrt-local-connector") {
            options.drrt_local_connector = requireValue(i, argc, argv, arg);
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
                std::stod(requireValue(i, argc, argv, arg));
        } else if (arg == "--arc-expansion-policy") {
            options.arc_expansion_policy =
                requireValue(i, argc, argv, arg);
        } else if (arg == "--arc-expansion-multipliers") {
            options.arc_expansion_multipliers =
                requireValue(i, argc, argv, arg);
        } else if (arg == "--arc-initial-valid-expansion-policy") {
            options.arc_initial_valid_expansion_policy =
                requireValue(i, argc, argv, arg);
        } else if (arg == "--arc-initial-valid-expansion-step") {
            options.arc_initial_valid_expansion_step =
                std::stod(requireValue(i, argc, argv, arg));
        } else if (arg == "--arc-initial-valid-expansion-multipliers") {
            options.arc_initial_valid_expansion_multipliers =
                requireValue(i, argc, argv, arg);
        } else if (arg == "--arc-initial-valid-symmetric-expansion") {
            options.arc_initial_valid_expansion_symmetric = true;
        } else if (arg == "--arc-initial-valid-asymmetric-expansion") {
            options.arc_initial_valid_expansion_symmetric = false;
        } else if (arg == "--arc-cspace-bound-margin") {
            options.arc_cspace_bound_margin =
                std::stod(requireValue(i, argc, argv, arg));
        } else if (arg == "--arc-min-cspace-bound-range") {
            options.arc_min_cspace_bound_range =
                std::stod(requireValue(i, argc, argv, arg));
        } else if (arg == "--arc-simplification-max-shortcut-steps") {
            options.arc_simplification_max_shortcut_steps =
                static_cast<unsigned int>(
                    std::stoul(requireValue(i, argc, argv, arg)));
        } else if (arg == "--arc-simplification-max-empty-steps") {
            options.arc_simplification_max_empty_steps =
                static_cast<unsigned int>(
                    std::stoul(requireValue(i, argc, argv, arg)));
        } else if (arg == "--arc-simplification-max-smooth-steps") {
            options.arc_simplification_max_smooth_steps =
                static_cast<unsigned int>(
                    std::stoul(requireValue(i, argc, argv, arg)));
        } else if (arg == "--arc-simplification-max-passes") {
            options.arc_simplification_max_passes =
                static_cast<unsigned int>(
                    std::stoul(requireValue(i, argc, argv, arg)));
        } else if (arg ==
                   "--arc-conflict-simplification-max-shortcut-steps") {
            options.arc_conflict_simplification_max_shortcut_steps =
                static_cast<unsigned int>(
                    std::stoul(requireValue(i, argc, argv, arg)));
            options.arc_conflict_simplification_options_explicit = true;
        } else if (arg == "--arc-conflict-simplification-max-empty-steps") {
            options.arc_conflict_simplification_max_empty_steps =
                static_cast<unsigned int>(
                    std::stoul(requireValue(i, argc, argv, arg)));
            options.arc_conflict_simplification_options_explicit = true;
        } else if (arg == "--arc-conflict-simplification-max-smooth-steps") {
            options.arc_conflict_simplification_max_smooth_steps =
                static_cast<unsigned int>(
                    std::stoul(requireValue(i, argc, argv, arg)));
            options.arc_conflict_simplification_options_explicit = true;
        } else if (arg == "--arc-conflict-simplification-max-passes") {
            options.arc_conflict_simplification_max_passes =
                static_cast<unsigned int>(
                    std::stoul(requireValue(i, argc, argv, arg)));
            options.arc_conflict_simplification_options_explicit = true;
        } else if (arg == "--arc-local-composite-max-samples") {
            options.arc_local_composite_max_samples = static_cast<unsigned int>(
                std::stoul(requireValue(i, argc, argv, arg)));
        } else if (arg == "--arc-local-composite-range") {
            options.arc_local_composite_range =
                std::stod(requireValue(i, argc, argv, arg));
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
        } else if (arg == "--arc-local-prioritized-return-first-solution") {
            options.arc_local_prioritized_return_first_solution =
                common::parseBoolValue(requireValue(i, argc, argv, arg));
        } else if (arg == "--arc-local-prioritized-rewiring") {
            options.arc_local_prioritized_rewiring =
                requireValue(i, argc, argv, arg);
        } else if (arg == "--arc-local-prioritized-persist-at-goal") {
            options.arc_local_prioritized_persist_at_goal = true;
        } else if (arg == "--no-arc-local-prioritized-persist-at-goal") {
            options.arc_local_prioritized_persist_at_goal = false;
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
        } else if (arg == "--parallel-arc-conflict-find-assignment") {
            options.parallel_arc_conflict_find_assignment =
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

    if (options.num_pandas == 0)
        throw std::runtime_error("--num-pandas is required");
    if (options.num_pandas != 4 && options.num_pandas != 8 &&
        options.num_pandas != 16)
        throw std::runtime_error("--num-pandas must be one of 4, 8, or 16");
    if (options.num_spheres == 0)
        options.num_spheres = 2 * options.num_pandas;
    if (options.num_spheres < 2 || options.num_spheres % 2 != 0)
        throw std::runtime_error("--num-spheres must be an even value at least 2");
    if (options.task_index < 0)
        throw std::runtime_error("--task-index must be non-negative");
    if (options.time_limit <= 0.0 && !options.output_endpoint_paths)
        throw std::runtime_error("--time-limit must be positive");
    if (options.resolution == 0)
        throw std::runtime_error("--resolution must be at least 1");
    common::validateSelectedPlannerOptions(options,
                                           !options.output_endpoint_paths);

    return options;
}

} // namespace

int main(int argc, char **argv) {
    try {
        setExecutablePath(argc > 0 ? argv[0] : nullptr);
        AppOptions options = parseArgs(argc, argv);

        json task_doc;
        std::string task_source;
        if (options.task_file) {
            task_doc = loadJsonFile(*options.task_file);
            task_source = *options.task_file;
        } else {
            task_source =
                resolveResourcePath(defaultTaskResource(options.num_pandas));
            task_doc = loadJsonFile(task_source);
        }

        const GeneratedScenario generated =
            loadScenarioFromDoc(task_doc, options, task_source);

        if (!options.urdf_explicit &&
            options.collision_backend ==
                comotion::CollisionChecker::Backend::Fcl) {
            options.urdf_rel = "panda/panda.urdf";
        }
        const std::string collision_urdf = resolveResourcePath(options.urdf_rel);
        const std::string visual_urdf =
            resolveResourcePath("panda/panda.urdf");
        const std::string srdf = resolveResourcePath(options.srdf_rel);

        const auto robots = loadRobots(generated, collision_urdf, srdf);

        auto problem =
            std::make_shared<comotion::MultiRobotProblem>(options.collision_backend);
        common::applyVampValidationStrategy(problem,
                                            options.vamp_validation_strategy);
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

        if (options.output_endpoint_paths) {
            const TrialMetrics metrics =
                common::makeEndpointPathMetrics(generated, context, problem);
            const auto endpoint_paths = common::makeEndpointPaths(generated);
            const std::string basename = outputBasename(generated, options);
            writePathArtifacts(metrics, generated, problem, endpoint_paths,
                               options.output_dir, basename, visual_urdf,
                               collision_urdf, srdf);
            const std::filesystem::path output_dir(options.output_dir);
            if (options.metrics_json_path)
                writeJson(metrics.toJson(), *options.metrics_json_path, 0);
            std::cout << "Wrote synthetic endpoint path artifacts to "
                      << output_dir << "\n";
            return 0;
        }

        PlannerBlueprint blueprint =
            common::makePlannerBlueprint(options, g_app_verbose);
        if (options.algorithm == "prioritized") {
            std::vector<int> panda_priorities;
            std::vector<int> sphere_priorities;
            panda_priorities.reserve(
                static_cast<std::size_t>(generated.num_pandas));
            sphere_priorities.reserve(
                static_cast<std::size_t>(generated.num_spheres));
            for (int robot = 0; robot < generated.num_pandas; ++robot)
                panda_priorities.push_back(robot);
            for (int robot = generated.num_pandas;
                 robot < generated.num_robots; ++robot) {
                sphere_priorities.push_back(robot);
            }

            const auto base_factory = blueprint.factory;
            blueprint.factory =
                [base_factory, panda_priorities, sphere_priorities]() {
                    auto planner = base_factory();
                    auto prioritized =
                        std::dynamic_pointer_cast<comotion::PrioritizedSTRRT>(
                            planner);
                    if (!prioritized) {
                        throw std::runtime_error(
                            "Prioritized planner factory returned an "
                            "unexpected planner type");
                    }
                    prioritized->setPriorityGroups(
                        {panda_priorities, sphere_priorities});
                    return planner;
                };
        }
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
