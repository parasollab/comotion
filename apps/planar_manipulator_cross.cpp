#include "planar_manipulator_cross_scenarios.hpp"

#include "benchmark_app_common.hpp"
#include "comotion/collision/CollisionChecker.h"
#include "comotion/planning/MultiRobotPlanner.h"
#include "comotion/planning/MultiRobotProblem.h"
#include "comotion/planning/OrParallelPlanner.h"
#include "comotion/planning/Path.h"
#include "comotion/planning/PlanningRng.h"
#include "comotion/robot/RobotModel.h"

#include <Eigen/Geometry>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

using json = nlohmann::json;
namespace common = comotion::benchmark_apps::common;
namespace planar = comotion::benchmark_apps::planar_manipulator_cross;

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
    std::string scenario = "cross";
    int num_robots = 0;
    std::string algorithm = "arc";
    comotion::CollisionChecker::Backend collision_backend =
        comotion::CollisionChecker::Backend::Vamp;
    std::string vamp_validation_strategy = "combined_rake";
    double time_limit = 60.0;
    std::uint32_t seed = 0;
    double line_distance = 1.35;
    double spacing = 1.2;
    std::size_t resolution = 128;
    std::string output_dir = "benchmarks/results/planar_manipulator_cross";
    bool output_paths = false;
    bool output_endpoint_paths = false;
    bool reverse_adaptive_endpoints = false;
    std::optional<std::string> metrics_json_path;
    bool exit_nonzero_without_exact_solution = false;

    unsigned int strrt_initial_batch_size = 4096;
    double strrt_initial_time_factor = 2.0;
    double strrt_time_bound_factor_increase = 2.0;
    bool strrt_shuffle_priority_order = true;
    bool strrt_return_first_solution = true;
    std::string strrt_rewiring = "off";
    int drrt_roadmap_size = 30;
    int drrt_iterations_per_batch = 1;
    std::string drrt_cost_metric = "makespan";
    std::string drrt_tensor_search = "drrt";
    std::string drrt_local_connector = "prioritized";
    bool drrt_exclude_roadmap_build_time = false;
    double composite_rrt_range = 0.0;
    bool composite_rrt_simplify_solution = false;
    bool composite_rrt_use_makespan_metric = false;
    std::size_t composite_aorrtc_max_internal_samples = 10000;
    std::size_t composite_aorrtc_max_internal_vertices = 10000;
    unsigned int cooperative_rrt_worker_threads = 2;
    int arc_initial_window = 100;
    double arc_expansion_step = 1.05;
    std::string arc_expansion_policy = "exponential";
    std::string arc_expansion_multipliers = "1,1,1,2,2,2,4,8";
    std::optional<std::string> arc_initial_valid_expansion_policy = "linear";
    std::optional<double> arc_initial_valid_expansion_step = 10.0;
    std::optional<std::string> arc_initial_valid_expansion_multipliers;
    bool arc_initial_valid_expansion_symmetric = false;
    double arc_cspace_bound_margin = 1.0;
    double arc_min_cspace_bound_range = 0.5;
    unsigned int arc_simplification_max_shortcut_steps = 128;
    unsigned int arc_simplification_max_empty_steps = 32;
    unsigned int arc_simplification_max_smooth_steps = 1;
    unsigned int arc_simplification_max_passes = 1;
    bool arc_conflict_simplification_options_explicit = false;
    unsigned int arc_conflict_simplification_max_shortcut_steps = 128;
    unsigned int arc_conflict_simplification_max_empty_steps = 32;
    unsigned int arc_conflict_simplification_max_smooth_steps = 1;
    unsigned int arc_conflict_simplification_max_passes = 1;
    unsigned int arc_local_composite_max_samples = 50000;
    double arc_local_composite_range = 0.0;
    bool arc_local_composite_use_makespan_metric = false;
    bool arc_simplify_initial_solutions = true;
    bool arc_simplify_conflict_solutions = true;
    std::string arc_local_solvers = "composite";
    unsigned int arc_local_prioritized_max_iterations = 5;
    std::uint64_t ao_arc_local_bound_epsilon_timesteps = 1;
    unsigned int or_parallel_worker_processes = 1;
    unsigned int parallel_arc_worker_processes = 2;
    bool parallel_arc_parallel_initial_plans = true;
    bool parallel_arc_initial_solution_or = true;
    bool parallel_arc_repair_duplicate_attempts = false;
    std::string parallel_arc_strategy = "synchronous";
    std::string parallel_arc_conflict_strategy = "greedy";
    std::string parallel_arc_conflict_find_mode = "segment_parallel";
    std::string parallel_arc_conflict_find_assignment =
        "cyclic_cover_greedy";
    std::string parallel_arc_conflict_batch_mode = "optimistic";
    std::size_t parallel_arc_conflict_find_horizon = 400;
    bool parallel_arc_conflict_ablation_only = false;
    int stcbs_max_ct_nodes = 5000;
    int stcbs_max_samples = 25000;
};

struct DrrtDefaultParameters {
    int roadmap_size;
    int iterations_per_batch;
};

std::optional<DrrtDefaultParameters>
tunedDrrtDefaults(const std::string &scenario_name, int num_robots) {
    const auto scenario = planar::parseScenarioName(scenario_name);
    if (scenario == planar::Scenario::Cross && num_robots == 4)
        return DrrtDefaultParameters{30, 8};
    return std::nullopt;
}

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
        std::string path = normalized.string();
        std::ifstream file(path);
        if (file.good())
            return path;
    }
    return std::string("resources/") + relative;
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

json cylinderObstaclesJson(
    const std::vector<comotion::ObstacleCylinder> &cylinders) {
    json obstacles = json::array();
    for (std::size_t i = 0; i < cylinders.size(); ++i) {
        const auto &cylinder = cylinders[i];
        json obstacle;
        obstacle["id"] = "cyl_" + std::to_string(i);
        obstacle["type"] = "cylinder";
        obstacle["pose"] = {
            {"position",
             {cylinder.center.x(), cylinder.center.y(), cylinder.center.z()}},
            {"axis", {cylinder.axis.x(), cylinder.axis.y(), cylinder.axis.z()}},
        };
        obstacle["geometry"] = {
            {"radius", cylinder.radius},
            {"half_height", cylinder.half_height},
        };
        obstacles.push_back(obstacle);
    }
    return obstacles;
}

json sphereObstaclesJson(const std::vector<comotion::ObstacleSphere> &spheres) {
    json obstacles = json::array();
    for (std::size_t i = 0; i < spheres.size(); ++i) {
        const auto &sphere = spheres[i];
        json obstacle;
        obstacle["id"] = "sphere_" + std::to_string(i);
        obstacle["type"] = "sphere";
        obstacle["pose"] = {
            {"position",
             {sphere.center.x(), sphere.center.y(), sphere.center.z()}},
            {"quaternion_xyzw", {0.0, 0.0, 0.0, 1.0}},
        };
        obstacle["geometry"] = {{"radius", sphere.radius}};
        obstacles.push_back(obstacle);
    }
    return obstacles;
}

json obstaclesJson(const std::vector<comotion::ObstacleSphere> &spheres,
                   const std::vector<comotion::ObstacleCylinder> &cylinders) {
    json obstacles = sphereObstaclesJson(spheres);
    const json cylinder_obstacles = cylinderObstaclesJson(cylinders);
    for (const auto &obstacle : cylinder_obstacles)
        obstacles.push_back(obstacle);
    return obstacles;
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

json basePoseJson(const planar::BasePose &pose) {
    json out = basePoseJson(planar::baseTransform(pose));
    out["yaw"] = pose.yaw;
    return out;
}

json benchmarkContextJson(const planar::GeneratedScenario &generated,
                          const AppOptions &options) {
    json base_poses = json::array();
    for (const auto &pose : generated.base_poses)
        base_poses.push_back(basePoseJson(pose));

    return json{
        {"suite", "planar_manipulator_cross"},
        {"scenario", generated.scenario_name},
        {"num_robots", generated.num_robots},
        {"robots_per_line", generated.robots_per_line},
        {"seed", options.seed},
        {"time_limit_seconds", options.time_limit},
        {"vamp_validation_strategy", options.vamp_validation_strategy},
        {"line_distance", generated.line_distance},
        {"spacing", generated.spacing},
        {"resolution", options.resolution},
        {"start_angle", generated.start_angle},
        {"goal_angle", generated.goal_angle},
        {"wall_radius", generated.wall_radius},
        {"boundary_margin", generated.boundary_margin},
        {"backboard_offset", generated.backboard_offset},
        {"adaptive_obstacle_radius", generated.adaptive_obstacle_radius},
        {"adaptive_obstacle_offset", generated.adaptive_obstacle_offset},
        {"reverse_adaptive_endpoints", generated.reverse_adaptive_endpoints},
        {"side_min_y", generated.side_min_y},
        {"side_max_y", generated.side_max_y},
        {"left_backboard_x", generated.left_backboard_x},
        {"right_backboard_x", generated.right_backboard_x},
        {"robot_names", generated.robot_names},
        {"base_poses", base_poses},
        {"starts", generated.starts},
        {"goals", generated.goals},
        {"obstacles", obstaclesJson(generated.sphere_obstacles,
                                    generated.cylinder_obstacles)},
        {"sphere_obstacles", sphereObstaclesJson(generated.sphere_obstacles)},
        {"cylinder_obstacles",
         cylinderObstaclesJson(generated.cylinder_obstacles)},
    };
}

std::string outputBasename(const planar::GeneratedScenario &generated,
                           const AppOptions &options) {
    return "planar_manipulator_cross_" + generated.scenario_name + "_n" +
           std::to_string(generated.num_robots) + "_seed" +
           std::to_string(options.seed);
}

std::shared_ptr<comotion::RobotModel>
loadPlanar3(const std::string &planning_urdf, const std::string &srdf,
            const planar::BasePose &pose) {
    auto robot = std::make_shared<comotion::RobotModel>();
    robot->loadURDF(planning_urdf);
    robot->loadSRDF(srdf);
    robot->setBaseTransform(planar::baseTransform(pose));
    return robot;
}

void validateEndpoints(
    const planar::GeneratedScenario &generated, const AppOptions &options,
    const std::vector<std::shared_ptr<comotion::RobotModel>> &robots) {
    comotion::CollisionChecker checker(options.collision_backend);
    common::applyVampValidationStrategy(checker,
                                        options.vamp_validation_strategy);
    checker.setObstacles(generated.sphere_obstacles);
    checker.setCylinderObstacles(generated.cylinder_obstacles);

    for (int i = 0; i < generated.num_robots; ++i) {
        const auto index = static_cast<std::size_t>(i);
        if (!checker.isValidSingleFull(*robots[index], generated.starts[index]))
            throw std::runtime_error("planar cross start is invalid for robot " +
                                     std::to_string(i));
        if (!checker.isValidSingleFull(*robots[index], generated.goals[index]))
            throw std::runtime_error("planar cross goal is invalid for robot " +
                                     std::to_string(i));
    }

    for (int i = 0; i < generated.num_robots; ++i) {
        for (int j = i + 1; j < generated.num_robots; ++j) {
            const auto a = static_cast<std::size_t>(i);
            const auto b = static_cast<std::size_t>(j);
            if (!checker.isValidPair(*robots[a], generated.starts[a],
                                     *robots[b], generated.starts[b])) {
                throw std::runtime_error(
                    "planar cross start pair is invalid for robots " +
                    std::to_string(i) + " and " + std::to_string(j));
            }
            if (!checker.isValidPair(*robots[a], generated.goals[a], *robots[b],
                                     generated.goals[b])) {
                throw std::runtime_error(
                    "planar cross goal pair is invalid for robots " +
                    std::to_string(i) + " and " + std::to_string(j));
            }
        }
    }
}

void writePathArtifacts(const TrialMetrics &metrics,
                        const planar::GeneratedScenario &generated,
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
    for (std::size_t r = 0; r < paths.size(); ++r) {
        const auto &path = paths[r];
        timesteps = std::max(timesteps, path.size());
        total_path_cost += path.path_cost();

        json robot_json;
        robot_json["name"] = r < generated.robot_names.size()
                                 ? generated.robot_names[r]
                                 : "planar3_" + std::to_string(r);
        robot_json["robot_type"] = "planar3";
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

    writeJson(out, output_dir / (basename + "_" + metrics.planner + "_result.json"),
              2);
}

TrialMetrics runPlanner(
    const planar::GeneratedScenario &generated, const AppOptions &options,
    const json &benchmark_context,
    const std::shared_ptr<comotion::MultiRobotProblem> &problem,
    const std::shared_ptr<comotion::MultiRobotPlanner> &planner,
    const std::string &planner_name, const std::string &visual_urdf,
    const std::string &collision_urdf,
    const std::string &srdf) {
    comotion::seedOmplGlobalFromUserPlanningSeed(options.seed);
    planner->setPlanningSeed(options.seed);
    planner->setProblem(problem);

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

TrialMetrics runParallelArcConflictAblation(
    const planar::GeneratedScenario &generated, const AppOptions &options,
    const json &benchmark_context,
    const std::shared_ptr<comotion::MultiRobotProblem> &problem,
    const std::shared_ptr<comotion::ParallelARC> &planner,
    const std::string &planner_name) {
    comotion::seedOmplGlobalFromUserPlanningSeed(options.seed);
    planner->setPlanningSeed(options.seed);
    planner->setProblem(problem);

    if (g_app_verbose)
        std::cout << "Running fair conflict-detection ablation for "
                  << planner_name << "\n";

    const bool completed = planner->runConflictDetectionAblation(
        options.time_limit);
    json planner_stats = planner->plannerStatsJson();
    double conflict_detection_wall_seconds = 0.0;
    if (planner_stats.contains("conflict_detection_times_seconds_wall_clock") &&
        planner_stats["conflict_detection_times_seconds_wall_clock"].is_number()) {
        conflict_detection_wall_seconds =
            planner_stats["conflict_detection_times_seconds_wall_clock"]
                .get<double>();
    }
    double conflict_detection_tree_cpu_seconds = 0.0;
    if (planner_stats.contains("conflict_find_timing") &&
        planner_stats["conflict_find_timing"].is_object()) {
        const auto &timing = planner_stats["conflict_find_timing"];
        if (timing.contains("process_tree_cpu_seconds_total") &&
            timing["process_tree_cpu_seconds_total"].is_number()) {
            conflict_detection_tree_cpu_seconds =
                timing["process_tree_cpu_seconds_total"].get<double>();
        }
    }

    TrialMetrics metrics;
    metrics.planner = planner_name;
    metrics.collision_backend = backendName(problem->collisionChecker().backend());
    metrics.planner_status =
        completed
            ? "ParallelARC conflict-detection ablation completed"
            : "ParallelARC conflict-detection ablation timed out while generating initial paths";
    metrics.success = completed;
    metrics.planning_time_seconds = conflict_detection_wall_seconds;
    metrics.solve_time_seconds = conflict_detection_wall_seconds;
    metrics.compute_time_seconds = conflict_detection_tree_cpu_seconds;
    metrics.sum_of_cost_timesteps = planner->sumOfCostTimesteps()
                                        ? json(*planner->sumOfCostTimesteps())
                                        : json(nullptr);
    metrics.makespan_timesteps = planner->makespanTimesteps()
                                     ? json(*planner->makespanTimesteps())
                                     : json(nullptr);
    metrics.planner_stats = std::move(planner_stats);
    metrics.benchmark_context = benchmark_context;
    metrics.solution_summary = common::solutionSummaryJson(metrics);

    std::cout << "Conflict-detection wall time: "
              << conflict_detection_wall_seconds << " seconds\n";
    std::cout << "Conflict-detection compute time: "
              << conflict_detection_tree_cpu_seconds << " seconds\n";

    if (options.metrics_json_path) {
        writeJson(metrics.toJson(), *options.metrics_json_path, 0);
    }
    if (options.output_paths && g_app_verbose) {
        std::cout << "Conflict-detection ablation mode skips path artifacts; "
                     "the measured object is the initial single conflict scan\n";
    }

    return metrics;
}

void printUsage(const char *prog) {
    std::cout
        << "Usage: " << prog
        << " --num-robots <N> [--scenario cross|adaptive] [options]\n"
        << "  --scenario <name>       cross or adaptive (default: cross)\n"
        << "  --algorithm <name>      composite, composite_rrtstar, composite_prmstar, composite_aorrtc, "
        << "cooperative_composite, prioritized, drrt, drrt_star, ao_drrt, arc, ao_arc, "
        << "parallel_arc, stcbs (default: arc)\n"
        << "  --collision-backend <b> sphere, fcl, vamp (default: vamp)\n"
        << "  --vamp-validation-strategy <s> combined_rake, combined_linear,\n"
        << "                         hierarchical_rake, hierarchical_linear\n"
        << "                         (default: combined_rake)\n"
        << "  --time-limit <sec>      Planning time limit (default: 60)\n"
        << "  --seed <n>              Planning seed (default: 0)\n"
        << "  --line-distance <d>     Distance between facing base lines "
        << "(default: 1.35)\n"
        << "  --spacing <d>           Base spacing along each line "
        << "(default: 1.2)\n"
        << "  --resolution <n>        Timesteps per second (default: 128)\n"
        << "  --metrics-json <path>   Write compact trial metrics JSON\n"
        << "  --output-paths          Write visualization result JSON and .pth files\n"
        << "  --output-endpoint-paths Write fake two-state start/goal paths and exit\n"
        << "  --reverse-adaptive-endpoints\n"
        << "                         Restore the original adaptive start/goal direction\n"
        << "  --output-dir <dir>      Output directory for path artifacts\n"
        << "      (default: benchmarks/results/planar_manipulator_cross)\n"
        << "  --strrt-initial-batch-size <n>  Initial STRRT batch size (default: 4096)\n"
        << "  --strrt-initial-time-factor <x> Initial STRRT time bound factor (default: 2)\n"
        << "  --strrt-time-bound-factor-increase <x> STRRT time bound growth factor (default: 2)\n"
        << "  --strrt-shuffle-priority-order / --no-strrt-shuffle-priority-order\n"
        << "                         Shuffle PrioritizedSTRRT priority order from --seed (default: on)\n"
        << "  --strrt-return-first-solution <0|1> Stop each STRRT solve at first solution (default: 1)\n"
        << "  --strrt-rewiring <off|radius|knearest> STRRT rewiring mode (default: off)\n"
        << "  --drrt-roadmap-size <n> Override tuned dRRT* roadmap size\n"
        << "  --drrt-iterations-per-batch <n> Override tuned dRRT* batch size\n"
        << "  --drrt-cost-metric <sum_of_costs|composite_l2|makespan>\n"
        << "  --drrt-tensor-search <drrt|astar|lazy_astar> (default: drrt)\n"
        << "  --drrt-local-connector <prioritized|synchronized> (default: prioritized)\n"
        << "  --drrt-exclude-roadmap-build-time\n"
        << "                         Give dRRT tensor search the full time limit after PRM* build\n"
        << "  --arc-initial-window <n> (default: 100)\n"
        << "  --arc-expansion-step <x> (default: 1.05)\n"
        << "  --arc-expansion-policy <linear|logarithmic|exponential|multiplied> (baseline ARC only; default: exponential)\n"
        << "  --arc-expansion-multipliers <csv> (baseline ARC only; default: 1,1,1,2,2,2,4,8)\n"
        << "  --arc-initial-valid-expansion-policy <linear|logarithmic|exponential|multiplied> (baseline ARC only; default: linear)\n"
        << "  --arc-initial-valid-expansion-step <x> (baseline ARC only; default: 10)\n"
        << "  --arc-initial-valid-expansion-multipliers <csv> (baseline ARC only; default: main multipliers)\n"
        << "  --arc-initial-valid-symmetric-expansion / --arc-initial-valid-asymmetric-expansion (baseline ARC only; default: asymmetric)\n"
        << "  --arc-cspace-bound-margin <x> (default: 1)\n"
        << "  --arc-min-cspace-bound-range <x> (default: 0.5)\n"
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
        << "  --arc-simplify-conflict-solutions / --no-arc-simplify-conflict-solutions (default: on)\n"
        << "  --composite-rrt-range <d> Explicit Composite RRT-C extension range\n"
        << "  --composite-rrt-use-makespan-metric\n"
        << "  --composite-rrt-simplify Run path simplification after Composite RRT-C succeeds\n"
        << "  --aorrtc-restart-effort <n> Set CompositeAORRTC sample/vertex caps\n"
        << "  --aorrtc-max-internal-samples <n>\n"
        << "  --aorrtc-max-internal-vertices <n>\n"
        << "  --arc-local-solvers <both|prioritized|composite> (default: composite)\n"
        << "  --arc-local-prioritized-max-iterations <n> (default: 5; 0 disables cap)\n"
        << "  --ao-arc-local-bound-epsilon-timesteps <n> (default: 1; 0 disables)\n"
        << "  --cooperative-rrt-worker-threads <n>\n"
        << "  --or-parallel-worker-processes <n>\n"
        << "  --parallel-arc-worker-processes <n>\n"
        << "  --parallel-arc-parallel-initial-plans / --no-parallel-arc-parallel-initial-plans (default: on)\n"
        << "  --parallel-arc-initial-solution-or / --no-parallel-arc-initial-solution-or (default: on)\n"
        << "  --parallel-arc-repair-duplicate-attempts / --no-parallel-arc-repair-duplicate-attempts (default: off)\n"
        << "  --parallel-arc-strategy <synchronous|asynchronous>\n"
        << "  --parallel-arc-conflict-strategy <greedy|spatial_distribution>\n"
        << "  --parallel-arc-conflict-find-mode <sequential|segment_parallel>\n"
        << "  --parallel-arc-conflict-find-assignment <auto|pair_cover|round_robin|balanced_pair_cover|pair_first_greedy|cyclic_cover_greedy> (default: cyclic_cover_greedy)\n"
        << "  --parallel-arc-conflict-batch-mode <optimistic|independent_only> (default: optimistic)\n"
        << "  --parallel-arc-conflict-find-horizon <n> (default: 400)\n"
        << "  --parallel-arc-conflict-ablation-only\n"
        << "                         Generate initial individual paths, then time only one\n"
        << "                         ParallelARC conflict-detection call on those fixed paths\n"
        << "  --exit-nonzero-without-exact-solution\n"
        << "  --verbose\n";
}

AppOptions parseArgs(int argc, char **argv) {
    AppOptions options;
    bool drrt_roadmap_size_explicit = false;
    bool drrt_iterations_per_batch_explicit = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--scenario") {
            options.scenario = requireValue(i, argc, argv, arg);
        } else if (arg == "--num-robots") {
            options.num_robots = std::stoi(requireValue(i, argc, argv, arg));
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
        } else if (arg == "--line-distance") {
            options.line_distance =
                std::stod(requireValue(i, argc, argv, arg));
        } else if (arg == "--spacing") {
            options.spacing = std::stod(requireValue(i, argc, argv, arg));
        } else if (arg == "--resolution") {
            options.resolution = static_cast<std::size_t>(
                std::stoul(requireValue(i, argc, argv, arg)));
        } else if (arg == "--metrics-json") {
            options.metrics_json_path = requireValue(i, argc, argv, arg);
        } else if (arg == "--output-paths") {
            options.output_paths = true;
        } else if (arg == "--output-endpoint-paths" ||
                   arg == "--output-fake-paths") {
            options.output_endpoint_paths = true;
        } else if (arg == "--reverse-adaptive-endpoints") {
            options.reverse_adaptive_endpoints = true;
        } else if (arg == "--output-dir") {
            options.output_dir = requireValue(i, argc, argv, arg);
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
            drrt_roadmap_size_explicit = true;
        } else if (arg == "--drrt-iterations-per-batch") {
            options.drrt_iterations_per_batch =
                std::stoi(requireValue(i, argc, argv, arg));
            drrt_iterations_per_batch_explicit = true;
        } else if (arg == "--drrt-cost-metric") {
            options.drrt_cost_metric = requireValue(i, argc, argv, arg);
        } else if (arg == "--drrt-tensor-search") {
            options.drrt_tensor_search = requireValue(i, argc, argv, arg);
        } else if (arg == "--drrt-local-connector") {
            options.drrt_local_connector = requireValue(i, argc, argv, arg);
        } else if (arg == "--drrt-exclude-roadmap-build-time") {
            options.drrt_exclude_roadmap_build_time = true;
        } else if (arg == "--composite-rrt-range") {
            options.composite_rrt_range =
                std::stod(requireValue(i, argc, argv, arg));
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
        } else if (arg == "--parallel-arc-conflict-batch-mode") {
            options.parallel_arc_conflict_batch_mode =
                requireValue(i, argc, argv, arg);
        } else if (arg == "--parallel-arc-conflict-find-horizon") {
            options.parallel_arc_conflict_find_horizon =
                static_cast<std::size_t>(
                    std::stoull(requireValue(i, argc, argv, arg)));
        } else if (arg == "--parallel-arc-conflict-ablation-only") {
            options.parallel_arc_conflict_ablation_only = true;
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
    if (const auto tuned =
            tunedDrrtDefaults(options.scenario, options.num_robots)) {
        if (!drrt_roadmap_size_explicit)
            options.drrt_roadmap_size = tuned->roadmap_size;
        if (!drrt_iterations_per_batch_explicit)
            options.drrt_iterations_per_batch = tuned->iterations_per_batch;
    }
    if (options.time_limit <= 0.0)
        throw std::runtime_error("--time-limit must be positive");
    if (options.line_distance <= 0.0)
        throw std::runtime_error("--line-distance must be positive");
    if (options.spacing <= 0.0)
        throw std::runtime_error("--spacing must be positive");
    if (options.resolution == 0)
        throw std::runtime_error("--resolution must be at least 1");
    if (options.strrt_initial_batch_size == 0)
        throw std::runtime_error(
            "--strrt-initial-batch-size must be at least 1");
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
        throw std::runtime_error(
            "--drrt-iterations-per-batch must be at least 1");
    (void)common::parseDrrtCostMetric(options.drrt_cost_metric);
    (void)common::parseDrrtTensorSearchMode(options.drrt_tensor_search);
    (void)common::parseDrrtLocalConnectorMode(options.drrt_local_connector);
    (void)common::parseVampValidationStrategy(
        options.vamp_validation_strategy);
    if (options.cooperative_rrt_worker_threads == 0)
        throw std::runtime_error(
            "--cooperative-rrt-worker-threads must be at least 1");
    if (options.composite_aorrtc_max_internal_samples == 0)
        throw std::runtime_error(
            "--aorrtc-max-internal-samples must be at least 1");
    if (options.composite_aorrtc_max_internal_vertices == 0)
        throw std::runtime_error(
            "--aorrtc-max-internal-vertices must be at least 1");
    if (options.composite_rrt_range < 0.0)
        throw std::runtime_error("--composite-rrt-range must be non-negative");
    if (options.arc_initial_window < 1)
        throw std::runtime_error("--arc-initial-window must be at least 1");
    if (!std::isfinite(options.arc_expansion_step) ||
        options.arc_expansion_step <= 0.0)
        throw std::runtime_error("--arc-expansion-step must be positive");
    (void)common::parseArcExpansionPolicy(options.arc_expansion_policy);
    (void)common::parseArcExpansionMultipliers(
        options.arc_expansion_multipliers);
    if (options.arc_initial_valid_expansion_policy) {
        (void)common::parseArcExpansionPolicy(
            *options.arc_initial_valid_expansion_policy);
    }
    if (options.arc_initial_valid_expansion_step &&
        (!std::isfinite(*options.arc_initial_valid_expansion_step) ||
         *options.arc_initial_valid_expansion_step <= 0.0)) {
        throw std::runtime_error(
            "--arc-initial-valid-expansion-step must be positive");
    }
    if (options.arc_initial_valid_expansion_multipliers) {
        (void)common::parseArcExpansionMultipliers(
            *options.arc_initial_valid_expansion_multipliers);
    }
    if (options.arc_cspace_bound_margin < 0.0)
        throw std::runtime_error(
            "--arc-cspace-bound-margin must be non-negative");
    if (options.arc_min_cspace_bound_range < 0.0)
        throw std::runtime_error(
            "--arc-min-cspace-bound-range must be non-negative");
    if (options.arc_local_composite_range < 0.0)
        throw std::runtime_error(
            "--arc-local-composite-range must be non-negative");
    (void)parseArcLocalSolverMode(options.arc_local_solvers);
    if (options.or_parallel_worker_processes == 0)
        throw std::runtime_error(
            "--or-parallel-worker-processes must be at least 1");
    if (options.parallel_arc_worker_processes == 0)
        throw std::runtime_error(
            "--parallel-arc-worker-processes must be at least 1");
    if (options.parallel_arc_conflict_find_mode == "segment_parallel" &&
        options.parallel_arc_conflict_find_horizon == 0) {
        throw std::runtime_error(
            "--parallel-arc-conflict-find-horizon must be at least 1 for "
            "segment_parallel mode");
    }
    const auto conflict_find_assignment =
        common::parseParallelArcConflictFindAssignment(
            options.parallel_arc_conflict_find_assignment);
    if ((conflict_find_assignment ==
             comotion::ConflictFindParallelAssignment::PairFirstGreedy ||
         conflict_find_assignment ==
             comotion::ConflictFindParallelAssignment::CyclicCoverGreedy) &&
        options.collision_backend !=
            comotion::CollisionChecker::Backend::Vamp) {
        throw std::runtime_error(
            options.parallel_arc_conflict_find_assignment +
            " conflict assignment currently requires the VAMP collision "
            "backend");
    }
    if (options.parallel_arc_conflict_ablation_only) {
        if (options.algorithm != "parallel_arc") {
            throw std::runtime_error(
                "--parallel-arc-conflict-ablation-only requires "
                "--algorithm parallel_arc");
        }
        if (options.or_parallel_worker_processes != 1) {
            throw std::runtime_error(
                "--parallel-arc-conflict-ablation-only does not support "
                "outer OR parallelism");
        }
    }
    if (options.stcbs_max_ct_nodes < 1)
        throw std::runtime_error("--stcbs-max-ct-nodes must be at least 1");
    if (options.stcbs_max_samples < 1)
        throw std::runtime_error("--stcbs-max-samples must be at least 1");

    return options;
}

} // namespace

int main(int argc, char **argv) {
    try {
        setExecutablePath(argc > 0 ? argv[0] : nullptr);
        AppOptions options = parseArgs(argc, argv);

        const auto generated = planar::generateScenario(
            planar::parseScenarioName(options.scenario), options.num_robots,
            options.line_distance, options.spacing,
            options.reverse_adaptive_endpoints);
        const json context = benchmarkContextJson(generated, options);

        const std::string visual_urdf = getResourcePath("planar3/planar3.urdf");
        const std::string planning_urdf =
            getResourcePath("planar3/planar3_spherized.urdf");
        const std::string srdf = getResourcePath("planar3/planar3.srdf");

        std::vector<std::shared_ptr<comotion::RobotModel>> robots;
        robots.reserve(generated.base_poses.size());
        for (const auto &pose : generated.base_poses)
            robots.push_back(loadPlanar3(planning_urdf, srdf, pose));
        if (!options.output_endpoint_paths)
            validateEndpoints(generated, options, robots);

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

        if (options.output_endpoint_paths) {
            const TrialMetrics metrics =
                common::makeEndpointPathMetrics(generated, context, problem);
            const auto endpoint_paths = common::makeEndpointPaths(generated);
            const std::string basename = outputBasename(generated, options);
            writePathArtifacts(metrics, generated, problem, endpoint_paths,
                               options.output_dir, basename, visual_urdf,
                               planning_urdf, srdf);
            const std::filesystem::path output_dir(options.output_dir);
            if (options.metrics_json_path) {
                writeJson(metrics.toJson(), *options.metrics_json_path, 0);
            }
            std::cout << "Wrote synthetic endpoint path artifacts to "
                      << output_dir << "\n";
            return 0;
        }

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
            options.parallel_arc_conflict_ablation_only
                ? runParallelArcConflictAblation(
                      generated, options, context, problem,
                      std::dynamic_pointer_cast<comotion::ParallelARC>(planner),
                      planner_name)
                : runPlanner(generated, options, context, problem, planner,
                             planner_name, visual_urdf, planning_urdf, srdf);
        if (options.exit_nonzero_without_exact_solution && !metrics.success)
            return 1;
        return 0;
    } catch (const std::exception &ex) {
        std::cerr << "error: " << ex.what() << "\n";
        return 1;
    }
}
