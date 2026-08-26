#include "mobile_robot_2d_crossing_scenarios.hpp"

#include "benchmark_app_common.hpp"
#include "comotion/collision/CollisionChecker.h"
#include "comotion/planning/MultiRobotPlanner.h"
#include "comotion/planning/MultiRobotProblem.h"
#include "comotion/planning/OrParallelPlanner.h"
#include "comotion/planning/Path.h"
#include "comotion/planning/PlanningRng.h"
#include "comotion/robot/FlyingSphere.h"

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
namespace crossing = comotion::benchmark_apps::mobile_robot_2d;

namespace {

bool g_app_verbose = false;

using common::PlannerBlueprint;
using common::TrialMetrics;
using common::backendName;
using common::parseArcLocalSolverMode;
using common::parseCollisionBackend;
using common::requireValue;
using common::writeJson;

struct AppOptions {
    std::string scenario;
    int num_robots = 0;
    std::string algorithm = "arc";
    comotion::CollisionChecker::Backend collision_backend =
        comotion::CollisionChecker::Backend::Vamp;
    std::string vamp_validation_strategy = "combined_rake";
    double time_limit = 60.0;
    std::uint32_t seed = 0;
    double robot_radius = 0.5;
    double spacing = 5.0;
    int vertical_hallways = -1;
    int horizontal_hallways = 0;
    double hallway_radius = -1.0;
    double hallway_width = -1.0;
    double hallway_length = -1.0;
    double intersection_width = -1.0;
    std::size_t resolution = 128;
    std::string output_dir = "benchmarks/results/mobile_robot_2d_crossing";
    bool output_paths = false;
    bool track_arc_history = false;
    bool output_endpoint_paths = false;
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
    unsigned int arc_local_composite_max_samples = 5000;
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
    double stcbs_range = 10.0;
    double stcbs_goal_threshold = 0.1;
    double stcbs_layer_dt = 1.0;
    double stcbs_lambda = 0.5;
    double stcbs_occupied_radius = 0.1;
};

struct DrrtDefaultParameters {
    int roadmap_size;
    int iterations_per_batch;
};

std::optional<DrrtDefaultParameters>
tunedDrrtDefaults(const std::string &scenario_name, int num_robots) {
    const auto scenario = crossing::parseScenarioName(scenario_name);
    if (scenario == crossing::CrossingScenario::Parallel && num_robots == 8)
        return DrrtDefaultParameters{30, 8};
    if (scenario == crossing::CrossingScenario::Circle && num_robots == 4)
        return DrrtDefaultParameters{30, 64};
    if (scenario == crossing::CrossingScenario::Inlet && num_robots == 2)
        return DrrtDefaultParameters{300, 4};
    return std::nullopt;
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

json benchmarkContextJson(const crossing::GeneratedScenario &generated,
                          const AppOptions &options) {
    json context = json{
        {"suite", "mobile_robot_2d_crossing"},
        {"scenario", generated.scenario_name},
        {"num_robots", generated.num_robots},
        {"seed", options.seed},
        {"time_limit_seconds", options.time_limit},
        {"vamp_validation_strategy", options.vamp_validation_strategy},
        {"robot_radius", generated.robot_radius},
        {"spacing", generated.spacing},
        {"resolution", options.resolution},
        {"starts", generated.starts},
        {"goals", generated.goals},
        {"environment", {{"min", generated.env_min}, {"max", generated.env_max}}},
        {"obstacles", cylinderObstaclesJson(generated.cylinder_obstacles)},
    };
    if (generated.scenario == crossing::CrossingScenario::Hallways) {
        context["vertical_hallways"] = generated.vertical_hallways;
        context["horizontal_hallways"] = generated.horizontal_hallways;
        context["hallway_radius"] = generated.hallway_radius;
    } else if (generated.scenario == crossing::CrossingScenario::Adaptive) {
        context["hallway_width"] = generated.hallway_width;
        context["intersection_width"] = generated.intersection_width;
    } else if (generated.scenario == crossing::CrossingScenario::Inlet) {
        context["hallway_width"] = generated.hallway_width;
        context["hallway_length"] = generated.hallway_length;
    }
    return context;
}

std::string outputBasename(const crossing::GeneratedScenario &generated,
                           const AppOptions &options) {
    return "mobile_robot_2d_crossing_" + generated.scenario_name + "_n" +
           std::to_string(generated.num_robots) + "_seed" +
           std::to_string(options.seed);
}

void writePathArtifacts(const TrialMetrics &metrics,
                        const crossing::GeneratedScenario &generated,
                        const std::shared_ptr<comotion::MultiRobotProblem> &problem,
                        const std::vector<comotion::Path> &paths,
                        const std::filesystem::path &output_dir,
                        const std::string &basename,
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
    out["obstacles"] =
        cylinderObstaclesJson(problem->collisionChecker().cylinders());

    std::size_t timesteps = 0;
    double total_path_cost = 0.0;
    for (std::size_t r = 0; r < export_paths.size(); ++r) {
        const auto &path = export_paths[r];
        timesteps = std::max(timesteps, path.size());
        total_path_cost += path.path_cost();

        json robot_json;
        robot_json["name"] = "sphere_" + std::to_string(r);
        robot_json["robot_type"] = "sphere";
        robot_json["robot_radius"] = generated.robot_radius;
        robot_json["base_pose"] = {
            {"position", {0.0, 0.0, 0.0}},
            {"quaternion_xyzw", {0.0, 0.0, 0.0, 1.0}},
        };
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

    writeJson(out, output_dir / (basename + "_" + metrics.planner + "_result.json"),
              2);
}

TrialMetrics runPlanner(
    const crossing::GeneratedScenario &generated, const AppOptions &options,
    const json &benchmark_context,
    const std::shared_ptr<comotion::MultiRobotProblem> &problem,
    const std::shared_ptr<comotion::MultiRobotPlanner> &planner,
    const std::string &planner_name) {
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
    if (options.metrics_json_path) {
        writeJson(metrics.toJson(), *options.metrics_json_path, 0);
    }

    if (options.output_paths) {
        const auto *history_paths = common::arcHistoryArtifactPaths(
            planner, options.output_paths, options.track_arc_history);
        if (metrics.success || history_paths) {
            const auto artifact_paths =
                metrics.success ? planner->getSolutionPaths() : *history_paths;
            writePathArtifacts(metrics, generated, problem,
                               artifact_paths, options.output_dir, basename,
                               planner, options.output_paths,
                               options.track_arc_history);
        } else if (g_app_verbose) {
            std::cout << "No complete path set; skipping path artifacts\n";
        }
    }

    return metrics;
}

TrialMetrics runParallelArcConflictAblation(
    const crossing::GeneratedScenario &generated, const AppOptions &options,
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
        << " --scenario <circle|parallel|perpendicular|hallways|adaptive|inlet> "
        << "--num-robots <N> [options]\n"
        << "  --algorithm <name>       composite, composite_rrtstar, composite_prmstar, composite_aorrtc, "
        << "cooperative_composite, prioritized, drrt, drrt_star, ao_drrt, arc, ao_arc, "
        << "parallel_arc, stcbs "
        << "(default: arc)\n"
        << "  --collision-backend <b>  sphere, fcl, vamp (default: vamp)\n"
        << "  --vamp-validation-strategy <s> combined_rake, combined_linear,\n"
        << "                         hierarchical_rake, hierarchical_linear\n"
        << "                         (default: combined_rake)\n"
        << "  --time-limit <sec>       Planning time limit (default: 60)\n"
        << "  --seed <n>               Planning seed (default: 0)\n"
        << "  --robot-radius <r>       Sphere radius (default: 0.5)\n"
        << "  --spacing <d>            Scenario spacing (default: 5.0)\n"
        << "  --vertical-hallways <n>  Hallways scenario only; -1 derives n/2 - 1\n"
        << "  --horizontal-hallways <n> Hallways scenario only (default: 0)\n"
        << "  --hallway-radius <r>     Hallways scenario only; -1 derives 2*robot radius\n"
        << "  --hallway-width <w>      Adaptive/inlet; -1 derives 4*robot radius\n"
        << "  --hallway-length <l>     Inlet only; -1 derives 4*spacing\n"
        << "  --intersection-width <w> Adaptive only; -1 derives 2*hallway width\n"
        << "  --resolution <n>         Timesteps per second (default: 128)\n"
        << "  --metrics-json <path>    Write compact trial metrics JSON\n"
        << "  --output-paths           Write visualization result JSON and .pth files\n"
        << "  --track-arc-history      With --output-paths, embed ARC process history\n"
        << "  --output-endpoint-paths  Write fake two-state start/goal paths and exit\n"
        << "  --output-dir <dir>       Output directory for path artifacts\n"
        << "      (default: benchmarks/results/mobile_robot_2d_crossing)\n"
        << "  --strrt-initial-batch-size <n>  Initial STRRT batch size (default: 4096)\n"
        << "  --strrt-initial-time-factor <x> Initial STRRT time bound factor (default: 2)\n"
        << "  --strrt-time-bound-factor-increase <x> STRRT time bound growth factor (default: 2)\n"
        << "  --strrt-shuffle-priority-order / --no-strrt-shuffle-priority-order\n"
        << "                         Shuffle PrioritizedSTRRT priority order from --seed (default: on)\n"
        << "  --strrt-return-first-solution <0|1> Stop each STRRT solve at first solution (default: 1)\n"
        << "  --strrt-rewiring <off|radius|knearest> STRRT rewiring mode (default: off)\n"
        << "  --drrt-roadmap-size <n>  Override tuned dRRT* roadmap size\n"
        << "  --drrt-iterations-per-batch <n> Override tuned dRRT* batch size\n"
        << "  --drrt-cost-metric <sum_of_costs|composite_l2|makespan>\n"
        << "  --drrt-tensor-search <drrt|astar|lazy_astar> (default: drrt)\n"
        << "  --drrt-local-connector <prioritized|synchronized> (default: prioritized)\n"
        << "  --drrt-exclude-roadmap-build-time\n"
        << "                         Give dRRT tensor search the full time limit after PRM* build\n"
        << "  --arc-initial-window <n> (default: 200)\n"
        << "  --arc-expansion-step <x> (default: 200)\n"
        << "  --arc-expansion-policy <linear|logarithmic|exponential|multiplied> (baseline ARC only; default: linear)\n"
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
        << "  --arc-local-composite-max-samples <n> (default: 5000)\n"
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
        << "  --stcbs-range <x>\n"
        << "  --stcbs-goal-threshold <x>\n"
        << "  --stcbs-layer-dt <sec>\n"
        << "  --stcbs-lambda <x>\n"
        << "  --stcbs-occupied-radius <x>\n"
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
        } else if (arg == "--robot-radius") {
            options.robot_radius = std::stod(requireValue(i, argc, argv, arg));
        } else if (arg == "--spacing") {
            options.spacing = std::stod(requireValue(i, argc, argv, arg));
        } else if (arg == "--vertical-hallways") {
            options.vertical_hallways =
                std::stoi(requireValue(i, argc, argv, arg));
        } else if (arg == "--horizontal-hallways") {
            options.horizontal_hallways =
                std::stoi(requireValue(i, argc, argv, arg));
        } else if (arg == "--hallway-radius") {
            options.hallway_radius =
                std::stod(requireValue(i, argc, argv, arg));
        } else if (arg == "--hallway-width") {
            options.hallway_width =
                std::stod(requireValue(i, argc, argv, arg));
        } else if (arg == "--hallway-length") {
            options.hallway_length =
                std::stod(requireValue(i, argc, argv, arg));
        } else if (arg == "--intersection-width") {
            options.intersection_width =
                std::stod(requireValue(i, argc, argv, arg));
        } else if (arg == "--resolution") {
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
        } else if (arg == "--stcbs-range") {
            options.stcbs_range =
                std::stod(requireValue(i, argc, argv, arg));
        } else if (arg == "--stcbs-goal-threshold") {
            options.stcbs_goal_threshold =
                std::stod(requireValue(i, argc, argv, arg));
        } else if (arg == "--stcbs-layer-dt") {
            options.stcbs_layer_dt =
                std::stod(requireValue(i, argc, argv, arg));
        } else if (arg == "--stcbs-lambda") {
            options.stcbs_lambda =
                std::stod(requireValue(i, argc, argv, arg));
        } else if (arg == "--stcbs-occupied-radius") {
            options.stcbs_occupied_radius =
                std::stod(requireValue(i, argc, argv, arg));
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

    if (options.scenario.empty())
        throw std::runtime_error("--scenario is required");
    if (options.num_robots == 0)
        throw std::runtime_error("--num-robots is required");
    if (const auto tuned =
            tunedDrrtDefaults(options.scenario, options.num_robots)) {
        if (!drrt_roadmap_size_explicit)
            options.drrt_roadmap_size = tuned->roadmap_size;
        if (!drrt_iterations_per_batch_explicit)
            options.drrt_iterations_per_batch = tuned->iterations_per_batch;
    }
    if (options.vertical_hallways < -1)
        throw std::runtime_error(
            "--vertical-hallways must be -1 or at least 1");
    if (options.horizontal_hallways < 0)
        throw std::runtime_error("--horizontal-hallways must be non-negative");
    if (options.hallway_radius != -1.0 && options.hallway_radius <= 0.0)
        throw std::runtime_error(
            "--hallway-radius must be -1 or positive");
    if (options.hallway_width != -1.0 && options.hallway_width <= 0.0)
        throw std::runtime_error(
            "--hallway-width must be -1 or positive");
    if (options.hallway_length != -1.0 && options.hallway_length <= 0.0)
        throw std::runtime_error(
            "--hallway-length must be -1 or positive");
    if (options.intersection_width != -1.0 &&
        options.intersection_width <= 0.0)
        throw std::runtime_error(
            "--intersection-width must be -1 or positive");
    if (options.time_limit <= 0.0)
        throw std::runtime_error("--time-limit must be positive");
    if (options.resolution == 0)
        throw std::runtime_error("--resolution must be at least 1");
    common::validateSelectedPlannerOptions(options,
                                           !options.output_endpoint_paths);
    if (options.algorithm == "stcbs") {
        if (options.stcbs_range <= 0.0)
            throw std::runtime_error("--stcbs-range must be positive");
        if (options.stcbs_goal_threshold <= 0.0)
            throw std::runtime_error("--stcbs-goal-threshold must be positive");
        if (options.stcbs_layer_dt <= 0.0)
            throw std::runtime_error("--stcbs-layer-dt must be positive");
        if (options.stcbs_lambda < 0.0 || options.stcbs_lambda > 1.0)
            throw std::runtime_error("--stcbs-lambda must be in [0, 1]");
        if (options.stcbs_occupied_radius < 0.0)
            throw std::runtime_error(
                "--stcbs-occupied-radius must be non-negative");
    }

    return options;
}

} // namespace

int main(int argc, char **argv) {
    try {
        AppOptions options = parseArgs(argc, argv);

        crossing::HallwayOptions hallway_options;
        hallway_options.vertical_hallways = options.vertical_hallways;
        hallway_options.horizontal_hallways = options.horizontal_hallways;
        hallway_options.hallway_radius = options.hallway_radius;
        crossing::AdaptiveOptions adaptive_options;
        adaptive_options.hallway_width = options.hallway_width;
        adaptive_options.intersection_width = options.intersection_width;
        crossing::InletOptions inlet_options;
        inlet_options.hallway_width = options.hallway_width;
        inlet_options.hallway_length = options.hallway_length;
        const auto generated = crossing::generateScenario(
            crossing::parseScenarioName(options.scenario), options.num_robots,
            options.robot_radius, options.spacing, hallway_options,
            adaptive_options, inlet_options);
        const json context = benchmarkContextJson(generated, options);

        auto problem =
            std::make_shared<comotion::MultiRobotProblem>(options.collision_backend);
        common::applyVampValidationStrategy(problem,
                                            options.vamp_validation_strategy);
        for (int i = 0; i < generated.num_robots; ++i) {
            auto sphere = std::make_shared<comotion::FlyingSphere>(
                generated.robot_radius, generated.env_min, generated.env_max);
            problem->addRobot(sphere, generated.starts[static_cast<std::size_t>(i)],
                              generated.goals[static_cast<std::size_t>(i)]);
        }
        problem->setObstacles({});
        problem->setCylinderObstacles(generated.cylinder_obstacles);
        problem->setResolution(options.resolution);

        if (options.output_endpoint_paths) {
            const TrialMetrics metrics =
                common::makeEndpointPathMetrics(generated, context, problem);
            const auto endpoint_paths = common::makeEndpointPaths(generated);
            const std::string basename = outputBasename(generated, options);
            writePathArtifacts(metrics, generated, problem, endpoint_paths,
                               options.output_dir, basename);
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
        if (auto stcbs = std::dynamic_pointer_cast<comotion::STCBS>(planner)) {
            stcbs->setRange(options.stcbs_range);
            stcbs->setGoalThreshold(options.stcbs_goal_threshold);
            stcbs->setLayerDtSeconds(options.stcbs_layer_dt);
            stcbs->setLambda(options.stcbs_lambda);
            stcbs->setOccupiedRadius(options.stcbs_occupied_radius);
        }

        const TrialMetrics metrics =
            options.parallel_arc_conflict_ablation_only
                ? runParallelArcConflictAblation(
                      generated, options, context, problem,
                      std::dynamic_pointer_cast<comotion::ParallelARC>(planner),
                      planner_name)
                : runPlanner(generated, options, context, problem, planner,
                             planner_name);
        if (options.exit_nonzero_without_exact_solution && !metrics.success)
            return 1;
        return 0;
    } catch (const std::exception &ex) {
        std::cerr << "error: " << ex.what() << "\n";
        return 1;
    }
}
