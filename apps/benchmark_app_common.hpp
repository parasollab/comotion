#pragma once

#include "comotion/collision/CollisionChecker.h"
#include "comotion/planning/AOARC.h"
#include "comotion/planning/ARC.h"
#include "comotion/planning/CompositeAORRTC.h"
#include "comotion/planning/CompositePRMStar.h"
#include "comotion/planning/CompositeRRT.h"
#include "comotion/planning/CompositeRRTStar.h"
#include "comotion/planning/CooperativeCompositeRRT.h"
#include "comotion/planning/MRdRRT.h"
#include "comotion/planning/MultiRobotProblem.h"
#include "comotion/planning/STCBS.h"
#include "comotion/planning/OrParallelPlanner.h"
#include "comotion/planning/ParallelARC.h"
#include "comotion/planning/Path.h"
#include "comotion/planning/PrioritizedSTRRT.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#if !defined(_WIN32)
#include <sys/resource.h>
#endif

namespace comotion::benchmark_apps::common {

using json = nlohmann::json;

inline json validationWorkStatsJson(const comotion::ValidationWorkStats &stats) {
    return {
        {"motion_timesteps_possible", stats.motion_timesteps_possible},
        {"motion_timesteps_checked", stats.motion_timesteps_checked},
        {"robot_state_checks_possible", stats.robot_state_checks_possible},
        {"robot_state_checks_completed", stats.robot_state_checks_completed},
        {"robot_pair_checks_possible", stats.robot_pair_checks_possible},
        {"robot_pair_checks_completed", stats.robot_pair_checks_completed},
        {"simd_packs_checked", stats.simd_packs_checked},
        {"simd_lanes_checked", stats.simd_lanes_checked},
    };
}

inline json
validationTimingStatsJson(const comotion::ValidationTimingStats &stats) {
    return {
        {"total_validation_time_seconds",
         stats.total_validation_time_seconds},
        {"total_validation_calls", stats.total_validation_calls},
        {"composite_state_seconds", stats.composite_state_seconds},
        {"composite_state_calls", stats.composite_state_calls},
        {"pair_path_seconds", stats.pair_path_seconds},
        {"pair_path_calls", stats.pair_path_calls},
        {"pair_path_conflict_seconds", stats.pair_path_conflict_seconds},
        {"pair_path_conflict_calls", stats.pair_path_conflict_calls},
        {"goal_hold_constraint_seconds", stats.goal_hold_constraint_seconds},
        {"goal_hold_constraint_calls", stats.goal_hold_constraint_calls},
        {"composite_motion_seconds", stats.composite_motion_seconds},
        {"composite_motion_calls", stats.composite_motion_calls},
        {"composite_motion_conflict_seconds",
         stats.composite_motion_conflict_seconds},
        {"composite_motion_conflict_calls",
         stats.composite_motion_conflict_calls},
        {"composite_paths_seconds", stats.composite_paths_seconds},
        {"composite_paths_calls", stats.composite_paths_calls},
        {"composite_path_conflict_seconds",
         stats.composite_path_conflict_seconds},
        {"composite_path_conflict_calls",
         stats.composite_path_conflict_calls},
        {"inter_robot_path_conflicts_scan_seconds",
         stats.inter_robot_path_conflicts_scan_seconds},
        {"inter_robot_path_conflicts_scan_calls",
         stats.inter_robot_path_conflicts_scan_calls},
        {"work", validationWorkStatsJson(stats.work)},
    };
}

struct TrialMetrics {
    std::string planner;
    std::string collision_backend;
    std::string planner_status;
    bool success = false;
    double planning_time_seconds = 0.0;
    double solve_time_seconds = 0.0;
    double compute_time_seconds = 0.0;
    double validation_time_seconds = 0.0;
    comotion::ValidationTimingStats validation_timing_stats;
    json sum_of_cost_timesteps = nullptr;
    json makespan_timesteps = nullptr;
    json planner_stats = json::object();
    json benchmark_context = json::object();
    json solution_summary = json::object();

    json toJson() const {
        return json{
            {"planner", planner},
            {"collision_backend", collision_backend},
            {"planner_status", planner_status},
            {"success", success},
            {"planning_time_seconds", planning_time_seconds},
            {"solve_time_seconds", solve_time_seconds},
            {"compute_time_seconds", compute_time_seconds},
            {"validation_time_seconds", validation_time_seconds},
            {"sum_of_cost_timesteps", sum_of_cost_timesteps},
            {"validation_timing",
             validationTimingStatsJson(validation_timing_stats)},
            {"makespan_timesteps", makespan_timesteps},
            {"planner_stats", planner_stats},
            {"benchmark_context", benchmark_context},
            {"solution_summary", solution_summary},
        };
    }
};

inline void resetValidationTimingForSolve() {
    comotion::CollisionChecker::resetValidationTimingStats();
}

inline void captureValidationTimingForSolve(TrialMetrics &metrics) {
    metrics.validation_timing_stats =
        comotion::CollisionChecker::validationTimingStats();
    metrics.validation_time_seconds =
        metrics.validation_timing_stats.total_validation_time_seconds;
}

inline json drrtLocalConnectorSummary(const TrialMetrics &metrics) {
    if (!metrics.planner_stats.is_object() ||
        !metrics.planner_stats.contains("local_connector_mode")) {
        return nullptr;
    }
    return {
        {"mode", metrics.planner_stats["local_connector_mode"]},
        {"attempts",
         metrics.planner_stats.value("local_connector_attempts",
                                     std::uint64_t{0})},
        {"successes",
         metrics.planner_stats.value("local_connector_successes",
                                     std::uint64_t{0})},
        {"used_for_solution",
         metrics.planner_stats.value("solution_used_local_connector", false)},
        {"priority_order",
         metrics.planner_stats.value("solution_local_connector_priority_order",
                                     std::vector<int>{})},
    };
}

struct ProcessCpuUsageSnapshot {
    double self_seconds = 0.0;
    double children_seconds = 0.0;
};

#if !defined(_WIN32)
inline double timevalSeconds(const timeval &value) {
    return static_cast<double>(value.tv_sec) +
           static_cast<double>(value.tv_usec) * 1e-6;
}

inline double rusageCpuSeconds(const rusage &usage) {
    return timevalSeconds(usage.ru_utime) + timevalSeconds(usage.ru_stime);
}
#endif

inline ProcessCpuUsageSnapshot processCpuUsageSnapshot() {
    ProcessCpuUsageSnapshot snapshot;
#if !defined(_WIN32)
    rusage self_usage {};
    if (getrusage(RUSAGE_SELF, &self_usage) == 0)
        snapshot.self_seconds = rusageCpuSeconds(self_usage);
    rusage children_usage {};
    if (getrusage(RUSAGE_CHILDREN, &children_usage) == 0)
        snapshot.children_seconds = rusageCpuSeconds(children_usage);
#endif
    return snapshot;
}

inline double elapsedProcessTreeCpuSeconds(
    const ProcessCpuUsageSnapshot &start,
    const ProcessCpuUsageSnapshot &finish) {
    const double elapsed =
        (finish.self_seconds - start.self_seconds) +
        (finish.children_seconds - start.children_seconds);
    return elapsed < 0.0 ? 0.0 : elapsed;
}

struct PlannerBlueprint {
    std::string planner_name;
    comotion::OrParallelPlanner::PlannerFactory factory;
    std::function<void(const std::shared_ptr<comotion::MultiRobotProblem> &)>
        prepare_problem = [](const std::shared_ptr<comotion::MultiRobotProblem> &) {};
    bool allow_outer_or_parallel = true;
};

inline std::string backendName(comotion::CollisionChecker::Backend backend) {
    switch (backend) {
    case comotion::CollisionChecker::Backend::Fcl:
        return "fcl";
    case comotion::CollisionChecker::Backend::Vamp:
        return "vamp";
    case comotion::CollisionChecker::Backend::Spheres:
        return "sphere";
    }
    return "unknown";
}

inline comotion::CollisionChecker::Backend
parseCollisionBackend(const std::string &value) {
    if (value == "fcl" || value == "FCL")
        return comotion::CollisionChecker::Backend::Fcl;
    if (value == "vamp" || value == "VAMP")
        return comotion::CollisionChecker::Backend::Vamp;
    if (value == "sphere" || value == "spheres" || value == "SPHERE" ||
        value == "SPHERES")
        return comotion::CollisionChecker::Backend::Spheres;
    throw std::runtime_error("Unknown collision backend: " + value);
}

inline comotion::ARC::LocalSolverMode
parseArcLocalSolverMode(const std::string &value) {
    if (value == "both")
        return comotion::ARC::LocalSolverMode::Both;
    if (value == "prioritized" || value == "prioritized_strrt")
        return comotion::ARC::LocalSolverMode::PrioritizedStrrtOnly;
    if (value == "composite" || value == "composite_rrt")
        return comotion::ARC::LocalSolverMode::CompositeRrtOnly;
    throw std::runtime_error("Unknown ARC local solver mode: " + value);
}

inline std::string lowerAscii(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char ch) {
                       return static_cast<char>(std::tolower(ch));
                   });
    return value;
}

inline comotion::ARC::ExpansionPolicy
parseArcExpansionPolicy(const std::string &value) {
    const std::string lowered = lowerAscii(value);
    if (lowered == "linear")
        return comotion::ARC::ExpansionPolicy::Linear;
    if (lowered == "logarithmic" || lowered == "log")
        return comotion::ARC::ExpansionPolicy::Logarithmic;
    if (lowered == "exponential" || lowered == "exp")
        return comotion::ARC::ExpansionPolicy::Exponential;
    if (lowered == "multiplied" || lowered == "custom" ||
        lowered == "custom_multiplied") {
        return comotion::ARC::ExpansionPolicy::CustomMultiplied;
    }
    throw std::runtime_error("Unknown ARC expansion policy: " + value);
}

inline std::vector<double>
parseArcExpansionMultipliers(const std::string &value) {
    std::vector<double> multipliers;
    std::size_t begin = 0;
    while (begin <= value.size()) {
        const std::size_t end = value.find(',', begin);
        const std::size_t count =
            end == std::string::npos ? std::string::npos : end - begin;
        std::string token = value.substr(begin, count);
        const auto first = token.find_first_not_of(" \t\r\n");
        const auto last = token.find_last_not_of(" \t\r\n");
        if (first == std::string::npos) {
            throw std::runtime_error(
                "ARC expansion multipliers must not contain empty values");
        }
        token = token.substr(first, last - first + 1);

        std::size_t consumed = 0;
        double multiplier = 0.0;
        try {
            multiplier = std::stod(token, &consumed);
        } catch (const std::exception &) {
            throw std::runtime_error(
                "Invalid ARC expansion multiplier: " + token);
        }
        if (consumed != token.size() || !std::isfinite(multiplier) ||
            multiplier <= 0.0) {
            throw std::runtime_error(
                "ARC expansion multipliers must be finite and positive: " +
                token);
        }
        multipliers.push_back(multiplier);

        if (end == std::string::npos)
            break;
        begin = end + 1;
    }
    if (multipliers.empty()) {
        throw std::runtime_error(
            "ARC expansion multiplier sequence must not be empty");
    }
    return multipliers;
}

inline bool parseBoolValue(const std::string &value) {
    const std::string lowered = lowerAscii(value);
    if (lowered == "1" || lowered == "true" || lowered == "yes" ||
        lowered == "on") {
        return true;
    }
    if (lowered == "0" || lowered == "false" || lowered == "no" ||
        lowered == "off") {
        return false;
    }
    throw std::runtime_error("Expected boolean value, got: " + value);
}

inline comotion::InterRobotConflictBatchMode
parseParallelArcConflictBatchMode(const std::string &value) {
    const std::string lowered = lowerAscii(value);
    if (lowered == "optimistic" ||
        lowered == "optimistic_independent" ||
        lowered == "optimistic-independent")
        return comotion::InterRobotConflictBatchMode::OptimisticIndependent;
    if (lowered == "independent_only" ||
        lowered == "independent-only" ||
        lowered == "strict" ||
        lowered == "strict_independent" ||
        lowered == "strict-independent")
        return comotion::InterRobotConflictBatchMode::IndependentOnly;
    throw std::runtime_error(
        "Unknown ParallelARC conflict batch mode: " + value);
}

inline comotion::ConflictFindParallelAssignment
parseParallelArcConflictFindAssignment(const std::string &value) {
    std::string lowered = lowerAscii(value);
    std::replace(lowered.begin(), lowered.end(), '-', '_');
    if (lowered == "auto")
        return comotion::ConflictFindParallelAssignment::Auto;
    if (lowered == "pair_cover")
        return comotion::ConflictFindParallelAssignment::PairCover;
    if (lowered == "round_robin") {
        return comotion::ConflictFindParallelAssignment::
            AllRobotsRoundRobin;
    }
    if (lowered == "balanced_pair_cover")
        return comotion::ConflictFindParallelAssignment::BalancedPairCover;
    if (lowered == "pair_first_greedy")
        return comotion::ConflictFindParallelAssignment::PairFirstGreedy;
    if (lowered == "cyclic_cover_greedy")
        return comotion::ConflictFindParallelAssignment::CyclicCoverGreedy;
    throw std::runtime_error(
        "Unknown ParallelARC conflict-find assignment: " + value);
}

template <typename Options>
auto parallelArcConflictBatchModeValueImpl(const Options &options, int)
    -> decltype(options.parallel_arc_conflict_batch_mode) {
    return options.parallel_arc_conflict_batch_mode;
}

template <typename Options>
std::string parallelArcConflictBatchModeValueImpl(const Options &, long) {
    return "optimistic";
}

template <typename Options>
std::string parallelArcConflictBatchModeValue(const Options &options) {
    return parallelArcConflictBatchModeValueImpl(options, 0);
}

inline std::string normalizedVampValidationStrategyName(std::string value) {
    value = lowerAscii(std::move(value));
    std::replace(value.begin(), value.end(), '-', '_');
    std::replace(value.begin(), value.end(), ' ', '_');
    const std::string prefix = "vamp_";
    if (value.rfind(prefix, 0) == 0)
        value.erase(0, prefix.size());
    return value;
}

inline comotion::VampValidationStrategy
parseVampValidationStrategy(const std::string &value) {
    const std::string normalized = normalizedVampValidationStrategyName(value);
    using comotion::VampBatchOrdering;
    using comotion::VampBatchPacking;

    if (normalized == "combined_rake")
        return {VampBatchOrdering::Combined, VampBatchPacking::Rake};
    if (normalized == "combined_linear")
        return {VampBatchOrdering::Combined, VampBatchPacking::Linear};
    if (normalized == "hierarchical_rake")
        return {VampBatchOrdering::Hierarchical, VampBatchPacking::Rake};
    if (normalized == "hierarchical_linear")
        return {VampBatchOrdering::Hierarchical, VampBatchPacking::Linear};

    throw std::runtime_error(
        "Unknown VAMP validation strategy: " + value +
        " (expected combined_rake, combined_linear, hierarchical_rake, "
        "or hierarchical_linear)");
}

inline std::string
vampValidationStrategyName(const comotion::VampValidationStrategy &strategy) {
    std::string out =
        strategy.ordering == comotion::VampBatchOrdering::Combined
            ? "combined"
            : "hierarchical";
    out += "_";
    out += strategy.packing == comotion::VampBatchPacking::Linear ? "linear"
                                                                 : "rake";
    return out;
}

inline void applyVampValidationStrategy(comotion::CollisionChecker &checker,
                                        const std::string &value) {
    checker.setVampValidationStrategy(parseVampValidationStrategy(value));
}

inline void applyVampValidationStrategy(
    const std::shared_ptr<comotion::MultiRobotProblem> &problem,
    const std::string &value) {
    applyVampValidationStrategy(problem->collisionChecker(), value);
}

inline comotion::StrrtRewiring parseStrrtRewiring(const std::string &value) {
    const std::string lowered = lowerAscii(value);
    if (lowered == "off")
        return comotion::StrrtRewiring::Off;
    if (lowered == "radius")
        return comotion::StrrtRewiring::Radius;
    if (lowered == "knearest" || lowered == "k_nearest" ||
        lowered == "k-nearest" || lowered == "k") {
        return comotion::StrrtRewiring::KNearest;
    }
    throw std::runtime_error("Unknown STRRT rewiring mode: " + value);
}

inline comotion::MRdRRT::CostMetric parseDrrtCostMetric(const std::string &value) {
    if (value == "sum" || value == "sum_of_costs" ||
        value == "sum-of-costs")
        return comotion::MRdRRT::CostMetric::SumOfCosts;
    if (value == "composite_l2" || value == "composite-l2" ||
        value == "l2")
        return comotion::MRdRRT::CostMetric::CompositeL2;
    if (value == "makespan")
        return comotion::MRdRRT::CostMetric::Makespan;
    throw std::runtime_error("Unknown dRRT cost metric: " + value);
}

inline comotion::MRdRRT::TensorSearchMode
parseDrrtTensorSearchMode(const std::string &value) {
    if (value == "drrt" || value == "rrt" || value == "randomized")
        return comotion::MRdRRT::TensorSearchMode::Drrt;
    if (value == "astar" || value == "a_star" || value == "a-star")
        return comotion::MRdRRT::TensorSearchMode::AStar;
    if (value == "lazy_astar" || value == "lazy-a-star" ||
        value == "lazy_a_star" || value == "lazy-astar")
        return comotion::MRdRRT::TensorSearchMode::LazyAStar;
    throw std::runtime_error("Unknown dRRT tensor search mode: " + value);
}

inline comotion::MRdRRT::LocalConnectorMode
parseDrrtLocalConnectorMode(const std::string &value) {
    const std::string lowered = lowerAscii(value);
    if (lowered == "prioritized" || lowered == "paper" ||
        lowered == "paper_prioritized" || lowered == "paper-prioritized") {
        return comotion::MRdRRT::LocalConnectorMode::PaperPrioritized;
    }
    if (lowered == "synchronized" || lowered == "synchronised" ||
        lowered == "direct" || lowered == "straight_line" ||
        lowered == "straight-line") {
        return comotion::MRdRRT::LocalConnectorMode::Synchronized;
    }
    throw std::runtime_error("Unknown dRRT local connector mode: " + value);
}

inline void writeJson(const json &doc, const std::filesystem::path &path,
                      int indent) {
    if (path.has_parent_path())
        std::filesystem::create_directories(path.parent_path());
    std::ofstream ofs(path);
    if (!ofs.good())
        throw std::runtime_error("Unable to write JSON to " + path.string());
    if (indent > 0)
        ofs << doc.dump(indent) << "\n";
    else
        ofs << doc.dump() << "\n";
}

inline std::string requireValue(int &index, int argc, char **argv,
                                const std::string &flag) {
    if (index + 1 >= argc)
        throw std::runtime_error(flag + " requires a value");
    return argv[++index];
}

inline std::optional<json> normalizedSolutionEvent(const json &event) {
    if (!event.is_object() || !event.contains("elapsed_seconds") ||
        !event["elapsed_seconds"].is_number() ||
        !event.contains("makespan_timesteps") ||
        !event["makespan_timesteps"].is_number()) {
        return std::nullopt;
    }

    const double elapsed = event["elapsed_seconds"].get<double>();
    const double makespan = event["makespan_timesteps"].get<double>();
    if (!std::isfinite(elapsed) || !std::isfinite(makespan))
        return std::nullopt;

    json out = {
        {"elapsed_seconds", std::max(0.0, elapsed)},
        {"makespan_timesteps", event["makespan_timesteps"]},
    };
    return out;
}

inline json solutionSummaryJson(const TrialMetrics &metrics) {
    json events = json::array();
    if (metrics.planner_stats.is_object() &&
        metrics.planner_stats.contains("solution_events") &&
        metrics.planner_stats["solution_events"].is_array()) {
        for (const auto &event : metrics.planner_stats["solution_events"]) {
            if (auto normalized = normalizedSolutionEvent(event))
                events.push_back(*normalized);
        }
    }

    if (events.empty() && metrics.success &&
        metrics.makespan_timesteps.is_number()) {
        events.push_back({
            {"elapsed_seconds", std::max(0.0, metrics.planning_time_seconds)},
            {"makespan_timesteps", metrics.makespan_timesteps},
        });
    }

    std::sort(events.begin(), events.end(), [](const json &lhs,
                                               const json &rhs) {
        return lhs["elapsed_seconds"].get<double>() <
               rhs["elapsed_seconds"].get<double>();
    });

    return json{
        {"solution_events", events},
    };
}

template <typename GeneratedScenario>
std::vector<comotion::Path> makeEndpointPaths(
    const GeneratedScenario &generated) {
    std::vector<comotion::Path> paths;
    paths.reserve(generated.starts.size());
    for (std::size_t i = 0; i < generated.starts.size(); ++i) {
        comotion::Path path;
        path.push_back(generated.starts[i]);
        path.push_back(generated.goals[i]);
        path.set_waypoint_timesteps({0, 1});
        paths.push_back(std::move(path));
    }
    return paths;
}

template <typename GeneratedScenario>
TrialMetrics makeEndpointPathMetrics(
    const GeneratedScenario &generated, const json &benchmark_context,
    const std::shared_ptr<comotion::MultiRobotProblem> &problem) {
    TrialMetrics metrics;
    metrics.planner = "EndpointPath";
    metrics.collision_backend =
        backendName(problem->collisionChecker().backend());
    metrics.planner_status = "Synthetic start-goal endpoint path";
    metrics.success = false;
    metrics.sum_of_cost_timesteps =
        static_cast<std::uint64_t>(generated.num_robots);
    metrics.makespan_timesteps = static_cast<std::uint64_t>(1);
    metrics.planner_stats = {
        {"synthetic_endpoint_path", true},
        {"waypoints_per_robot", 2},
        {"skipped_planning", true},
    };
    metrics.benchmark_context = benchmark_context;
    metrics.solution_summary = solutionSummaryJson(metrics);
    return metrics;
}

template <typename Options>
PlannerBlueprint makePlannerBlueprint(const Options &options,
                                       bool app_verbose) {
    (void)app_verbose;

    if (options.algorithm == "composite") {
        PlannerBlueprint blueprint;
        blueprint.planner_name = "CompositeRRT";
        blueprint.factory = [&options]() {
            auto planner = std::make_shared<comotion::CompositeRRT>();
            planner->setSimplifySolution(options.composite_rrt_simplify_solution);
            planner->setUseMakespanMetric(
                options.composite_rrt_use_makespan_metric);
            if (options.composite_rrt_range > 0.0)
                planner->setRange(options.composite_rrt_range);
            return planner;
        };
        return blueprint;
    }

    if (options.algorithm == "composite_rrtstar" ||
        options.algorithm == "composite_rrt_star" ||
        options.algorithm == "composite_rrtstar_l2" ||
        options.algorithm == "composite_rrt_star_l2") {
        PlannerBlueprint blueprint;
        blueprint.planner_name = "CompositeRRTStar";
        blueprint.factory = [&options]() {
            auto planner = std::make_shared<comotion::CompositeRRTStar>();
            planner->setSimplifySolution(false);
            if (options.algorithm == "composite_rrtstar_l2" ||
                options.algorithm == "composite_rrt_star_l2") {
                planner->setMetricMode(
                    comotion::CompositeRRTStar::MetricMode::PlainL2);
            }
            return planner;
        };
        return blueprint;
    }

    if (options.algorithm == "composite_prmstar" ||
        options.algorithm == "composite_prm_star" ||
        options.algorithm == "composite_prmstar_l2" ||
        options.algorithm == "composite_prm_star_l2") {
        PlannerBlueprint blueprint;
        blueprint.planner_name = "CompositePRMStar";
        blueprint.factory = [&options]() {
            auto planner = std::make_shared<comotion::CompositePRMStar>();
            planner->setSimplifySolution(false);
            if (options.algorithm == "composite_prmstar_l2" ||
                options.algorithm == "composite_prm_star_l2") {
                planner->setMetricMode(
                    comotion::CompositePRMStar::MetricMode::PlainL2);
            }
            return planner;
        };
        return blueprint;
    }

    if (options.algorithm == "composite_aorrtc") {
        PlannerBlueprint blueprint;
        blueprint.planner_name = "CompositeAORRTC";
        blueprint.factory = [&options]() {
            auto planner = std::make_shared<comotion::CompositeAORRTC>();
            planner->setSimplifySolution(false);
            planner->setMaxInternalSamples(
                options.composite_aorrtc_max_internal_samples);
            planner->setMaxInternalVertices(
                options.composite_aorrtc_max_internal_vertices);
            return planner;
        };
        return blueprint;
    }

    if (options.algorithm == "cooperative_composite") {
        PlannerBlueprint blueprint;
        blueprint.planner_name = "CooperativeCompositeRRT";
        blueprint.factory = [&options]() {
            auto planner = std::make_shared<comotion::CooperativeCompositeRRT>();
            planner->setSimplifySolution(false);
            planner->setWorkerThreads(options.cooperative_rrt_worker_threads);
            return planner;
        };
        return blueprint;
    }

    if (options.algorithm == "prioritized") {
        PlannerBlueprint blueprint;
        blueprint.planner_name = "PrioritizedSTRRT";
        blueprint.factory = [&options]() {
            auto planner = std::make_shared<comotion::PrioritizedSTRRT>();
            planner->setUseUnboundedTime(true);
            planner->setInflateInitialBatchFromMinGoalTime(true);
            planner->setShufflePriorityOrder(
                options.strrt_shuffle_priority_order);
            planner->setReturnFirstSolution(
                options.strrt_return_first_solution);
            planner->setStrrtRewiring(
                parseStrrtRewiring(options.strrt_rewiring));
            planner->setStrrtInitialBatchSize(
                options.strrt_initial_batch_size);
            planner->setStrrtInitialTimeBoundFactor(
                options.strrt_initial_time_factor);
            planner->setStrrtTimeBoundFactorIncrease(
                options.strrt_time_bound_factor_increase);
            planner->setStrrtMaxInflatedBatchMultiplier(64);
            return planner;
        };
        blueprint.prepare_problem =
            [](const std::shared_ptr<comotion::MultiRobotProblem> &problem) {
                problem->setVmax(1.0);
            };
        return blueprint;
    }

    if (options.algorithm == "ao_arc") {
        PlannerBlueprint blueprint;
        blueprint.planner_name = "AOARC";
        blueprint.factory = [&options]() {
            auto planner = std::make_shared<comotion::AOARC>();
            planner->setInitialWindow(options.arc_initial_window);
            planner->setExpansionStep(options.arc_expansion_step);
            planner->setExpansionPolicy(
                parseArcExpansionPolicy(options.arc_expansion_policy));
            planner->setCustomExpansionMultipliers(
                parseArcExpansionMultipliers(
                    options.arc_expansion_multipliers));
            if (options.arc_initial_valid_expansion_policy) {
                planner->setInitialValidWindowExpansionPolicy(
                    parseArcExpansionPolicy(
                        *options.arc_initial_valid_expansion_policy));
            }
            if (options.arc_initial_valid_expansion_step) {
                planner->setInitialValidWindowExpansionStep(
                    *options.arc_initial_valid_expansion_step);
            }
            if (options.arc_initial_valid_expansion_multipliers) {
                planner->setInitialValidWindowExpansionMultipliers(
                    parseArcExpansionMultipliers(
                        *options.arc_initial_valid_expansion_multipliers));
            }
            planner->setInitialValidWindowExpansionSymmetric(
                options.arc_initial_valid_expansion_symmetric);
            planner->setLocalCompositeRrtMaxSamples(
                options.arc_local_composite_max_samples);
            planner->setLocalCompositeRrtRange(
                options.arc_local_composite_range);
            planner->setLocalCompositeRrtUseMakespanMetric(
                options.arc_local_composite_use_makespan_metric);
            planner->setLocalSolverMode(
                parseArcLocalSolverMode(options.arc_local_solvers));
            planner->setLocalPrioritizedStrrtMaxIterations(
                options.arc_local_prioritized_max_iterations);
            planner->setBoundedLocalRepairEpsilonTimesteps(
                options.ao_arc_local_bound_epsilon_timesteps);
            planner->setSimplifyInitialSolutions(
                options.arc_simplify_initial_solutions);
            planner->setSimplifyConflictSolutions(
                options.arc_simplify_conflict_solutions);
            planner->setUseCspaceBounds(true);
            planner->setCspaceBoundMargin(
                static_cast<float>(options.arc_cspace_bound_margin));
            planner->setMinCspaceBoundRange(
                options.arc_min_cspace_bound_range);
            planner->setPathSimplificationOptions(
                {options.arc_simplification_max_shortcut_steps,
                 options.arc_simplification_max_empty_steps,
                 options.arc_simplification_max_smooth_steps,
                 options.arc_simplification_max_passes});
            if (options.arc_conflict_simplification_options_explicit) {
                planner->setConflictPathSimplificationOptions(
                    {options.arc_conflict_simplification_max_shortcut_steps,
                     options.arc_conflict_simplification_max_empty_steps,
                     options.arc_conflict_simplification_max_smooth_steps,
                     options.arc_conflict_simplification_max_passes});
            }
            return planner;
        };
        return blueprint;
    }

    if (options.algorithm == "drrt") {
        PlannerBlueprint blueprint;
        blueprint.planner_name = "MRdRRT";
        blueprint.factory = [&options]() {
            auto planner = std::make_shared<comotion::MRdRRT>();
            planner->setRoadmapSize(options.drrt_roadmap_size);
            planner->setIterationsPerBatch(options.drrt_iterations_per_batch);
            planner->setCostMetric(parseDrrtCostMetric(options.drrt_cost_metric));
            planner->setTensorSearchMode(
                parseDrrtTensorSearchMode(options.drrt_tensor_search));
            planner->setLocalConnectorMode(
                parseDrrtLocalConnectorMode(options.drrt_local_connector));
            planner->setExcludeRoadmapBuildTimeFromBudget(
                options.drrt_exclude_roadmap_build_time);
            return planner;
        };
        return blueprint;
    }

    if (options.algorithm == "drrt_star" || options.algorithm == "drrtstar" ||
        options.algorithm == "ao_drrt" || options.algorithm == "ao-drrt") {
        PlannerBlueprint blueprint;
        blueprint.planner_name = "MRdRRTStar";
        blueprint.factory = [&options]() {
            auto planner = std::make_shared<comotion::MRdRRTStar>();
            planner->setRoadmapSize(options.drrt_roadmap_size);
            planner->setIterationsPerBatch(options.drrt_iterations_per_batch);
            planner->setCostMetric(parseDrrtCostMetric(options.drrt_cost_metric));
            planner->setTensorSearchMode(
                parseDrrtTensorSearchMode(options.drrt_tensor_search));
            planner->setLocalConnectorMode(
                parseDrrtLocalConnectorMode(options.drrt_local_connector));
            planner->setExcludeRoadmapBuildTimeFromBudget(
                options.drrt_exclude_roadmap_build_time);
            return planner;
        };
        return blueprint;
    }

    if (options.algorithm == "arc") {
        PlannerBlueprint blueprint;
        blueprint.planner_name = "ARC";
        blueprint.factory = [&options]() {
            auto planner = std::make_shared<comotion::ARC>();
            planner->setInitialWindow(options.arc_initial_window);
            planner->setExpansionStep(options.arc_expansion_step);
            planner->setExpansionPolicy(
                parseArcExpansionPolicy(options.arc_expansion_policy));
            planner->setCustomExpansionMultipliers(
                parseArcExpansionMultipliers(
                    options.arc_expansion_multipliers));
            if (options.arc_initial_valid_expansion_policy) {
                planner->setInitialValidWindowExpansionPolicy(
                    parseArcExpansionPolicy(
                        *options.arc_initial_valid_expansion_policy));
            }
            if (options.arc_initial_valid_expansion_step) {
                planner->setInitialValidWindowExpansionStep(
                    *options.arc_initial_valid_expansion_step);
            }
            if (options.arc_initial_valid_expansion_multipliers) {
                planner->setInitialValidWindowExpansionMultipliers(
                    parseArcExpansionMultipliers(
                        *options.arc_initial_valid_expansion_multipliers));
            }
            planner->setInitialValidWindowExpansionSymmetric(
                options.arc_initial_valid_expansion_symmetric);
            planner->setLocalCompositeRrtMaxSamples(
                options.arc_local_composite_max_samples);
            planner->setLocalCompositeRrtRange(
                options.arc_local_composite_range);
            planner->setLocalCompositeRrtUseMakespanMetric(
                options.arc_local_composite_use_makespan_metric);
            planner->setLocalSolverMode(
                parseArcLocalSolverMode(options.arc_local_solvers));
            planner->setLocalPrioritizedStrrtMaxIterations(
                options.arc_local_prioritized_max_iterations);
            planner->setSimplifyInitialSolutions(
                options.arc_simplify_initial_solutions);
            planner->setSimplifyConflictSolutions(
                options.arc_simplify_conflict_solutions);
            planner->setUseCspaceBounds(true);
            planner->setCspaceBoundMargin(
                static_cast<float>(options.arc_cspace_bound_margin));
            planner->setMinCspaceBoundRange(
                options.arc_min_cspace_bound_range);
            planner->setPathSimplificationOptions(
                {options.arc_simplification_max_shortcut_steps,
                 options.arc_simplification_max_empty_steps,
                 options.arc_simplification_max_smooth_steps,
                 options.arc_simplification_max_passes});
            if (options.arc_conflict_simplification_options_explicit) {
                planner->setConflictPathSimplificationOptions(
                    {options.arc_conflict_simplification_max_shortcut_steps,
                     options.arc_conflict_simplification_max_empty_steps,
                     options.arc_conflict_simplification_max_smooth_steps,
                     options.arc_conflict_simplification_max_passes});
            }
            return planner;
        };
        return blueprint;
    }

    if (options.algorithm == "parallel_arc") {
        PlannerBlueprint blueprint;
        blueprint.planner_name = "ParallelARC";
        blueprint.factory = [&options]() {
            auto planner = std::make_shared<comotion::ParallelARC>();
            planner->setInitialWindow(options.arc_initial_window);
            planner->setExpansionStep(options.arc_expansion_step);
            planner->setExpansionPolicy(
                parseArcExpansionPolicy(options.arc_expansion_policy));
            planner->setCustomExpansionMultipliers(
                parseArcExpansionMultipliers(
                    options.arc_expansion_multipliers));
            if (options.arc_initial_valid_expansion_policy) {
                planner->setInitialValidWindowExpansionPolicy(
                    parseArcExpansionPolicy(
                        *options.arc_initial_valid_expansion_policy));
            }
            if (options.arc_initial_valid_expansion_step) {
                planner->setInitialValidWindowExpansionStep(
                    *options.arc_initial_valid_expansion_step);
            }
            if (options.arc_initial_valid_expansion_multipliers) {
                planner->setInitialValidWindowExpansionMultipliers(
                    parseArcExpansionMultipliers(
                        *options.arc_initial_valid_expansion_multipliers));
            }
            planner->setInitialValidWindowExpansionSymmetric(
                options.arc_initial_valid_expansion_symmetric);
            planner->setLocalCompositeRrtMaxSamples(
                options.arc_local_composite_max_samples);
            planner->setLocalCompositeRrtRange(
                options.arc_local_composite_range);
            planner->setLocalCompositeRrtUseMakespanMetric(
                options.arc_local_composite_use_makespan_metric);
            planner->setLocalSolverMode(
                parseArcLocalSolverMode(options.arc_local_solvers));
            planner->setLocalPrioritizedStrrtMaxIterations(
                options.arc_local_prioritized_max_iterations);
            planner->setSimplifyInitialSolutions(
                options.arc_simplify_initial_solutions);
            planner->setSimplifyConflictSolutions(
                options.arc_simplify_conflict_solutions);
            planner->setUseCspaceBounds(true);
            planner->setCspaceBoundMargin(
                static_cast<float>(options.arc_cspace_bound_margin));
            planner->setMinCspaceBoundRange(
                options.arc_min_cspace_bound_range);
            planner->setPathSimplificationOptions(
                {options.arc_simplification_max_shortcut_steps,
                 options.arc_simplification_max_empty_steps,
                 options.arc_simplification_max_smooth_steps,
                 options.arc_simplification_max_passes});
            if (options.arc_conflict_simplification_options_explicit) {
                planner->setConflictPathSimplificationOptions(
                    {options.arc_conflict_simplification_max_shortcut_steps,
                     options.arc_conflict_simplification_max_empty_steps,
                     options.arc_conflict_simplification_max_smooth_steps,
                     options.arc_conflict_simplification_max_passes});
            }
            planner->setWorkerProcesses(options.parallel_arc_worker_processes);
            planner->setParallelizeInitialIndividualPlans(
                options.parallel_arc_parallel_initial_plans);
            planner->setInitialSolutionOr(
                options.parallel_arc_initial_solution_or);
            planner->setRepairDuplicateAttempts(
                options.parallel_arc_repair_duplicate_attempts);
            if (options.parallel_arc_strategy == "synchronous") {
                planner->setParallelStrategy(
                    comotion::ParallelArcParallelStrategy::Synchronous);
            } else if (options.parallel_arc_strategy == "asynchronous") {
                planner->setParallelStrategy(
                    comotion::ParallelArcParallelStrategy::Asynchronous);
            } else {
                throw std::runtime_error("Unknown ParallelARC strategy: " +
                                         options.parallel_arc_strategy);
            }
            if (options.parallel_arc_conflict_strategy == "greedy") {
                planner->setConflictSelectionStrategy(
                    comotion::ParallelArcConflictSelectionStrategy::Greedy);
            } else if (options.parallel_arc_conflict_strategy ==
                       "spatial_distribution") {
                planner->setConflictSelectionStrategy(
                    comotion::ParallelArcConflictSelectionStrategy::
                        SpatialDistribution);
            } else {
                throw std::runtime_error(
                    "Unknown ParallelARC conflict strategy: " +
                    options.parallel_arc_conflict_strategy);
            }
            if (options.parallel_arc_conflict_find_mode == "sequential") {
                planner->setConflictFindMode(
                    comotion::ParallelArcConflictFindMode::Sequential);
            } else if (options.parallel_arc_conflict_find_mode ==
                       "segment_parallel") {
                planner->setConflictFindMode(
                    comotion::ParallelArcConflictFindMode::SegmentParallel);
            } else {
                throw std::runtime_error(
                    "Unknown ParallelARC conflict-find mode: " +
                    options.parallel_arc_conflict_find_mode);
            }
            planner->setConflictFindHorizon(
                options.parallel_arc_conflict_find_horizon);
            planner->setConflictFindParallelAssignment(
                parseParallelArcConflictFindAssignment(
                    options.parallel_arc_conflict_find_assignment));
            planner->setConflictBatchMode(parseParallelArcConflictBatchMode(
                parallelArcConflictBatchModeValue(options)));
            return planner;
        };
        return blueprint;
    }

    if (options.algorithm == "stcbs") {
        PlannerBlueprint blueprint;
        blueprint.planner_name = "STCBS";
        blueprint.factory = [&options]() {
            auto planner = std::make_shared<comotion::STCBS>();
            planner->setMaxCTNodes(options.stcbs_max_ct_nodes);
            planner->setMaxSamples(options.stcbs_max_samples);
            return planner;
        };
        blueprint.prepare_problem =
            [](const std::shared_ptr<comotion::MultiRobotProblem> &problem) {
                problem->setVmax(1.0);
            };
        return blueprint;
    }

    throw std::runtime_error("Unknown algorithm: " + options.algorithm);
}

} // namespace comotion::benchmark_apps::common
