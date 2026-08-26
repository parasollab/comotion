#include "../apps/benchmark_app_common.hpp"

#include <functional>
#include <iostream>
#include <optional>
#include <string>

namespace common = comotion::benchmark_apps::common;

namespace {

struct Options {
    std::string algorithm = "arc";
    comotion::CollisionChecker::Backend collision_backend =
        comotion::CollisionChecker::Backend::Spheres;
    std::string vamp_validation_strategy = "not_a_vamp_strategy";

    unsigned int strrt_initial_batch_size = 0;
    double strrt_initial_time_factor = 0.0;
    double strrt_time_bound_factor_increase = 0.0;
    std::string strrt_rewiring = "not_a_strrt_mode";

    int drrt_roadmap_size = 0;
    int drrt_iterations_per_batch = 0;
    std::string drrt_cost_metric = "not_a_drrt_metric";
    std::string drrt_tensor_search = "not_a_drrt_search";
    std::string drrt_local_connector = "not_a_drrt_connector";

    double composite_rrt_range = -1.0;
    std::size_t composite_aorrtc_max_internal_samples = 0;
    std::size_t composite_aorrtc_max_internal_vertices = 0;
    unsigned int cooperative_rrt_worker_threads = 0;

    int arc_initial_window = 1;
    double arc_expansion_step = 1.0;
    std::string arc_expansion_policy = "linear";
    std::string arc_expansion_multipliers = "1";
    std::optional<std::string> arc_initial_valid_expansion_policy;
    std::optional<double> arc_initial_valid_expansion_step;
    std::optional<std::string> arc_initial_valid_expansion_multipliers;
    double arc_cspace_bound_margin = 0.0;
    double arc_min_cspace_bound_range = 0.0;
    double arc_local_composite_range = 0.0;
    std::string arc_local_solvers = "composite";
    std::string arc_local_prioritized_rewiring = "knearest";

    unsigned int or_parallel_worker_processes = 1;
    unsigned int parallel_arc_worker_processes = 0;
    std::string parallel_arc_strategy = "not_a_parallel_strategy";
    std::string parallel_arc_conflict_strategy = "not_a_conflict_strategy";
    std::string parallel_arc_conflict_find_mode = "segment_parallel";
    std::string parallel_arc_conflict_find_assignment = "not_an_assignment";
    std::string parallel_arc_conflict_batch_mode = "not_a_batch_mode";
    std::size_t parallel_arc_conflict_find_horizon = 0;
    bool parallel_arc_conflict_ablation_only = true;

    int stcbs_max_ct_nodes = 0;
    int stcbs_max_samples = 0;
};

bool expectNoThrow(const std::string &label,
                   const std::function<void()> &operation) {
    try {
        operation();
        return true;
    } catch (const std::exception &error) {
        std::cerr << label << " unexpectedly failed: " << error.what() << '\n';
        return false;
    }
}

bool expectThrow(const std::string &label,
                 const std::function<void()> &operation) {
    try {
        operation();
    } catch (const std::exception &) {
        return true;
    }
    std::cerr << label << " unexpectedly passed\n";
    return false;
}

} // namespace

int main() {
    bool ok = true;

    ok &= !common::arcHistoryRequested(false, false);
    ok &= !common::arcHistoryRequested(true, false);
    ok &= !common::arcHistoryRequested(false, true);
    ok &= common::arcHistoryRequested(true, true);

    auto arc = std::make_shared<comotion::ARC>();
    common::enableArcHistoryTracking(arc, true, false);
    ok &= !arc->visualizationTraceEnabled();
    common::enableArcHistoryTracking(arc, false, true);
    ok &= !arc->visualizationTraceEnabled();
    common::enableArcHistoryTracking(arc, true, true);
    ok &= arc->visualizationTraceEnabled();

    nlohmann::json artifact = nlohmann::json::object();
    common::appendArcVisualization(artifact, arc, true, false, 128, 1.0);
    ok &= !artifact.contains("arc_visualization");
    common::appendArcVisualization(artifact, arc, false, true, 128, 1.0);
    ok &= !artifact.contains("arc_visualization");

    comotion::Path sparse_path;
    sparse_path.push_back({0.0});
    sparse_path.push_back({2.0});
    sparse_path.push_back({4.0});
    sparse_path.set_waypoint_timesteps({0, 2, 4});
    const auto dense_path =
        common::densePathForExport(sparse_path, 128, 1.0);
    ok &= dense_path.size() == 5;
    ok &= dense_path.has_implicit_dense_timesteps();
    ok &= dense_path[1][0] == 1.0;
    ok &= dense_path[3][0] == 3.0;
    ok &= sparse_path.size() == 3;
    ok &= sparse_path.has_explicit_timesteps();

    comotion::Path index_timed_path;
    index_timed_path.push_back({0.0});
    index_timed_path.push_back({1.0});
    const auto unchanged_path =
        common::densePathForExport(index_timed_path, 128, 1.0);
    ok &= unchanged_path.size() == index_timed_path.size();

    Options options;
    ok &= expectNoThrow("ARC with unrelated invalid options", [&]() {
        common::validateSelectedPlannerOptions(options);
    });

    options.collision_backend = comotion::CollisionChecker::Backend::Fcl;
    options.parallel_arc_conflict_find_assignment = "cyclic_cover_greedy";
    ok &= expectNoThrow("ARC/FCL with a VAMP-only ParallelARC assignment",
                        [&]() {
                            common::validateSelectedPlannerOptions(options);
                        });

    options.algorithm = "parallel_arc";
    options.parallel_arc_worker_processes = 1;
    options.parallel_arc_strategy = "synchronous";
    options.parallel_arc_conflict_strategy = "greedy";
    options.parallel_arc_conflict_batch_mode = "optimistic";
    options.parallel_arc_conflict_find_horizon = 1;
    options.parallel_arc_conflict_ablation_only = false;
    ok &= expectThrow("ParallelARC/FCL with a VAMP-only assignment", [&]() {
        common::validateSelectedPlannerOptions(options);
    });

    options.algorithm = "arc";
    options.collision_backend = comotion::CollisionChecker::Backend::Vamp;
    ok &= expectThrow("VAMP with an invalid VAMP strategy", [&]() {
        common::validateSelectedPlannerOptions(options);
    });

    options.collision_backend = comotion::CollisionChecker::Backend::Spheres;
    options.algorithm = "prioritized";
    ok &= expectThrow("PrioritizedSTRRT with invalid STRRT options", [&]() {
        common::validateSelectedPlannerOptions(options);
    });

    options.algorithm = "drrt";
    ok &= expectThrow("dRRT with invalid dRRT options", [&]() {
        common::validateSelectedPlannerOptions(options);
    });

    options.algorithm = "stcbs";
    ok &= expectThrow("STCBS with invalid STCBS options", [&]() {
        common::validateSelectedPlannerOptions(options);
    });

    return ok ? 0 : 1;
}
