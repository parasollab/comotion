#include "comotion/planning/ARC.h"
#include "comotion/planning/AORRTCUtils.h"
#include "comotion/planning/CompositeRRT.h"
#include "comotion/planning/PlanningSeed.h"
#include "comotion/planning/PrioritizedSTRRT.h"
#include "comotion/planning/detail/PlannerInvariantUtils.h"
#include "comotion/planning/detail/SeededOmpl.h"
#include <ompl/util/RandomNumbers.h>
#include "comotion/collision/ValidationTypes.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/ScopedState.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <set>
#include <tuple>
#if !defined(_WIN32)
#include <sys/resource.h>
#endif
namespace {

using Clock = std::chrono::steady_clock;

std::uint64_t elapsedNanoseconds(const Clock::time_point &start) {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() -
                                                             start)
            .count());
}

double sumSeconds(const std::vector<double> &values) {
    double total = 0.0;
    for (const double value : values)
        total += value;
    return total;
}

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

std::uint64_t processCpuElapsedNanoseconds(double start) {
    return static_cast<std::uint64_t>(elapsedProcessCpuSeconds(start) * 1e9);
}

double timevalSeconds(const timeval &value) {
    return static_cast<double>(value.tv_sec) +
           static_cast<double>(value.tv_usec) * 1e-6;
}

const char *arcLocalSolverModeStr(comotion::ARC::LocalSolverMode mode) {
    switch (mode) {
    case comotion::ARC::LocalSolverMode::Both:
        return "both";
    case comotion::ARC::LocalSolverMode::PrioritizedStrrtOnly:
        return "prioritized";
    case comotion::ARC::LocalSolverMode::CompositeRrtOnly:
        return "composite";
    }
    return "unknown";
}

const char *strrtRewiringStr(comotion::StrrtRewiring mode) {
    switch (mode) {
    case comotion::StrrtRewiring::Off:
        return "off";
    case comotion::StrrtRewiring::Radius:
        return "radius";
    case comotion::StrrtRewiring::KNearest:
        return "knearest";
    }
    return "unknown";
}

const char *arcExpansionPolicyStr(comotion::ARC::ExpansionPolicy policy) {
    switch (policy) {
    case comotion::ARC::ExpansionPolicy::Linear:
        return "linear";
    case comotion::ARC::ExpansionPolicy::Logarithmic:
        return "logarithmic";
    case comotion::ARC::ExpansionPolicy::Exponential:
        return "exponential";
    case comotion::ARC::ExpansionPolicy::CustomMultiplied:
        return "multiplied";
    }
    return "unknown";
}

bool arcRepairWindowsIntersect(int lhs_start, int lhs_end, int rhs_start,
                               int rhs_end) {
    return lhs_start <= rhs_end && rhs_start <= lhs_end;
}

std::pair<std::uint64_t, std::uint64_t>
arcSolutionMetricsFromArrivalTimesteps(
    const std::vector<std::uint64_t> &arrival_timesteps) {
    std::uint64_t sum_of_cost_timesteps = 0;
    std::uint64_t makespan_timesteps = 0;
    for (const auto arrival_timestep : arrival_timesteps) {
        sum_of_cost_timesteps += arrival_timestep;
        makespan_timesteps = std::max(makespan_timesteps, arrival_timestep);
    }
    return {sum_of_cost_timesteps, makespan_timesteps};
}

} // namespace

namespace comotion {

namespace {

void validateArcExpansionMultipliers(
    const std::vector<double> &multipliers, const char *schedule_name) {
    if (multipliers.empty()) {
        std::ostringstream message;
        message << "ARC " << schedule_name
                << " expansion multiplier sequence must not be empty";
        throw std::invalid_argument(message.str());
    }
    for (std::size_t i = 0; i < multipliers.size(); ++i) {
        if (!std::isfinite(multipliers[i]) || multipliers[i] <= 0.0) {
            std::ostringstream message;
            message << "ARC " << schedule_name << " expansion multiplier " << i
                    << " must be finite and positive";
            throw std::invalid_argument(message.str());
        }
    }
}

} // namespace

void ARC::setCustomExpansionMultipliers(std::vector<double> multipliers) {
    validateArcExpansionMultipliers(multipliers, "custom");
    custom_expansion_multipliers_ = std::move(multipliers);
}

void ARC::setInitialValidWindowExpansionMultipliers(
    std::vector<double> multipliers) {
    validateArcExpansionMultipliers(multipliers, "initial-valid custom");
    initial_valid_window_expansion_multipliers_ = std::move(multipliers);
}

std::pair<int, int>
ARC::nextExpansionWindow(int start_t, int end_t, std::size_t max_t,
                         std::size_t expansion_index) const {
    return nextExpansionWindowWithSettings(
        start_t, end_t, max_t, expansion_index, expansion_policy_,
        expansion_step_, custom_expansion_multipliers_);
}

std::pair<int, int> ARC::nextInitialValidExpansionWindow(
    int start_t, int end_t, std::size_t max_t,
    std::size_t expansion_index) const {
    return nextExpansionWindowWithSettings(
        start_t, end_t, max_t, expansion_index,
        initialValidWindowExpansionPolicy(),
        initialValidWindowExpansionStep(),
        initialValidWindowExpansionMultipliers());
}

std::pair<int, int> ARC::symmetricWindowFromGeometry(
    std::int64_t center_twice, std::int64_t half_width_twice,
    std::size_t max_t) const {
    const long double raw_start =
        (static_cast<long double>(center_twice) -
         static_cast<long double>(half_width_twice)) /
        2.0L;
    const long double raw_end =
        (static_cast<long double>(center_twice) +
         static_cast<long double>(half_width_twice)) /
        2.0L;
    const int start_t = static_cast<int>(std::max<long double>(
        0.0L, std::floor(raw_start)));
    const int end_t = static_cast<int>(std::min<long double>(
        static_cast<long double>(max_t), std::ceil(raw_end)));
    return {start_t, end_t};
}

std::pair<int, int> ARC::absoluteExpansionWindow(
    int start_t, int end_t, std::size_t max_t,
    std::size_t expansion_index, ExpansionPolicy policy,
    double configured_expansion_step,
    const std::vector<double> &custom_multipliers,
    std::int64_t center_twice,
    std::int64_t base_half_width_twice) const {
    const auto global_window =
        std::make_pair(0, static_cast<int>(max_t));
    if (start_t == 0 &&
        static_cast<std::size_t>(std::max(0, end_t)) >= max_t) {
        return global_window;
    }

    long double target_half_width_twice =
        static_cast<long double>(
            std::max<std::int64_t>(1, base_half_width_twice));
    const long double expansion_step =
        static_cast<long double>(
            std::isfinite(configured_expansion_step) &&
                    configured_expansion_step > 0.0
                ? configured_expansion_step
                : 1.0);
    switch (policy) {
    case ExpansionPolicy::Linear:
        target_half_width_twice +=
            2.0L * expansion_step *
            (static_cast<long double>(expansion_index) + 1.0L);
        break;
    case ExpansionPolicy::Logarithmic:
        target_half_width_twice +=
            2.0L * expansion_step *
            std::log2(static_cast<long double>(expansion_index) + 2.0L);
        break;
    case ExpansionPolicy::Exponential:
        target_half_width_twice +=
            2.0L * std::ldexp(
                         expansion_step,
                         expansion_index >=
                                 static_cast<std::size_t>(
                                     std::numeric_limits<int>::max())
                             ? std::numeric_limits<int>::max()
                             : static_cast<int>(expansion_index));
        break;
    case ExpansionPolicy::CustomMultiplied:
        if (expansion_index >= custom_multipliers.size())
            return global_window;
        target_half_width_twice =
            static_cast<long double>(
                std::max<std::int64_t>(1, base_half_width_twice)) *
            static_cast<long double>(
                custom_multipliers[expansion_index]);
        break;
    }

    const auto half_width_twice =
        !std::isfinite(target_half_width_twice) ||
                target_half_width_twice >=
                    static_cast<long double>(
                        std::numeric_limits<std::int64_t>::max())
            ? std::numeric_limits<std::int64_t>::max()
            : static_cast<std::int64_t>(
                  std::ceil(target_half_width_twice));
    auto next = symmetricWindowFromGeometry(
        center_twice, std::max<std::int64_t>(1, half_width_twice), max_t);

    // Repeated custom entries intentionally restart the local solver at the
    // same bounds. For formula-based schedules, a repeated/clipped size means
    // no further useful local growth, so make the next attempt global.
    if (policy != ExpansionPolicy::CustomMultiplied &&
        next.first == start_t && next.second == end_t) {
        return global_window;
    }

    if (policy == ExpansionPolicy::Logarithmic) {
        const std::size_t ten_percent = max_t / 10;
        const std::size_t left_gap =
            static_cast<std::size_t>(std::max(0, next.first));
        const std::size_t right_gap =
            max_t - std::min(
                        static_cast<std::size_t>(
                            std::max(0, next.second)),
                        max_t);
        if (left_gap <= ten_percent && right_gap <= ten_percent)
            return global_window;
    }
    return next;
}

void ARC::establishMainWindowGeometry(
    int start_t, int end_t, ExpansionScheduleState &state) const {
    if (state.initial_valid_window_established)
        return;
    state.initial_valid_window_established = true;
    state.main_window_center_twice =
        static_cast<std::int64_t>(start_t) +
        static_cast<std::int64_t>(end_t);
    state.main_base_half_width_twice =
        std::max<std::int64_t>(
            1, static_cast<std::int64_t>(end_t) -
                   static_cast<std::int64_t>(start_t));
}

std::pair<int, int> ARC::nextInitialValidExpansionWindow(
    int start_t, int end_t, std::size_t max_t,
    std::size_t expansion_index, bool start_valid, bool goal_valid,
    const ExpansionScheduleState &state) const {
    const auto policy = initialValidWindowExpansionPolicy();
    const auto candidate = absoluteExpansionWindow(
        start_t, end_t, max_t, expansion_index, policy,
        initialValidWindowExpansionStep(),
        initialValidWindowExpansionMultipliers(),
        state.initial_search_center_twice,
        state.initial_search_half_width_twice);

    // Endpoint search never shrinks a side. In asymmetric mode, a side whose
    // endpoint is already valid remains anchored while only invalid sides grow.
    const bool expand_start =
        initial_valid_window_expansion_symmetric_ || !start_valid;
    const bool expand_goal =
        initial_valid_window_expansion_symmetric_ || !goal_valid;
    const std::pair<int, int> next{
        expand_start ? std::min(start_t, candidate.first) : start_t,
        expand_goal ? std::max(end_t, candidate.second) : end_t,
    };
    if (policy != ExpansionPolicy::CustomMultiplied &&
        next.first == start_t && next.second == end_t) {
        return {0, static_cast<int>(max_t)};
    }
    return next;
}

std::pair<int, int> ARC::nextMainExpansionWindow(
    int start_t, int end_t, std::size_t max_t,
    std::size_t expansion_index,
    const ExpansionScheduleState &state) const {
    return absoluteExpansionWindow(
        start_t, end_t, max_t, expansion_index, expansion_policy_,
        expansion_step_, custom_expansion_multipliers_,
        state.main_window_center_twice,
        state.main_base_half_width_twice);
}

std::pair<int, int> ARC::nextExpansionWindowAfterAttempt(
    int start_t, int end_t, std::size_t max_t,
    bool start_valid, bool goal_valid,
    ExpansionScheduleState &state) const {
    if (!state.initial_search_geometry_initialized) {
        state.initial_search_geometry_initialized = true;
        state.initial_search_center_twice =
            static_cast<std::int64_t>(start_t) +
            static_cast<std::int64_t>(end_t);
        state.initial_search_half_width_twice =
            std::max<std::int64_t>(
                1, static_cast<std::int64_t>(end_t) -
                       static_cast<std::int64_t>(start_t));
    }
    if (start_valid && goal_valid)
        establishMainWindowGeometry(start_t, end_t, state);

    state.last_expansion_used_initial_valid_schedule =
        !state.initial_valid_window_established;
    if (state.last_expansion_used_initial_valid_schedule) {
        const auto next = nextInitialValidExpansionWindow(
            start_t, end_t, max_t, state.initial_valid_expansion_index,
            start_valid, goal_valid, state);
        ++state.initial_valid_expansion_index;
        return next;
    }

    const auto next = nextMainExpansionWindow(
        start_t, end_t, max_t, state.main_expansion_index, state);
    ++state.main_expansion_index;
    return next;
}

std::pair<int, int> ARC::nextExpansionWindowWithSettings(
    int start_t, int end_t, std::size_t max_t,
    std::size_t expansion_index, ExpansionPolicy policy,
    double configured_expansion_step,
    const std::vector<double> &custom_multipliers) const {
    const auto center_twice =
        static_cast<std::int64_t>(start_t) +
        static_cast<std::int64_t>(end_t);
    const auto base_half_width_twice =
        std::max<std::int64_t>(
            1, static_cast<std::int64_t>(end_t) -
                   static_cast<std::int64_t>(start_t));
    return absoluteExpansionWindow(
        start_t, end_t, max_t, expansion_index, policy,
        configured_expansion_step, custom_multipliers,
        center_twice, base_half_width_twice);
}

void ARC::resetArcSolveState() {
    resetPlannerRunMetrics();
    solution_paths_.clear();
    repair_window_schedule_.clear();
    applied_repair_history_events_.clear();
    conflict_scan_robot_count_ = 0;
    pair_conflict_scan_start_t_.clear();
    true_arrival_timesteps_.clear();
    last_subproblem_window_start_ = -1;
    warned_bounded_prioritized_disabled_ = false;
    num_conflicts_ = 0;
    num_subproblem_attempts_ = 0;
    num_temporal_expansions_ = 0;
    num_initial_valid_temporal_expansions_ = 0;
    num_main_temporal_expansions_ = 0;
    next_repair_attempt_id_ = 0;
    repair_attempt_events_.clear();
    initial_solution_times_seconds_wall_clock_ = 0.0;
    initial_solution_times_seconds_cpu_ = 0.0;
    initial_simplification_times_seconds_wall_clock_ = 0.0;
    local_composite_simplification_times_seconds_wall_clock_ = 0.0;
    conflict_detection_times_seconds_.clear();
    conflict_detection_times_cpu_seconds_.clear();
    conflict_find_main_process_wall_seconds_.clear();
    conflict_find_process_tree_cpu_seconds_.clear();
    conflict_find_build_worker_wall_seconds_.clear();
    conflict_find_build_worker_cpu_seconds_.clear();
    conflict_find_collision_worker_wall_seconds_.clear();
    conflict_find_collision_worker_cpu_seconds_.clear();
    conflict_find_critical_worker_index_.clear();
    conflict_find_critical_worker_build_wall_seconds_.clear();
    conflict_find_critical_worker_collision_wall_seconds_.clear();
    conflict_find_critical_worker_total_wall_seconds_.clear();
    conflict_resolution_times_seconds_.clear();
    conflict_resolution_times_cpu_seconds_.clear();
    visualization_trace_.clear();
}

void ARC::startVisualizationIteration(const std::vector<Path> &paths) {
    if (!visualization_trace_enabled_)
        return;
    VisualizationIteration iteration;
    iteration.paths = paths;
    visualization_trace_.push_back(std::move(iteration));
}

void ARC::setVisualizationConflicts(
    const std::vector<SubproblemConflict> &conflicts) {
    if (!visualization_trace_enabled_)
        return;
    if (visualization_trace_.empty())
        throw std::runtime_error(
            "ARC visualization conflict batch has no path snapshot");
    auto &iteration = visualization_trace_.back();
    iteration.conflict_scan_completed = true;
    iteration.conflicts = conflicts;
}

void ARC::appendVisualizationRepair(
    std::size_t conflict_index, const std::vector<int> &robots,
    int window_start_t, int window_end_t,
    const std::vector<Path> &local_paths) {
    if (!visualization_trace_enabled_)
        return;
    if (visualization_trace_.empty())
        throw std::runtime_error(
            "ARC visualization repair has no path snapshot");
    if (robots.size() != local_paths.size())
        throw std::runtime_error(
            "ARC visualization repair path count does not match robot count");
    VisualizationRepair repair;
    repair.conflict_index = conflict_index;
    repair.robots = robots;
    repair.window_start_t = window_start_t;
    repair.window_end_t = window_end_t;
    repair.local_paths = local_paths;
    visualization_trace_.back().repairs.push_back(std::move(repair));
}

void ARC::initializeConflictScanStarts(std::size_t robot_count) {
    conflict_scan_robot_count_ = robot_count;
    pair_conflict_scan_start_t_.assign(pairFrontierSize(robot_count), 0);
    updateDerivedConflictScanStart();
}

CompositePathValidationOptions ARC::conflictScanOptions() const {
    CompositePathValidationOptions options;
    options.check_environment = false;
    options.t_begin =
        last_subproblem_window_start_ > 0
            ? static_cast<std::size_t>(last_subproblem_window_start_)
            : 0;
    if (!pair_conflict_scan_start_t_.empty()) {
        options.per_pair_t_begin.reserve(pair_conflict_scan_start_t_.size());
        for (const int start_t : pair_conflict_scan_start_t_) {
            options.per_pair_t_begin.push_back(
                static_cast<std::size_t>(std::max(0, start_t)));
        }
    }
    return options;
}

void ARC::applyConflictScanProgress(
    const std::vector<std::size_t> &next_t_begin_by_pair) {
    if (next_t_begin_by_pair.empty())
        return;
    if (next_t_begin_by_pair.size() !=
        pairFrontierSize(conflict_scan_robot_count_)) {
        throw std::runtime_error(
            "ARC conflict scan progress size mismatch");
    }

    pair_conflict_scan_start_t_.resize(next_t_begin_by_pair.size(), 0);
    for (std::size_t i = 0; i < next_t_begin_by_pair.size(); ++i) {
        const auto next_t = next_t_begin_by_pair[i];
        pair_conflict_scan_start_t_[i] =
            next_t > static_cast<std::size_t>(std::numeric_limits<int>::max())
                ? std::numeric_limits<int>::max()
                : static_cast<int>(next_t);
    }
    updateDerivedConflictScanStart();
}

void ARC::resetConflictScanStartsForRobots(const std::vector<int> &robots,
                                           int start_t) {
    const int clamped_start = std::max(0, start_t);
    std::vector<int> unique_robots = robots;
    std::sort(unique_robots.begin(), unique_robots.end());
    unique_robots.erase(std::unique(unique_robots.begin(), unique_robots.end()),
                        unique_robots.end());
    for (const int robot : robots) {
        if (robot < 0 ||
            static_cast<std::size_t>(robot) >= conflict_scan_robot_count_) {
            throw std::runtime_error(
                "ARC conflict scan reset robot index out of range");
        }
    }
    for (const int robot : unique_robots) {
        const std::size_t robot_index = static_cast<std::size_t>(robot);
        for (std::size_t other = 0; other < conflict_scan_robot_count_; ++other) {
            if (other == robot_index)
                continue;
            const std::size_t pair_index =
                pairFrontierIndex(robot_index, other,
                                  conflict_scan_robot_count_);
            pair_conflict_scan_start_t_[pair_index] = std::min(
                pair_conflict_scan_start_t_[pair_index], clamped_start);
        }
    }
    updateDerivedConflictScanStart();
}

void ARC::updateDerivedConflictScanStart() {
    if (pair_conflict_scan_start_t_.empty()) {
        last_subproblem_window_start_ = -1;
        return;
    }
    last_subproblem_window_start_ = *std::min_element(
        pair_conflict_scan_start_t_.begin(),
        pair_conflict_scan_start_t_.end());
}

void ARC::recordAppliedRepairHistory(const std::vector<int> &robots,
                                     int window_start_t, int window_end_t) {
    if (robots.empty())
        return;
    if (window_end_t < window_start_t) {
        throw std::runtime_error(
            "ARC repair history window end precedes window start");
    }

    std::set<int> unique_robots;
    for (const int robot : robots) {
        if (robot < 0) {
            throw std::runtime_error("ARC repair history robot index negative");
        }
        unique_robots.insert(robot);
    }
    if (unique_robots.empty())
        return;

    AppliedRepairHistoryEvent history_event;
    history_event.event_id =
        static_cast<int>(applied_repair_history_events_.size());
    history_event.robots.assign(unique_robots.begin(), unique_robots.end());
    history_event.window_start_t = window_start_t;
    history_event.window_end_t = window_end_t;
    applied_repair_history_events_.push_back(history_event);

    const auto insertWindow = [&](int robot_a, int robot_b) {
        auto &windows = repair_window_schedule_[robot_a][robot_b];
        RepairWindow merged;
        merged.window_start_t = window_start_t;
        merged.window_end_t = window_end_t;
        merged.history_event_ids.push_back(history_event.event_id);
        std::vector<RepairWindow> next;
        next.reserve(windows.size() + 1);
        bool inserted = false;

        for (const auto &window : windows) {
            if (window.window_end_t < merged.window_start_t) {
                next.push_back(window);
                continue;
            }
            if (merged.window_end_t < window.window_start_t) {
                if (!inserted) {
                    next.push_back(merged);
                    inserted = true;
                }
                next.push_back(window);
                continue;
            }

            merged.window_start_t =
                std::min(merged.window_start_t, window.window_start_t);
            merged.window_end_t =
                std::max(merged.window_end_t, window.window_end_t);
            merged.history_event_ids.insert(merged.history_event_ids.end(),
                                            window.history_event_ids.begin(),
                                            window.history_event_ids.end());
        }

        std::sort(merged.history_event_ids.begin(),
                  merged.history_event_ids.end());
        merged.history_event_ids.erase(
            std::unique(merged.history_event_ids.begin(),
                        merged.history_event_ids.end()),
            merged.history_event_ids.end());
        if (!inserted)
            next.push_back(merged);
        windows = std::move(next);
    };

    for (auto it_a = unique_robots.begin(); it_a != unique_robots.end();
         ++it_a) {
        auto it_b = it_a;
        ++it_b;
        for (; it_b != unique_robots.end(); ++it_b) {
            insertWindow(*it_a, *it_b);
            insertWindow(*it_b, *it_a);
        }
    }
}

bool ARC::planIndividualPaths(const Clock::time_point &solve_start,
                              double timeLimit,
                              std::vector<Path> &working_paths) {
    int n = problem_->numRobots();
    working_paths.resize(n);
    true_arrival_timesteps_.assign(static_cast<std::size_t>(n), 0);
    const auto initial_wall_start = Clock::now();
    const double initial_cpu_start = processCpuSeconds();
    const auto finishInitialTiming = [&]() {
        initial_solution_times_seconds_wall_clock_ +=
            std::chrono::duration<double>(Clock::now() - initial_wall_start)
                .count();
        initial_solution_times_seconds_cpu_ +=
            elapsedProcessCpuSeconds(initial_cpu_start);
    };

    for (int i = 0; i < n; ++i) {
        double elapsed_s = std::chrono::duration<double>(
                               std::chrono::steady_clock::now() - solve_start)
                               .count();
        double remaining = timeLimit - elapsed_s;
        if (remaining <= 0.0) {
            finishInitialTiming();
            return false;
        }
        auto result = planIndividualPath(i, remaining);
        recordInitialIndividualPlanStats(result);
        if (!result.success) {
            finishInitialTiming();
            return false;
        }
        working_paths[i] = std::move(result.path);
        true_arrival_timesteps_[static_cast<std::size_t>(i)] =
            result.arrival_timestep;
    }

    finishInitialIndividualPaths(working_paths);
    finishInitialTiming();
    return true;
}

ARC::IndividualPlanResult ARC::planIndividualPath(
    int robot_index, double solve_budget_seconds,
    std::optional<std::uint32_t> local_seed) {
    IndividualPlanResult result;
    result.status_type =
        static_cast<int>(ompl::base::PlannerStatus::UNKNOWN);
    const double cpu_start = processCpuSeconds();

    const auto &robot = problem_->robot(robot_index);
    int ndof = robot.model->numJoints();

    if (global_makespan_bound_timesteps_) {
        aorrtc::SolveOptions options;
        options.planning_seed = local_seed.value_or(planning_seed_);
        options.cost_bound_timesteps = *global_makespan_bound_timesteps_;
        options.simplify_solution = simplify_initial_solutions_;
        options.simplification_options = simplification_options_;
        if (local_composite_rrt_max_samples_ > 0) {
            options.max_internal_samples = local_composite_rrt_max_samples_;
            options.max_internal_vertices = local_composite_rrt_max_samples_;
        }

        const auto solve_call_start = Clock::now();
        auto bounded = aorrtc::solveSingleRobotBounded(
            *problem_, robot_index, solve_budget_seconds, options);
        result.solve_ns = elapsedNanoseconds(solve_call_start);
        result.simplify_ns = static_cast<std::uint64_t>(
            std::max(0.0, bounded.simplify_seconds) * 1e9);
        result.status_type =
            static_cast<int>(static_cast<ompl::base::PlannerStatus::StatusType>(
                bounded.status));
        result.status_message = bounded.status.asString();
        result.cpu_seconds = elapsedProcessCpuSeconds(cpu_start);
        if (bounded.status != ompl::base::PlannerStatus::EXACT_SOLUTION ||
            bounded.paths.empty()) {
            return result;
        }

        result.path = std::move(bounded.paths.front());
        result.arrival_timestep =
            static_cast<std::uint64_t>(result.path.arrival_timestep());
        if (result.path.empty()) {
            std::ostringstream msg;
            msg << "ARC exact bounded individual path missing for robot "
                << robot_index;
            throw std::runtime_error(msg.str());
        }
        constexpr double kEndpointTolerance = 1e-6;
        std::ostringstream start_context;
        start_context << "ARC exact bounded individual start mismatch for robot "
                      << robot_index;
        comotion::detail::requireConfigNear(result.path.front(), robot.start,
                                        kEndpointTolerance,
                                        start_context.str());
        std::ostringstream goal_context;
        goal_context << "ARC exact bounded individual goal mismatch for robot "
                     << robot_index;
        comotion::detail::requireConfigNear(result.path.back(), robot.goal,
                                        kEndpointTolerance,
                                        goal_context.str());
        result.success = true;
        return result;
    }

    auto si = problem_->createSpaceInfo(robot_index);
    auto space = si->getStateSpace();

    if (local_seed.has_value()) {
        const auto sampler_seed = omplLocalSeedFromUserPlanningSeed(
            *local_seed, 17);
        space->setStateSamplerAllocator(
            [sampler_seed](const ompl::base::StateSpace *sampler_space) {
                return std::make_shared<detail::SeededRealVectorStateSampler>(
                    sampler_space, sampler_seed);
            });
    }

    ompl::geometric::SimpleSetup setup(si);
    if (local_seed.has_value()) {
        setup.setPlanner(std::make_shared<detail::SeededRRTConnect>(
            si, omplLocalSeedFromUserPlanningSeed(*local_seed, 23)));
    } else {
        setup.setPlanner(
            std::make_shared<ompl::geometric::RRTConnect>(si));
    }

    ompl::base::ScopedState<> start(space);
    ompl::base::ScopedState<> goal(space);
    for (int d = 0; d < ndof; ++d) {
        start->as<ompl::base::RealVectorStateSpace::StateType>()->values[d] =
            robot.start[d];
        goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[d] =
            robot.goal[d];
    }
    setup.setStartAndGoalStates(start, goal);

    const auto solve_call_start = Clock::now();
    auto status = setup.solve(solve_budget_seconds);
    result.solve_ns = elapsedNanoseconds(solve_call_start);
    result.status_type =
        static_cast<int>(static_cast<ompl::base::PlannerStatus::StatusType>(
            status));
    result.status_message = status.asString();
    if (status != ompl::base::PlannerStatus::EXACT_SOLUTION) {
        result.cpu_seconds = elapsedProcessCpuSeconds(cpu_start);
        return result;
    }

    if (simplify_initial_solutions_) {
        const auto simplify_start = std::chrono::steady_clock::now();
        if (local_seed.has_value()) {
            auto simplifier = std::make_shared<detail::SeededPathSimplifier>(
                si, omplLocalSeedFromUserPlanningSeed(*local_seed, 29),
                setup.getGoal(), setup.getOptimizationObjective());
            detail::simplifyPathBounded(setup.getSolutionPath(), simplifier,
                                        simplification_options_);
        } else {
            detail::simplifySolutionBounded(setup, simplification_options_);
        }
        result.simplify_ns = elapsedNanoseconds(simplify_start);
    }
    auto &gpath = setup.getSolutionPath();
    gpath.interpolate();

    const size_t res = problem_->resolution();
    const double vmax = problem_->vmax();
    result.path = omplPathToPath(gpath, ndof, res, vmax);
    result.arrival_timestep =
        static_cast<std::uint64_t>(result.path.arrival_timestep());
    if (result.path.empty()) {
        std::ostringstream msg;
        msg << "ARC exact individual path missing for robot " << robot_index;
        throw std::runtime_error(msg.str());
    }
    constexpr double kEndpointTolerance = 1e-6;
    std::ostringstream start_context;
    start_context << "ARC exact individual start mismatch for robot "
                  << robot_index;
    comotion::detail::requireConfigNear(result.path.front(), robot.start,
                                    kEndpointTolerance, start_context.str());
    std::ostringstream goal_context;
    goal_context << "ARC exact individual goal mismatch for robot "
                 << robot_index;
    comotion::detail::requireConfigNear(result.path.back(), robot.goal,
                                    kEndpointTolerance, goal_context.str());
    result.success = true;
    result.cpu_seconds = elapsedProcessCpuSeconds(cpu_start);
    return result;
}

void ARC::recordInitialIndividualPlanStats(
    const IndividualPlanResult &result) {
    if (result.simplify_ns > 0) {
        initial_simplification_times_seconds_wall_clock_ +=
            static_cast<double>(result.simplify_ns) * 1e-9;
    }
}

void ARC::finishInitialIndividualPaths(std::vector<Path> &working_paths) {
    const auto [sum_of_cost_timesteps, makespan_timesteps] =
        arcSolutionMetricsFromArrivalTimesteps(true_arrival_timesteps_);
    setSolutionMetrics(sum_of_cost_timesteps, makespan_timesteps);
}

bool ARC::solveSubproblemOnPaths(const SubproblemConflict &conflict,
                                 const Clock::time_point &solve_start,
                                 double global_time_limit,
                                 std::vector<Path> &working_paths,
                                 int *window_start_t_out,
                                 int *window_end_t_out,
                                 std::vector<Path> *local_paths_out,
                                 bool apply_solution_to_paths,
                                 CancellationCallback cancel_requested) {
    const auto &involved_robots = conflict.robots;
    const auto isCancelled = [&]() {
        return cancel_requested && cancel_requested();
    };

    // max_t = max path length over involved robots (match mr-vamp)
    size_t max_t = 0;
    for (int r : involved_robots) {
        const auto &path = working_paths[static_cast<std::size_t>(r)];
        const std::size_t path_horizon =
            path.empty() ? 0 : path.arrival_timestep() + 1;
        max_t = std::max(max_t, path_horizon);
    }
    if (max_t == 0)
        return false;

    int start_t = std::max(0, conflict.window_begin_t);
    int end_t = std::max(conflict.conflict_timestep, conflict.window_end_t);
    end_t = static_cast<int>(std::min(static_cast<size_t>(std::max(0, end_t)),
                                      max_t));
    if (end_t == 0 && max_t > 0)
        end_t = 1;
    ExpansionScheduleState expansion_schedule_state;
    const std::uint64_t repair_id = next_repair_attempt_id_++;
    std::uint64_t repair_attempt_index = 0;
    RepairAttemptPhase current_attempt_phase =
        RepairAttemptPhase::InitialWindow;
    std::optional<std::size_t> current_expansion_index;

    const auto recordWindow = [&]() {
        if (window_start_t_out)
            *window_start_t_out = start_t;
        if (window_end_t_out)
            *window_end_t_out = end_t;
    };

    for (;;) {
        recordWindow();
        ++num_subproblem_attempts_;
        RepairAttemptEvent attempt_event;
        attempt_event.repair_id = repair_id;
        attempt_event.attempt_index = repair_attempt_index++;
        attempt_event.attempt_root_planning_seed =
            arcRepairAttemptPlanningSeed(planning_seed_, repair_id,
                                         attempt_event.attempt_index);
        attempt_event.seed_robot_i = conflict.seed_robot_i;
        attempt_event.seed_robot_j = conflict.seed_robot_j;
        attempt_event.conflict_timestep = conflict.conflict_timestep;
        attempt_event.robots = involved_robots;
        attempt_event.phase = current_attempt_phase;
        attempt_event.expansion_index = current_expansion_index;
        attempt_event.window_start_t = start_t;
        attempt_event.window_end_t = end_t;
        attempt_event.max_t = max_t;
        attempt_event.effective_global =
            start_t == 0 && static_cast<std::size_t>(std::max(0, end_t)) >=
                                max_t;
        repair_attempt_events_.push_back(std::move(attempt_event));
        auto &current_event = repair_attempt_events_.back();

        const double elapsed_s = std::chrono::duration<double>(
                                       std::chrono::steady_clock::now() - solve_start)
                                       .count();
        const double time_remaining = global_time_limit - elapsed_s;
        if (isCancelled()) {
            current_event.outcome = "cancelled";
            return false;
        }
        if (time_remaining <= 0.0) {
            current_event.outcome = "budget_exhausted";
            return false;
        }

        // Build local subproblem
        auto sub_problem = std::make_shared<MultiRobotProblem>(
            problem_->collisionChecker().backend());
        sub_problem->setObstacles(
            std::vector<ObstacleSphere>(
                problem_->collisionChecker().obstacles().begin(),
                problem_->collisionChecker().obstacles().end()));
        sub_problem->setCylinderObstacles(
            std::vector<ObstacleCylinder>(
                problem_->collisionChecker().cylinders().begin(),
                problem_->collisionChecker().cylinders().end()));
        sub_problem->setVmax(problem_->vmax());
        sub_problem->setResolution(problem_->resolution());

        for (int r : involved_robots) {
            auto &robot = problem_->robot(r);
            const auto &path = working_paths[static_cast<std::size_t>(r)];
            auto start_config =
                path.config_at_timestep(static_cast<std::size_t>(start_t));
            auto goal_config =
                path.config_at_timestep(static_cast<std::size_t>(end_t));
            sub_problem->addRobot(robot.model, start_config, goal_config);
        }

        auto sub_ptrs = sub_problem->robotModelPtrs();
        std::vector<std::vector<double>> joint_start, joint_goal;
        joint_start.reserve(sub_ptrs.size());
        joint_goal.reserve(sub_ptrs.size());
        for (int si = 0; si < sub_problem->numRobots(); ++si) {
            joint_start.push_back(sub_problem->robot(si).start);
            joint_goal.push_back(sub_problem->robot(si).goal);
        }
        auto &cc_sub = sub_problem->collisionChecker();
        const bool start_composite_ok =
            sub_ptrs.empty() ||
            cc_sub.isValidComposite(sub_ptrs, joint_start);
        const bool goal_composite_ok =
            sub_ptrs.empty() ||
            cc_sub.isValidComposite(sub_ptrs, joint_goal);
        const bool local_endpoints_valid =
            start_composite_ok && goal_composite_ok;
        current_event.validity_checked = true;
        current_event.start_valid = start_composite_ok;
        current_event.goal_valid = goal_composite_ok;
        current_event.endpoints_valid = local_endpoints_valid;
        if (local_endpoints_valid &&
            !expansion_schedule_state.initial_valid_window_established) {
            establishMainWindowGeometry(
                start_t, end_t, expansion_schedule_state);
        }
        if (expansion_schedule_state.initial_valid_window_established) {
            current_event.main_window_center_twice =
                expansion_schedule_state.main_window_center_twice;
            current_event.main_base_half_width_twice =
                expansion_schedule_state.main_base_half_width_twice;
        }

        const bool temporal_full_window =
            (start_t == 0 && max_t > 0 &&
             static_cast<size_t>(end_t) >= max_t - 1);
        current_event.temporal_full_window = temporal_full_window;

        if (use_cspace_bounds_) {
            size_t li = 0;
            for (int r : involved_robots) {
                auto &robot = problem_->robot(r);
                const auto &path = working_paths[static_cast<std::size_t>(r)];
                if (path.empty()) {
                    ++li;
                    continue;
                }
                int ndof = robot.model->numJoints();
                if (temporal_full_window) {
                    ++li;
                    continue;
                }
                std::vector<double> lo(ndof, std::numeric_limits<double>::max());
                std::vector<double> hi(ndof, std::numeric_limits<double>::lowest());
                auto include_config = [&](const std::vector<double> &config) {
                    for (int d = 0; d < ndof; ++d) {
                        double v = config[static_cast<size_t>(d)];
                        lo[d] = std::min(lo[d], v);
                        hi[d] = std::max(hi[d], v);
                    }
                };
                std::vector<double> sampled_config;
                path.config_at_timestep(static_cast<std::size_t>(start_t),
                                        sampled_config);
                include_config(sampled_config);
                path.config_at_timestep(static_cast<std::size_t>(end_t),
                                        sampled_config);
                include_config(sampled_config);
                if (path.has_explicit_timesteps()) {
                    for (size_t waypoint = 0; waypoint < path.size();
                         ++waypoint) {
                        const size_t waypoint_t = path.timestep_at(waypoint);
                        if (waypoint_t < static_cast<size_t>(start_t) ||
                            waypoint_t > static_cast<size_t>(end_t)) {
                            continue;
                        }
                        include_config(path[waypoint]);
                    }
                } else {
                    const size_t seg_start =
                        std::min(static_cast<size_t>(start_t), path.size() - 1);
                    const size_t seg_end =
                        std::min(static_cast<size_t>(end_t), path.size() - 1);
                    for (size_t t = seg_start; t <= seg_end && t < path.size();
                         ++t) {
                        include_config(path[t]);
                    }
                }
                double margin = static_cast<double>(cspace_bound_margin_);
                const double min_span = min_cspace_bound_range_;
                for (int d = 0; d < ndof; ++d) {
                    double range_d = hi[d] - lo[d];
                    double expansion = margin * range_d;
                    double jlo = robot.model->jointLower(d);
                    double jhi = robot.model->jointUpper(d);
                    double pre_lo = lo[d] - expansion;
                    double pre_hi = hi[d] + expansion;
                    lo[d] = std::max(pre_lo, jlo);
                    hi[d] = std::min(pre_hi, jhi);
                    if (lo[d] > hi[d])
                        lo[d] = hi[d] = 0.5 * (lo[d] + hi[d]);

                    if (min_span > 0.0 && hi[d] - lo[d] < min_span) {
                        const double mid = 0.5 * (lo[d] + hi[d]);
                        const double half = 0.5 * min_span;
                        const double joint_span = jhi - jlo;
                        if (joint_span >= min_span) {
                            const double m =
                                std::clamp(mid, jlo + half, jhi - half);
                            lo[d] = m - half;
                            hi[d] = m + half;
                        } else {
                            lo[d] = jlo;
                            hi[d] = jhi;
                        }
                    }
                }
                sub_problem->setCspaceBoundsForRobot(static_cast<int>(li), lo, hi);
                ++li;
            }
        }
        std::optional<std::uint64_t> local_makespan_bound_timesteps;
        bool skip_bounded_local_attempt_for_epsilon = false;
        if (global_makespan_bound_timesteps_) {
            std::uint64_t min_slack =
                std::numeric_limits<std::uint64_t>::max();
            for (const int r : involved_robots) {
                const auto arrival =
                    static_cast<std::size_t>(r) < true_arrival_timesteps_.size()
                        ? true_arrival_timesteps_[static_cast<std::size_t>(r)]
                        : static_cast<std::uint64_t>(
                              working_paths[static_cast<std::size_t>(r)]
                                  .arrival_timestep());
                const std::uint64_t slack =
                    *global_makespan_bound_timesteps_ > arrival
                        ? *global_makespan_bound_timesteps_ - arrival
                        : 0;
                min_slack = std::min(min_slack, slack);
            }
            if (min_slack == std::numeric_limits<std::uint64_t>::max())
                min_slack = 0;
            std::uint64_t raw_local_bound =
                static_cast<std::uint64_t>(std::max(0, end_t - start_t)) +
                min_slack;
            if (!temporal_full_window &&
                bounded_local_repair_epsilon_timesteps_ > 0) {
                if (raw_local_bound <=
                    bounded_local_repair_epsilon_timesteps_) {
                    skip_bounded_local_attempt_for_epsilon = true;
                } else {
                    raw_local_bound -=
                        bounded_local_repair_epsilon_timesteps_;
                }
            }
            local_makespan_bound_timesteps = raw_local_bound;
        }
        current_event.bounded_epsilon_skipped =
            skip_bounded_local_attempt_for_epsilon;

        const auto remainingWall = [&]() {
            const double elapsed = std::chrono::duration<double>(
                                       std::chrono::steady_clock::now() - solve_start)
                                       .count();
            const double rem = global_time_limit - elapsed;
            return rem > 0.0 ? rem : 0.0;
        };

        if (local_endpoints_valid &&
            !skip_bounded_local_attempt_for_epsilon) {
            const bool bounded_arc_subproblem =
                global_makespan_bound_timesteps_.has_value();
            const bool prioritized_requested =
                local_solver_mode_ != LocalSolverMode::CompositeRrtOnly;
            if (bounded_arc_subproblem && prioritized_requested &&
                !warned_bounded_prioritized_disabled_) {
                std::cerr
                    << "Warning: bounded ARC ignores PrioritizedSTRRT local "
                       "repair requests and uses only the composite bounded "
                       "subproblem solver.\n";
                warned_bounded_prioritized_disabled_ = true;
            }
            const bool use_prioritized =
                prioritized_requested && !bounded_arc_subproblem;
            const bool use_composite =
                bounded_arc_subproblem ||
                local_solver_mode_ != LocalSolverMode::PrioritizedStrrtOnly;
            // Layer 1: Prioritized ST-RRT*
            if (use_prioritized) {
                const size_t res_st = problem_->resolution();
                const int window_span_t = std::max(0, end_t - start_t);
                const double window_span_sec =
                    static_cast<double>(window_span_t) /
                    static_cast<double>(res_st);
                const double strrt_time_ub =
                    std::max(window_span_sec * strrt_space_time_span_factor_,
                             1.0 / static_cast<double>(res_st));

                auto planner = std::make_shared<PrioritizedSTRRT>();
                planner->setProblem(sub_problem);
                const auto prioritized_seed =
                    arcRepairPrioritizedPlanningSeed(
                        current_event.attempt_root_planning_seed);
                current_event.prioritized_planning_seed = prioritized_seed;
                planner->setPlanningSeed(prioritized_seed);
                planner->setPersistAtGoal(
                    local_prioritized_strrt_persist_at_goal_);
                planner->setEqualizePaths(false); // Return different-length paths
                planner->setUseUnboundedTime(false);
                planner->setSpaceTimeUpperBound(strrt_time_ub);
                planner->setSimplifyAfterPlan(
                    simplify_conflict_solutions_);
                planner->setPathSimplificationOptions(
                    conflict_simplification_options_.value_or(
                        simplification_options_));
                planner->setStrrtRewiring(
                    local_prioritized_strrt_rewiring_);
                planner->setReturnFirstSolution(
                    local_prioritized_strrt_return_first_solution_);
                planner->setStrrtMaxIterations(
                    local_prioritized_strrt_max_iterations_);
                planner->setCancellationCallback(cancel_requested);
                double strrt_wall = remainingWall();
                if (strrt_wall > 0.0) {
                    current_event.prioritized_invoked = true;
                    current_event.solver_invoked = true;
                    auto status = planner->solve(strrt_wall);
                    if (status == ompl::base::PlannerStatus::EXACT_SOLUTION) {
                        auto local_paths = planner->getSolutionPaths();
                        if (local_paths_out)
                            *local_paths_out = local_paths;
                        if (apply_solution_to_paths) {
                            spliceSolutionIntoPaths(involved_robots, start_t,
                                                    end_t, local_paths,
                                                    working_paths);
                        }
                        if (window_start_t_out)
                            *window_start_t_out = start_t;
                        if (window_end_t_out)
                            *window_end_t_out = end_t;
                        current_event.resolved = true;
                        current_event.solved_by = "prioritized_strrt";
                        current_event.outcome = "solved";
                        return true;
                    }
                    if (isCancelled()) {
                        current_event.outcome = "cancelled";
                        return false;
                    }
                }
            }

            // Layer 2: Composite RRT-C / bounded Composite AO-RRT-C
            if (use_composite) {
                const double composite_wall = remainingWall();
                if (isCancelled()) {
                    current_event.outcome = "cancelled";
                    return false;
                }
                if (composite_wall <= 0.0) {
                    // No wall time left for this local attempt; treat as failure.
                } else {
                    current_event.composite_invoked = true;
                    current_event.solver_invoked = true;
                    ompl::base::PlannerStatus status{
                        ompl::base::PlannerStatus::TIMEOUT};
                    std::vector<Path> local_paths;
                    const auto &conflict_simplification_options =
                        conflict_simplification_options_.value_or(
                            simplification_options_);
                    if (global_makespan_bound_timesteps_) {
                        const auto composite_seed =
                            arcRepairCompositePlanningSeed(
                                current_event.attempt_root_planning_seed);
                        current_event.composite_planning_seed = composite_seed;
                        aorrtc::SolveOptions options;
                        options.use_makespan_metric = true;
                        options.planning_seed = composite_seed;
                        options.simplify_solution = simplify_conflict_solutions_;
                        options.simplification_options =
                            conflict_simplification_options;
                        options.cost_bound_timesteps =
                            local_makespan_bound_timesteps;
                        if (local_composite_rrt_max_samples_ > 0) {
                            options.max_internal_samples =
                                local_composite_rrt_max_samples_;
                            options.max_internal_vertices =
                                local_composite_rrt_max_samples_;
                        }
                        auto bounded = aorrtc::solveCompositeBounded(
                            *sub_problem, {}, composite_wall, options);
                        status = bounded.status;
                        const double simplify_seconds =
                            std::max(0.0, bounded.simplify_seconds);
                        local_composite_simplification_times_seconds_wall_clock_ +=
                            simplify_seconds;
                        local_paths = std::move(bounded.paths);
                    } else {
                        auto planner = std::make_shared<CompositeRRT>();
                        planner->setProblem(sub_problem);
                        const auto composite_seed =
                            arcRepairCompositePlanningSeed(
                                current_event.attempt_root_planning_seed);
                        current_event.composite_planning_seed = composite_seed;
                        planner->setPlanningSeed(composite_seed);
                        planner->setSimplifySolution(simplify_conflict_solutions_);
                        planner->setPathSimplificationOptions(
                            conflict_simplification_options);
                        if (local_composite_rrt_range_)
                            planner->setRange(*local_composite_rrt_range_);
                        planner->setUseMakespanMetric(
                            local_composite_rrt_use_makespan_metric_);
                        planner->setCancellationCallback(cancel_requested);
                        if (temporal_full_window)
                            planner->setMaxRrtConnectIterations(0);
                        else if (local_composite_rrt_max_samples_ > 0)
                            planner->setMaxRrtConnectIterations(
                                local_composite_rrt_max_samples_);
                        else
                            planner->setMaxRrtConnectIterations(0);
                        status = planner->solve(composite_wall);
                        const auto local_stats = planner->plannerStatsJson();
                        if (local_stats.is_object()) {
                            const double simplify_seconds =
                                std::max(0.0, local_stats.value(
                                                  "simplify_wall_seconds", 0.0));
                            local_composite_simplification_times_seconds_wall_clock_ +=
                                simplify_seconds;
                            if (local_stats.contains("state_sampler_seed")) {
                                current_event.composite_state_sampler_seed =
                                    local_stats["state_sampler_seed"]
                                        .get<std::uint_fast32_t>();
                            }
                            if (local_stats.contains("rrt_connect_seed")) {
                                current_event.composite_rrt_connect_seed =
                                    local_stats["rrt_connect_seed"]
                                        .get<std::uint_fast32_t>();
                            }
                            if (local_stats.contains("path_simplifier_seed")) {
                                current_event.composite_path_simplifier_seed =
                                    local_stats["path_simplifier_seed"]
                                        .get<std::uint_fast32_t>();
                            }
                        }
                        if (status == ompl::base::PlannerStatus::EXACT_SOLUTION)
                            local_paths = planner->getSolutionPaths();
                    }
                    if (status == ompl::base::PlannerStatus::EXACT_SOLUTION) {
                        if (local_paths_out)
                            *local_paths_out = local_paths;
                        if (apply_solution_to_paths) {
                            spliceSolutionIntoPaths(involved_robots, start_t,
                                                    end_t, local_paths,
                                                    working_paths);
                        }
                        if (window_start_t_out)
                            *window_start_t_out = start_t;
                        if (window_end_t_out)
                            *window_end_t_out = end_t;
                        current_event.resolved = true;
                        current_event.solved_by = "composite_rrt";
                        current_event.outcome = "solved";
                        return true;
                    }
                    if (isCancelled()) {
                        current_event.outcome = "cancelled";
                        return false;
                    }
                }
            }
        } // local_endpoints_valid

        const double elapsed_after_local = std::chrono::duration<double>(
                                               std::chrono::steady_clock::now() -
                                               solve_start)
                                               .count();
        const double time_remaining_after_local =
            global_time_limit - elapsed_after_local;
        if (isCancelled()) {
            current_event.outcome = "cancelled";
            return false;
        }
        if (time_remaining_after_local <= 0.0) {
            current_event.outcome = "budget_exhausted";
            return false;
        }

        // Expand the temporal subproblem according to the configured schedule.
        if (start_t == 0 && static_cast<size_t>(end_t) >= max_t) {
            current_event.outcome = "global_window_failed";
            recordWindow();
            return false;
        }
        const int prev_start = start_t;
        const size_t prev_end = static_cast<size_t>(end_t);
        const std::size_t initial_valid_index_before =
            expansion_schedule_state.initial_valid_expansion_index;
        const std::size_t main_index_before =
            expansion_schedule_state.main_expansion_index;
        std::tie(start_t, end_t) =
            nextExpansionWindowAfterAttempt(
                start_t, end_t, max_t, start_composite_ok,
                goal_composite_ok,
                expansion_schedule_state);
        if (expansion_schedule_state
                .last_expansion_used_initial_valid_schedule) {
            current_attempt_phase = RepairAttemptPhase::InitialValid;
            current_expansion_index = initial_valid_index_before;
        } else {
            current_attempt_phase = RepairAttemptPhase::Main;
            current_expansion_index = main_index_before;
        }
        if (!(start_t == prev_start &&
              static_cast<size_t>(end_t) == prev_end)) {
            ++num_temporal_expansions_;
            if (expansion_schedule_state
                    .last_expansion_used_initial_valid_schedule) {
                ++num_initial_valid_temporal_expansions_;
            } else {
                ++num_main_temporal_expansions_;
            }
        }
        const bool repeated_custom_window =
            start_t == prev_start &&
            static_cast<size_t>(end_t) == prev_end &&
            ((expansion_schedule_state
                      .last_expansion_used_initial_valid_schedule &&
              initialValidWindowExpansionPolicy() ==
                  ExpansionPolicy::CustomMultiplied &&
              initial_valid_index_before <
                  initialValidWindowExpansionMultipliers().size()) ||
             (!expansion_schedule_state
                       .last_expansion_used_initial_valid_schedule &&
              expansion_policy_ == ExpansionPolicy::CustomMultiplied &&
              main_index_before <
                  custom_expansion_multipliers_.size()));
        if (start_t == prev_start && static_cast<size_t>(end_t) == prev_end &&
            !repeated_custom_window) {
            current_event.outcome = "expansion_exhausted";
            recordWindow();
            return false;
        }
        current_event.outcome = "expanded";
    }
}

void ARC::spliceSolutionIntoPaths(const std::vector<int> &involved_robots,
                                  int start_t, int end_t,
                                  const std::vector<Path> &local_paths,
                                  std::vector<Path> &working_paths) {
    std::vector<const Path *> local_path_ptrs;
    local_path_ptrs.reserve(local_paths.size());
    for (const auto &path : local_paths)
        local_path_ptrs.push_back(&path);
    spliceSolutionIntoPaths(involved_robots, start_t, end_t, local_path_ptrs,
                            working_paths);
}

void ARC::spliceSolutionIntoPaths(const std::vector<int> &involved_robots,
                                  int start_t, int end_t,
                                  const std::vector<const Path *> &local_paths,
                                  std::vector<Path> &working_paths) {
    for (size_t i = 0; i < involved_robots.size(); ++i) {
        int r = involved_robots[i];
        auto &global = working_paths[static_cast<std::size_t>(r)];
        if (i >= local_paths.size() || local_paths[i] == nullptr)
            throw std::runtime_error("ARC splice missing local path");
        const auto &local = *local_paths[i];

        if (local.empty())
            continue;

        if (global.empty())
            continue;

        const size_t window_start =
            static_cast<size_t>(std::max(0, start_t));
        const size_t window_end =
            static_cast<size_t>(std::max(std::max(0, start_t), end_t));
        const size_t original_arrival = global.arrival_timestep();
        const bool has_suffix = window_end < original_arrival;

        std::vector<double> global_start_config;
        std::vector<double> global_end_config;
        global.config_at_timestep(window_start, global_start_config);
        global.config_at_timestep(window_end, global_end_config);

        constexpr double kAnchorTolerance = 1e-6;
        if (local.size() > 1) {
            std::ostringstream context;
            context << "ARC splice robot " << r << " window [" << start_t
                    << "," << end_t << "] local start anchor mismatch";
            comotion::detail::requireConfigNear(
                local.front(), global_start_config,
                kAnchorTolerance, context.str());
        }
        if (local.size() > 1) {
            std::ostringstream context;
            context << "ARC splice robot " << r << " window [" << start_t
                    << "," << end_t << "] local end anchor mismatch";
            comotion::detail::requireConfigNear(
                local.back(), global_end_config, kAnchorTolerance,
                context.str());
        }

        Path rebuilt;
        std::vector<size_t> rebuilt_timesteps;
        rebuilt.reserve(global.size() + local.size() + 2);
        rebuilt_timesteps.reserve(global.size() + local.size() + 2);

        auto appendWaypoint = [&](size_t timestep,
                                  const std::vector<double> &config) {
            if (!rebuilt_timesteps.empty()) {
                if (timestep < rebuilt_timesteps.back()) {
                    throw std::runtime_error(
                        "ARC splice produced non-monotone sparse timesteps");
                }
                if (timestep == rebuilt_timesteps.back()) {
                    rebuilt.back() = config;
                    return;
                }
            }
            rebuilt.push_back(config);
            rebuilt_timesteps.push_back(timestep);
        };

        for (size_t waypoint = 0; waypoint < global.size(); ++waypoint) {
            const size_t waypoint_t = global.timestep_at(waypoint);
            if (waypoint_t >= window_start)
                break;
            appendWaypoint(waypoint_t, global[waypoint]);
        }
        appendWaypoint(window_start, global_start_config);

        const size_t local_t0 = local.timestep_at(0);
        for (size_t waypoint = 0; waypoint < local.size(); ++waypoint) {
            const size_t local_t = local.timestep_at(waypoint);
            const auto local_offset = comotion::detail::signedTimestepDelta(
                local_t0, local_t, "ARC sparse splice local waypoint offset");
            const size_t mapped_t =
                comotion::detail::applySignedTimestepShift(
                    window_start, local_offset,
                    "ARC sparse splice local waypoint");
            appendWaypoint(mapped_t, local[waypoint]);
        }

        const size_t local_end_t = rebuilt_timesteps.empty()
                                       ? window_start
                                       : rebuilt_timesteps.back();
        const auto suffix_shift = comotion::detail::signedTimestepDelta(
            window_end, local_end_t, "ARC sparse splice suffix shift");
        if (has_suffix) {
            appendWaypoint(local_end_t, global_end_config);
            for (size_t waypoint = 0; waypoint < global.size(); ++waypoint) {
                const size_t waypoint_t = global.timestep_at(waypoint);
                if (waypoint_t <= window_end)
                    continue;
                const size_t shifted_t =
                    comotion::detail::applySignedTimestepShift(
                        waypoint_t, suffix_shift,
                        "ARC sparse splice suffix waypoint");
                appendWaypoint(shifted_t, global[waypoint]);
            }
        }

        global = std::move(rebuilt);
        global.set_waypoint_timesteps(rebuilt_timesteps);
        if (static_cast<std::size_t>(r) < true_arrival_timesteps_.size()) {
            true_arrival_timesteps_[static_cast<std::size_t>(r)] =
                static_cast<std::uint64_t>(global.arrival_timestep());
        }

    }
    const auto [sum_of_cost_timesteps, makespan_timesteps] =
        arcSolutionMetricsFromArrivalTimesteps(true_arrival_timesteps_);
    setSolutionMetrics(sum_of_cost_timesteps, makespan_timesteps);
}

ARC::RepairOutcome ARC::resolveConflictOnPaths(
    const SubproblemConflict &conflict, const Clock::time_point &solve_start,
    double global_time_limit, std::vector<Path> &working_paths,
    bool apply_solution_to_paths, CancellationCallback cancel_requested) {
    RepairOutcome outcome;
    outcome.final_involved_robots = conflict.robots;

    int sub_window_start_t = 0;
    int sub_window_end_t = 0;
    std::vector<Path> local_patch_paths;
    auto *local_patch_paths_out =
        (!apply_solution_to_paths || visualization_trace_enabled_)
            ? &local_patch_paths
            : nullptr;
    outcome.resolved =
        solveSubproblemOnPaths(conflict, solve_start, global_time_limit,
                               working_paths,
                               &sub_window_start_t, &sub_window_end_t,
                               local_patch_paths_out,
                               apply_solution_to_paths, cancel_requested);
    if (!outcome.resolved) {
        outcome.window_start_t = sub_window_start_t;
        outcome.window_end_t = sub_window_end_t;
        return outcome;
    }

    outcome.window_start_t = sub_window_start_t;
    outcome.window_end_t = sub_window_end_t;
    if (local_patch_paths_out)
        outcome.local_patch_paths = std::move(local_patch_paths);
    return outcome;
}

ARC::ArcPlannerStatsSummary ARC::currentArcPlannerStatsSummary() const {
    ArcPlannerStatsSummary summary;
    summary.local_solver_mode = local_solver_mode_;
    summary.local_prioritized_strrt_max_iterations =
        local_prioritized_strrt_max_iterations_;
    summary.local_prioritized_strrt_return_first_solution =
        local_prioritized_strrt_return_first_solution_;
    summary.local_prioritized_strrt_rewiring =
        local_prioritized_strrt_rewiring_;
    summary.local_prioritized_strrt_persist_at_goal =
        local_prioritized_strrt_persist_at_goal_;
    summary.local_composite_rrt_use_makespan_metric =
        local_composite_rrt_use_makespan_metric_;
    summary.bounded_local_repair_epsilon_timesteps =
        bounded_local_repair_epsilon_timesteps_;
    summary.num_conflicts = num_conflicts_;
    summary.subproblem_attempts = num_subproblem_attempts_;
    summary.temporal_expansions = num_temporal_expansions_;
    summary.initial_valid_temporal_expansions =
        num_initial_valid_temporal_expansions_;
    summary.main_temporal_expansions = num_main_temporal_expansions_;
    summary.initial_solution_times_seconds_wall_clock =
        initial_solution_times_seconds_wall_clock_;
    summary.initial_solution_times_seconds_cpu =
        initial_solution_times_seconds_cpu_;
    summary.initial_simplification_times_seconds_wall_clock =
        initial_simplification_times_seconds_wall_clock_;
    summary.local_composite_simplification_times_seconds_wall_clock =
        local_composite_simplification_times_seconds_wall_clock_;
    summary.conflict_detection_times_seconds_wall_clock =
        sumSeconds(conflict_detection_times_seconds_);
    summary.conflict_detection_times_seconds_cpu =
        sumSeconds(conflict_detection_times_cpu_seconds_);
    summary.conflict_resolution_times_seconds_wall_clock =
        sumSeconds(conflict_resolution_times_seconds_);
    summary.conflict_resolution_times_seconds_total =
        summary.conflict_resolution_times_seconds_wall_clock;
    summary.conflict_resolution_times_seconds_cpu =
        sumSeconds(conflict_resolution_times_cpu_seconds_);
    summary.subproblem_batches = num_subproblem_attempts_;
    return summary;
}

nlohmann::json ARC::plannerStatsJsonFromSummary(
    const ArcPlannerStatsSummary &summary,
    const std::vector<double> *conflict_resolution_times_seconds,
    const std::vector<double> *conflict_detection_times_seconds) {
    nlohmann::json stats = nlohmann::json::object();
    stats["local_solver_mode"] = arcLocalSolverModeStr(summary.local_solver_mode);
    stats["local_prioritized_strrt_max_iterations"] =
        summary.local_prioritized_strrt_max_iterations;
    stats["local_prioritized_strrt_return_first_solution"] =
        summary.local_prioritized_strrt_return_first_solution;
    stats["local_prioritized_strrt_rewiring"] =
        strrtRewiringStr(summary.local_prioritized_strrt_rewiring);
    stats["local_prioritized_strrt_persist_at_goal"] =
        summary.local_prioritized_strrt_persist_at_goal;
    stats["local_composite_rrt_use_makespan_metric"] =
        summary.local_composite_rrt_use_makespan_metric;
    stats["bounded_local_repair_epsilon_timesteps"] =
        summary.bounded_local_repair_epsilon_timesteps;
    stats["num_conflicts"] = summary.num_conflicts;
    stats["subproblem_attempts"] = summary.subproblem_attempts;
    stats["temporal_expansions"] = summary.temporal_expansions;
    stats["initial_valid_temporal_expansions"] =
        summary.initial_valid_temporal_expansions;
    stats["main_temporal_expansions"] =
        summary.main_temporal_expansions;
    stats["initial_solution_times_seconds_wall_clock"] =
        summary.initial_solution_times_seconds_wall_clock;
    stats["initial_solution_times_seconds_cpu"] =
        summary.initial_solution_times_seconds_cpu;
    stats["initial_simplification_times_seconds_wall_clock"] =
        summary.initial_simplification_times_seconds_wall_clock;
    stats["local_composite_simplification_times_seconds_wall_clock"] =
        summary.local_composite_simplification_times_seconds_wall_clock;
    stats["total_simplification_times_seconds_wall_clock"] =
        summary.initial_simplification_times_seconds_wall_clock +
        summary.local_composite_simplification_times_seconds_wall_clock;
    stats["conflict_detection_times_seconds_wall_clock"] =
        summary.conflict_detection_times_seconds_wall_clock;
    stats["conflict_detection_times_seconds_cpu"] =
        summary.conflict_detection_times_seconds_cpu;
    stats["conflict_find_times_seconds_wall_clock"] =
        summary.conflict_detection_times_seconds_wall_clock;
    stats["conflict_find_times_seconds_cpu"] =
        summary.conflict_detection_times_seconds_cpu;
    stats["conflict_resolution_times_seconds_wall_clock"] =
        summary.conflict_resolution_times_seconds_wall_clock;
    stats["conflict_resolution_times_seconds_total"] =
        summary.conflict_resolution_times_seconds_total;
    stats["conflict_resolution_times_seconds_cpu"] =
        summary.conflict_resolution_times_seconds_cpu;
    stats["subproblem_resolution_times_seconds_wall_clock"] =
        summary.conflict_resolution_times_seconds_wall_clock;
    stats["subproblem_resolution_times_seconds_total_worker_wall"] =
        summary.conflict_resolution_times_seconds_total;
    stats["subproblem_resolution_times_seconds_cpu"] =
        summary.conflict_resolution_times_seconds_cpu;
    stats["subproblem_batches"] = summary.subproblem_batches;

    // Preserve legacy ARC field names for existing tooling.
    stats["num_subproblem_attempts"] = summary.subproblem_attempts;
    stats["num_temporal_expansions"] = summary.temporal_expansions;
    stats["num_initial_valid_temporal_expansions"] =
        summary.initial_valid_temporal_expansions;
    stats["num_main_temporal_expansions"] =
        summary.main_temporal_expansions;
    if (conflict_resolution_times_seconds) {
        stats["conflict_resolution_times_seconds"] =
            *conflict_resolution_times_seconds;
    }
    if (conflict_detection_times_seconds) {
        stats["conflict_detection_times_seconds"] =
            *conflict_detection_times_seconds;
        stats["conflict_find_times_seconds"] =
            *conflict_detection_times_seconds;
    }
    return stats;
}

ARC::ProcessTreeCpuUsageSnapshot ARC::processTreeCpuUsageSnapshot() {
    ProcessTreeCpuUsageSnapshot snapshot;
#if !defined(_WIN32)
    rusage self_usage {};
    if (::getrusage(RUSAGE_SELF, &self_usage) == 0) {
        snapshot.self_seconds =
            timevalSeconds(self_usage.ru_utime) +
            timevalSeconds(self_usage.ru_stime);
    }
    rusage children_usage {};
    if (::getrusage(RUSAGE_CHILDREN, &children_usage) == 0) {
        snapshot.children_seconds =
            timevalSeconds(children_usage.ru_utime) +
            timevalSeconds(children_usage.ru_stime);
    }
#endif
    return snapshot;
}

double ARC::elapsedProcessTreeCpuSeconds(
    const ProcessTreeCpuUsageSnapshot &start,
    const ProcessTreeCpuUsageSnapshot &finish) {
    const double elapsed =
        (finish.self_seconds - start.self_seconds) +
        (finish.children_seconds - start.children_seconds);
    return elapsed < 0.0 ? 0.0 : elapsed;
}

nlohmann::json ARC::conflictFindTimingJson(
    const std::vector<double> &main_process_wall_seconds,
    const std::vector<double> &process_tree_cpu_seconds,
    const std::vector<double> &build_worker_wall_seconds,
    const std::vector<double> &build_worker_cpu_seconds,
    const std::vector<double> &collision_worker_wall_seconds,
    const std::vector<double> &collision_worker_cpu_seconds,
    const std::vector<int> &critical_worker_index,
    const std::vector<double> &critical_worker_build_wall_seconds,
    const std::vector<double> &critical_worker_collision_wall_seconds,
    const std::vector<double> &critical_worker_total_wall_seconds) {
    return {
        {"round_count", main_process_wall_seconds.size()},
        {"main_process_wall_seconds_total", sumSeconds(main_process_wall_seconds)},
        {"main_process_wall_seconds_by_round", main_process_wall_seconds},
        {"process_tree_cpu_seconds_total",
         sumSeconds(process_tree_cpu_seconds)},
        {"process_tree_cpu_seconds_by_round", process_tree_cpu_seconds},
        {"build_worker_wall_seconds_total",
         sumSeconds(build_worker_wall_seconds)},
        {"build_worker_wall_seconds_by_round", build_worker_wall_seconds},
        {"build_worker_cpu_seconds_total",
         sumSeconds(build_worker_cpu_seconds)},
        {"build_worker_cpu_seconds_by_round", build_worker_cpu_seconds},
        {"collision_worker_wall_seconds_total",
         sumSeconds(collision_worker_wall_seconds)},
        {"collision_worker_wall_seconds_by_round",
         collision_worker_wall_seconds},
        {"collision_worker_cpu_seconds_total",
         sumSeconds(collision_worker_cpu_seconds)},
        {"collision_worker_cpu_seconds_by_round",
         collision_worker_cpu_seconds},
        {"critical_worker_index_by_round", critical_worker_index},
        {"critical_worker_build_wall_seconds_total",
         sumSeconds(critical_worker_build_wall_seconds)},
        {"critical_worker_build_wall_seconds_by_round",
         critical_worker_build_wall_seconds},
        {"critical_worker_collision_wall_seconds_total",
         sumSeconds(critical_worker_collision_wall_seconds)},
        {"critical_worker_collision_wall_seconds_by_round",
         critical_worker_collision_wall_seconds},
        {"critical_worker_total_wall_seconds_total",
         sumSeconds(critical_worker_total_wall_seconds)},
        {"critical_worker_total_wall_seconds_by_round",
         critical_worker_total_wall_seconds},
    };
}

nlohmann::json ARC::repairAttemptEventsJson() const {
    const auto phaseName = [](RepairAttemptPhase phase) {
        switch (phase) {
        case RepairAttemptPhase::InitialWindow:
            return "initial_window";
        case RepairAttemptPhase::InitialValid:
            return "initial_valid";
        case RepairAttemptPhase::Main:
            return "main";
        }
        return "unknown";
    };

    nlohmann::json events = nlohmann::json::array();
    for (const auto &event : repair_attempt_events_) {
        events.push_back({
            {"repair_id", event.repair_id},
            {"attempt_index", event.attempt_index},
            {"seed_robot_i", event.seed_robot_i},
            {"seed_robot_j", event.seed_robot_j},
            {"conflict_timestep", event.conflict_timestep},
            {"robots", event.robots},
            {"phase", phaseName(event.phase)},
            {"expansion_index",
             event.expansion_index
                 ? nlohmann::json(*event.expansion_index)
                 : nlohmann::json(nullptr)},
            {"window_start_t", event.window_start_t},
            {"window_end_t", event.window_end_t},
            {"max_t", event.max_t},
            {"effective_global", event.effective_global},
            {"temporal_full_window", event.temporal_full_window},
            {"validity_checked", event.validity_checked},
            {"start_valid", event.start_valid},
            {"goal_valid", event.goal_valid},
            {"endpoints_valid", event.endpoints_valid},
            {"bounded_epsilon_skipped", event.bounded_epsilon_skipped},
            {"prioritized_invoked", event.prioritized_invoked},
            {"composite_invoked", event.composite_invoked},
            {"solver_invoked", event.solver_invoked},
            {"resolved", event.resolved},
            {"attempt_root_planning_seed",
             event.attempt_root_planning_seed},
            {"prioritized_planning_seed",
             event.prioritized_planning_seed
                 ? nlohmann::json(*event.prioritized_planning_seed)
                 : nlohmann::json(nullptr)},
            {"composite_planning_seed",
             event.composite_planning_seed
                 ? nlohmann::json(*event.composite_planning_seed)
                 : nlohmann::json(nullptr)},
            {"composite_state_sampler_seed",
             event.composite_state_sampler_seed
                 ? nlohmann::json(*event.composite_state_sampler_seed)
                 : nlohmann::json(nullptr)},
            {"composite_rrt_connect_seed",
             event.composite_rrt_connect_seed
                 ? nlohmann::json(*event.composite_rrt_connect_seed)
                 : nlohmann::json(nullptr)},
            {"composite_path_simplifier_seed",
             event.composite_path_simplifier_seed
                 ? nlohmann::json(*event.composite_path_simplifier_seed)
                 : nlohmann::json(nullptr)},
            {"main_window_center_twice",
             event.main_window_center_twice
                 ? nlohmann::json(*event.main_window_center_twice)
                 : nlohmann::json(nullptr)},
            {"main_base_half_width_twice",
             event.main_base_half_width_twice
                 ? nlohmann::json(*event.main_base_half_width_twice)
                 : nlohmann::json(nullptr)},
            {"solved_by",
             event.solved_by.empty() ? nlohmann::json(nullptr)
                                     : nlohmann::json(event.solved_by)},
            {"outcome", event.outcome},
        });
    }
    return events;
}

nlohmann::json ARC::conflictResolutionEventsJson() const {
    struct ResolutionEvent {
        std::uint64_t repair_id = 0;
        std::uint64_t attempt_count = 0;
        std::uint64_t solver_calls = 0;
        std::uint64_t composite_solver_calls = 0;
        std::uint64_t prioritized_solver_calls = 0;
        bool used_initial_valid_expansion = false;
        bool used_main_expansion = false;
        bool reached_global_window = false;
        bool resolved = false;
        bool resolved_by_composite = false;
    };

    std::vector<ResolutionEvent> resolutions(
        conflict_resolution_times_seconds_.size());
    for (std::size_t repair_id = 0; repair_id < resolutions.size();
         ++repair_id) {
        resolutions[repair_id].repair_id = repair_id;
    }

    for (const auto &attempt : repair_attempt_events_) {
        if (attempt.repair_id >= resolutions.size())
            continue;
        auto &resolution =
            resolutions[static_cast<std::size_t>(attempt.repair_id)];
        ++resolution.attempt_count;
        if (attempt.solver_invoked)
            ++resolution.solver_calls;
        if (attempt.composite_invoked)
            ++resolution.composite_solver_calls;
        if (attempt.prioritized_invoked)
            ++resolution.prioritized_solver_calls;
        resolution.used_initial_valid_expansion =
            resolution.used_initial_valid_expansion ||
            attempt.phase == RepairAttemptPhase::InitialValid;
        resolution.used_main_expansion =
            resolution.used_main_expansion ||
            attempt.phase == RepairAttemptPhase::Main;
        resolution.reached_global_window =
            resolution.reached_global_window || attempt.effective_global;
        resolution.resolved = resolution.resolved || attempt.resolved;
        resolution.resolved_by_composite =
            resolution.resolved_by_composite ||
            (attempt.resolved && attempt.composite_invoked);
    }

    nlohmann::json events = nlohmann::json::array();
    for (std::size_t repair_id = 0; repair_id < resolutions.size();
         ++repair_id) {
        const auto &resolution = resolutions[repair_id];
        const double cpu_seconds =
            repair_id < conflict_resolution_times_cpu_seconds_.size()
                ? conflict_resolution_times_cpu_seconds_[repair_id]
                : 0.0;
        events.push_back({
            {"repair_id", resolution.repair_id},
            {"wall_seconds",
             conflict_resolution_times_seconds_[repair_id]},
            {"cpu_seconds", cpu_seconds},
            {"attempt_count", resolution.attempt_count},
            {"solver_calls", resolution.solver_calls},
            {"composite_solver_calls",
             resolution.composite_solver_calls},
            {"prioritized_solver_calls",
             resolution.prioritized_solver_calls},
            {"used_initial_valid_expansion",
             resolution.used_initial_valid_expansion},
            {"used_main_expansion", resolution.used_main_expansion},
            {"reached_global_window",
             resolution.reached_global_window},
            {"resolved", resolution.resolved},
            {"resolved_by_composite",
             resolution.resolved_by_composite},
            {"solved_on_first_composite_call",
             resolution.resolved_by_composite &&
                 resolution.composite_solver_calls == 1},
            {"solved_without_main_expansion",
             resolution.resolved &&
                 !resolution.used_main_expansion},
        });
    }
    return events;
}

nlohmann::json ARC::conflictSolveCountsByExpansionStageJson() const {
    struct StageCounts {
        std::uint64_t attempts = 0;
        std::uint64_t validity_checked_attempts = 0;
        std::uint64_t endpoint_valid_attempts = 0;
        std::uint64_t effective_global_attempts = 0;
        std::uint64_t temporal_full_window_attempts = 0;
        std::uint64_t solver_attempts = 0;
        std::uint64_t solver_invocations = 0;
        std::uint64_t resolved_conflicts = 0;
    };

    const auto addEvent = [](StageCounts &counts,
                             const RepairAttemptEvent &event) {
        ++counts.attempts;
        if (event.validity_checked)
            ++counts.validity_checked_attempts;
        if (event.validity_checked && event.endpoints_valid)
            ++counts.endpoint_valid_attempts;
        if (event.effective_global)
            ++counts.effective_global_attempts;
        if (event.temporal_full_window)
            ++counts.temporal_full_window_attempts;
        if (event.solver_invoked)
            ++counts.solver_attempts;
        if (event.prioritized_invoked)
            ++counts.solver_invocations;
        if (event.composite_invoked)
            ++counts.solver_invocations;
        if (event.resolved)
            ++counts.resolved_conflicts;
    };
    const auto countsJson = [](const StageCounts &counts) {
        return nlohmann::json{
            {"attempts", counts.attempts},
            {"validity_checked_attempts",
             counts.validity_checked_attempts},
            {"endpoint_valid_attempts", counts.endpoint_valid_attempts},
            {"effective_global_attempts",
             counts.effective_global_attempts},
            {"temporal_full_window_attempts",
             counts.temporal_full_window_attempts},
            {"solver_attempts", counts.solver_attempts},
            {"solver_invocations", counts.solver_invocations},
            {"resolved_conflicts", counts.resolved_conflicts},
        };
    };

    StageCounts initial_window_counts;
    StageCounts initial_valid_counts;
    StageCounts global_counts;
    StageCounts unindexed_main_counts;
    StageCounts main_total_counts;
    StageCounts total_counts;
    std::map<std::size_t, StageCounts> index_counts;
    if (expansion_policy_ == ExpansionPolicy::CustomMultiplied) {
        for (std::size_t index = 0;
             index < custom_expansion_multipliers_.size(); ++index) {
            index_counts[index];
        }
    }

    for (const auto &event : repair_attempt_events_) {
        addEvent(total_counts, event);
        switch (event.phase) {
        case RepairAttemptPhase::InitialWindow:
            addEvent(initial_window_counts, event);
            break;
        case RepairAttemptPhase::InitialValid:
            addEvent(initial_valid_counts, event);
            break;
        case RepairAttemptPhase::Main:
            addEvent(main_total_counts, event);
            if (event.effective_global) {
                addEvent(global_counts, event);
            } else if (event.expansion_index) {
                addEvent(index_counts[*event.expansion_index], event);
            } else {
                addEvent(unindexed_main_counts, event);
            }
            break;
        }
    }

    nlohmann::json indices = nlohmann::json::array();
    for (const auto &[index, counts] : index_counts) {
        auto entry = countsJson(counts);
        entry["index"] = index;
        entry["multiplier"] =
            expansion_policy_ == ExpansionPolicy::CustomMultiplied &&
                    index < custom_expansion_multipliers_.size()
                ? nlohmann::json(custom_expansion_multipliers_[index])
                : nlohmann::json(nullptr);
        indices.push_back(std::move(entry));
    }

    return {
        {"initial_window", countsJson(initial_window_counts)},
        {"initial_valid", countsJson(initial_valid_counts)},
        {"indices", std::move(indices)},
        {"global", countsJson(global_counts)},
        {"unindexed_main", countsJson(unindexed_main_counts)},
        {"main_total", countsJson(main_total_counts)},
        {"total", countsJson(total_counts)},
        {"global_bucket_rule", "phase_main_and_effective_global"},
    };
}

std::vector<int> ARC::subproblemRobotsForConflict(int robot_i, int robot_j,
                                                  int window_start_t,
                                                  int window_end_t,
                                                  std::vector<SubproblemConflict::
                                                                  ExpansionTraceStep> *
                                                      trace_out) const {
    if (window_end_t < window_start_t) {
        throw std::runtime_error(
            "ARC conflict expansion window end precedes window start");
    }

    std::set<int> team;
    std::vector<int> worklist;

    const auto enqueueRobot = [&](int robot, std::vector<int> &pending,
                                  std::set<int> &current_team) {
        if (robot < 0)
            return;
        if (current_team.insert(robot).second)
            pending.push_back(robot);
    };

    enqueueRobot(robot_i, worklist, team);
    enqueueRobot(robot_j, worklist, team);

    for (std::size_t cursor = 0; cursor < worklist.size(); ++cursor) {
        const int robot = worklist[cursor];
        if (robot < 0)
            continue;

        const auto robot_it = repair_window_schedule_.find(robot);
        if (robot_it == repair_window_schedule_.end())
            continue;

        for (const auto &[teammate, windows] : robot_it->second) {
            for (const auto &window : windows) {
                if (!arcRepairWindowsIntersect(
                        window_start_t, window_end_t, window.window_start_t,
                        window.window_end_t)) {
                    continue;
                }
                const bool inserted = team.insert(teammate).second;
                if (inserted) {
                    worklist.push_back(teammate);
                    if (trace_out) {
                        SubproblemConflict::ExpansionTraceStep step;
                        step.from_robot = robot;
                        step.added_robot = teammate;
                        step.window_robot_a = robot;
                        step.window_robot_b = teammate;
                        step.window_start_t = window.window_start_t;
                        step.window_end_t = window.window_end_t;
                        step.history_event_ids = window.history_event_ids;
                        trace_out->push_back(std::move(step));
                    }
                }
                break;
            }
        }
    }

    return std::vector<int>(team.begin(), team.end());
}

const std::vector<ARC::RepairWindow> *
ARC::repairWindowsForRobots(int robot_i, int robot_j) const {
    const auto robot_it = repair_window_schedule_.find(robot_i);
    if (robot_it == repair_window_schedule_.end())
        return nullptr;
    const auto teammate_it = robot_it->second.find(robot_j);
    if (teammate_it == robot_it->second.end())
        return nullptr;
    return &teammate_it->second;
}

SubproblemConflict ARC::expandConflictForSubproblem(
    const Conflict &conflict) const {
    SubproblemConflict expanded;
    expanded.conflict_timestep = conflict.timestep;
    expanded.window_begin_t = std::max(0, conflict.timestep - initial_window_);
    expanded.window_end_t = conflict.timestep + initial_window_;
    expanded.robots = subproblemRobotsForConflict(
        conflict.robot_i, conflict.robot_j, expanded.window_begin_t,
        expanded.window_end_t, &expanded.expansion_trace);
    expanded.seed_robot_i = conflict.robot_i;
    expanded.seed_robot_j = conflict.robot_j;
    expanded.alpha = conflict.alpha;
    expanded.kind = conflict.kind;
    expanded.config_i = conflict.config_i;
    expanded.config_j = conflict.config_j;
    return expanded;
}

ompl::base::PlannerStatus ARC::solve(double timeLimit) {
    resetArcSolveState();
    if (global_makespan_bound_timesteps_ &&
        local_solver_mode_ != LocalSolverMode::CompositeRrtOnly) {
        std::cerr
            << "Warning: bounded ARC ignores PrioritizedSTRRT local "
               "repair requests and uses only the composite bounded "
               "subproblem solver.\n";
        warned_bounded_prioritized_disabled_ = true;
    }
    auto start_time = Clock::now();
    const auto finalizePlannerStats = [&](bool exact_solution = false) {
        auto stats = plannerStatsJsonFromSummary(
            currentArcPlannerStatsSummary(),
            &conflict_resolution_times_seconds_,
            &conflict_detection_times_seconds_);
        // Keep the detailed timing in a nested block so consumers can opt in.
        stats["conflict_find_timing"] =
            conflictFindTimingJson(
                conflict_find_main_process_wall_seconds_,
                conflict_find_process_tree_cpu_seconds_,
                conflict_find_build_worker_wall_seconds_,
                conflict_find_build_worker_cpu_seconds_,
                conflict_find_collision_worker_wall_seconds_,
                conflict_find_collision_worker_cpu_seconds_,
                conflict_find_critical_worker_index_,
                conflict_find_critical_worker_build_wall_seconds_,
                conflict_find_critical_worker_collision_wall_seconds_,
                conflict_find_critical_worker_total_wall_seconds_);
        stats["solution_events"] = nlohmann::json::array();
        if (exact_solution && makespanTimesteps()) {
            stats["solution_events"].push_back({
                {"elapsed_seconds",
                 std::chrono::duration<double>(Clock::now() - start_time)
                     .count()},
                {"makespan_timesteps", *makespanTimesteps()},
                {"sum_of_cost_timesteps",
                 sumOfCostTimesteps() ? nlohmann::json(*sumOfCostTimesteps())
                                      : nlohmann::json(nullptr)},
                {"kind", "first_solution"},
            });
        }
        const auto conflict_simplification_options =
            conflict_simplification_options_.value_or(
                simplification_options_);
        stats["path_simplification"] = {
            {"initial_enabled", simplify_initial_solutions_},
            {"conflict_enabled", simplify_conflict_solutions_},
            {"max_shortcut_steps", simplification_options_.max_shortcut_steps},
            {"max_empty_steps", simplification_options_.max_empty_steps},
            {"max_smooth_steps", simplification_options_.max_smooth_steps},
            {"max_passes", simplification_options_.max_passes},
            {"initial",
             {
                 {"max_shortcut_steps",
                  simplification_options_.max_shortcut_steps},
                 {"max_empty_steps",
                  simplification_options_.max_empty_steps},
                 {"max_smooth_steps",
                  simplification_options_.max_smooth_steps},
                 {"max_passes", simplification_options_.max_passes},
             }},
            {"conflict",
             {
                 {"inherits_initial_options",
                  !conflict_simplification_options_.has_value()},
                 {"max_shortcut_steps",
                  conflict_simplification_options.max_shortcut_steps},
                 {"max_empty_steps",
                  conflict_simplification_options.max_empty_steps},
                 {"max_smooth_steps",
                  conflict_simplification_options.max_smooth_steps},
                 {"max_passes",
                  conflict_simplification_options.max_passes},
             }},
        };
        stats["expansion_policy"] =
            arcExpansionPolicyStr(expansion_policy_);
        stats["initial_window"] = initial_window_;
        stats["expansion_step"] = expansion_step_;
        stats["custom_expansion_multipliers"] =
            custom_expansion_multipliers_;
        stats["initial_valid_expansion_policy"] =
            arcExpansionPolicyStr(initialValidWindowExpansionPolicy());
        stats["initial_valid_expansion_step"] =
            initialValidWindowExpansionStep();
        stats["initial_valid_custom_expansion_multipliers"] =
            initialValidWindowExpansionMultipliers();
        stats["initial_valid_expansion_symmetric"] =
            initialValidWindowExpansionSymmetric();
        stats["initial_valid_expansion_inherits_main"] = {
            {"policy", initialValidWindowExpansionPolicyInheritsMain()},
            {"step", initialValidWindowExpansionStepInheritsMain()},
            {"multipliers",
             initialValidWindowExpansionMultipliersInheritMain()},
        };
        stats["repair_attempt_events"] = repairAttemptEventsJson();
        stats["conflict_resolution_events"] =
            conflictResolutionEventsJson();
        stats["conflict_solve_counts_by_expansion_stage"] =
            conflictSolveCountsByExpansionStageJson();
        stats["num_solution_events"] = stats["solution_events"].size();
        setPlannerStatsJson(std::move(stats));
    };

    // Step 1: Plan individual paths (each robot gets remaining wall time)
    if (!planIndividualPaths(start_time, timeLimit, solution_paths_)) {
        finalizePlannerStats();
        return ompl::base::PlannerStatus::TIMEOUT;
    }
    initializeConflictScanStarts(solution_paths_.size());

    auto ptrs = problem_->robotModelPtrs();
    // Step 2: Iteratively find and resolve conflicts
    while (true) {
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        double elapsed_s = std::chrono::duration<double>(elapsed).count();
        if (elapsed_s >= timeLimit) break;

        startVisualizationIteration(solution_paths_);

        // Scan the current timestep-synchronized global paths directly.
        CompositePathValidationOptions options = conflictScanOptions();
        options.stop_requested = [&]() {
            return std::chrono::duration<double>(
                       Clock::now() - start_time)
                       .count() >= timeLimit;
        };
        ConflictChecker conflict_checker(problem_->collisionChecker());
        std::vector<std::size_t> next_t_begin_by_pair;
        const auto conflict_detection_start = Clock::now();
        const double conflict_detection_cpu_start = processCpuSeconds();
        const auto conflict_detection_tree_cpu_start =
            processTreeCpuUsageSnapshot();
        ConflictFindTimingInstrumentation conflict_find_timing;
        if (validationInstrumentationEnabled()) {
            options.conflict_find_timing_instrumentation =
                &conflict_find_timing;
        }
        const auto conflicts = conflict_checker.findConflicts(
            solution_paths_, ptrs, options, 0, 1, true,
            [this](const Conflict &conflict) {
                return expandConflictForSubproblem(conflict);
            },
            nullptr, &next_t_begin_by_pair);
        setVisualizationConflicts(conflicts);
        applyConflictScanProgress(next_t_begin_by_pair);
        const double conflict_detection_wall_seconds =
            std::chrono::duration<double>(Clock::now() -
                                          conflict_detection_start)
                .count();
        conflict_detection_times_seconds_.push_back(
            conflict_detection_wall_seconds);
        conflict_detection_times_cpu_seconds_.push_back(
            elapsedProcessCpuSeconds(conflict_detection_cpu_start));
        conflict_find_main_process_wall_seconds_.push_back(
            conflict_detection_wall_seconds);
        conflict_find_process_tree_cpu_seconds_.push_back(
            elapsedProcessTreeCpuSeconds(conflict_detection_tree_cpu_start,
                                         processTreeCpuUsageSnapshot()));
        conflict_find_build_worker_wall_seconds_.push_back(
            conflict_find_timing.build_worker_wall_seconds);
        conflict_find_build_worker_cpu_seconds_.push_back(
            conflict_find_timing.build_worker_cpu_seconds);
        conflict_find_collision_worker_wall_seconds_.push_back(
            conflict_find_timing.collision_worker_wall_seconds);
        conflict_find_collision_worker_cpu_seconds_.push_back(
            conflict_find_timing.collision_worker_cpu_seconds);
        conflict_find_critical_worker_index_.push_back(
            conflict_find_timing.criticalWorkerIndex());
        conflict_find_critical_worker_build_wall_seconds_.push_back(
            conflict_find_timing.criticalWorkerBuildWallSeconds());
        conflict_find_critical_worker_collision_wall_seconds_.push_back(
            conflict_find_timing.criticalWorkerCollisionWallSeconds());
        conflict_find_critical_worker_total_wall_seconds_.push_back(
            conflict_find_timing.criticalWorkerTotalWallSeconds());
        elapsed = std::chrono::steady_clock::now() - start_time;
        elapsed_s = std::chrono::duration<double>(elapsed).count();
        if (elapsed_s >= timeLimit) {
            finalizePlannerStats();
            return ompl::base::PlannerStatus::TIMEOUT;
        }
        if (conflicts.empty()) {
            finalizePlannerStats(true);
            return ompl::base::PlannerStatus::EXACT_SOLUTION;
        }
        ++num_conflicts_;

        const auto subproblem_conflict = conflicts.front();

        const auto conflict_resolution_start = Clock::now();
        const double conflict_resolution_cpu_start = processCpuSeconds();
        const auto outcome = resolveConflictOnPaths(
            subproblem_conflict, start_time, timeLimit, solution_paths_);
        const double conflict_resolution_cpu_seconds =
            elapsedProcessCpuSeconds(conflict_resolution_cpu_start);
        const double conflict_resolution_wall_seconds =
            std::chrono::duration<double>(Clock::now() -
                                          conflict_resolution_start)
                .count();
        conflict_resolution_times_seconds_.push_back(
            conflict_resolution_wall_seconds);
        conflict_resolution_times_cpu_seconds_.push_back(
            conflict_resolution_cpu_seconds);

        if (!outcome.resolved) {
            finalizePlannerStats();
            return ompl::base::PlannerStatus::TIMEOUT;
        }

        appendVisualizationRepair(0, outcome.final_involved_robots,
                                  outcome.window_start_t,
                                  outcome.window_end_t,
                                  outcome.local_patch_paths);

        resetConflictScanStartsForRobots(outcome.final_involved_robots,
                                         outcome.window_start_t);
        recordAppliedRepairHistory(outcome.final_involved_robots,
                                   outcome.window_start_t,
                                   outcome.window_end_t);
    }

    finalizePlannerStats();
    return ompl::base::PlannerStatus::TIMEOUT;
}

std::vector<Path> ARC::getSolutionPaths() const {
    return solution_paths_;
}

} // namespace comotion
