#include "comotion/planning/ARC.h"
#include "comotion/planning/MultiRobotProblem.h"
#include "comotion/planning/PlanningRng.h"
#include "comotion/planning/detail/PlannerInvariantUtils.h"
#include "comotion/robot/FlyingSphere.h"

#include <ompl/util/RandomNumbers.h>

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace ob = ompl::base;

namespace {

class ArcHistoryProbe : public comotion::ARC {
public:
    using ExpansionState = ExpansionScheduleState;

    void recordHistory(const std::vector<int> &robots, int window_start_t,
                       int window_end_t) {
        recordAppliedRepairHistory(robots, window_start_t, window_end_t);
    }

    std::vector<int> expandTeamWindow(int robot_i, int robot_j,
                                      int window_start_t,
                                      int window_end_t) const {
        return subproblemRobotsForConflict(robot_i, robot_j, window_start_t,
                                           window_end_t);
    }

    comotion::SubproblemConflict expandConflict(const comotion::Conflict &conflict) const {
        return expandConflictForSubproblem(conflict);
    }

    std::vector<std::pair<int, int>> repairWindows(int robot_i,
                                                   int robot_j) const {
        std::vector<std::pair<int, int>> out;
        const auto *windows = repairWindowsForRobots(robot_i, robot_j);
        if (!windows)
            return out;
        for (const auto &window : *windows)
            out.push_back({window.window_start_t, window.window_end_t});
        return out;
    }

    void spliceRaggedLocalPaths(const std::vector<int> &robots, int start_t,
                                int end_t,
                                const std::vector<comotion::Path> &local_paths,
                                std::vector<comotion::Path> &working_paths) {
        true_arrival_timesteps_.clear();
        true_arrival_timesteps_.reserve(working_paths.size());
        for (const auto &path : working_paths) {
            true_arrival_timesteps_.push_back(
                static_cast<std::uint64_t>(path.arrival_timestep()));
        }
        spliceSolutionIntoPaths(robots, start_t, end_t, local_paths,
                                working_paths);
    }

    bool solveProbeSubproblem(const comotion::SubproblemConflict &conflict,
                              double global_time_limit,
                              std::vector<comotion::Path> &working_paths) {
        true_arrival_timesteps_.clear();
        true_arrival_timesteps_.reserve(working_paths.size());
        for (const auto &path : working_paths) {
            true_arrival_timesteps_.push_back(
                static_cast<std::uint64_t>(path.arrival_timestep()));
        }
        int window_start_t = 0;
        int window_end_t = 0;
        return solveSubproblemOnPaths(conflict, Clock::now(), global_time_limit,
                                      working_paths, &window_start_t,
                                      &window_end_t);
    }

    std::pair<int, int>
    expandWindow(int start_t, int end_t, std::size_t max_t,
                 std::size_t expansion_index) const {
        return nextExpansionWindow(start_t, end_t, max_t, expansion_index);
    }

    std::pair<int, int>
    expandInitialValidWindow(int start_t, int end_t, std::size_t max_t,
                             std::size_t expansion_index) const {
        return nextInitialValidExpansionWindow(start_t, end_t, max_t,
                                               expansion_index);
    }

    std::pair<int, int>
    expandAfterAttempt(int start_t, int end_t, std::size_t max_t,
                       bool local_endpoints_valid,
                       ExpansionState &state) const {
        return nextExpansionWindowAfterAttempt(
            start_t, end_t, max_t, local_endpoints_valid, state);
    }

    std::pair<int, int>
    expandAfterAttempt(int start_t, int end_t, std::size_t max_t,
                       bool start_valid, bool goal_valid,
                       ExpansionState &state) const {
        return nextExpansionWindowAfterAttempt(
            start_t, end_t, max_t, start_valid, goal_valid, state);
    }

    nlohmann::json repairAttemptEvents() const {
        return repairAttemptEventsJson();
    }

    nlohmann::json conflictResolutionEvents(double wall_seconds,
                                            double cpu_seconds) {
        conflict_resolution_times_seconds_ = {wall_seconds};
        conflict_resolution_times_cpu_seconds_ = {cpu_seconds};
        return conflictResolutionEventsJson();
    }

    nlohmann::json conflictSolveCountsByExpansionStage() const {
        return conflictSolveCountsByExpansionStageJson();
    }
};

std::shared_ptr<comotion::FlyingSphere> makeSphereRobot(double radius = 1.0) {
    return std::make_shared<comotion::FlyingSphere>(
        radius, std::vector<double>{-12.0, -12.0, 0.0},
        std::vector<double>{12.0, 12.0, 1.5});
}

bool expectTrue(const std::string &label, bool value) {
    if (!value) {
        std::cerr << "arc_exact_only_regression: " << label
                  << " expected true\n";
        return false;
    }
    return true;
}

bool expectEq(const std::string &label, std::size_t actual,
              std::size_t expected) {
    if (actual != expected) {
        std::cerr << "arc_exact_only_regression: " << label
                  << " expected " << expected << " got " << actual << "\n";
        return false;
    }
    return true;
}

bool expectGreaterThanZero(const std::string &label, std::uint64_t value) {
    if (value == 0) {
        std::cerr << "arc_exact_only_regression: " << label
                  << " expected > 0\n";
        return false;
    }
    return true;
}

bool expectWindowEq(const std::string &label,
                    const std::pair<int, int> &actual, int expected_start,
                    int expected_end) {
    if (actual.first != expected_start || actual.second != expected_end) {
        std::cerr << "arc_exact_only_regression: " << label << " expected ["
                  << expected_start << ", " << expected_end << "] got ["
                  << actual.first << ", " << actual.second << "]\n";
        return false;
    }
    return true;
}

bool expectVectorEq(const std::string &label, const std::vector<int> &actual,
                    const std::vector<int> &expected) {
    if (actual != expected) {
        std::cerr << "arc_exact_only_regression: " << label << " expected [";
        for (std::size_t i = 0; i < expected.size(); ++i)
            std::cerr << (i ? ", " : "") << expected[i];
        std::cerr << "] got [";
        for (std::size_t i = 0; i < actual.size(); ++i)
            std::cerr << (i ? ", " : "") << actual[i];
        std::cerr << "]\n";
        return false;
    }
    return true;
}

bool testSignedTimestepShiftHelpers() {
    const auto delta =
        comotion::detail::signedTimestepDelta(42, 30, "test negative delta");
    if (delta != -12) {
        std::cerr << "arc_exact_only_regression: negative delta expected -12 "
                     "got "
                  << delta << "\n";
        return false;
    }

    const auto shifted =
        comotion::detail::applySignedTimestepShift(42, delta, "test negative shift");
    if (!expectEq("negative shift result", shifted, 30))
        return false;

    try {
        (void)comotion::detail::applySignedTimestepShift(
            5, -6, "test underflow");
        std::cerr << "arc_exact_only_regression: expected runtime_error for "
                     "negative timestep underflow\n";
        return false;
    } catch (const std::runtime_error &) {
    }

    return true;
}

bool testConfigMismatchInvariantThrows() {
    try {
        comotion::detail::requireConfigNear({0.0, 1.0}, {0.0, 2.0}, 1e-6,
                                        "test config mismatch");
        std::cerr << "arc_exact_only_regression: expected runtime_error for "
                     "config mismatch invariant\n";
        return false;
    } catch (const std::runtime_error &) {
    }

    return true;
}

bool testRecursiveHistoryCascadeClosesTransitively() {
    ArcHistoryProbe probe;
    probe.recordHistory({0, 1}, 10, 20);
    probe.recordHistory({1, 2}, 18, 30);

    return expectVectorEq("recursive history closure",
                          probe.expandTeamWindow(0, 3, 15, 19),
                          {0, 1, 2, 3});
}

bool testHistoryCascadeIgnoresUnrelatedWindows() {
    ArcHistoryProbe probe;
    probe.recordHistory({0, 1}, 10, 20);
    probe.recordHistory({1, 4}, 30, 40);
    probe.recordHistory({3, 5}, 14, 16);

    return expectVectorEq("history cascade excludes unrelated windows",
                          probe.expandTeamWindow(0, 3, 15, 16),
                          {0, 1, 3, 5});
}

bool testHistoryCascadeHandlesCycles() {
    ArcHistoryProbe probe;
    probe.recordHistory({0, 1}, 10, 20);
    probe.recordHistory({1, 0, 2}, 10, 20);

    return expectVectorEq("history cascade handles cycles",
                          probe.expandTeamWindow(0, 3, 15, 15),
                          {0, 1, 2, 3});
}

bool testHistoryCascadeUsesWindowIntersection() {
    ArcHistoryProbe probe;
    probe.setInitialWindow(10);
    probe.recordHistory({0, 1}, 10, 20);

    const auto expanded = probe.expandConflict(comotion::Conflict{0, 3, 25});
    if (!expectVectorEq("window intersection expansion", expanded.robots,
                        {0, 1, 3}))
        return false;
    return expectTrue("window intersection keeps proposed patch window",
                      expanded.window_begin_t == 15 &&
                          expanded.window_end_t == 35);
}

bool testRepairWindowScheduleMergesOverlaps() {
    ArcHistoryProbe probe;
    probe.recordHistory({0, 1}, 10, 20);
    probe.recordHistory({0, 1}, 15, 30);
    probe.recordHistory({0, 1}, 40, 45);

    const auto windows = probe.repairWindows(0, 1);
    if (!expectTrue("merged window count", windows.size() == 2))
        return false;
    if (!expectTrue("first merged window",
                    windows[0].first == 10 && windows[0].second == 30))
        return false;
    return expectTrue("second non-overlap window",
                      windows[1].first == 40 && windows[1].second == 45);
}

bool testArcExpansionPolicies() {
    ArcHistoryProbe probe;

    probe.setExpansionStep(7);
    probe.setExpansionPolicy(comotion::ARC::ExpansionPolicy::Linear);
    auto window = probe.expandWindow(50, 70, 200, 0);
    if (!expectWindowEq("linear first expansion", window, 43, 77))
        return false;
    window = probe.expandWindow(50, 70, 200, 1);
    if (!expectWindowEq("linear second expansion", window, 36, 84))
        return false;
    if (!expectWindowEq("linear clips to global",
                        probe.expandWindow(2, 198, 200, 2), 0, 200))
        return false;

    probe.setExpansionStep(20);
    probe.setExpansionPolicy(comotion::ARC::ExpansionPolicy::Logarithmic);
    window = probe.expandWindow(80, 120, 300, 0);
    if (!expectWindowEq("logarithmic absolute half-width 40",
                        window, 60, 140))
        return false;
    window = probe.expandWindow(80, 120, 300, 1);
    if (!expectWindowEq("logarithmic absolute half-width 52",
                        window, 48, 152))
        return false;
    window = probe.expandWindow(80, 120, 300, 2);
    if (!expectWindowEq("logarithmic absolute half-width 60",
                        window, 40, 160))
        return false;
    window = probe.expandWindow(80, 120, 300, 3);
    if (!expectWindowEq("logarithmic absolute half-width 67",
                        window, 33, 167))
        return false;
    if (!expectWindowEq("logarithmic ten-percent global jump",
                        probe.expandWindow(30, 170, 200, 0), 0, 200))
        return false;
    if (!expectWindowEq("logarithmic left gap outside threshold",
                        probe.expandWindow(41, 160, 200, 0), 21, 180))
        return false;
    if (!expectWindowEq("logarithmic right gap outside threshold",
                        probe.expandWindow(40, 159, 200, 0), 20, 179))
        return false;

    probe.setExpansionStep(3);
    probe.setExpansionPolicy(comotion::ARC::ExpansionPolicy::Exponential);
    window = probe.expandWindow(50, 70, 200, 0);
    if (!expectWindowEq("exponential delta 3", window, 47, 73))
        return false;
    window = probe.expandWindow(50, 70, 200, 1);
    if (!expectWindowEq("exponential absolute half-width 16", window, 44, 76))
        return false;
    window = probe.expandWindow(50, 70, 200, 2);
    if (!expectWindowEq("exponential absolute half-width 22", window, 38, 82))
        return false;
    if (!expectWindowEq(
            "exponential saturates without overflow",
            probe.expandWindow(
                50, 70, 200, std::numeric_limits<std::size_t>::digits),
            0, 200))
        return false;

    probe.setExpansionStep(1.5);
    probe.setExpansionPolicy(comotion::ARC::ExpansionPolicy::Exponential);
    ArcHistoryProbe::ExpansionState fractional_state;
    window = probe.expandAfterAttempt(
        80, 120, 300, true, fractional_state);
    if (!expectWindowEq(
            "fractional exponential index zero rounds outward",
            window, 78, 122))
        return false;
    window = probe.expandAfterAttempt(
        window.first, window.second, 300, true, fractional_state);
    if (!expectWindowEq(
            "fractional exponential index one uses original base",
            window, 77, 123))
        return false;
    window = probe.expandAfterAttempt(
        window.first, window.second, 300, true, fractional_state);
    if (!expectWindowEq(
            "fractional exponential index two uses original base",
            window, 74, 126))
        return false;

    probe.setExpansionStep(0.01);
    probe.setExpansionPolicy(comotion::ARC::ExpansionPolicy::Logarithmic);
    ArcHistoryProbe::ExpansionState repeated_log_state;
    window = probe.expandAfterAttempt(
        50, 70, 200, true, repeated_log_state);
    if (!expectWindowEq(
            "small logarithmic increment rounds to first larger window",
            window, 49, 71))
        return false;
    window = probe.expandAfterAttempt(
        window.first, window.second, 200, true, repeated_log_state);
    if (!expectWindowEq(
            "repeated logarithmic window falls back to global",
            window, 0, 200))
        return false;

    probe.setInitialWindow(5);
    probe.setCustomExpansionMultipliers({1.0, 1.0, 2.0});
    probe.setExpansionPolicy(
        comotion::ARC::ExpansionPolicy::CustomMultiplied);
    window = probe.expandWindow(50, 70, 200, 0);
    if (!expectWindowEq("multiplied first absolute window", window, 50, 70))
        return false;
    window = probe.expandWindow(window.first, window.second, 200, 1);
    if (!expectWindowEq("multiplied repeated absolute window", window, 50, 70))
        return false;
    window = probe.expandWindow(window.first, window.second, 200, 2);
    if (!expectWindowEq("multiplied doubled absolute window", window, 40, 80))
        return false;
    if (!expectWindowEq("multiplied exhaustion jumps global",
                        probe.expandWindow(window.first, window.second, 200, 3),
                        0, 200))
        return false;

    const auto expectInvalidMultipliers =
        [&probe](std::vector<double> multipliers) {
            try {
                probe.setCustomExpansionMultipliers(std::move(multipliers));
            } catch (const std::invalid_argument &) {
                return true;
            }
            return false;
        };
    if (!expectTrue("empty multiplied sequence rejected",
                    expectInvalidMultipliers({})))
        return false;
    if (!expectTrue("zero multiplier rejected",
                    expectInvalidMultipliers({1.0, 0.0})))
        return false;
    if (!expectTrue(
            "non-finite multiplier rejected",
            expectInvalidMultipliers(
                {1.0, std::numeric_limits<double>::infinity()})))
        return false;

    return true;
}

bool testArcInitialValidExpansionSchedule() {
    ArcHistoryProbe probe;
    probe.setInitialWindow(10);
    probe.setExpansionStep(7);
    probe.setExpansionPolicy(comotion::ARC::ExpansionPolicy::Exponential);
    probe.setCustomExpansionMultipliers({1.0, 3.0});

    if (!expectTrue(
            "initial-valid policy initially inherits main",
            probe.initialValidWindowExpansionPolicyInheritsMain() &&
                probe.initialValidWindowExpansionPolicy() ==
                    comotion::ARC::ExpansionPolicy::Exponential))
        return false;
    if (!expectTrue(
            "initial-valid step initially inherits main",
            probe.initialValidWindowExpansionStepInheritsMain() &&
                probe.initialValidWindowExpansionStep() == 7))
        return false;
    if (!expectTrue(
            "initial-valid multipliers initially inherit main",
            probe.initialValidWindowExpansionMultipliersInheritMain() &&
                probe.initialValidWindowExpansionMultipliers() ==
                    std::vector<double>({1.0, 3.0})))
        return false;

    probe.setExpansionStep(11);
    probe.setExpansionPolicy(comotion::ARC::ExpansionPolicy::Linear);
    probe.setCustomExpansionMultipliers({2.0, 4.0});
    if (!expectTrue(
            "inherited initial-valid settings follow later main changes",
            probe.initialValidWindowExpansionStep() == 11 &&
                probe.initialValidWindowExpansionPolicy() ==
                    comotion::ARC::ExpansionPolicy::Linear &&
                probe.initialValidWindowExpansionMultipliers() ==
                    std::vector<double>({2.0, 4.0})))
        return false;

    probe.setInitialValidWindowExpansionStep(10);
    probe.setInitialValidWindowExpansionPolicy(
        comotion::ARC::ExpansionPolicy::Logarithmic);
    probe.setInitialValidWindowExpansionMultipliers({1.0, 2.0});
    if (!expectTrue(
            "explicit initial-valid settings are independent",
            !probe.initialValidWindowExpansionStepInheritsMain() &&
                !probe.initialValidWindowExpansionPolicyInheritsMain() &&
                !probe.initialValidWindowExpansionMultipliersInheritMain() &&
                probe.initialValidWindowExpansionStep() == 10 &&
                probe.initialValidWindowExpansionPolicy() ==
                    comotion::ARC::ExpansionPolicy::Logarithmic &&
                probe.initialValidWindowExpansionMultipliers() ==
                    std::vector<double>({1.0, 2.0})))
        return false;

    probe.setExpansionPolicy(
        comotion::ARC::ExpansionPolicy::CustomMultiplied);
    probe.setCustomExpansionMultipliers(
        {1.0, 1.0, 2.0, 1.0, 1.0, 2.0, 4.0, 4.0, 8.0, 8.0});

    ArcHistoryProbe::ExpansionState state;
    auto window = probe.expandAfterAttempt(50, 70, 200, false, state);
    if (!expectWindowEq("invalid endpoint uses initial log index 0",
                        window, 40, 80))
        return false;
    if (!expectTrue(
            "first invalid endpoint leaves main sequence untouched",
            state.initial_valid_expansion_index == 1 &&
                state.main_expansion_index == 0 &&
                !state.initial_valid_window_established &&
                state.last_expansion_used_initial_valid_schedule))
        return false;

    window =
        probe.expandAfterAttempt(window.first, window.second, 200, false, state);
    if (!expectWindowEq("invalid endpoint uses initial log index 1",
                        window, 34, 86))
        return false;
    if (!expectTrue(
            "second invalid endpoint still leaves main sequence untouched",
            state.initial_valid_expansion_index == 2 &&
                state.main_expansion_index == 0))
        return false;

    window =
        probe.expandAfterAttempt(window.first, window.second, 200, true, state);
    if (!expectWindowEq(
            "first valid failed attempt restarts discovered base at index 0",
            window, 34, 86))
        return false;
    if (!expectTrue(
            "valid endpoint permanently hands off to main sequence",
            state.initial_valid_window_established &&
                state.initial_valid_expansion_index == 2 &&
                state.main_expansion_index == 1 &&
                !state.last_expansion_used_initial_valid_schedule))
        return false;

    window =
        probe.expandAfterAttempt(window.first, window.second, 200, false, state);
    if (!expectWindowEq(
            "later invalid endpoint remains on repeated main index 1",
            window, 34, 86))
        return false;
    if (!expectTrue(
            "validity prefix is not re-entered after handoff",
            state.initial_valid_expansion_index == 2 &&
                state.main_expansion_index == 2 &&
                !state.last_expansion_used_initial_valid_schedule))
        return false;

    probe.clearInitialValidWindowExpansionStep();
    probe.clearInitialValidWindowExpansionPolicy();
    probe.clearInitialValidWindowExpansionMultipliers();
    if (!expectTrue(
            "clearing initial-valid overrides restores inheritance",
            probe.initialValidWindowExpansionStepInheritsMain() &&
                probe.initialValidWindowExpansionPolicyInheritsMain() &&
                probe.initialValidWindowExpansionMultipliersInheritMain() &&
                probe.initialValidWindowExpansionPolicy() ==
                    comotion::ARC::ExpansionPolicy::CustomMultiplied &&
                probe.initialValidWindowExpansionStep() == 11 &&
                probe.initialValidWindowExpansionMultipliers() ==
                    std::vector<double>(
                        {1.0, 1.0, 2.0, 1.0, 1.0, 2.0, 4.0, 4.0, 8.0,
                         8.0})))
        return false;

    try {
        probe.setInitialValidWindowExpansionMultipliers({1.0, 0.0});
        std::cerr
            << "arc_exact_only_regression: expected invalid initial-valid "
               "multiplier rejection\n";
        return false;
    } catch (const std::invalid_argument &) {
    }

    return true;
}

bool testArcAsymmetricInitialValidExpansionRecentersMainWindow() {
    ArcHistoryProbe probe;
    probe.setInitialWindow(20);
    probe.setInitialValidWindowExpansionPolicy(
        comotion::ARC::ExpansionPolicy::Linear);
    probe.setInitialValidWindowExpansionStep(10);
    probe.setInitialValidWindowExpansionSymmetric(false);
    probe.setExpansionPolicy(
        comotion::ARC::ExpansionPolicy::CustomMultiplied);
    probe.setCustomExpansionMultipliers(
        {1.0, 1.0, 2.0, 1.0, 1.0, 2.0, 4.0});

    ArcHistoryProbe::ExpansionState state;
    auto window =
        probe.expandAfterAttempt(80, 120, 300, true, false, state);
    if (!expectWindowEq(
            "asymmetric endpoint search expands only invalid goal side",
            window, 80, 130))
        return false;
    window = probe.expandAfterAttempt(
        window.first, window.second, 300, true, false, state);
    if (!expectWindowEq(
            "asymmetric endpoint search keeps valid start anchored",
            window, 80, 140))
        return false;

    window = probe.expandAfterAttempt(
        window.first, window.second, 300, true, true, state);
    if (!expectWindowEq(
            "custom index zero restores discovered symmetric base",
            window, 80, 140))
        return false;
    if (!expectTrue(
            "main window recenters on discovered interval",
            state.initial_valid_window_established &&
                state.main_window_center_twice == 220 &&
                state.main_base_half_width_twice == 60))
        return false;

    window = probe.expandAfterAttempt(
        window.first, window.second, 300, true, true, state);
    if (!expectWindowEq(
            "repeated multiplier retries identical recentered window",
            window, 80, 140))
        return false;
    window = probe.expandAfterAttempt(
        window.first, window.second, 300, true, true, state);
    if (!expectWindowEq(
            "multiplier two doubles discovered half-width",
            window, 50, 170))
        return false;
    window = probe.expandAfterAttempt(
        window.first, window.second, 300, true, true, state);
    return expectWindowEq(
        "smaller later multiplier may deliberately return to base window",
        window, 80, 140);
}

std::shared_ptr<comotion::MultiRobotProblem> makeArcApproximateRepairProblem() {
    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(128);
    problem->setVmax(2.0);
    problem->setObstacles({comotion::ObstacleSphere{{0.0, 0.0, 0.75}, 1.5}});

    const std::vector<std::vector<double>> starts = {
        {10.0, 0.0, 0.75},
        {7.07, 7.07, 0.75},
        {0.0, 10.0, 0.75},
        {-7.07, 7.07, 0.75},
        {-10.0, 0.0, 0.75},
        {-7.07, -7.07, 0.75},
        {0.0, -10.0, 0.75},
        {7.07, -7.07, 0.75},
    };
    const std::vector<std::vector<double>> goals = {
        {-10.0, 0.0, 0.75},
        {-7.07, -7.07, 0.75},
        {0.0, -10.0, 0.75},
        {7.07, -7.07, 0.75},
        {10.0, 0.0, 0.75},
        {7.07, 7.07, 0.75},
        {0.0, 10.0, 0.75},
        {-7.07, 7.07, 0.75},
    };

    for (size_t i = 0; i < starts.size(); ++i)
        problem->addRobot(makeSphereRobot(), starts[i], goals[i]);

    return problem;
}

bool testArcRejectsApproximateLocalRepairWithoutSplicing() {
    // Match composite_rrt_approximate_regression seed mapping (OMPL root 7).
    comotion::seedOmplGlobalFromUserPlanningSeed(6);

    auto problem = makeArcApproximateRepairProblem();
    comotion::ARC planner;
    planner.setPlanningSeed(6);
    planner.setProblem(problem);
    planner.setInitialWindow(2000);
    planner.setExpansionStep(2000);
    planner.setUseCspaceBounds(true);
    planner.setCspaceBoundMargin(2.0f);
    planner.setMinCspaceBoundRange(2.0);

    // Tight global wall budget so the first local CompositeRRT call (which gets
    // the full remaining time) is in the same ~0.5s regime as
    // composite_rrt_approximate_rejection, producing an approximate solution that
    // CompositeRRT rejects without ARC splicing.
    constexpr double kTightArcGlobalBudgetSec = 0.32;
    const auto status = planner.solve(kTightArcGlobalBudgetSec);

    if (status != ob::PlannerStatus::TIMEOUT &&
        status != ob::PlannerStatus::EXACT_SOLUTION) {
        std::cerr << "arc_exact_only_regression: expected ARC exact solution "
                     "or timeout, got "
                  << status.asString() << "\n";
        return false;
    }

    return true;
}

bool testArcSolutionMetricsMatchReturnedRaggedPaths() {
    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(32);
    problem->setVmax(2.0);

    problem->addRobot(makeSphereRobot(0.5), {-6.0, -6.0, 0.75},
                      {-2.0, -6.0, 0.75});
    problem->addRobot(makeSphereRobot(0.5), {6.0, 6.0, 0.75},
                      {6.0, -6.0, 0.75});

    comotion::ARC planner;
    planner.setProblem(problem);
    planner.setPlanningSeed(11);
    planner.setVisualizationTraceEnabled(true);
    planner.setLocalPrioritizedStrrtReturnFirstSolution(false);
    planner.setLocalPrioritizedStrrtRewiring(comotion::StrrtRewiring::Radius);
    planner.setLocalPrioritizedStrrtPersistAtGoal(true);
    planner.setInitialWindow(16);
    planner.setExpansionStep(16);
    comotion::PathSimplificationOptions simplification_options;
    simplification_options.max_shortcut_steps = 4;
    simplification_options.max_empty_steps = 2;
    simplification_options.max_smooth_steps = 1;
    planner.setPathSimplificationOptions(simplification_options);

    comotion::seedOmplGlobalFromUserPlanningSeed(11);
    const auto status = planner.solve(5.0);
    if (status != ob::PlannerStatus::EXACT_SOLUTION) {
        std::cerr << "arc_exact_only_regression: expected exact ARC solution "
                     "for solution-metrics test, got "
                  << status.asString() << "\n";
        return false;
    }

    if (!expectTrue("ARC sum_of_cost metric populated",
                    planner.sumOfCostTimesteps().has_value()))
        return false;
    if (!expectTrue("ARC makespan metric populated",
                    planner.makespanTimesteps().has_value()))
        return false;

    const auto paths = planner.getSolutionPaths();
    if (!expectTrue(
            "ARC visualization trace captures a conflict-free iteration",
            planner.visualizationTrace().size() == 1 &&
                planner.visualizationTrace().front().conflict_scan_completed &&
                planner.visualizationTrace().front().conflicts.empty() &&
                planner.visualizationTrace().front().paths.size() ==
                    problem->numRobots()))
        return false;
    std::uint64_t returned_sum = 0;
    std::uint64_t returned_makespan = 0;
    for (const auto &path : paths) {
        if (!expectTrue("ARC returned path has timesteps", path.has_timesteps()))
            return false;
        const auto arrival_timestep =
            static_cast<std::uint64_t>(path.arrival_timestep());
        returned_sum += arrival_timestep;
        returned_makespan = std::max(returned_makespan, arrival_timestep);
    }

    if (!expectTrue("ARC sum_of_cost matches returned ragged paths",
                    *planner.sumOfCostTimesteps() == returned_sum))
        return false;
    if (!expectTrue("ARC makespan matches returned ragged paths",
                    *planner.makespanTimesteps() == returned_makespan))
        return false;

    const auto &planner_stats = planner.plannerStatsJson();
    if (!expectTrue("ARC planner stats include num_conflicts",
                    planner_stats.contains("num_conflicts")))
        return false;
    if (!expectTrue("ARC planner stats include num_subproblem_attempts",
                    planner_stats.contains("num_subproblem_attempts")))
        return false;
    if (!expectTrue("ARC planner stats report zero conflicts for disjoint paths",
                    planner_stats["num_conflicts"].get<std::uint64_t>() == 0))
        return false;
    if (!expectTrue("ARC planner stats include path_simplification",
                    planner_stats.contains("path_simplification")))
        return false;
    const auto &path_simplification = planner_stats["path_simplification"];
    if (!expectTrue("ARC initial simplification enabled by default",
                    path_simplification["initial_enabled"].get<bool>()))
        return false;
    if (!expectTrue("ARC conflict simplification disabled by default",
                    !path_simplification["conflict_enabled"].get<bool>()))
        return false;
    if (!expectTrue("ARC simplification max shortcut steps reported",
                    path_simplification["max_shortcut_steps"]
                            .get<unsigned int>() == 4))
        return false;
    if (!expectTrue("ARC simplification max empty steps reported",
                    path_simplification["max_empty_steps"].get<unsigned int>() ==
                        2))
        return false;
    if (!expectTrue("ARC conflict simplification inherits initial options",
                    path_simplification["conflict"]
                        ["inherits_initial_options"]
                            .get<bool>()))
        return false;
    if (!expectTrue("ARC conflict simplification max shortcut steps reported",
                    path_simplification["conflict"]["max_shortcut_steps"]
                            .get<unsigned int>() == 4))
        return false;
    if (!expectTrue("ARC expansion policy reported",
                    planner_stats["expansion_policy"].get<std::string>() ==
                        "linear"))
        return false;
    if (!expectTrue("ARC initial window reported",
                    planner_stats["initial_window"].get<int>() == 16))
        return false;
    if (!expectTrue(
            "ARC local PrioritizedSTRRT controls are reported",
            !planner_stats["local_prioritized_strrt_return_first_solution"]
                 .get<bool>() &&
                planner_stats["local_prioritized_strrt_rewiring"]
                        .get<std::string>() == "radius" &&
                planner_stats["local_prioritized_strrt_persist_at_goal"]
                    .get<bool>()))
        return false;
    if (!expectTrue("ARC expansion step reported",
                    planner_stats["expansion_step"].get<int>() == 16))
        return false;
    if (!expectTrue("ARC custom expansion sequence reported",
                    planner_stats["custom_expansion_multipliers"].size() == 8))
        return false;
    if (!expectTrue(
            "ARC effective initial-valid expansion policy reported",
            planner_stats["initial_valid_expansion_policy"]
                    .get<std::string>() == "linear"))
        return false;
    if (!expectTrue(
            "ARC effective initial-valid expansion step reported",
            planner_stats["initial_valid_expansion_step"].get<int>() == 16))
        return false;
    if (!expectTrue(
            "ARC initial-valid expansion inheritance reported",
            planner_stats["initial_valid_expansion_inherits_main"]["policy"]
                    .get<bool>() &&
                planner_stats["initial_valid_expansion_inherits_main"]["step"]
                    .get<bool>() &&
                planner_stats["initial_valid_expansion_inherits_main"]
                             ["multipliers"]
                                 .get<bool>()))
        return false;
    if (!expectTrue(
            "ARC phase-specific temporal expansion counts reported",
            planner_stats.contains("initial_valid_temporal_expansions") &&
                planner_stats.contains("main_temporal_expansions") &&
                planner_stats["initial_valid_temporal_expansions"]
                        .get<std::uint64_t>() == 0 &&
                planner_stats["main_temporal_expansions"]
                        .get<std::uint64_t>() == 0))
        return false;
    if (!expectTrue(
            "ARC repair attempt telemetry reported for no-conflict solve",
            planner_stats.contains("repair_attempt_events") &&
                planner_stats["repair_attempt_events"].is_array() &&
                planner_stats["repair_attempt_events"].empty()))
        return false;
    if (!expectTrue(
            "ARC per-conflict timing telemetry reported for no-conflict solve",
            planner_stats.contains(
                "conflict_resolution_times_seconds_wall_clock") &&
                planner_stats
                        ["conflict_resolution_times_seconds_wall_clock"]
                            .get<double>() == 0.0 &&
                planner_stats.contains("conflict_resolution_times_seconds") &&
                planner_stats["conflict_resolution_times_seconds"].empty() &&
                planner_stats.contains("conflict_resolution_events") &&
                planner_stats["conflict_resolution_events"].empty()))
        return false;
    if (!expectTrue(
            "ARC expansion-stage solve counts reported for no-conflict solve",
            planner_stats.contains(
                "conflict_solve_counts_by_expansion_stage") &&
                planner_stats["conflict_solve_counts_by_expansion_stage"]
                             ["total"]["attempts"]
                                 .get<std::uint64_t>() == 0 &&
                planner_stats["conflict_solve_counts_by_expansion_stage"]
                             ["total"]["resolved_conflicts"]
                                 .get<std::uint64_t>() == 0))
        return false;
    if (!expectTrue("ARC simplification time reported",
                    planner_stats.contains(
                        "initial_simplification_times_seconds_wall_clock") &&
                        planner_stats
                            ["initial_simplification_times_seconds_wall_clock"]
                                .get<double>() >= 0.0))
        return false;

    return true;
}

comotion::Path makeTimedLinePath(double y, std::size_t count) {
    comotion::Path path;
    for (std::size_t t = 0; t < count; ++t) {
        path.push_back({static_cast<double>(t), y, 0.75});
    }
    path.markDenseTimestepsImplicit();
    return path;
}

comotion::Path
makeDensePath(const std::vector<std::vector<double>> &configurations) {
    comotion::Path path;
    for (const auto &configuration : configurations)
        path.push_back(configuration);
    path.markDenseTimestepsImplicit();
    return path;
}

bool testArcRepairAttemptTelemetryTracksConsumedIndicesAndGlobal() {
    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(1);
    problem->setVmax(100.0);
    problem->addRobot(makeSphereRobot(0.5), {0.0, 0.0, 0.75},
                      {0.0, 0.0, 0.75});
    problem->addRobot(makeSphereRobot(0.5), {0.0, 0.0, 0.75},
                      {0.0, 0.0, 0.75});

    const std::vector<double> colliding{0.0, 0.0, 0.75};
    const std::vector<double> separated{3.0, 0.0, 0.75};
    std::vector<comotion::Path> working_paths = {
        makeDensePath(std::vector<std::vector<double>>(7, colliding)),
        makeDensePath({colliding, colliding, separated, separated, colliding,
                       colliding, colliding}),
    };

    comotion::SubproblemConflict conflict;
    conflict.seed_robot_i = 0;
    conflict.seed_robot_j = 1;
    conflict.conflict_timestep = 2;
    conflict.window_begin_t = 2;
    conflict.window_end_t = 3;
    conflict.robots = {0, 1};

    ArcHistoryProbe probe;
    probe.setProblem(problem);
    probe.setPlanningSeed(29);
    probe.setInitialWindow(1);
    probe.setExpansionPolicy(
        comotion::ARC::ExpansionPolicy::CustomMultiplied);
    probe.setCustomExpansionMultipliers({1.0, 1.0});
    probe.setInitialValidWindowExpansionPolicy(
        comotion::ARC::ExpansionPolicy::Linear);
    probe.setInitialValidWindowExpansionStep(1);
    probe.setLocalSolverMode(
        comotion::ARC::LocalSolverMode::CompositeRrtOnly);
    probe.setUseCspaceBounds(false);
    probe.setGlobalMakespanBoundTimesteps(0);
    probe.setBoundedLocalRepairEpsilonTimesteps(100);

    if (!expectTrue(
            "invalid expanded endpoints leave telemetry probe unresolved",
            !probe.solveProbeSubproblem(conflict, 2.0, working_paths)))
        return false;

    const auto events = probe.repairAttemptEvents();
    if (!expectEq("repair attempt telemetry event count", events.size(), 4))
        return false;

    const auto &initial = events[0];
    if (!expectTrue(
            "valid base window remains pre-main and skips bounded solve",
            initial["phase"].get<std::string>() == "initial_window" &&
                initial["expansion_index"].is_null() &&
                initial["endpoints_valid"].get<bool>() &&
                initial["bounded_epsilon_skipped"].get<bool>() &&
                !initial["solver_invoked"].get<bool>()))
        return false;

    for (std::size_t index = 0; index < 2; ++index) {
        const auto &event = events[index + 1];
        if (!expectTrue(
                "repeated main window consumes its source index",
                event["phase"].get<std::string>() == "main" &&
                    event["expansion_index"].get<std::size_t>() == index &&
                    !event["effective_global"].get<bool>() &&
                    event["validity_checked"].get<bool>() &&
                    event["endpoints_valid"].get<bool>() &&
                    !event["solver_invoked"].get<bool>() &&
                    !event["resolved"].get<bool>()))
            return false;
    }

    const auto &global = events[3];
    if (!expectTrue(
            "custom exhaustion retains source index and marks global",
            global["phase"].get<std::string>() == "main" &&
                global["expansion_index"].get<std::size_t>() == 2 &&
                global["effective_global"].get<bool>() &&
                !global["endpoints_valid"].get<bool>() &&
                !global["solver_invoked"].get<bool>() &&
                global["outcome"].get<std::string>() ==
                    "global_window_failed"))
        return false;

    const auto counts = probe.conflictSolveCountsByExpansionStage();
    if (!expectTrue(
            "main index telemetry counts repeated windows without solver calls",
            counts["indices"].size() == 2 &&
                counts["indices"][0]["index"].get<std::size_t>() == 0 &&
                counts["indices"][0]["attempts"].get<std::uint64_t>() == 1 &&
                counts["indices"][0]["endpoint_valid_attempts"]
                        .get<std::uint64_t>() == 1 &&
                counts["indices"][0]["solver_invocations"]
                        .get<std::uint64_t>() == 0 &&
                counts["indices"][1]["index"].get<std::size_t>() == 1 &&
                counts["indices"][1]["attempts"].get<std::uint64_t>() == 1 &&
                counts["indices"][1]["solver_invocations"]
                        .get<std::uint64_t>() == 0))
        return false;
    if (!expectTrue(
            "effective global window has a separate custom-main bucket",
            counts["global"]["attempts"].get<std::uint64_t>() == 1 &&
                counts["global"]["solver_invocations"]
                        .get<std::uint64_t>() == 0 &&
                counts["global"]["resolved_conflicts"]
                        .get<std::uint64_t>() == 0))
        return false;

    const auto resolution_events =
        probe.conflictResolutionEvents(0.125, 0.1);
    if (!expectTrue(
            "per-conflict timing event aggregates expansion telemetry",
            resolution_events.size() == 1 &&
                resolution_events[0]["repair_id"].get<std::uint64_t>() == 0 &&
                resolution_events[0]["wall_seconds"].get<double>() == 0.125 &&
                resolution_events[0]["cpu_seconds"].get<double>() == 0.1 &&
                resolution_events[0]["attempt_count"]
                        .get<std::uint64_t>() == 4 &&
                resolution_events[0]["used_main_expansion"].get<bool>() &&
                resolution_events[0]["reached_global_window"].get<bool>() &&
                !resolution_events[0]["resolved"].get<bool>() &&
                !resolution_events[0]["solved_on_first_composite_call"]
                     .get<bool>() &&
                !resolution_events[0]["solved_without_main_expansion"]
                     .get<bool>()))
        return false;

    return true;
}

bool testArcRepairAttemptTelemetrySeparatesInitialValidGlobal() {
    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(1);
    problem->setVmax(100.0);
    problem->addRobot(makeSphereRobot(0.5), {0.0, 0.0, 0.75},
                      {0.0, 0.0, 0.75});
    problem->addRobot(makeSphereRobot(0.5), {3.0, 0.0, 0.75},
                      {3.0, 0.0, 0.75});

    const std::vector<double> robot_zero{0.0, 0.0, 0.75};
    const std::vector<double> colliding{0.0, 0.0, 0.75};
    const std::vector<double> separated{3.0, 0.0, 0.75};
    std::vector<comotion::Path> working_paths = {
        makeDensePath(std::vector<std::vector<double>>(7, robot_zero)),
        makeDensePath({separated, colliding, colliding, colliding, colliding,
                       colliding, separated}),
    };

    comotion::SubproblemConflict conflict;
    conflict.seed_robot_i = 0;
    conflict.seed_robot_j = 1;
    conflict.conflict_timestep = 2;
    conflict.window_begin_t = 2;
    conflict.window_end_t = 3;
    conflict.robots = {0, 1};

    ArcHistoryProbe probe;
    probe.setProblem(problem);
    probe.setPlanningSeed(41);
    probe.setInitialWindow(1);
    probe.setExpansionPolicy(
        comotion::ARC::ExpansionPolicy::CustomMultiplied);
    probe.setCustomExpansionMultipliers({1.0});
    probe.setInitialValidWindowExpansionPolicy(
        comotion::ARC::ExpansionPolicy::Linear);
    probe.setInitialValidWindowExpansionStep(100);
    probe.setLocalSolverMode(
        comotion::ARC::LocalSolverMode::CompositeRrtOnly);
    probe.setUseCspaceBounds(false);

    if (!expectTrue(
            "initial-valid expansion can resolve at global window",
            probe.solveProbeSubproblem(conflict, 2.0, working_paths)))
        return false;

    const auto events = probe.repairAttemptEvents();
    const auto counts = probe.conflictSolveCountsByExpansionStage();
    if (!expectTrue(
            "initial-valid global attempt retains its phase",
            events.size() == 2 &&
                events[0]["phase"].get<std::string>() == "initial_window" &&
                !events[0]["endpoints_valid"].get<bool>() &&
                !events[0]["solver_invoked"].get<bool>() &&
                events[1]["phase"].get<std::string>() == "initial_valid" &&
                events[1]["expansion_index"].get<std::size_t>() == 0 &&
                events[1]["effective_global"].get<bool>() &&
                events[1]["solver_invoked"].get<bool>() &&
                events[1]["resolved"].get<bool>()))
        return false;
    if (!expectTrue(
            "initial-valid global resolution is excluded from main global",
            counts["initial_valid"]["resolved_conflicts"]
                    .get<std::uint64_t>() == 1 &&
                counts["initial_valid"]["effective_global_attempts"]
                        .get<std::uint64_t>() == 1 &&
                counts["global"]["attempts"].get<std::uint64_t>() == 0 &&
                counts["global"]["resolved_conflicts"]
                        .get<std::uint64_t>() == 0 &&
                counts["main_total"]["attempts"].get<std::uint64_t>() == 0))
        return false;

    return true;
}

bool testArcRepairAttemptTelemetryCountsMainIndexSolutions() {
    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(1);
    problem->setVmax(100.0);
    const std::vector<double> stationary{0.0, 0.0, 0.75};
    problem->addRobot(makeSphereRobot(0.25), stationary, stationary);

    comotion::SubproblemConflict conflict;
    conflict.seed_robot_i = 0;
    conflict.seed_robot_j = 0;
    conflict.conflict_timestep = 2;
    conflict.window_begin_t = 2;
    conflict.window_end_t = 3;
    conflict.robots = {0};

    ArcHistoryProbe probe;
    probe.setProblem(problem);
    probe.setPlanningSeed(31);
    probe.setInitialWindow(1);
    probe.setExpansionPolicy(
        comotion::ARC::ExpansionPolicy::CustomMultiplied);
    probe.setCustomExpansionMultipliers({3.0, 3.0});
    probe.setLocalSolverMode(
        comotion::ARC::LocalSolverMode::CompositeRrtOnly);
    probe.setUseCspaceBounds(false);
    probe.setGlobalMakespanBoundTimesteps(0);
    probe.setBoundedLocalRepairEpsilonTimesteps(1);

    for (std::size_t repair = 0; repair < 2; ++repair) {
        std::vector<comotion::Path> working_paths = {
            makeDensePath(
                std::vector<std::vector<double>>(7, stationary)),
        };
        if (!expectTrue(
                "main index zero bounded repair succeeds",
                probe.solveProbeSubproblem(conflict, 2.0, working_paths)))
            return false;
    }

    const auto events = probe.repairAttemptEvents();
    if (!expectEq("two repair traces contain base and main attempts",
                  events.size(), 4))
        return false;
    for (std::size_t repair = 0; repair < 2; ++repair) {
        const auto &base = events[repair * 2];
        const auto &main = events[repair * 2 + 1];
        if (!expectTrue(
                "each repair resets local attempt and main indices",
                base["repair_id"].get<std::uint64_t>() == repair &&
                    base["attempt_index"].get<std::uint64_t>() == 0 &&
                    base["phase"].get<std::string>() == "initial_window" &&
                    base["expansion_index"].is_null() &&
                    !base["solver_invoked"].get<bool>() &&
                    main["repair_id"].get<std::uint64_t>() == repair &&
                    main["attempt_index"].get<std::uint64_t>() == 1 &&
                    main["phase"].get<std::string>() == "main" &&
                    main["expansion_index"].get<std::size_t>() == 0 &&
                    !main["effective_global"].get<bool>() &&
                    main["solver_invoked"].get<bool>() &&
                    main["composite_invoked"].get<bool>() &&
                    main["resolved"].get<bool>()))
            return false;
    }

    const auto counts = probe.conflictSolveCountsByExpansionStage();
    if (!expectTrue(
            "main index zero counts solver invocations and resolutions",
            counts["indices"][0]["index"].get<std::size_t>() == 0 &&
                counts["indices"][0]["attempts"].get<std::uint64_t>() == 2 &&
                counts["indices"][0]["endpoint_valid_attempts"]
                        .get<std::uint64_t>() == 2 &&
                counts["indices"][0]["solver_invocations"]
                        .get<std::uint64_t>() == 2 &&
                counts["indices"][0]["resolved_conflicts"]
                        .get<std::uint64_t>() == 2 &&
                counts["global"]["resolved_conflicts"]
                        .get<std::uint64_t>() == 0))
        return false;

    ArcHistoryProbe global_probe;
    global_probe.setProblem(problem);
    global_probe.setPlanningSeed(37);
    global_probe.setInitialWindow(1);
    global_probe.setExpansionPolicy(
        comotion::ARC::ExpansionPolicy::CustomMultiplied);
    global_probe.setCustomExpansionMultipliers({100.0});
    global_probe.setLocalSolverMode(
        comotion::ARC::LocalSolverMode::CompositeRrtOnly);
    global_probe.setUseCspaceBounds(false);
    global_probe.setGlobalMakespanBoundTimesteps(0);
    global_probe.setBoundedLocalRepairEpsilonTimesteps(1);
    std::vector<comotion::Path> global_working_paths = {
        makeDensePath(std::vector<std::vector<double>>(7, stationary)),
    };
    if (!expectTrue(
            "main index that clips global resolves in global bucket",
            global_probe.solveProbeSubproblem(conflict, 2.0,
                                              global_working_paths)))
        return false;
    const auto global_events = global_probe.repairAttemptEvents();
    const auto global_counts =
        global_probe.conflictSolveCountsByExpansionStage();
    if (!expectTrue(
            "global clipping preserves source index without double counting",
            global_events.size() == 2 &&
                global_events[1]["phase"].get<std::string>() == "main" &&
                global_events[1]["expansion_index"].get<std::size_t>() == 0 &&
                global_events[1]["effective_global"].get<bool>() &&
                global_events[1]["solver_invoked"].get<bool>() &&
                global_events[1]["resolved"].get<bool>() &&
                global_counts["indices"][0]["resolved_conflicts"]
                        .get<std::uint64_t>() == 0 &&
                global_counts["global"]["resolved_conflicts"]
                        .get<std::uint64_t>() == 1))
        return false;

    probe.clearGlobalMakespanBoundTimesteps();
    const auto public_status = probe.solve(1.0);
    if (!expectTrue(
            "public ARC solve resets prior repair attempt telemetry",
            public_status == ob::PlannerStatus::EXACT_SOLUTION &&
                probe.repairAttemptEvents().empty() &&
                probe.conflictSolveCountsByExpansionStage()["total"]
                         ["attempts"]
                             .get<std::uint64_t>() == 0))
        return false;

    return true;
}

bool testArcSplicesRaggedLocalPathsWithoutEqualizing() {
    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(1);
    problem->setVmax(100.0);
    problem->addRobot(makeSphereRobot(0.1), {0.0, 0.0, 0.75},
                      {10.0, 0.0, 0.75});
    problem->addRobot(makeSphereRobot(0.1), {0.0, 5.0, 0.75},
                      {10.0, 5.0, 0.75});

    std::vector<comotion::Path> working_paths = {
        makeTimedLinePath(0.0, 11),
        makeTimedLinePath(5.0, 11),
    };

    comotion::Path short_local;
    short_local.push_back({2.0, 0.0, 0.75});
    short_local.push_back({4.0, 0.0, 0.75});
    short_local.push_back({6.0, 0.0, 0.75});
    short_local.waypoint_timesteps_ = {0, 1, 2};

    comotion::Path long_local;
    long_local.push_back({2.0, 5.0, 0.75});
    long_local.push_back({3.0, 5.0, 0.75});
    long_local.push_back({4.0, 5.0, 0.75});
    long_local.push_back({5.0, 5.0, 0.75});
    long_local.push_back({6.0, 5.0, 0.75});
    long_local.waypoint_timesteps_ = {0, 1, 2, 3, 4};

    if (!expectTrue("test local paths are ragged before splice",
                    short_local.size() != long_local.size())) {
        return false;
    }

    ArcHistoryProbe probe;
    probe.setProblem(problem);
    probe.spliceRaggedLocalPaths({0, 1}, 2, 6, {short_local, long_local},
                                 working_paths);

    if (!expectTrue("short local path shifts robot 0 arrival earlier",
                    working_paths[0].arrival_timestep() == 8)) {
        return false;
    }
    if (!expectTrue("long local path preserves robot 1 arrival",
                    working_paths[1].arrival_timestep() == 10)) {
        return false;
    }
    if (!expectTrue("in-place splice keeps dense robot 0 timesteps implicit",
                    working_paths[0].has_implicit_dense_timesteps()))
        return false;
    if (!expectTrue("in-place splice keeps dense robot 1 timesteps implicit",
                    working_paths[1].has_implicit_dense_timesteps()))
        return false;
    return expectTrue("ragged local splice keeps different global arrivals",
                      working_paths[0].arrival_timestep() !=
                          working_paths[1].arrival_timestep());
}

std::shared_ptr<comotion::MultiRobotProblem> makeArcLocalModeProblem() {
    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(1);
    problem->setVmax(1.0);
    problem->addRobot(makeSphereRobot(0.1), {0.0, 0.0, 0.75},
                      {10.0, 0.0, 0.75});
    return problem;
}

comotion::SubproblemConflict makeSingleRobotFullWindowConflict() {
    comotion::SubproblemConflict conflict;
    conflict.seed_robot_i = 0;
    conflict.seed_robot_j = 0;
    conflict.conflict_timestep = 0;
    conflict.window_begin_t = 0;
    conflict.window_end_t = 1;
    conflict.robots = {0};
    return conflict;
}

bool runLocalModeProbe(comotion::ARC::LocalSolverMode mode, bool &success) {
    auto problem = makeArcLocalModeProblem();
    std::vector<comotion::Path> working_paths;
    comotion::Path path;
    path.push_back(problem->robot(0).start);
    path.push_back(problem->robot(0).goal);
    path.waypoint_timesteps_ = {0, 1};
    working_paths.push_back(path);

    ArcHistoryProbe probe;
    probe.setProblem(problem);
    probe.setPlanningSeed(17);
    probe.setLocalSolverMode(mode);
    probe.setInitialWindow(1);
    probe.setExpansionStep(1);
    probe.setUseCspaceBounds(false);
    probe.setStrrtSpaceTimeSpanFactor(1.0);
    probe.setLocalPrioritizedStrrtMaxIterations(1);

    success = probe.solveProbeSubproblem(makeSingleRobotFullWindowConflict(),
                                         2.0, working_paths);
    return true;
}

bool testArcLocalRepairsUsePerSubproblemSeeds() {
    auto problem = makeArcLocalModeProblem();
    ArcHistoryProbe probe;
    probe.setProblem(problem);
    probe.setPlanningSeed(17);
    probe.setLocalSolverMode(
        comotion::ARC::LocalSolverMode::CompositeRrtOnly);
    probe.setInitialWindow(1);
    probe.setExpansionStep(1);
    probe.setUseCspaceBounds(false);

    for (int repair = 0; repair < 2; ++repair) {
        comotion::Path path;
        path.push_back(problem->robot(0).start);
        path.push_back(problem->robot(0).goal);
        path.waypoint_timesteps_ = {0, 1};
        std::vector<comotion::Path> working_paths{path};
        if (!expectTrue(
                "seed telemetry local repair succeeds",
                probe.solveProbeSubproblem(
                    makeSingleRobotFullWindowConflict(), 2.0,
                    working_paths))) {
            return false;
        }
    }

    const auto events = probe.repairAttemptEvents();
    if (!expectEq("seed telemetry repair count", events.size(), 2))
        return false;

    std::uint32_t previous_root = 0;
    std::uint32_t previous_composite = 0;
    for (std::size_t index = 0; index < events.size(); ++index) {
        const auto root =
            events[index]["attempt_root_planning_seed"].get<std::uint32_t>();
        const auto composite =
            events[index]["composite_planning_seed"].get<std::uint32_t>();
        if (!expectTrue(
                "ARC invokes composite with its per-attempt derived seed",
                composite ==
                    comotion::arcRepairCompositePlanningSeed(root))) {
            return false;
        }
        if (index > 0 &&
            !expectTrue("separate ARC repairs use different root seeds",
                        root != previous_root &&
                            composite != previous_composite)) {
            return false;
        }
        previous_root = root;
        previous_composite = composite;
    }
    return true;
}

bool testArcLocalSolverModeGating() {
    bool success = false;

    runLocalModeProbe(comotion::ARC::LocalSolverMode::CompositeRrtOnly, success);
    if (!expectTrue("composite-only local probe succeeds", success))
        return false;

    runLocalModeProbe(comotion::ARC::LocalSolverMode::PrioritizedStrrtOnly,
                      success);
    if (!expectTrue("prioritized-only local probe fails bounded short window",
                    !success))
        return false;

    runLocalModeProbe(comotion::ARC::LocalSolverMode::Both, success);
    if (!expectTrue("both-mode local probe falls back to composite", success))
        return false;

    return true;
}

} // namespace

int main() {
    if (!testSignedTimestepShiftHelpers())
        return 1;
    if (!testConfigMismatchInvariantThrows())
        return 1;
    if (!testRecursiveHistoryCascadeClosesTransitively())
        return 1;
    if (!testHistoryCascadeIgnoresUnrelatedWindows())
        return 1;
    if (!testHistoryCascadeHandlesCycles())
        return 1;
    if (!testHistoryCascadeUsesWindowIntersection())
        return 1;
    if (!testRepairWindowScheduleMergesOverlaps())
        return 1;
    if (!testArcExpansionPolicies())
        return 1;
    if (!testArcInitialValidExpansionSchedule())
        return 1;
    if (!testArcAsymmetricInitialValidExpansionRecentersMainWindow())
        return 1;
    if (!testArcRejectsApproximateLocalRepairWithoutSplicing())
        return 1;
    if (!testArcSolutionMetricsMatchReturnedRaggedPaths())
        return 1;
    if (!testArcRepairAttemptTelemetryTracksConsumedIndicesAndGlobal())
        return 1;
    if (!testArcRepairAttemptTelemetrySeparatesInitialValidGlobal())
        return 1;
    if (!testArcRepairAttemptTelemetryCountsMainIndexSolutions())
        return 1;
    if (!testArcSplicesRaggedLocalPathsWithoutEqualizing())
        return 1;
    if (!testArcLocalSolverModeGating())
        return 1;
    if (!testArcLocalRepairsUsePerSubproblemSeeds())
        return 1;

    std::cout << "arc_exact_only_regression: OK\n";
    return 0;
}
