#include "comotion/collision/ValidationTypes.h"
#include "comotion/planning/AOARC.h"
#include "comotion/planning/MultiRobotProblem.h"
#include "comotion/planning/PlanningRng.h"
#include "comotion/robot/FlyingSphere.h"

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace {

bool expectTrue(const std::string &label, bool condition) {
    if (!condition) {
        std::cerr << "ao_arc_selective_regression: " << label << "\n";
        return false;
    }
    return true;
}

template <typename T>
bool expectEq(const std::string &label, const T &actual, const T &expected) {
    if (actual != expected) {
        std::cerr << "ao_arc_selective_regression: " << label
                  << " expected " << expected << " got " << actual << "\n";
        return false;
    }
    return true;
}

class AOArcProbe : public comotion::AOARC {
public:
    using comotion::ARC::AppliedRepairHistoryEvent;
    using comotion::AOARC::boundedInitialPathReuseBoundTimesteps;
    using comotion::AOARC::incorporateAcceptedRepairHistory;
    using comotion::AOARC::oneHopRepairPartnerRobots;
    using comotion::AOARC::randomFullRestartForAttempt;
    using comotion::AOARC::repairPartnerRobotsWithinDepth;
};

class ArcProbe : public comotion::ARC {
public:
    using comotion::ARC::conflictScanOptions;
    using comotion::ARC::initializeConflictScanStartsForChangedRobots;
    using comotion::ARC::localMakespanBoundForRobot;
    using comotion::ARC::resetConflictScanStartsForRobots;

    bool initializeFromSeeds(
        std::shared_ptr<comotion::MultiRobotProblem> problem,
        std::vector<comotion::Path> seeds, std::uint64_t global_bound,
        std::uint64_t reuse_bound, double time_limit,
        std::vector<comotion::Path> &working_paths,
        std::vector<int> forced_replanning_robots = {}) {
        setProblem(std::move(problem));
        setGlobalMakespanBoundTimesteps(global_bound);
        setBoundedInitialPathReuseBoundTimesteps(reuse_bound);
        setBoundedInitialPaths(std::move(seeds));
        setBoundedInitialForcedReplanningRobots(
            std::move(forced_replanning_robots));
        return planIndividualPaths(Clock::now(), time_limit, working_paths);
    }

    const std::vector<int> &scanStarts() const {
        return pair_conflict_scan_start_t_;
    }

    const std::vector<int> &initiallyReplannedRobots() const {
        return initially_replanned_robots_;
    }

    const std::vector<int> &initiallyReplanAttemptedRobots() const {
        return initially_replan_attempted_robots_;
    }

    const std::vector<int> &initiallySelectedForReplanningRobots() const {
        return initially_selected_for_replanning_robots_;
    }

    const std::vector<int> &initiallyForcedReplanningRobots() const {
        return initially_forced_replanning_robots_;
    }

    std::uint64_t numInitialPathsReused() const {
        return num_initial_paths_reused_;
    }

    std::uint64_t numInitialConflictPairsSkipped() const {
        return num_initial_conflict_pairs_skipped_;
    }
};

comotion::Path makeTimedPath(const std::vector<double> &start,
                             const std::vector<double> &goal,
                             std::size_t arrival_timestep) {
    comotion::Path path;
    path.push_back(start);
    path.push_back(goal);
    path.set_waypoint_timesteps({0, arrival_timestep});
    return path;
}

bool samePath(const comotion::Path &lhs, const comotion::Path &rhs) {
    return static_cast<const comotion::Path::Base &>(lhs) ==
               static_cast<const comotion::Path::Base &>(rhs) &&
           lhs.waypoint_timesteps_ == rhs.waypoint_timesteps_ &&
           lhs.has_implicit_dense_timesteps() ==
               rhs.has_implicit_dense_timesteps();
}

bool testAOArcToggleDefaultsAndRoundTrips() {
    AOArcProbe planner;
    if (!expectTrue("selective bounded replanning defaults on",
                    planner.selectiveBoundedReplanning()))
        return false;
    if (!expectTrue("selective initial conflict scan defaults on",
                    planner.selectiveInitialConflictScan()))
        return false;
    if (!expectTrue("repair-history replanning expansion defaults off",
                    !planner.expandReplanningFromRepairHistory()))
        return false;
    if (!expectEq<std::size_t>("repair-history depth defaults to zero",
                               planner.repairHistoryReplanningDepth(), 0))
        return false;
    if (!expectTrue("random full restart defaults off",
                    planner.randomFullRestartProbability() == 0.0))
        return false;

    planner.setSelectiveBoundedReplanning(false);
    if (!expectTrue("selective bounded replanning setter disables",
                    !planner.selectiveBoundedReplanning()))
        return false;
    if (!expectTrue("bounded replanning setter leaves scan toggle unchanged",
                    planner.selectiveInitialConflictScan()))
        return false;

    planner.setSelectiveInitialConflictScan(false);
    if (!expectTrue("selective initial conflict scan setter disables",
                    !planner.selectiveInitialConflictScan()))
        return false;

    planner.setExpandReplanningFromRepairHistory(true);
    if (!expectTrue("repair-history replanning expansion setter enables",
                    planner.expandReplanningFromRepairHistory() &&
                        planner.repairHistoryReplanningDepth() == 1))
        return false;

    planner.setRepairHistoryReplanningDepth(2);
    if (!expectTrue("repair-history depth supports two hops",
                    planner.expandReplanningFromRepairHistory() &&
                        planner.repairHistoryReplanningDepth() == 2))
        return false;
    planner.setRandomFullRestartProbability(0.25);
    if (!expectTrue("random full-restart probability round-trips",
                    planner.randomFullRestartProbability() == 0.25))
        return false;

    planner.setSelectiveBoundedReplanning(true);
    planner.setSelectiveInitialConflictScan(true);
    planner.setExpandReplanningFromRepairHistory(false);
    planner.setRandomFullRestartProbability(0.0);
    return expectTrue("AO-ARC behavior toggles round-trip",
                      planner.selectiveBoundedReplanning() &&
                          planner.selectiveInitialConflictScan() &&
                          !planner.expandReplanningFromRepairHistory() &&
                          planner.repairHistoryReplanningDepth() == 0 &&
                          planner.randomFullRestartProbability() == 0.0);
}

bool testRepairPartnerExpansionUsesExactBreadthFirstDepth() {
    using Event = AOArcProbe::AppliedRepairHistoryEvent;
    const std::vector<Event> history = {
        Event{0, {0, 1}, 10, 20},
        Event{1, {1, 2}, 20, 30},
        Event{2, {0, 3, 4}, 30, 40},
        Event{3, {4, 5}, 40, 50},
        Event{4, {6, 7}, 50, 60},
        Event{5, {0, 1}, 60, 70},
    };

    const auto from_zero =
        AOArcProbe::repairPartnerRobotsWithinDepth({0}, history, 1);
    if (!expectTrue(
            "one-hop expansion adds sorted unique direct repair partners",
            from_zero == std::vector<int>({1, 3, 4})))
        return false;

    const auto from_zero_two_hops =
        AOArcProbe::repairPartnerRobotsWithinDepth({0}, history, 2);
    if (!expectTrue("two-hop expansion adds distance-two repair partners",
                    from_zero_two_hops ==
                        std::vector<int>({1, 2, 3, 4, 5})))
        return false;

    auto reversed_history = history;
    std::reverse(reversed_history.begin(), reversed_history.end());
    if (!expectTrue(
            "breadth-first expansion is independent of history order",
            AOArcProbe::repairPartnerRobotsWithinDepth(
                {0}, reversed_history, 2) == from_zero_two_hops))
        return false;

    if (!expectTrue("depth zero is violators-only",
                    AOArcProbe::repairPartnerRobotsWithinDepth(
                        {0}, history, 0)
                        .empty()))
        return false;
    if (!expectTrue(
            "one-hop expansion does not recurse through newly added partners",
            std::find(from_zero.begin(), from_zero.end(), 2) ==
                    from_zero.end() &&
                std::find(from_zero.begin(), from_zero.end(), 5) ==
                    from_zero.end()))
        return false;

    const auto from_zero_and_two =
        AOArcProbe::repairPartnerRobotsWithinDepth({2, 0, 2}, history, 1);
    if (!expectTrue("each original violator contributes direct partners only",
                    from_zero_and_two == std::vector<int>({1, 3, 4})))
        return false;
    if (!expectTrue("original violators are excluded from expansion output",
                    std::find(from_zero_and_two.begin(),
                              from_zero_and_two.end(), 0) ==
                            from_zero_and_two.end() &&
                        std::find(from_zero_and_two.begin(),
                                  from_zero_and_two.end(), 2) ==
                            from_zero_and_two.end()))
        return false;

    return expectTrue(
        "empty bound-violator set cannot expand from repair history",
        AOArcProbe::repairPartnerRobotsWithinDepth({}, history, 10).empty());
}

bool testRandomFullRestartDecisionsAreBoundedAndReplayable() {
    AOArcProbe planner;
    const auto rejects_probability = [&](double probability) {
        try {
            planner.setRandomFullRestartProbability(probability);
        } catch (const std::invalid_argument &) {
            return true;
        }
        return false;
    };
    if (!expectTrue("negative restart probability is rejected",
                    rejects_probability(-0.01)))
        return false;
    if (!expectTrue("restart probability above one is rejected",
                    rejects_probability(1.01)))
        return false;
    if (!expectTrue("nonfinite restart probability is rejected",
                    rejects_probability(
                        std::numeric_limits<double>::quiet_NaN()) &&
                        rejects_probability(
                            std::numeric_limits<double>::infinity())))
        return false;

    std::vector<bool> first_sequence;
    std::vector<bool> replayed_sequence;
    std::vector<bool> other_seed_sequence;
    for (std::uint64_t attempt = 0; attempt < 64; ++attempt) {
        if (!expectTrue("probability zero never restarts",
                        !AOArcProbe::randomFullRestartForAttempt(
                            73, attempt, 0.0)))
            return false;
        if (!expectTrue("probability one always restarts",
                        AOArcProbe::randomFullRestartForAttempt(
                            73, attempt, 1.0)))
            return false;
        first_sequence.push_back(AOArcProbe::randomFullRestartForAttempt(
            73, attempt, 0.25));
        replayed_sequence.push_back(AOArcProbe::randomFullRestartForAttempt(
            73, attempt, 0.25));
        other_seed_sequence.push_back(
            AOArcProbe::randomFullRestartForAttempt(74, attempt, 0.25));
    }
    if (!expectTrue("restart decision sequence is replayable",
                    first_sequence == replayed_sequence))
        return false;
    if (!expectTrue("restart decision stream is seed-dependent",
                    first_sequence != other_seed_sequence))
        return false;
    const std::vector<bool> expected_prefix = {
        true,  true,  false, false, false, false, false, true,
        false, true,  false, true,  true,  false, false, false,
    };
    if (!expectTrue(
            "quarter-probability restart schedule remains replay-compatible",
            std::equal(expected_prefix.begin(), expected_prefix.end(),
                       first_sequence.begin())))
        return false;
    return expectTrue(
        "quarter-probability sequence contains both decisions",
        std::find(first_sequence.begin(), first_sequence.end(), true) !=
                first_sequence.end() &&
            std::find(first_sequence.begin(), first_sequence.end(), false) !=
                first_sequence.end());
}

bool testAcceptedRepairHistoryAppendsOnlyWhenIncumbentPathsRemain() {
    using Event = AOArcProbe::AppliedRepairHistoryEvent;
    const Event old_event{0, {0, 1}, 10, 20};
    const Event new_event{1, {1, 2}, 20, 30};

    std::vector<Event> retained_history{old_event};
    AOArcProbe::incorporateAcceptedRepairHistory(
        retained_history, {new_event}, true);
    if (!expectTrue("selective improvement appends repair history",
                    retained_history.size() == 2 &&
                        retained_history[0].event_id == 0 &&
                        retained_history[1].event_id == 1))
        return false;

    std::vector<Event> discarded_history{old_event};
    AOArcProbe::incorporateAcceptedRepairHistory(
        discarded_history, {new_event}, false);
    return expectTrue("full restart replaces discarded repair history",
                      discarded_history.size() == 1 &&
                          discarded_history[0].event_id == 1);
}

bool testStrictDiscreteReuseTarget() {
    if (!expectEq<std::uint64_t>(
            "epsilon zero still tightens by one timestep",
            AOArcProbe::boundedInitialPathReuseBoundTimesteps(10, 0), 9))
        return false;
    if (!expectEq<std::uint64_t>(
            "epsilon one tightens by one timestep",
            AOArcProbe::boundedInitialPathReuseBoundTimesteps(10, 1), 9))
        return false;
    if (!expectEq<std::uint64_t>(
            "epsilon greater than one is the discrete reduction",
            AOArcProbe::boundedInitialPathReuseBoundTimesteps(10, 4), 6))
        return false;
    if (!expectEq<std::uint64_t>(
            "one-timestep incumbent saturates at zero",
            AOArcProbe::boundedInitialPathReuseBoundTimesteps(1, 0), 0))
        return false;
    if (!expectEq<std::uint64_t>(
            "epsilon equal to incumbent saturates at zero",
            AOArcProbe::boundedInitialPathReuseBoundTimesteps(3, 3), 0))
        return false;
    if (!expectEq<std::uint64_t>(
            "epsilon above incumbent saturates without underflow",
            AOArcProbe::boundedInitialPathReuseBoundTimesteps(3, 4), 0))
        return false;
    if (!expectEq<std::uint64_t>(
            "zero incumbent remains zero",
            AOArcProbe::boundedInitialPathReuseBoundTimesteps(0, 9), 0))
        return false;

    const auto maximum = std::numeric_limits<std::uint64_t>::max();
    return expectEq<std::uint64_t>(
        "maximum incumbent tightens without overflow",
        AOArcProbe::boundedInitialPathReuseBoundTimesteps(maximum, 0),
        maximum - 1);
}

bool expectBound(const std::string &label,
                 const std::optional<std::uint64_t> &actual,
                 std::uint64_t expected) {
    return expectTrue(label + " exists", actual.has_value()) &&
           expectEq<std::uint64_t>(label, *actual, expected);
}

bool testLocalMakespanBoundUsesDirectPrefixAndSuffix() {
    if (!expectBound("full-window bound keeps the inclusive global budget",
                     ArcProbe::localMakespanBoundForRobot(10, 10, 0, 10),
                     10))
        return false;
    if (!expectBound("global sentinel beyond arrival keeps global budget",
                     ArcProbe::localMakespanBoundForRobot(10, 10, 0, 11),
                     10))
        return false;
    if (!expectBound("prefix and suffix are both charged",
                     ArcProbe::localMakespanBoundForRobot(100, 80, 20, 50),
                     50))
        return false;
    if (!expectBound("arrival inside window has no suffix charge",
                     ArcProbe::localMakespanBoundForRobot(100, 40, 20, 50),
                     80))
        return false;
    if (!expectBound("negative window start clamps prefix to zero",
                     ArcProbe::localMakespanBoundForRobot(100, 30, -5, 10),
                     80))
        return false;
    if (!expectBound("reversed window clamps end to its start",
                     ArcProbe::localMakespanBoundForRobot(100, 80, 20, 10),
                     20))
        return false;
    if (!expectBound("exact fixed-cost boundary permits zero local budget",
                     ArcProbe::localMakespanBoundForRobot(50, 80, 20, 50),
                     0))
        return false;
    if (!expectTrue("fixed prefix and suffix exceeding bound is infeasible",
                    !ArcProbe::localMakespanBoundForRobot(49, 80, 20, 50)
                         .has_value()))
        return false;
    return expectTrue("prefix alone exceeding bound is infeasible",
                      !ArcProbe::localMakespanBoundForRobot(19, 20, 20, 20)
                           .has_value());
}

bool testSelectiveInitialPairFrontiersAndRepairReset() {
    std::vector<comotion::Path> paths = {
        makeTimedPath({0.0}, {1.0}, 3),
        makeTimedPath({1.0}, {2.0}, 8),
        makeTimedPath({2.0}, {3.0}, 5),
        makeTimedPath({3.0}, {4.0}, 6),
    };

    ArcProbe probe;
    probe.initializeConflictScanStartsForChangedRobots(paths, {1});
    const auto &initial = probe.scanStarts();
    if (!expectEq<std::size_t>("one frontier per robot pair", initial.size(),
                               comotion::pairFrontierSize(4)))
        return false;

    const auto frontier = [&](std::size_t i, std::size_t j) {
        return initial[comotion::pairFrontierIndex(i, j, 4)];
    };
    if (!expectTrue("pairs touching replanned robot start at zero",
                    frontier(0, 1) == 0 && frontier(1, 2) == 0 &&
                        frontier(1, 3) == 0))
        return false;
    if (!expectTrue("unchanged pairs start one past maximum arrival",
                    frontier(0, 2) == 9 && frontier(0, 3) == 9 &&
                        frontier(2, 3) == 9))
        return false;
    if (!expectEq<std::uint64_t>("three unchanged pairs are skipped",
                                 probe.numInitialConflictPairsSkipped(), 3))
        return false;

    const auto initial_options = probe.conflictScanOptions();
    if (!expectTrue("pair frontiers are forwarded to conflict scanning",
                    initial_options.t_begin == 0 &&
                        initial_options.per_pair_t_begin ==
                            std::vector<std::size_t>{0, 9, 9, 0, 0, 9}))
        return false;

    ArcProbe all_changed;
    all_changed.initializeConflictScanStartsForChangedRobots(
        paths, {0, 1, 2, 3});
    if (!expectTrue("all replanned robots leave every pair frontier at zero",
                    all_changed.scanStarts() == std::vector<int>(6, 0) &&
                        all_changed.numInitialConflictPairsSkipped() == 0))
        return false;

    probe.resetConflictScanStartsForRobots({2}, 4);
    const auto &after_repair = probe.scanStarts();
    const auto repaired_frontier = [&](std::size_t i, std::size_t j) {
        return after_repair[comotion::pairFrontierIndex(i, j, 4)];
    };
    if (!expectTrue("repair lowers every incident frontier",
                    repaired_frontier(0, 2) == 4 &&
                        repaired_frontier(2, 3) == 4))
        return false;
    if (!expectTrue("repair cannot advance an already-earlier frontier",
                    repaired_frontier(1, 2) == 0))
        return false;
    return expectTrue("repair preserves unrelated pair frontiers",
                      repaired_frontier(0, 1) == 0 &&
                          repaired_frontier(0, 3) == 9 &&
                          repaired_frontier(1, 3) == 0);
}

std::shared_ptr<comotion::MultiRobotProblem> makeReuseProblem() {
    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(1);
    problem->setVmax(1.0);
    auto robot = []() {
        return std::make_shared<comotion::FlyingSphere>(0.1, -20.0, 20.0);
    };
    problem->addRobot(robot(), {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
    problem->addRobot(robot(), {0.0, 5.0, 0.0}, {1.0, 5.0, 0.0});
    return problem;
}

std::vector<comotion::Path> makeReuseSeeds(std::size_t second_arrival) {
    return {
        makeTimedPath({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, 4),
        makeTimedPath({0.0, 5.0, 0.0}, {1.0, 5.0, 0.0}, second_arrival),
    };
}

bool testSeedPathReuseIsInclusiveAtTargetAndSelectiveAboveIt() {
    const auto reusable_seeds = makeReuseSeeds(6);
    std::vector<comotion::Path> working_paths;
    ArcProbe all_reused;
    if (!expectTrue(
            "all qualifying seeds initialize successfully with zero budget",
            all_reused.initializeFromSeeds(makeReuseProblem(), reusable_seeds,
                                           10, 6, 0.0, working_paths)))
        return false;
    if (!expectTrue("path exactly at reuse threshold is reused",
                    all_reused.initiallyReplannedRobots().empty() &&
                        all_reused.numInitialPathsReused() == 2))
        return false;
    if (!expectTrue("reused seeds are retained verbatim",
                    working_paths.size() == reusable_seeds.size() &&
                        samePath(working_paths[0], reusable_seeds[0]) &&
                        samePath(working_paths[1], reusable_seeds[1])))
        return false;
    if (!expectTrue("reused arrivals drive exact team metrics",
                    all_reused.sumOfCostTimesteps().has_value() &&
                        *all_reused.sumOfCostTimesteps() == 10 &&
                        all_reused.makespanTimesteps().has_value() &&
                        *all_reused.makespanTimesteps() == 6))
        return false;

    ArcProbe one_selected;
    working_paths.clear();
    const bool completed = one_selected.initializeFromSeeds(
        makeReuseProblem(), makeReuseSeeds(7), 10, 6, 0.0, working_paths);
    if (!expectTrue("target-plus-one seed is not treated as reusable",
                    !completed && one_selected.numInitialPathsReused() == 1))
        return false;
    return expectTrue(
        "only target-plus-one robot is selected for replanning",
        one_selected.initiallySelectedForReplanningRobots() ==
                std::vector<int>{1} &&
            one_selected.initiallyReplanAttemptedRobots().empty() &&
            one_selected.initiallyReplannedRobots().empty());
}

bool testRepairHistoryPartnerIsForcedPastReuseThreshold() {
    const auto reusable_seeds = makeReuseSeeds(6);
    std::vector<comotion::Path> working_paths;
    ArcProbe probe;
    const bool completed = probe.initializeFromSeeds(
        makeReuseProblem(), reusable_seeds, 10, 6, 0.0, working_paths, {1});

    if (!expectTrue(
            "forced repair-history partner is selected despite reuse threshold",
            !completed &&
                probe.initiallySelectedForReplanningRobots() ==
                    std::vector<int>{1}))
        return false;
    if (!expectTrue("forced partner is identified independently of bound violations",
                    probe.initiallyForcedReplanningRobots() ==
                        std::vector<int>{1}))
        return false;
    if (!expectTrue("only the unforced qualifying path is reused",
                    probe.numInitialPathsReused() == 1))
        return false;
    return expectTrue(
        "zero budget stops before attempting the forced path deterministically",
        probe.initiallyReplanAttemptedRobots().empty() &&
            probe.initiallyReplannedRobots().empty());
}

bool testSelectedPathIsReplannedAgainstGlobalBound() {
    constexpr std::uint64_t global_bound = 10;
    constexpr std::uint64_t reuse_bound = global_bound - 1;
    comotion::seedOmplGlobalFromUserPlanningSeed(73);

    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(1);
    problem->setVmax(1.0);
    problem->addRobot(
        std::make_shared<comotion::FlyingSphere>(0.1, -1000.0, 1000.0),
        {0.0, 0.0, 0.0}, {9.6, 0.0, 0.0});
    const std::vector<comotion::Path> seeds = {
        makeTimedPath({0.0, 0.0, 0.0}, {9.6, 0.0, 0.0}, global_bound),
    };

    ArcProbe probe;
    probe.setPlanningSeed(73);
    probe.setSimplifyInitialSolutions(false);
    std::vector<comotion::Path> working_paths;
    if (!expectTrue(
            "seed above reuse threshold can replan at inclusive global bound",
            probe.initializeFromSeeds(problem, seeds, global_bound,
                                      reuse_bound, 0.5, working_paths)))
        return false;
    if (!expectTrue("global-bound path was selected, attempted, and replaced",
                    probe.numInitialPathsReused() == 0 &&
                        probe.initiallySelectedForReplanningRobots() ==
                            std::vector<int>{0} &&
                        probe.initiallyReplanAttemptedRobots() ==
                            std::vector<int>{0} &&
                        probe.initiallyReplannedRobots() ==
                            std::vector<int>{0}))
        return false;
    return expectTrue("minimum discrete arrival may equal inclusive global B",
                      working_paths.size() == 1 &&
                          working_paths[0].arrival_timestep() == global_bound &&
                          probe.makespanTimesteps().has_value() &&
                          *probe.makespanTimesteps() == global_bound);
}

} // namespace

int main() {
    if (!testAOArcToggleDefaultsAndRoundTrips())
        return 1;
    if (!testRepairPartnerExpansionUsesExactBreadthFirstDepth())
        return 1;
    if (!testRandomFullRestartDecisionsAreBoundedAndReplayable())
        return 1;
    if (!testAcceptedRepairHistoryAppendsOnlyWhenIncumbentPathsRemain())
        return 1;
    if (!testStrictDiscreteReuseTarget())
        return 1;
    if (!testLocalMakespanBoundUsesDirectPrefixAndSuffix())
        return 1;
    if (!testSelectiveInitialPairFrontiersAndRepairReset())
        return 1;
    if (!testSeedPathReuseIsInclusiveAtTargetAndSelectiveAboveIt())
        return 1;
    if (!testRepairHistoryPartnerIsForcedPastReuseThreshold())
        return 1;
    if (!testSelectedPathIsReplannedAgainstGlobalBound())
        return 1;
    return 0;
}
