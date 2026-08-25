#include "comotion/collision/ConflictChecker.h"
#include "comotion/planning/MultiRobotProblem.h"
#include "comotion/planning/ParallelARC.h"
#include "comotion/planning/PlanningRng.h"
#include "comotion/robot/FlyingSphere.h"

#include <nlohmann/json.hpp>

#include <cstdint>
#include <iostream>
#include <memory>
#include <sstream>
#include <set>
#include <vector>

namespace ob = ompl::base;

namespace {

bool expectTrue(const char *label, bool condition) {
    if (!condition) {
        std::cerr << "parallel_arc_regression: " << label << "\n";
        return false;
    }
    return true;
}

std::shared_ptr<comotion::RobotModel> makeSphereRobot() {
    return std::make_shared<comotion::FlyingSphere>(1.0, -20.0, 20.0);
}

class ParallelArcProbe : public comotion::ParallelARC {
public:
    using BatchConflictTask = comotion::ParallelARC::BatchConflictTask;
    using comotion::ARC::applyConflictScanProgress;
    using comotion::ARC::conflictScanOptions;
    using comotion::ARC::initializeConflictScanStarts;
    using comotion::ARC::resetConflictScanStartsForRobots;

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

    std::vector<BatchConflictTask>
    select(const std::vector<comotion::SubproblemConflict> &conflicts) const {
        return selectConflictBatch(conflicts);
    }

    const std::vector<int> &scanStarts() const {
        return pair_conflict_scan_start_t_;
    }
};

bool testSharedExpansionPreservesWindowAndCascades() {
    ParallelArcProbe probe;
    probe.setInitialWindow(10);
    probe.setConflictSelectionStrategy(
        comotion::ParallelArcConflictSelectionStrategy::Greedy);
    probe.recordHistory({0, 1, 4}, 20, 40);
    probe.recordHistory({2, 3, 4}, 35, 50);

    const auto conflict = probe.expandConflict(comotion::Conflict{0, 1, 30});
    if (!expectTrue("shared expansion cascades through intersecting windows",
                    conflict.robots == std::vector<int>({0, 1, 2, 3, 4})))
        return false;
    return expectTrue("shared expansion keeps original conflict window",
                      conflict.window_begin_t == 20 &&
                          conflict.window_end_t == 40);
}

bool testRecursiveHistoryClosureCreatesOverlapAtMatchingTimestep() {
    ParallelArcProbe probe;
    probe.recordHistory({0, 1}, 10, 20);
    probe.recordHistory({1, 2}, 10, 20);

    return expectTrue("recursive history closure includes chained robot",
                      probe.expandTeamWindow(0, 3, 15, 15) ==
                          std::vector<int>({0, 1, 2, 3}));
}

bool testRecursiveHistoryClosureSkipsUnrelatedWindows() {
    ParallelArcProbe probe;
    probe.recordHistory({0, 1}, 10, 20);
    probe.recordHistory({1, 2}, 30, 40);

    return expectTrue("recursive history closure skips unrelated window",
                      probe.expandTeamWindow(0, 3, 15, 15) ==
                          std::vector<int>({0, 1, 3}));
}

bool testParallelArcSolvesIndependentConflicts() {
    comotion::seedOmplGlobalFromUserPlanningSeed(7);

    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(8);
    problem->setVmax(1.0);

    problem->addRobot(makeSphereRobot(), {-4.0, 0.0, 0.0}, {4.0, 0.0, 0.0});
    problem->addRobot(makeSphereRobot(), {4.0, 0.0, 0.0}, {-4.0, 0.0, 0.0});
    problem->addRobot(makeSphereRobot(), {-4.0, 8.0, 0.0}, {4.0, 8.0, 0.0});
    problem->addRobot(makeSphereRobot(), {4.0, 8.0, 0.0}, {-4.0, 8.0, 0.0});

    comotion::ParallelARC planner;
    planner.setPlanningSeed(7);
    planner.setProblem(problem);
    planner.setWorkerProcesses(2);
    planner.setParallelStrategy(
        comotion::ParallelArcParallelStrategy::Synchronous);
    planner.setConflictSelectionStrategy(
        comotion::ParallelArcConflictSelectionStrategy::Greedy);
    planner.setInitialWindow(12);
    planner.setExpansionStep(12);
    planner.setUseCspaceBounds(true);
    planner.setCspaceBoundMargin(2.0f);
    planner.setMinCspaceBoundRange(2.0);
    planner.setVisualizationTraceEnabled(true);

    const auto status = planner.solve(10.0);
    if (!expectTrue("ParallelARC exact solution",
                    status == ob::PlannerStatus::EXACT_SOLUTION)) {
        return false;
    }

    const auto paths = planner.getSolutionPaths();
    if (!expectTrue("ParallelARC visualization trace is captured",
                    !planner.visualizationTrace().empty() &&
                        planner.visualizationTrace().back()
                            .conflict_scan_completed &&
                        planner.visualizationTrace().back().conflicts.empty()))
        return false;
    comotion::ConflictChecker checker(problem->collisionChecker());
    auto ptrs = problem->robotModelPtrs();
    const auto conflict = checker.findConflict(paths, ptrs);
    if (!expectTrue("ParallelARC final paths conflict-free", !conflict.has_value()))
        return false;

    const auto planner_json = planner.plannerStatsJson();
    const auto total_expansions =
        planner_json.value("temporal_expansions", std::uint64_t{0});
    const auto initial_valid_expansions = planner_json.value(
        "initial_valid_temporal_expansions", std::uint64_t{0});
    const auto main_expansions =
        planner_json.value("main_temporal_expansions", std::uint64_t{0});
    if (!expectTrue(
            "ParallelARC phase expansion counters sum to total",
            total_expansions ==
                initial_valid_expansions + main_expansions)) {
        return false;
    }
    if (!expectTrue("ParallelARC default conflict finder is segment-parallel",
                    planner_json.value("parallel_arc_conflict_find_mode", "") ==
                        "segment_parallel")) {
        return false;
    }
    if (!expectTrue("ParallelARC default initial parallel planning on",
                    planner_json.value(
                        "parallel_arc_parallel_initial_individual_plans",
                        false))) {
        return false;
    }
    if (!expectTrue("ParallelARC default initial worker count",
                    planner_json.value(
                        "parallel_arc_initial_individual_workers", 0) == 2)) {
        return false;
    }
    if (!expectTrue("ParallelARC default initial solution OR off",
                    !planner_json.value("parallel_arc_initial_solution_or",
                                        true))) {
        return false;
    }
    if (!expectTrue("ParallelARC default initial OR parallelism off",
                    !planner_json.value(
                        "parallel_arc_initial_individual_or_parallelism",
                        true))) {
        return false;
    }
    if (!expectTrue("ParallelARC default initial duplicate attempt count",
                    planner_json.value(
                        "parallel_arc_initial_individual_duplicate_attempt_count",
                        1) == 0)) {
        return false;
    }
    if (!expectTrue("ParallelARC default conflict finder horizon",
                    planner_json.value("parallel_arc_conflict_find_horizon",
                                       0) == 400)) {
        return false;
    }
    if (!expectTrue("ParallelARC default conflict finder worker count",
                    planner_json.value("parallel_arc_conflict_find_workers",
                                       0) == 2)) {
        return false;
    }
    if (!expectTrue("ParallelARC default conflict finder ipc",
                    planner_json.value("parallel_arc_conflict_find_ipc", "")
                        == "pipes")) {
        return false;
    }
    if (!expectTrue(
            "ParallelARC default conflict finder process lifecycle",
            planner_json
                .value("parallel_arc_conflict_find_process_lifecycle", "")
                == "per_find_call")) {
        return false;
    }
    if (!expectTrue(
            "ParallelARC omits fixed conflict assignment metadata",
            !planner_json.contains(
                "parallel_arc_conflict_find_assignment_setting") &&
                !planner_json.contains(
                    "parallel_arc_conflict_find_assignment_strategy") &&
                !planner_json.contains(
                    "parallel_arc_conflict_find_logical_bucket_count"))) {
        return false;
    }
    if (!expectTrue("ParallelARC default optimistic conflict batch mode",
                    planner_json.value("parallel_arc_conflict_batch_mode",
                                       "") == "optimistic_independent")) {
        return false;
    }
    if (!expectTrue("ParallelARC repair OR parallelism enabled",
                    planner_json.value("parallel_arc_repair_or_parallelism",
                                       false))) {
        return false;
    }
    if (!expectTrue("ParallelARC repair result transport is pipes",
                    planner_json.value("parallel_arc_result_transport", "") ==
                        "pipes")) {
        return false;
    }
    if (!expectTrue(
            "ParallelARC repair worker lifecycle is persistent",
            planner_json.value("parallel_arc_repair_process_lifecycle", "") ==
                "persistent_batch_pool")) {
        return false;
    }
    if (!expectTrue("ParallelARC repair ipc is pipes",
                    planner_json.value("parallel_arc_repair_ipc", "") ==
                        "pipes")) {
        return false;
    }
    if (!expectTrue(
            "ParallelARC repair cancellation is cooperative",
            planner_json.value("parallel_arc_repair_cancellation", "") ==
                "cooperative_signal_with_terminate_fallback")) {
        return false;
    }
    if (!expectTrue(
            "ParallelARC repair assignment strategy",
            planner_json.value("parallel_arc_repair_assignment_strategy", "") ==
                "round_robin_active_subproblems")) {
        return false;
    }
    return true;
}

bool testParallelArcInitialSolutionOrCanBeEnabled() {
    comotion::seedOmplGlobalFromUserPlanningSeed(37);

    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(8);
    problem->setVmax(1.0);

    problem->addRobot(makeSphereRobot(), {-4.0, 0.0, 0.0}, {4.0, 0.0, 0.0});

    comotion::ParallelARC planner;
    planner.setPlanningSeed(37);
    planner.setProblem(problem);
    planner.setWorkerProcesses(3);
    planner.setInitialSolutionOr(true);

    const auto status = planner.solve(10.0);
    if (!expectTrue("ParallelARC initial solution OR exact solution",
                    status == ob::PlannerStatus::EXACT_SOLUTION)) {
        return false;
    }

    const auto planner_json = planner.plannerStatsJson();
    if (!expectTrue("ParallelARC initial solution OR reports enabled",
                    planner_json.value("parallel_arc_initial_solution_or",
                                       false))) {
        return false;
    }
    if (!expectTrue("ParallelARC initial solution OR uses all workers",
                    planner_json.value(
                        "parallel_arc_initial_individual_workers", 0) == 3)) {
        return false;
    }
    if (!expectTrue("ParallelARC initial solution OR launches duplicates",
                    planner_json.value(
                        "parallel_arc_initial_individual_duplicate_attempt_count",
                        0) == 2)) {
        return false;
    }
    if (!expectTrue("ParallelARC initial solution OR parallelism active",
                    planner_json.value(
                        "parallel_arc_initial_individual_or_parallelism",
                        false))) {
        return false;
    }
    return expectTrue(
        "ParallelARC initial solution OR assignment strategy",
        planner_json.value(
            "parallel_arc_initial_individual_assignment_strategy", "") ==
            "dynamic_robot_queue_with_or");
}

bool testParallelArcInnerOrRepairsSingleConflict() {
    comotion::seedOmplGlobalFromUserPlanningSeed(29);

    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(8);
    problem->setVmax(1.0);

    problem->addRobot(makeSphereRobot(), {-4.0, 0.0, 0.0}, {4.0, 0.0, 0.0});
    problem->addRobot(makeSphereRobot(), {4.0, 0.0, 0.0}, {-4.0, 0.0, 0.0});

    comotion::ParallelARC planner;
    planner.setPlanningSeed(29);
    planner.setProblem(problem);
    planner.setWorkerProcesses(2);
    planner.setParallelStrategy(
        comotion::ParallelArcParallelStrategy::Synchronous);
    planner.setConflictSelectionStrategy(
        comotion::ParallelArcConflictSelectionStrategy::Greedy);
    planner.setInitialWindow(12);
    planner.setExpansionStep(12);
    planner.setUseCspaceBounds(true);
    planner.setCspaceBoundMargin(2.0f);
    planner.setMinCspaceBoundRange(2.0);

    const auto status = planner.solve(10.0);
    if (!expectTrue("ParallelARC inner OR exact solution",
                    status == ob::PlannerStatus::EXACT_SOLUTION)) {
        return false;
    }

    comotion::ConflictChecker checker(problem->collisionChecker());
    auto ptrs = problem->robotModelPtrs();
    const auto conflict = checker.findConflict(planner.getSolutionPaths(), ptrs);
    if (!expectTrue("ParallelARC inner OR final paths conflict-free",
                    !conflict.has_value())) {
        return false;
    }

    const auto planner_json = planner.plannerStatsJson();
    if (!expectTrue("ParallelARC inner OR stats enabled",
                    planner_json.value("parallel_arc_repair_or_parallelism",
                                       false))) {
        return false;
    }
    if (!expectTrue(
            "ParallelARC inner OR duplicate attempts enabled",
            planner_json.value("parallel_arc_repair_duplicate_attempts",
                               false))) {
        return false;
    }
    if (!expectTrue(
            "ParallelARC inner OR assignment strategy",
            planner_json.value("parallel_arc_repair_assignment_strategy", "") ==
                "round_robin_active_subproblems")) {
        return false;
    }
    if (!expectTrue("ParallelARC inner OR repair lifecycle",
                    planner_json.value("parallel_arc_repair_process_lifecycle",
                                       "") == "persistent_batch_pool")) {
        return false;
    }
    if (!expectTrue("ParallelARC inner OR pipe transport",
                    planner_json.value("parallel_arc_result_transport", "") ==
                        "pipes")) {
        return false;
    }
    if (!expectTrue("ParallelARC inner OR cooperative cancellation metadata",
                    planner_json.value("parallel_arc_repair_cancellation", "") ==
                        "cooperative_signal_with_terminate_fallback")) {
        return false;
    }

    return true;
}

bool testParallelArcRepairDuplicateAttemptsCanBeDisabled() {
    comotion::seedOmplGlobalFromUserPlanningSeed(31);

    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(8);
    problem->setVmax(1.0);

    problem->addRobot(makeSphereRobot(), {-4.0, 0.0, 0.0}, {4.0, 0.0, 0.0});
    problem->addRobot(makeSphereRobot(), {4.0, 0.0, 0.0}, {-4.0, 0.0, 0.0});

    comotion::ParallelARC planner;
    planner.setPlanningSeed(31);
    planner.setProblem(problem);
    planner.setWorkerProcesses(2);
    planner.setRepairDuplicateAttempts(false);
    planner.setParallelStrategy(
        comotion::ParallelArcParallelStrategy::Synchronous);
    planner.setConflictSelectionStrategy(
        comotion::ParallelArcConflictSelectionStrategy::Greedy);
    planner.setInitialWindow(12);
    planner.setExpansionStep(12);
    planner.setUseCspaceBounds(true);
    planner.setCspaceBoundMargin(2.0f);
    planner.setMinCspaceBoundRange(2.0);

    const auto status = planner.solve(10.0);
    if (!expectTrue("ParallelARC no duplicate repair exact solution",
                    status == ob::PlannerStatus::EXACT_SOLUTION)) {
        return false;
    }

    comotion::ConflictChecker checker(problem->collisionChecker());
    auto ptrs = problem->robotModelPtrs();
    const auto conflict = checker.findConflict(planner.getSolutionPaths(), ptrs);
    if (!expectTrue("ParallelARC no duplicate repair final paths conflict-free",
                    !conflict.has_value())) {
        return false;
    }

    const auto planner_json = planner.plannerStatsJson();
    if (!expectTrue(
            "ParallelARC no duplicate repair OR stats disabled",
            !planner_json.value("parallel_arc_repair_or_parallelism", true))) {
        return false;
    }
    if (!expectTrue(
            "ParallelARC no duplicate repair duplicate attempts disabled",
            !planner_json.value("parallel_arc_repair_duplicate_attempts",
                                true))) {
        return false;
    }
    if (!expectTrue(
            "ParallelARC no duplicate repair assignment strategy",
            planner_json.value("parallel_arc_repair_assignment_strategy", "") ==
                "round_robin_one_live_attempt_per_subproblem")) {
        return false;
    }
    return true;
}

bool testParallelArcSequentialInitialIndividualPlansOverride() {
    comotion::seedOmplGlobalFromUserPlanningSeed(19);

    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(8);
    problem->setVmax(1.0);

    problem->addRobot(makeSphereRobot(), {-4.0, 0.0, 0.0}, {4.0, 0.0, 0.0});
    problem->addRobot(makeSphereRobot(), {4.0, 0.0, 0.0}, {-4.0, 0.0, 0.0});
    problem->addRobot(makeSphereRobot(), {-4.0, 8.0, 0.0}, {4.0, 8.0, 0.0});
    problem->addRobot(makeSphereRobot(), {4.0, 8.0, 0.0}, {-4.0, 8.0, 0.0});

    comotion::ParallelARC planner;
    planner.setPlanningSeed(19);
    planner.setProblem(problem);
    planner.setWorkerProcesses(2);
    planner.setParallelizeInitialIndividualPlans(false);
    planner.setParallelStrategy(
        comotion::ParallelArcParallelStrategy::Synchronous);
    planner.setConflictSelectionStrategy(
        comotion::ParallelArcConflictSelectionStrategy::Greedy);
    planner.setInitialWindow(12);
    planner.setExpansionStep(12);
    planner.setUseCspaceBounds(true);
    planner.setCspaceBoundMargin(2.0f);
    planner.setMinCspaceBoundRange(2.0);

    const auto status = planner.solve(10.0);
    if (!expectTrue("ParallelARC sequential initial override exact solution",
                    status == ob::PlannerStatus::EXACT_SOLUTION)) {
        return false;
    }

    comotion::ConflictChecker checker(problem->collisionChecker());
    auto ptrs = problem->robotModelPtrs();
    const auto conflict = checker.findConflict(planner.getSolutionPaths(), ptrs);
    if (!expectTrue("ParallelARC sequential initial override final paths conflict-free",
                    !conflict.has_value())) {
        return false;
    }

    const auto planner_json = planner.plannerStatsJson();
    if (!expectTrue("ParallelARC sequential initial override reports disabled",
                    !planner_json.value(
                        "parallel_arc_parallel_initial_individual_plans",
                        true))) {
        return false;
    }
    if (!expectTrue("ParallelARC sequential initial override worker count",
                    planner_json.value(
                        "parallel_arc_initial_individual_workers", -1) == 0)) {
        return false;
    }
    if (!expectTrue("ParallelARC sequential initial override lifecycle",
                    planner_json.value(
                        "parallel_arc_initial_individual_process_lifecycle",
                        "unset") == "")) {
        return false;
    }
    if (!expectTrue("ParallelARC sequential initial override assignment",
                    planner_json.value(
                        "parallel_arc_initial_individual_assignment_strategy",
                        "unset") == "")) {
        return false;
    }
    if (!expectTrue("ParallelARC sequential initial override payload",
                    planner_json.value(
                        "parallel_arc_initial_individual_payload", "unset") ==
                        "")) {
        return false;
    }
    return true;
}

bool testParallelArcSegmentParallelConflictFindSolves() {
    comotion::seedOmplGlobalFromUserPlanningSeed(11);

    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(8);
    problem->setVmax(1.0);

    problem->addRobot(makeSphereRobot(), {-4.0, 0.0, 0.0}, {4.0, 0.0, 0.0});
    problem->addRobot(makeSphereRobot(), {4.0, 0.0, 0.0}, {-4.0, 0.0, 0.0});
    problem->addRobot(makeSphereRobot(), {-4.0, 8.0, 0.0}, {4.0, 8.0, 0.0});
    problem->addRobot(makeSphereRobot(), {4.0, 8.0, 0.0}, {-4.0, 8.0, 0.0});

    comotion::ParallelARC planner;
    planner.setPlanningSeed(11);
    planner.setProblem(problem);
    planner.setWorkerProcesses(2);
    planner.setParallelStrategy(
        comotion::ParallelArcParallelStrategy::Synchronous);
    planner.setConflictSelectionStrategy(
        comotion::ParallelArcConflictSelectionStrategy::Greedy);
    planner.setConflictFindMode(
        comotion::ParallelArcConflictFindMode::SegmentParallel);
    planner.setConflictBatchMode(
        comotion::InterRobotConflictBatchMode::IndependentOnly);
    planner.setConflictFindHorizon(4);
    planner.setInitialWindow(12);
    planner.setExpansionStep(12);
    planner.setUseCspaceBounds(true);
    planner.setCspaceBoundMargin(2.0f);
    planner.setMinCspaceBoundRange(2.0);

    const auto status = planner.solve(10.0);
    if (!expectTrue("ParallelARC segment-parallel exact solution",
                    status == ob::PlannerStatus::EXACT_SOLUTION)) {
        return false;
    }

    comotion::ConflictChecker checker(problem->collisionChecker());
    auto ptrs = problem->robotModelPtrs();
    const auto conflict = checker.findConflict(planner.getSolutionPaths(), ptrs);
    if (!expectTrue("ParallelARC segment-parallel final paths conflict-free",
                    !conflict.has_value())) {
        return false;
    }

    const auto planner_json = planner.plannerStatsJson();
    if (!expectTrue("ParallelARC segment-parallel stats mode",
                    planner_json.value("parallel_arc_conflict_find_mode", "") ==
                        "segment_parallel")) {
        return false;
    }
    if (!expectTrue("ParallelARC segment-parallel stats horizon",
                    planner_json.value("parallel_arc_conflict_find_horizon",
                                       0) == 4)) {
        return false;
    }
    if (!expectTrue("ParallelARC segment-parallel stats workers",
                    planner_json.value("parallel_arc_conflict_find_workers",
                                       0) == 2)) {
        return false;
    }
    if (!expectTrue("ParallelARC segment-parallel stats ipc",
                    planner_json.value("parallel_arc_conflict_find_ipc", "") ==
                        "pipes")) {
        return false;
    }
    if (!expectTrue(
            "ParallelARC segment-parallel stats process lifecycle",
            planner_json.value("parallel_arc_conflict_find_process_lifecycle",
                               "") == "per_find_call")) {
        return false;
    }
    if (!expectTrue("ParallelARC segment-parallel stats conflict batch mode",
                    planner_json.value("parallel_arc_conflict_batch_mode",
                                       "") == "independent_only")) {
        return false;
    }
    return true;
}

bool testParallelArcVampSegmentParallelConflictFindSolves() {
#if COMOTION_HAVE_VAMP
    comotion::seedOmplGlobalFromUserPlanningSeed(13);

    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Vamp);
    problem->setResolution(8);
    problem->setVmax(1.0);

    problem->addRobot(makeSphereRobot(), {-4.0, 0.0, 0.0}, {4.0, 0.0, 0.0});
    problem->addRobot(makeSphereRobot(), {4.0, 0.0, 0.0}, {-4.0, 0.0, 0.0});
    problem->addRobot(makeSphereRobot(), {-4.0, 8.0, 0.0}, {4.0, 8.0, 0.0});
    problem->addRobot(makeSphereRobot(), {4.0, 8.0, 0.0}, {-4.0, 8.0, 0.0});

    comotion::ParallelARC planner;
    planner.setPlanningSeed(13);
    planner.setProblem(problem);
    planner.setWorkerProcesses(2);
    planner.setParallelStrategy(
        comotion::ParallelArcParallelStrategy::Synchronous);
    planner.setConflictSelectionStrategy(
        comotion::ParallelArcConflictSelectionStrategy::Greedy);
    planner.setConflictFindMode(
        comotion::ParallelArcConflictFindMode::SegmentParallel);
    planner.setConflictFindHorizon(4);
    planner.setInitialWindow(12);
    planner.setExpansionStep(12);
    planner.setUseCspaceBounds(true);
    planner.setCspaceBoundMargin(2.0f);
    planner.setMinCspaceBoundRange(2.0);

    const auto status = planner.solve(10.0);
    if (!expectTrue("ParallelARC VAMP segment-parallel exact solution",
                    status == ob::PlannerStatus::EXACT_SOLUTION)) {
        return false;
    }

    comotion::ConflictChecker checker(problem->collisionChecker());
    auto ptrs = problem->robotModelPtrs();
    const auto conflict = checker.findConflict(planner.getSolutionPaths(), ptrs);
    if (!expectTrue(
            "ParallelARC VAMP segment-parallel final paths conflict-free",
            !conflict.has_value())) {
        return false;
    }

    const auto planner_json = planner.plannerStatsJson();
    if (!expectTrue(
            "ParallelARC VAMP segment-parallel stats mode",
            planner_json.value("parallel_arc_conflict_find_mode", "") ==
                "segment_parallel")) {
        return false;
    }
    if (!expectTrue("ParallelARC VAMP segment-parallel stats horizon",
                    planner_json.value("parallel_arc_conflict_find_horizon",
                                       0) == 4)) {
        return false;
    }
    if (!expectTrue("ParallelARC VAMP segment-parallel stats workers",
                    planner_json.value("parallel_arc_conflict_find_workers",
                                       0) == 2)) {
        return false;
    }

    return expectTrue("ParallelARC VAMP segment-parallel stats ipc",
                      planner_json.value("parallel_arc_conflict_find_ipc",
                                         "") == "pipes");
#else
    return true;
#endif
}

bool testArcPerPairScanRestartState() {
    ParallelArcProbe probe;
    probe.initializeConflictScanStarts(4);
    probe.applyConflictScanProgress({50, 50, 50, 50, 50, 50});
    probe.resetConflictScanStartsForRobots({0, 1}, 10);

    const auto &starts = probe.scanStarts();
    if (!expectTrue("ARC per-pair scan start size", starts.size() == 6))
        return false;
    if (!expectTrue("ARC per-pair scan start repaired pair reset",
                    starts[comotion::pairFrontierIndex(0, 1, 4)] == 10))
        return false;
    if (!expectTrue("ARC per-pair scan start adjacent pairs reset",
                    starts[comotion::pairFrontierIndex(0, 2, 4)] == 10 &&
                        starts[comotion::pairFrontierIndex(0, 3, 4)] == 10 &&
                        starts[comotion::pairFrontierIndex(1, 2, 4)] == 10 &&
                        starts[comotion::pairFrontierIndex(1, 3, 4)] == 10)) {
        return false;
    }
    if (!expectTrue("ARC per-pair scan start unrelated pair preserved",
                    starts[comotion::pairFrontierIndex(2, 3, 4)] == 50)) {
        return false;
    }

    const auto options = probe.conflictScanOptions();
    return expectTrue("ARC conflict scan options expose per-pair starts",
                      options.t_begin == 10 &&
                          options.per_pair_t_begin ==
                              std::vector<std::size_t>{10, 10, 10, 10, 10, 50});
}

bool testParallelArcPerPairBatchRestartState() {
    ParallelArcProbe probe;
    probe.initializeConflictScanStarts(6);
    probe.applyConflictScanProgress(
        std::vector<std::size_t>(comotion::pairFrontierSize(6), 50));
    probe.resetConflictScanStartsForRobots({0, 1}, 10);
    probe.resetConflictScanStartsForRobots({2, 3}, 40);

    const auto &starts = probe.scanStarts();
    if (!expectTrue("ParallelARC per-pair scan start group A pair",
                    starts[comotion::pairFrontierIndex(0, 1, 6)] == 10))
        return false;
    if (!expectTrue("ParallelARC per-pair scan start group B pair",
                    starts[comotion::pairFrontierIndex(2, 3, 6)] == 40))
        return false;
    if (!expectTrue("ParallelARC per-pair scan start cross pairs use min reset",
                    starts[comotion::pairFrontierIndex(0, 2, 6)] == 10 &&
                        starts[comotion::pairFrontierIndex(1, 3, 6)] == 10)) {
        return false;
    }
    if (!expectTrue("ParallelARC per-pair scan start later repair pairs reset",
                    starts[comotion::pairFrontierIndex(2, 4, 6)] == 40 &&
                        starts[comotion::pairFrontierIndex(3, 5, 6)] == 40)) {
        return false;
    }
    if (!expectTrue("ParallelARC per-pair scan start untouched pair preserved",
                    starts[comotion::pairFrontierIndex(4, 5, 6)] == 50)) {
        return false;
    }

    const auto options = probe.conflictScanOptions();
    return expectTrue("ParallelARC conflict scan options preserve later frontiers",
                      options.t_begin == 10 &&
                          options.per_pair_t_begin ==
                              std::vector<std::size_t>{
                                  10, 10, 10, 10, 10,
                                  10, 10, 10, 10,
                                  40, 40, 40,
                                  40, 40,
                                  50});
}

} // namespace

int main() {
    if (!testSharedExpansionPreservesWindowAndCascades())
        return 1;
    if (!testRecursiveHistoryClosureCreatesOverlapAtMatchingTimestep())
        return 1;
    if (!testRecursiveHistoryClosureSkipsUnrelatedWindows())
        return 1;
    if (!testArcPerPairScanRestartState())
        return 1;
    if (!testParallelArcPerPairBatchRestartState())
        return 1;
    if (!testParallelArcSolvesIndependentConflicts())
        return 1;
    if (!testParallelArcInitialSolutionOrCanBeEnabled())
        return 1;
    if (!testParallelArcInnerOrRepairsSingleConflict())
        return 1;
    if (!testParallelArcRepairDuplicateAttemptsCanBeDisabled())
        return 1;
    if (!testParallelArcSequentialInitialIndividualPlansOverride())
        return 1;
    if (!testParallelArcSegmentParallelConflictFindSolves())
        return 1;
    if (!testParallelArcVampSegmentParallelConflictFindSolves())
        return 1;

    std::cout << "parallel_arc_regression: OK\n";
    return 0;
}
