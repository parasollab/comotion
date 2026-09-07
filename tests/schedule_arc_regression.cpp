#include "comotion/planning/ARC.h"
#include "comotion/planning/MultiRobotProblem.h"
#include "comotion/planning/ScheduleARC.h"
#include "comotion/robot/FlyingSphere.h"

#include <Eigen/Core>
#include <iostream>
#include <memory>
#include <type_traits>
#include <vector>

namespace {

bool expect(bool condition, const char *message) {
    if (!condition)
        std::cerr << "schedule_arc_regression: " << message << '\n';
    return condition;
}

std::shared_ptr<comotion::FlyingSphere> makeRobot() {
    return std::make_shared<comotion::FlyingSphere>(
        0.2, std::vector<double>{-3.0, -3.0, -3.0},
        std::vector<double>{3.0, 3.0, 3.0});
}

std::shared_ptr<comotion::MultiRobotProblem> makeProblem() {
    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(8);
    problem->setVmax(1.0);
    return problem;
}

comotion::ScheduleArcMotion sparseMotion() {
    comotion::ScheduleArcMotion motion;
    motion.label = "sparse";
    motion.robot_name = "robot";
    motion.model = makeRobot();
    motion.path.push_back({-1.0, 0.0, 0.0});
    motion.path.push_back({1.0, 0.0, 0.0});
    motion.path.set_waypoint_timesteps({0, 4});
    motion.start_t = 10;
    motion.end_t = 14;
    motion.moving_entities = {"robot"};
    return motion;
}

bool sparseTimestepsAndProblemEnvironmentAreValidated() {
    auto problem = makeProblem();
    problem->setObstacles({comotion::ObstacleSphere{
        Eigen::Vector3d{0.0, 0.0, 0.0}, 0.15}});

    comotion::ScheduleARC planner;
    planner.setProblem(problem);
    planner.setMotions({sparseMotion()});

    const auto &normalized = planner.motions().front().path;
    const auto conflict = planner.findFirstConflict();
    bool ok = true;
    ok &= expect(normalized.size() == 5,
                 "sparse path was not normalized to its schedule duration");
    ok &= expect(normalized.has_implicit_dense_timesteps(),
                 "normalized schedule path is not explicitly marked dense");
    ok &= expect(normalized[2] == std::vector<double>({0.0, 0.0, 0.0}),
                 "explicit waypoint times were not used during normalization");
    ok &= expect(conflict.has_value(),
                 "base problem obstacle collision was not detected");
    if (conflict) {
        ok &= expect(conflict->type ==
                         comotion::ScheduleArcConflictType::EnvironmentCollision,
                     "base obstacle collision has the wrong conflict type");
        ok &= expect(conflict->timestep == 12,
                     "base obstacle conflict has the wrong global timestep");
    }
    return ok;
}

bool currentPairPathScannerFindsScheduleConflict() {
    auto lhs = sparseMotion();
    lhs.label = "lhs";
    lhs.robot_name = "lhs";
    lhs.moving_entities = {"lhs"};
    auto rhs = sparseMotion();
    rhs.label = "rhs";
    rhs.robot_name = "rhs";
    rhs.moving_entities = {"rhs"};
    rhs.path = comotion::Path{};
    rhs.path.push_back({1.0, 0.0, 0.0});
    rhs.path.push_back({-1.0, 0.0, 0.0});
    rhs.path.set_waypoint_timesteps({0, 4});

    comotion::ScheduleARC planner;
    planner.setProblem(makeProblem());
    planner.setMotions({lhs, rhs});
    const auto conflict = planner.findFirstConflict();
    return expect(conflict &&
                      conflict->type ==
                          comotion::ScheduleArcConflictType::MovingCollision &&
                      conflict->timestep == 12,
                  "current pair-path scanner missed the first schedule collision");
}

bool currentArcConfigurationIsInherited() {
    static_assert(std::is_base_of_v<comotion::ARC, comotion::ScheduleARC>);
    comotion::ScheduleARC planner;
    planner.setExpansionPolicy(comotion::ARC::ExpansionPolicy::Exponential);
    planner.setInitialValidWindowExpansionPolicy(
        comotion::ARC::ExpansionPolicy::Logarithmic);
    planner.setLocalSolverMode(
        comotion::ARC::LocalSolverMode::PrioritizedStrrtOnly);
    planner.setLocalCompositeRrtUseMakespanMetric(true);
    planner.setUseCspaceBounds(true);
    return expect(
               planner.expansionPolicy() ==
                   comotion::ARC::ExpansionPolicy::Exponential,
               "ScheduleARC did not retain the current ARC expansion policy") &&
           expect(planner.localSolverMode() ==
                      comotion::ARC::LocalSolverMode::PrioritizedStrrtOnly,
                  "ScheduleARC did not retain the current ARC solver mode");
}

bool fixedScheduleHonorsGlobalMakespanBound() {
    auto problem = makeProblem();
    auto motion = sparseMotion();

    comotion::ScheduleARC planner;
    planner.setProblem(problem);
    planner.setMotions({motion});
    planner.setGlobalMakespanBoundTimesteps(13);
    const auto status = planner.solve(0.1);
    return expect(status != ompl::base::PlannerStatus::EXACT_SOLUTION,
                  "fixed schedule exceeding the ARC makespan bound was accepted");
}

comotion::Path densePath(const std::vector<std::vector<double>> &configs) {
    comotion::Path path;
    for (const auto &config : configs)
        path.push_back(config);
    path.markDenseTimestepsImplicit();
    return path;
}

bool coupledMotionsStayInTheRepairTeam() {
    auto problem = makeProblem();
    comotion::ScheduleArcMotion moving;
    moving.label = "coupled-moving";
    moving.robot_name = "moving";
    moving.model = makeRobot();
    moving.path = densePath({
        {-1.0, 0.0, 0.0}, {-0.75, 0.0, 0.0}, {-0.5, 0.0, 0.0},
        {-0.25, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.25, 0.0, 0.0},
        {0.5, 0.0, 0.0}, {0.75, 0.0, 0.0}, {1.0, 0.0, 0.0}});
    moving.start_t = 0;
    moving.end_t = 8;
    moving.coupling_id = 7;
    moving.moving_entities = {"moving"};

    comotion::ScheduleArcMotion partner;
    partner.label = "coupled-partner";
    partner.robot_name = "partner";
    partner.model = makeRobot();
    partner.path = densePath(std::vector<std::vector<double>>(
        9, std::vector<double>{0.0, 2.0, 0.0}));
    partner.start_t = 0;
    partner.end_t = 8;
    partner.coupling_id = 7;
    partner.moving_entities = {"partner"};

    comotion::ScheduleARC planner;
    planner.setProblem(problem);
    planner.setPlanningSeed(17);
    planner.setInitialWindow(8);
    planner.setLocalSolverMode(comotion::ARC::LocalSolverMode::CompositeRrtOnly);
    planner.setLocalSolveTimeLimit(2.0);
    planner.setVisualizationTraceEnabled(true);
    planner.setMotions({moving, partner});
    planner.setStationaryEntities({comotion::ScheduleArcStationaryEntity{
        "blocker", 0, 8,
        {comotion::ObstacleSphere{Eigen::Vector3d{0.0, 0.0, 0.0}, 0.15}},
        {}}});

    const auto status = planner.solve(3.0);
    const auto &stats = planner.plannerStatsJson();
    bool ok = true;
    ok &= expect(status == ompl::base::PlannerStatus::EXACT_SOLUTION,
                 "coupled schedule repair did not find an exact solution");
    ok &= expect(stats.value("max_repair_team_size", 0u) == 2,
                 "a coupled sibling was omitted from the repair team");
    ok &= expect(!planner.findFirstConflict().has_value(),
                 "coupled schedule still has an external conflict after repair");
    ok &= expect(!planner.visualizationTrace().empty() &&
                     !planner.visualizationTrace().front().repairs.empty(),
                 "schedule repair was not captured in the ARC visualization trace");
    return ok;
}

comotion::ScheduleArcRobotHold hold(const std::string &name,
    const std::vector<double> &configuration, std::size_t begin, std::size_t end) {
    comotion::ScheduleArcRobotHold result;
    result.robot_name = name;
    result.model = makeRobot();
    result.configuration = configuration;
    result.start_t = begin;
    result.end_t = end;
    return result;
}

bool fullOccupancyRejectsMissingCoverageAndDiscontinuity() {
    comotion::ScheduleARC planner;
    planner.setProblem(makeProblem());
    planner.setFullOccupancy({4, {"idle"}, {}});
    bool missing = false, gap = false, discontinuity = false, overlap = false;
    try { planner.findFirstConflict(); } catch (const std::invalid_argument &) { missing = true; }
    planner.setFullOccupancy({4, {"idle"}, {
        hold("idle", {0, 0, 0}, 0, 1), hold("idle", {0, 0, 0}, 2, 4)}});
    try { planner.findFirstConflict(); } catch (const std::invalid_argument &) { gap = true; }
    planner.setFullOccupancy({4, {"idle"}, {
        hold("idle", {0, 0, 0}, 0, 2), hold("idle", {1, 0, 0}, 2, 4)}});
    try { planner.findFirstConflict(); } catch (const std::invalid_argument &) { discontinuity = true; }
    planner.setFullOccupancy({4, {"idle"}, {
        hold("idle", {0, 0, 0}, 0, 3), hold("idle", {0, 0, 0}, 2, 4)}});
    try { planner.findFirstConflict(); } catch (const std::invalid_argument &) { overlap = true; }
    return expect(missing && gap && discontinuity && overlap,
                  "incomplete or contradictory full occupancy was accepted");
}

bool zeroMotionAndTerminalHoldsAreChecked() {
    comotion::ScheduleARC planner;
    planner.setProblem(makeProblem());
    planner.setFullOccupancy({0, {"left", "right"}, {
        hold("left", {0, 0, 0}, 0, 0), hold("right", {0.1, 0, 0}, 0, 0)}});
    const auto collision = planner.findFirstConflict();
    bool ok = expect(collision && collision->hold_i && collision->hold_j &&
                          collision->timestep == 0,
                      "zero-time held-held collision was ignored");
    ok &= expect(planner.solve(0.1) != ompl::base::PlannerStatus::EXACT_SOLUTION,
                 "colliding zero-motion schedule was accepted");
    planner.setFullOccupancy({12, {"idle"}, {hold("idle", {0, 0, 0}, 0, 12)}});
    planner.setStationaryEntities({{"object", 9, 12,
        {{Eigen::Vector3d{0, 0, 0}, 0.15}}, {}}});
    const auto terminal = planner.findFirstConflict();
    ok &= expect(terminal && terminal->hold_i && terminal->timestep == 9 &&
                      terminal->stationary_entity == "object",
                 "object collision after the last active motion was ignored");
    planner.setStationaryEntities({});
    ok &= expect(planner.solve(0.1) == ompl::base::PlannerStatus::EXACT_SOLUTION,
                 "valid zero-motion full occupancy did not solve");
    return ok;
}

bool movingHeldCollisionRepairsOnlyTheMovingRobot() {
    auto motion = sparseMotion();
    motion.start_t = 0;
    motion.end_t = 8;
    motion.robot_name = "moving";
    motion.moving_entities = {"moving"};
    comotion::ScheduleARC planner;
    planner.setProblem(makeProblem());
    planner.setPlanningSeed(17);
    planner.setInitialWindow(8);
    planner.setLocalSolverMode(comotion::ARC::LocalSolverMode::CompositeRrtOnly);
    planner.setLocalSolveTimeLimit(2.0);
    planner.setMotions({motion});
    planner.setFullOccupancy({8, {"moving", "waiting"}, {
        hold("waiting", {0, 0, 0}, 0, 8)}});
    const auto conflict = planner.findFirstConflict();
    bool ok = expect(conflict && conflict->motion_i == 0 && conflict->hold_j,
                     "moving-held robot collision was ignored");
    const auto status = planner.solve(3.0);
    ok &= expect(status == ompl::base::PlannerStatus::EXACT_SOLUTION,
                 "moving-held conflict could not be repaired with fixed context");
    ok &= expect(planner.motions().size() == 1 &&
                     planner.motions()[0].path.front() == motion.path.front() &&
                     planner.motions()[0].path.back() == motion.path.back(),
                 "repair invented a waiting-robot path or changed endpoints");
    ok &= expect(!planner.findFirstConflict(),
                 "moving-held repair did not pass full revalidation");
    return ok;
}

bool fixedContextSurvivesCopiesAndMotionValidation() {
    auto problem = makeProblem();
    problem->setFixedRobots({{makeRobot(), {0, 0, 0}}});
    auto moving = makeRobot();
    const std::vector<double> left{-1, 0, 0}, right{1, 0, 0};
    const comotion::CollisionChecker copy(problem->collisionChecker());
    bool ok = expect(!copy.isValidSingleFull(*moving, {0, 0, 0}),
                      "copied checker forgot native fixed robot context");
    ok &= expect(!copy.isMotionValid(*moving, left, right),
                 "single edge skipped a fixed robot");
    ok &= expect(!copy.isCompositeMotionValid({moving.get()}, {left}, {right}),
                 "composite edge skipped a fixed robot");
    ok &= expect(!copy.isRobotPathValid(*moving, densePath({left, right})),
                 "sparse path skipped a fixed robot between endpoints");
    const auto conflict = copy.findFirstCompositePathConflict(
        {densePath({left, right})}, {moving.get()});
    ok &= expect(conflict && conflict->scope == comotion::ConflictScope::Environment &&
                      conflict->robot_i == 0 && conflict->robot_j == -1,
                 "fixed-context conflict exposed an unplanned robot DOF");
    problem->addRobot(moving, left, right);
    const auto si = problem->createSpaceInfo(0);
    auto *from = si->allocState();
    auto *to = si->allocState();
    for (std::size_t i = 0; i < left.size(); ++i) {
        from->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = left[i];
        to->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = right[i];
    }
    ok &= expect(!si->checkMotion(from, to),
                 "OMPL planning/simplification edge bypassed fixed context");
    si->freeState(from);
    si->freeState(to);
    return ok;
}

bool completedMotionLosesItsCouplingAndContactExemptions() {
    auto left = sparseMotion();
    left.robot_name = "left";
    left.moving_entities = {"left"};
    left.coupling_id = 7;
    left.start_t = 0;
    left.end_t = 4;
    left.path = densePath({{-1, 0, 0}, {0, 0, 0}});
    auto right = sparseMotion();
    right.robot_name = "right";
    right.moving_entities = {"right"};
    right.coupling_id = 7;
    right.start_t = 4;
    right.end_t = 8;
    right.path = densePath({{1, 0, 0}, {-1, 0, 0}});
    comotion::ScheduleARC planner;
    planner.setProblem(makeProblem());
    planner.setMotions({left, right});
    planner.setFullOccupancy({8, {"left", "right"}, {
        hold("left", {0, 0, 0}, 4, 8), hold("right", {1, 0, 0}, 0, 4)}});
    const auto collision = planner.findFirstConflict();
    bool ok = expect(collision && collision->motion_i == 1 && collision->hold_j,
        "completed coupled robot disappeared before another robot's final move");
    left.path = densePath({{0, 0, 0}, {0, 0, 0}});
    left.ignored_stationary_entities = {"contact"};
    planner.setMotions({left});
    planner.setFullOccupancy({8, {"left"}, {hold("left", {0, 0, 0}, 4, 8)}});
    planner.setStationaryEntities({{"contact", 0, 8,
        {{Eigen::Vector3d{0, 0, 0}, 0.15}}, {}}});
    const auto expired = planner.findFirstConflict();
    ok &= expect(expired && expired->hold_i && expired->timestep == 4,
                 "expired action contact allowance leaked into terminal hold");
    return ok;
}

bool heldAttachmentsRemainNativeCollisionGeometry() {
    auto carrying = hold("carrier", {0, 0, 0}, 0, 0);
    comotion::AttachedBody object;
    object.name = "carried-object";
    object.link_name = "link_z";
    object.spheres.push_back({Eigen::Vector3d{1, 0, 0}, 0.2});
    carrying.model->setAttachment(object);
    carrying.attached_entities = {object.name};
    comotion::ScheduleARC planner;
    planner.setProblem(makeProblem());
    planner.setFullOccupancy({0, {"carrier", "other"}, {
        carrying, hold("other", {1, 0, 0}, 0, 0)}});
    const auto conflict = planner.findFirstConflict();
    return expect(conflict && conflict->hold_i && conflict->hold_j,
                  "held attached object was omitted from native pair checking");
}

bool attachmentContactsAreDirectedAndExpireWithThePhase() {
    auto owner = makeRobot();
    auto receiver = makeRobot();
    comotion::AttachedBody object;
    object.name = "object";
    object.link_name = "link_z";
    object.spheres.push_back({Eigen::Vector3d{1, 0, 0}, 0.2});
    owner->setAttachment(object);
    comotion::CollisionChecker checker;
    checker.setAttachmentContacts({{owner, receiver, "object", {"link_z"}}});
    comotion::CollisionChecker copied(checker);
    bool ok = expect(copied.isValidComposite(
        {owner.get(), receiver.get()}, {{0, 0, 0}, {1, 0, 0}}),
        "directed object/gripper contact was lost in a checker copy");
    ok &= expect(copied.isCompositeMotionValid({owner.get(), receiver.get()},
        {{0, 0, 0}, {1, 0, 0}}, {{0, 0, 0}, {1.5, 0, 0}}),
        "directed contact was lost in native motion checking");
    ok &= expect(!copied.isValidPair(*owner, {0, 0, 0}, *receiver, {0, 0, 0}),
        "attachment contact exception hid a bare robot collision");
    checker.setAttachmentContacts({{owner, receiver, "object", {"link_y"}}});
    ok &= expect(!checker.isValidPair(*owner, {0, 0, 0}, *receiver, {1, 0, 0}),
        "attachment contact exception included an unlisted link");
    auto carrying = sparseMotion();
    carrying.model = owner;
    carrying.robot_name = "owner";
    carrying.moving_entities = {"owner", "object"};
    carrying.start_t = 0;
    carrying.end_t = 1;
    carrying.path = densePath({{0, 0, 0}, {0, 0, 0}});
    carrying.allowed_attachment_contacts = {{"object", "receiver", {"link_z"}}};
    auto touching = carrying;
    touching.model = receiver;
    touching.robot_name = "receiver";
    touching.moving_entities = {"receiver"};
    touching.allowed_attachment_contacts.clear();
    touching.path = densePath({{1.5, 0, 0}, {1, 0, 0}});
    auto owner_hold = hold("owner", {0, 0, 0}, 1, 2);
    owner_hold.model = owner;
    owner_hold.attached_entities = {"object"};
    comotion::ScheduleARC planner;
    planner.setProblem(makeProblem());
    planner.setMotions({carrying, touching});
    planner.setFullOccupancy({2, {"owner", "receiver"}, {
        owner_hold, hold("receiver", {1, 0, 0}, 1, 2)}});
    const auto expired = planner.findFirstConflict();
    ok &= expect(expired && expired->timestep == 1 && expired->hold_i && expired->hold_j,
        "phase contact exemption leaked into a later hold");
    auto separation_owner = carrying;
    separation_owner.start_t = 1;
    separation_owner.end_t = 2;
    auto separation_receiver = touching;
    separation_receiver.start_t = 1;
    separation_receiver.end_t = 2;
    separation_receiver.path = densePath({{1, 0, 0}, {2, 0, 0}});
    planner.setMotions({carrying, touching, separation_owner, separation_receiver});
    planner.setFullOccupancy({2, {"owner", "receiver"}, {}});
    ok &= expect(!planner.findFirstConflict(),
        "explicit transfer/separation contacts were not bounded to their phases");
    return ok;
}

} // namespace

int main() {
    bool ok = true;
    ok = sparseTimestepsAndProblemEnvironmentAreValidated() && ok;
    ok = currentPairPathScannerFindsScheduleConflict() && ok;
    ok = currentArcConfigurationIsInherited() && ok;
    ok = fixedScheduleHonorsGlobalMakespanBound() && ok;
    ok = coupledMotionsStayInTheRepairTeam() && ok;
    ok = fullOccupancyRejectsMissingCoverageAndDiscontinuity() && ok;
    ok = zeroMotionAndTerminalHoldsAreChecked() && ok;
    ok = movingHeldCollisionRepairsOnlyTheMovingRobot() && ok;
    ok = fixedContextSurvivesCopiesAndMotionValidation() && ok;
    ok = completedMotionLosesItsCouplingAndContactExemptions() && ok;
    ok = heldAttachmentsRemainNativeCollisionGeometry() && ok;
    ok = attachmentContactsAreDirectedAndExpireWithThePhase() && ok;
    return ok ? 0 : 1;
}
