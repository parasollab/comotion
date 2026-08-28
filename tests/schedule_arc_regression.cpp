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

} // namespace

int main() {
    bool ok = true;
    ok = sparseTimestepsAndProblemEnvironmentAreValidated() && ok;
    ok = currentPairPathScannerFindsScheduleConflict() && ok;
    ok = currentArcConfigurationIsInherited() && ok;
    ok = fixedScheduleHonorsGlobalMakespanBound() && ok;
    ok = coupledMotionsStayInTheRepairTeam() && ok;
    return ok ? 0 : 1;
}
