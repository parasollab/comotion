#include "comotion/collision/CollisionChecker.h"
#include "comotion/collision/ConflictChecker.h"
#include "comotion/collision/ObstacleShapes.h"
#include "comotion/collision/ValidationTypes.h"
#include "comotion/planning/AOARC.h"
#include "comotion/planning/AORRTCUtils.h"
#include "comotion/planning/ARC.h"
#include "comotion/planning/CompositeAORRTC.h"
#include "comotion/planning/CompositePRMStar.h"
#include "comotion/planning/CompositeRRT.h"
#include "comotion/planning/CompositeRRTStar.h"
#include "comotion/planning/MRdRRT.h"
#include "comotion/planning/MultiRobotPlanner.h"
#include "comotion/planning/MultiRobotProblem.h"
#include "comotion/planning/ParallelARC.h"
#include "comotion/planning/Path.h"
#include "comotion/planning/PathSimplification.h"
#include "comotion/planning/PlanningRng.h"
#include "comotion/planning/PlanningSeed.h"
#include "comotion/planning/PrioritizedSTRRT.h"
#include "comotion/planning/STCBS.h"
#include "comotion/planning/USTRRTstar.h"
#include "comotion/robot/FlyingSphere.h"
#include "comotion/robot/IKSolver.h"
#include "comotion/robot/RobotModel.h"

#include <Eigen/Core>

#include <cstdint>
#include <memory>
#include <random>
#include <vector>

int main() {
    auto robot = std::make_shared<comotion::FlyingSphere>(
        0.25, std::vector<double>{-1.0, -1.0, 0.0},
        std::vector<double>{1.0, 1.0, 0.0});

    comotion::MultiRobotProblem problem(
        comotion::CollisionChecker::Backend::Spheres);
    problem.addRobot(robot, {-0.5, 0.0, 0.0}, {0.5, 0.0, 0.0});
    problem.setResolution(16);
    problem.setVmax(1.0);

    comotion::ObstacleSphere sphere{Eigen::Vector3d{0.0, 0.0, 0.0}, 0.1};
    comotion::ObstacleCylinder cylinder{Eigen::Vector3d{0.0, 0.0, 0.0},
                                        Eigen::Vector3d{0.0, 0.0, 1.0},
                                        0.1,
                                        0.5};
    problem.setObstacles({sphere});
    problem.setCylinderObstacles({cylinder});

    comotion::CollisionChecker checker(
        comotion::CollisionChecker::Backend::Spheres);
    comotion::ConflictChecker conflict_checker(checker);
    comotion::CompositePathValidationOptions validation_options;
    validation_options.conflict_find_parallel_assignment =
        comotion::ConflictFindParallelAssignment::Auto;

    comotion::Path path;
    path.push_back({-0.5, 0.0, 0.0});
    path.push_back({0.5, 0.0, 0.0});
    path.computeTimestepsFromDistance(problem.resolution(), problem.vmax());

    comotion::PathSimplificationOptions simplification_options;
    simplification_options.max_shortcut_steps = 1;

    comotion::ARC arc;
    arc.setProblem(std::make_shared<comotion::MultiRobotProblem>(problem));
    arc.setPathSimplificationOptions(simplification_options);

    comotion::AOARC ao_arc;
    ao_arc.setProblem(std::make_shared<comotion::MultiRobotProblem>(problem));

    comotion::CompositeRRT composite_rrt;
    composite_rrt.setProblem(std::make_shared<comotion::MultiRobotProblem>(problem));
    composite_rrt.setRobotIndices({0});

    comotion::CompositeRRTStar composite_rrt_star;
    composite_rrt_star.setMetricMode(
        comotion::CompositeRRTStar::MetricMode::Makespan);

    comotion::CompositePRMStar composite_prm_star;
    composite_prm_star.setMetricMode(
        comotion::CompositePRMStar::MetricMode::PlainL2);

    comotion::CompositeAORRTC composite_aorrtc;
    composite_aorrtc.setMaxInternalSamples(1);

    comotion::MRdRRT drrt;
    drrt.setCostMetric(comotion::MRdRRT::CostMetric::SumOfCosts);

    comotion::MRdRRTStar drrt_star;
    drrt_star.setTensorSearchMode(comotion::MRdRRT::TensorSearchMode::Drrt);

    comotion::ParallelARC parallel_arc;
    parallel_arc.setWorkerProcesses(1);
    parallel_arc.setInitialSolutionOr(false);
    parallel_arc.setParallelStrategy(
        comotion::ParallelArcParallelStrategy::Synchronous);

    comotion::PrioritizedSTRRT prioritized_strrt;
    prioritized_strrt.setPriorityOrder({0});
    prioritized_strrt.setStrrtRewiring(comotion::StrrtRewiring::Off);

    comotion::STCBS stcbs;
    stcbs.setRewireMode(comotion::USTRRTstar::RewireMode::KNearest);

    comotion::USTRRTstar::Params ust_params;
    ust_params.rewire_mode = comotion::USTRRTstar::RewireMode::KNearest;
    comotion::USTRRTstar::BranchConstraint branch_constraint;
    branch_constraint.constrained_agent_id = 0;

    comotion::aorrtc::SolveOptions aorrtc_options;
    aorrtc_options.simplification_options = simplification_options;

    comotion::IKRequest ik_request;
    ik_request.target_position = Eigen::Vector3d{0.0, 0.0, 0.0};
    comotion::IKResult ik_result;
    std::mt19937 rng(42);

    const std::uint_fast32_t root_seed =
        comotion::omplRootSeedFromUserPlanningSeed(42);
    const std::uint_fast32_t local_seed =
        comotion::omplLocalSeedFromUserPlanningSeed(42, 0);
    auto ompl_rng = comotion::makeOmplRngLocal(42, 0);

    (void)conflict_checker;
    (void)validation_options;
    (void)path;
    (void)ao_arc;
    (void)composite_rrt;
    (void)composite_rrt_star;
    (void)composite_prm_star;
    (void)composite_aorrtc;
    (void)drrt;
    (void)drrt_star;
    (void)parallel_arc;
    (void)prioritized_strrt;
    (void)stcbs;
    (void)ust_params;
    (void)branch_constraint;
    (void)aorrtc_options;
    (void)ik_request;
    (void)ik_result;
    (void)rng;
    (void)root_seed;
    (void)local_seed;
    (void)ompl_rng;

    return problem.numRobots() == 1 ? 0 : 1;
}
