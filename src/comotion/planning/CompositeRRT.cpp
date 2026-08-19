#include "comotion/planning/CompositeRRT.h"
#include "comotion/planning/PlanningSeed.h"
#include "comotion/planning/detail/SeededOmpl.h"
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/ScopedState.h>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace comotion {

namespace {

using Clock = std::chrono::steady_clock;

inline std::uint64_t elapsedNanoseconds(Clock::time_point start) {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            Clock::now() - start)
            .count());
}

class FixedIterationRRTConnect final : public og::RRTConnect {
public:
    FixedIterationRRTConnect(const ob::SpaceInformationPtr &si,
                             std::uint_fast32_t seed)
        : og::RRTConnect(si) {
        rng_.setLocalSeed(seed);
    }

    void setExactIterations(unsigned iterations) { exact_iterations_ = iterations; }
    unsigned exactIterations() const { return exact_iterations_; }
    unsigned iterationsPerformed() const { return iterations_performed_; }
    unsigned firstSolutionIteration() const { return first_solution_iteration_; }
    bool foundExactSolution() const { return first_solution_iteration_ > 0; }

    ob::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc) override {
        checkValidity();
        auto *goal =
            dynamic_cast<ob::GoalSampleableRegion *>(pdef_->getGoal().get());
        if (goal == nullptr) {
            OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
            return ob::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
        }

        while (const ob::State *st = pis_.nextStart()) {
            auto *motion = new Motion(si_);
            si_->copyState(motion->state, st);
            motion->root = motion->state;
            tStart_->add(motion);
        }

        if (tStart_->size() == 0) {
            OMPL_ERROR("%s: Motion planning start tree could not be initialized!",
                       getName().c_str());
            return ob::PlannerStatus::INVALID_START;
        }

        if (!goal->couldSample()) {
            OMPL_ERROR("%s: Insufficient states in sampleable goal region",
                       getName().c_str());
            return ob::PlannerStatus::INVALID_GOAL;
        }

        if (!sampler_)
            sampler_ = si_->allocStateSampler();

        TreeGrowingInfo tgi;
        tgi.xstate = si_->allocState();

        Motion *approxsol = nullptr;
        double approxdif = std::numeric_limits<double>::infinity();
        auto *rmotion = new Motion(si_);
        ob::State *rstate = rmotion->state;
        bool solved = false;
        ob::PlannerStatus::StatusType status = ob::PlannerStatus::TIMEOUT;

        iterations_performed_ = 0;
        first_solution_iteration_ = 0;

        while (iterations_performed_ < exact_iterations_ && !ptc) {
            TreeData &tree = startTree_ ? tStart_ : tGoal_;
            tgi.start = startTree_;
            startTree_ = !startTree_;
            TreeData &otherTree = startTree_ ? tStart_ : tGoal_;

            if (tGoal_->size() == 0 ||
                pis_.getSampledGoalsCount() < tGoal_->size() / 2) {
                const ob::State *st =
                    tGoal_->size() == 0 ? pis_.nextGoal(ptc) : pis_.nextGoal();
                if (st != nullptr) {
                    auto *motion = new Motion(si_);
                    si_->copyState(motion->state, st);
                    motion->root = motion->state;
                    tGoal_->add(motion);
                }

                if (tGoal_->size() == 0) {
                    OMPL_ERROR("%s: Unable to sample any valid states for goal tree",
                               getName().c_str());
                    status = ob::PlannerStatus::INVALID_GOAL;
                    break;
                }
            }

            ++iterations_performed_;
            sampler_->sampleUniform(rstate);

            GrowState gs = growTree(tree, tgi, rmotion);
            if (gs == TRAPPED)
                continue;

            Motion *addedMotion = tgi.xmotion;
            if (gs != REACHED)
                si_->copyState(rstate, tgi.xstate);

            tgi.start = startTree_;
            GrowState gsc = growTree(otherTree, tgi, rmotion);
            if (gsc == TRAPPED)
                tgi.start = !tgi.start;

            while (gsc == ADVANCED)
                gsc = growTree(otherTree, tgi, rmotion);

            const double newDist =
                tree->getDistanceFunction()(addedMotion,
                                            otherTree->nearest(addedMotion));
            if (newDist < distanceBetweenTrees_)
                distanceBetweenTrees_ = newDist;

            Motion *startMotion = tgi.start ? tgi.xmotion : addedMotion;
            Motion *goalMotion = tgi.start ? addedMotion : tgi.xmotion;

            if (gsc == REACHED &&
                goal->isStartGoalPairValid(startMotion->root,
                                           goalMotion->root)) {
                if (!solved) {
                    if (startMotion->parent != nullptr)
                        startMotion = startMotion->parent;
                    else
                        goalMotion = goalMotion->parent;

                    connectionPoint_ =
                        std::make_pair(startMotion->state, goalMotion->state);

                    std::vector<Motion *> mpath1;
                    for (Motion *solution = startMotion; solution != nullptr;
                         solution = solution->parent) {
                        mpath1.push_back(solution);
                    }

                    std::vector<Motion *> mpath2;
                    for (Motion *solution = goalMotion; solution != nullptr;
                         solution = solution->parent) {
                        mpath2.push_back(solution);
                    }

                    auto path = std::make_shared<og::PathGeometric>(si_);
                    path->getStates().reserve(mpath1.size() + mpath2.size());
                    for (int i = static_cast<int>(mpath1.size()) - 1; i >= 0;
                         --i) {
                        path->append(mpath1[static_cast<std::size_t>(i)]->state);
                    }
                    for (auto *motion : mpath2)
                        path->append(motion->state);

                    pdef_->addSolutionPath(path, false, 0.0, getName());
                    solved = true;
                    first_solution_iteration_ = iterations_performed_;
                }
            } else if (tgi.start) {
                double dist = 0.0;
                goal->isSatisfied(tgi.xmotion->state, &dist);
                if (dist < approxdif) {
                    approxdif = dist;
                    approxsol = tgi.xmotion;
                }
            }
        }

        si_->freeState(tgi.xstate);
        si_->freeState(rstate);
        delete rmotion;

        if (approxsol && !solved) {
            std::vector<Motion *> mpath;
            while (approxsol != nullptr) {
                mpath.push_back(approxsol);
                approxsol = approxsol->parent;
            }

            auto path = std::make_shared<og::PathGeometric>(si_);
            for (int i = static_cast<int>(mpath.size()) - 1; i >= 0; --i)
                path->append(mpath[static_cast<std::size_t>(i)]->state);
            pdef_->addSolutionPath(path, true, approxdif, getName());
            return ob::PlannerStatus::APPROXIMATE_SOLUTION;
        }

        return solved ? ob::PlannerStatus::EXACT_SOLUTION : status;
    }

private:
    unsigned exact_iterations_{0};
    unsigned iterations_performed_{0};
    unsigned first_solution_iteration_{0};
};

} // namespace

ompl::base::PlannerStatus CompositeRRT::solve(double timeLimit) {
    resetPlannerRunMetrics();
    solution_paths_.clear();

    std::vector<int> indices = robot_indices_;
    if (indices.empty()) {
        for (int i = 0; i < problem_->numRobots(); ++i)
            indices.push_back(i);
    }

    auto si = use_makespan_metric_
                  ? problem_->createMakespanCompositeSpaceInfo(indices)
                  : problem_->createCompositeSpaceInfo(indices);
    auto space = si->getStateSpace();

    const auto state_sampler_seed =
        compositeRrtStateSamplerSeed(planning_seed_);
    const auto rrt_connect_seed = compositeRrtPlannerSeed(planning_seed_);
    const auto path_simplifier_seed =
        compositeRrtPathSimplifierSeed(planning_seed_);
    space->setStateSamplerAllocator(
        [state_sampler_seed](const ob::StateSpace *sampler_space) {
            return std::make_shared<detail::SeededRealVectorStateSampler>(
                sampler_space, state_sampler_seed);
        });

    og::SimpleSetup setup(si);
    std::shared_ptr<FixedIterationRRTConnect> fixed_iteration_planner;
    std::shared_ptr<og::RRTConnect> planner;
    if (continue_after_solution_until_iteration_cap_ &&
        max_rrt_connect_iterations_ > 0) {
        fixed_iteration_planner = std::make_shared<FixedIterationRRTConnect>(
            si, rrt_connect_seed);
        fixed_iteration_planner->setExactIterations(max_rrt_connect_iterations_);
        planner = fixed_iteration_planner;
    } else {
        planner =
            std::make_shared<detail::SeededRRTConnect>(si, rrt_connect_seed);
    }
    if (range_ && *range_ > 0.0)
        planner->setRange(*range_);
    setup.setPlanner(planner);

    // Build composite start and goal states
    ompl::base::ScopedState<> start(space);
    ompl::base::ScopedState<> goal(space);
    int offset = 0;
    for (int idx : indices) {
        auto &r = problem_->robot(idx);
        int ndof = r.model->numJoints();
        for (int d = 0; d < ndof; ++d) {
            start->as<ompl::base::RealVectorStateSpace::StateType>()
                ->values[offset + d] = r.start[d];
            goal->as<ompl::base::RealVectorStateSpace::StateType>()
                ->values[offset + d] = r.goal[d];
        }
        offset += ndof;
    }

    setup.setStartAndGoalStates(start, goal);

    const auto solve_wall_start = Clock::now();
    ob::PlannerStatus raw_status;
    if (fixed_iteration_planner) {
        auto time_ptc = ob::timedPlannerTerminationCondition(timeLimit);
        auto ptc = ob::PlannerTerminationCondition([&, time_ptc]() {
            return time_ptc || cancellationRequested();
        });
        raw_status = setup.solve(ptc);
    } else if (max_rrt_connect_iterations_ == 0 && !cancel_requested_) {
        raw_status = setup.solve(timeLimit);
    } else {
        const unsigned cap = max_rrt_connect_iterations_;
        std::size_t outer_iters = 0;
        auto time_ptc = ob::timedPlannerTerminationCondition(timeLimit);
        auto ptc = ob::PlannerTerminationCondition([&, cap, time_ptc]() {
            return time_ptc || cancellationRequested() ||
                   (cap > 0 && ++outer_iters > static_cast<std::size_t>(cap));
        });
        raw_status = setup.solve(ptc);
    }
    const auto solve_wall_ns = elapsedNanoseconds(solve_wall_start);
    auto exported_status = raw_status;
    if (raw_status == ob::PlannerStatus::APPROXIMATE_SOLUTION)
        exported_status = ob::PlannerStatus::TIMEOUT;
    std::uint64_t simplify_wall_ns = 0;

    if (raw_status == ob::PlannerStatus::EXACT_SOLUTION) {
        if (simplify_solution_) {
            const auto simplify_start = Clock::now();
            auto simplifier = std::make_shared<detail::SeededPathSimplifier>(
                si, path_simplifier_seed, setup.getGoal(),
                setup.getOptimizationObjective());
            detail::simplifyPathBounded(setup.getSolutionPath(), simplifier,
                                        simplification_options_);
            simplify_wall_ns = elapsedNanoseconds(simplify_start);
        }
        auto &path = setup.getSolutionPath();
        path.interpolate();

        solution_paths_ = splitCompositePathToRobotPaths(path, *problem_,
                                                         indices,
                                                         "CompositeRRT");
        setSolutionMetricsFromPaths(solution_paths_);
    }

    nlohmann::json stats = nlohmann::json::object();
    stats["solve_wall_seconds"] = static_cast<double>(solve_wall_ns) * 1e-9;
    stats["simplify_wall_seconds"] =
        static_cast<double>(simplify_wall_ns) * 1e-9;
    stats["simplify_solution"] = simplify_solution_;
    stats["planning_seed"] = planning_seed_;
    stats["state_sampler_seed"] = state_sampler_seed;
    stats["rrt_connect_seed"] = rrt_connect_seed;
    stats["path_simplifier_seed"] = path_simplifier_seed;
    stats["path_simplification"] = {
        {"max_shortcut_steps", simplification_options_.max_shortcut_steps},
        {"max_empty_steps", simplification_options_.max_empty_steps},
        {"max_smooth_steps", simplification_options_.max_smooth_steps},
        {"max_passes", simplification_options_.max_passes},
    };
    stats["rrt_connect_range"] = planner->getRange();
    stats["rrt_connect_range_explicit"] =
        static_cast<bool>(range_ && *range_ > 0.0);
    stats["use_makespan_metric"] = use_makespan_metric_;
    stats["max_rrt_connect_iterations"] = max_rrt_connect_iterations_;
    stats["continue_after_solution_until_iteration_cap"] =
        continue_after_solution_until_iteration_cap_;
    if (fixed_iteration_planner) {
        stats["rrt_connect_iterations"] =
            fixed_iteration_planner->iterationsPerformed();
        stats["rrt_connect_exact_iteration_cap"] =
            fixed_iteration_planner->exactIterations();
        stats["rrt_connect_first_solution_iteration"] =
            fixed_iteration_planner->foundExactSolution()
                ? nlohmann::json(fixed_iteration_planner
                                      ->firstSolutionIteration())
                : nlohmann::json(nullptr);
    }
    stats["state_space_extent"] = space->getMaximumExtent();
    stats["solution_events"] = nlohmann::json::array();
    if (exported_status == ob::PlannerStatus::EXACT_SOLUTION &&
        makespanTimesteps()) {
        stats["solution_events"].push_back({
            {"elapsed_seconds", static_cast<double>(solve_wall_ns) * 1e-9},
            {"makespan_timesteps", *makespanTimesteps()},
            {"sum_of_cost_timesteps",
             sumOfCostTimesteps() ? nlohmann::json(*sumOfCostTimesteps())
                                  : nlohmann::json(nullptr)},
            {"kind", "first_solution"},
        });
    }
    stats["num_solution_events"] = stats["solution_events"].size();
    setPlannerStatsJson(std::move(stats));

    return exported_status;
}

std::vector<Path> CompositeRRT::getSolutionPaths() const {
    return solution_paths_;
}

} // namespace comotion
