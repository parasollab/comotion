#include "comotion/planning/PrioritizedSTRRT.h"
#include "comotion/planning/PlanningSeed.h"
#include "comotion/planning/detail/SeededOmpl.h"
#include "comotion/planning/detail/StrrtBatchInflation.h"
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/STRRTstar.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/spaces/SpaceTimeStateSpace.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/base/ScopedState.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <random>
#include <stdexcept>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace comotion {

namespace {

class LimitedSTRRTstar : public og::STRRTstar {
public:
    explicit LimitedSTRRTstar(const ob::SpaceInformationPtr &si)
        : og::STRRTstar(si) {}

    unsigned int iterationCount() const { return numIterations_; }

    void setLocalSeeds(std::uint_fast32_t planner_seed,
                       std::uint_fast32_t conditional_sampler_seed) {
        rng_.setLocalSeed(planner_seed);
        sampler_.setLocalSeed(conditional_sampler_seed);
    }
};

double pathDurationSeconds(const Path &path, size_t resolution) {
    if (path.empty() || resolution == 0)
        return 0.0;
    return static_cast<double>(path.size() - 1) /
           static_cast<double>(resolution);
}

const char *strrtRewiringName(StrrtRewiring mode) {
    switch (mode) {
    case StrrtRewiring::Off:
        return "off";
    case StrrtRewiring::Radius:
        return "radius";
    case StrrtRewiring::KNearest:
        return "knearest";
    }
    return "unknown";
}

// Motion validator that enforces forward-in-time and velocity constraints,
// plus collision checking against dynamic obstacles.
class STMotionValidator : public ob::MotionValidator {
public:
    STMotionValidator(const ob::SpaceInformationPtr &si, double vMax,
                      const RobotModel *model, const CollisionChecker *cc,
                      std::shared_ptr<std::vector<std::pair<int, Path>>> planned,
                      std::shared_ptr<std::vector<const RobotModel *>> robots_ptr,
                      size_t resolution, bool persist_at_goal)
        : ob::MotionValidator(si), vMax_(vMax),
          stateSpace_(si->getStateSpace().get()), model_(model), cc_(cc),
          planned_(std::move(planned)), robots_ptr_(std::move(robots_ptr)),
          resolution_(resolution), persist_at_goal_(persist_at_goal) {}

    bool checkMotion(const ob::State *s1, const ob::State *s2) const override {
        if (!si_->isValid(s2)) {
            invalid_++;
            return false;
        }

        auto *space = stateSpace_->as<ob::SpaceTimeStateSpace>();
        double deltaPos = space->distanceSpace(s1, s2);
        double t1 = ob::SpaceTimeStateSpace::getStateTime(s1);
        double t2 = ob::SpaceTimeStateSpace::getStateTime(s2);
        double deltaT = t2 - t1;

        if (!(deltaT > 0 && deltaPos / deltaT <= vMax_)) {
            invalid_++;
            return false;
        }

        const auto *cs1 = s1->as<ob::CompoundState>();
        const auto *rv1 = cs1->as<ob::RealVectorStateSpace::StateType>(0);
        const auto *cs2 = s2->as<ob::CompoundState>();
        const auto *rv2 = cs2->as<ob::RealVectorStateSpace::StateType>(0);
        const int ndof = model_->numJoints();

        std::vector<double> start_cfg(static_cast<std::size_t>(ndof));
        std::vector<double> goal_cfg(static_cast<std::size_t>(ndof));
        for (int i = 0; i < ndof; ++i) {
            start_cfg[static_cast<std::size_t>(i)] = rv1->values[i];
            goal_cfg[static_cast<std::size_t>(i)] = rv2->values[i];
        }

        const int steps_by_space =
            std::max(1, static_cast<int>(std::ceil(deltaPos / 0.05)));
        const int steps_by_time =
            std::max(1, static_cast<int>(std::ceil(deltaT * static_cast<double>(resolution_))));
        const int num_checks = std::max(steps_by_space, steps_by_time);

        if (!cc_->isMotionValid(*model_, start_cfg, goal_cfg, num_checks)) {
            invalid_++;
            return false;
        }

        Path current_path;
        std::vector<int> sample_timesteps;
        current_path.reserve(static_cast<std::size_t>(num_checks) + 1);
        sample_timesteps.reserve(static_cast<std::size_t>(num_checks) + 1);
        for (int step = 0; step <= num_checks; ++step) {
            const double frac = static_cast<double>(step) /
                                static_cast<double>(num_checks);
            const double t_sec = t1 + frac * deltaT;
            current_path.push_back(interpolateConfig(start_cfg, goal_cfg, frac));
            sample_timesteps.push_back(static_cast<int>(std::round(
                t_sec * static_cast<double>(resolution_))));
        }

        for (const auto &[other_idx, other_path] : *planned_) {
            if (other_path.empty())
                continue;

            Path current_window;
            Path other_window;
            current_window.reserve(current_path.size());
            other_window.reserve(current_path.size());
            for (std::size_t sample = 0; sample < current_path.size(); ++sample) {
                const int timestep = sample_timesteps[sample];
                if (!persist_at_goal_ &&
                    timestep >= static_cast<int>(other_path.size())) {
                    continue;
                }

                int clamped_t =
                    std::min(timestep, static_cast<int>(other_path.size()) - 1);
                clamped_t = std::max(0, clamped_t);
                current_window.push_back(current_path[sample]);
                other_window.push_back(other_path[static_cast<std::size_t>(clamped_t)]);
            }

            if (current_window.empty())
                continue;

            CompositePathValidationOptions options;
            options.check_environment = false;
            std::vector<Path> pair_paths{current_window, other_window};
            std::vector<const RobotModel *> pair_robots{
                model_, (*robots_ptr_)[static_cast<std::size_t>(other_idx)]};
            if (!cc_->validateCompositePaths(pair_paths, pair_robots, options)) {
                invalid_++;
                return false;
            }
        }

        valid_++;
        return true;
    }

    bool checkMotion(const ob::State *s1, const ob::State *s2,
                     std::pair<ob::State *, double> &lastValid) const override {
        lastValid.first = nullptr;
        lastValid.second = 0.0;
        return checkMotion(s1, s2);
    }

private:
    double vMax_;
    ob::StateSpace *stateSpace_;
    const RobotModel *model_;
    const CollisionChecker *cc_;
    std::shared_ptr<std::vector<std::pair<int, Path>>> planned_;
    std::shared_ptr<std::vector<const RobotModel *>> robots_ptr_;
    size_t resolution_;
    bool persist_at_goal_;
};

} // anonymous namespace

ompl::base::PlannerStatus PrioritizedSTRRT::solve(double timeLimit) {
    resetPlannerRunMetrics();
    solution_paths_.clear();
    last_robot_solve_times_seconds_.clear();
    int n = problem_->numRobots();
    std::vector<int> order;
    if (!priority_groups_.empty()) {
        std::vector<bool> seen(static_cast<std::size_t>(n), false);
        std::mt19937 rng(planning_seed_);
        for (const auto &group : priority_groups_) {
            std::vector<int> group_order = group;
            if (shuffle_priority_order_)
                std::shuffle(group_order.begin(), group_order.end(), rng);
            for (const int robot : group_order) {
                if (robot < 0 || robot >= n ||
                    seen[static_cast<std::size_t>(robot)]) {
                    throw std::runtime_error(
                        "PrioritizedSTRRT priority groups must contain each "
                        "robot exactly once");
                }
                seen[static_cast<std::size_t>(robot)] = true;
                order.push_back(robot);
            }
        }
        if (order.size() != static_cast<std::size_t>(n)) {
            throw std::runtime_error(
                "PrioritizedSTRRT priority groups must contain each robot "
                "exactly once");
        }
    } else {
        order = priority_order_;
    }
    if (priority_groups_.empty() &&
        (shuffle_priority_order_ || order.empty())) {
        order.clear();
        for (int i = 0; i < n; ++i)
            order.push_back(i);
        if (shuffle_priority_order_) {
            std::mt19937 rng(planning_seed_);
            std::shuffle(order.begin(), order.end(), rng);
        }
    }
    std::vector<std::uint_fast32_t> state_sampler_seeds;
    std::vector<std::uint_fast32_t> time_sampler_seeds;
    std::vector<std::uint_fast32_t> planner_local_seeds;
    std::vector<std::uint_fast32_t> conditional_sampler_seeds;
    std::vector<std::uint_fast32_t> simplifier_local_seeds;
    state_sampler_seeds.reserve(order.size());
    time_sampler_seeds.reserve(order.size());
    planner_local_seeds.reserve(order.size());
    conditional_sampler_seeds.reserve(order.size());
    simplifier_local_seeds.reserve(order.size());
    const auto finalizePlannerStats = [&]() {
        nlohmann::json stats = nlohmann::json::object();
        stats["priority_order"] = order;
        stats["priority_groups"] = priority_groups_;
        stats["shuffle_priority_order"] = shuffle_priority_order_;
        stats["return_first_solution"] = return_first_solution_;
        stats["persist_at_goal"] = persist_at_goal_;
        stats["simplify_after_plan"] = simplify_after_plan_;
        stats["path_simplification"] = {
            {"max_shortcut_steps",
             simplification_options_.max_shortcut_steps},
            {"max_empty_steps", simplification_options_.max_empty_steps},
            {"max_smooth_steps", simplification_options_.max_smooth_steps},
            {"max_passes", simplification_options_.max_passes},
        };
        stats["strrt_rewiring"] = strrtRewiringName(strrt_rewiring_);
        stats["use_unbounded_time"] = use_unbounded_time_;
        stats["inflate_initial_batch_from_min_goal_time"] =
            inflate_initial_batch_from_min_goal_time_;
        stats["strrt_initial_batch_size"] = strrt_initial_batch_size_;
        stats["strrt_initial_time_bound_factor"] =
            strrt_initial_time_bound_factor_;
        stats["strrt_time_bound_factor_increase"] =
            strrt_time_bound_factor_increase_;
        stats["strrt_max_inflated_batch_multiplier"] =
            strrt_max_inflated_batch_multiplier_;
        stats["strrt_max_iterations"] = strrt_max_iterations_;
        stats["per_robot_time_fraction"] = per_robot_fraction_;
        stats["robot_solve_times_seconds"] = last_robot_solve_times_seconds_;
        stats["planning_seed"] = planning_seed_;
        stats["state_sampler_seeds"] = state_sampler_seeds;
        stats["time_sampler_seeds"] = time_sampler_seeds;
        stats["planner_local_seeds"] = planner_local_seeds;
        stats["conditional_sampler_seeds"] = conditional_sampler_seeds;
        stats["simplifier_local_seeds"] = simplifier_local_seeds;
        setPlannerStatsJson(std::move(stats));
    };

    // All planned paths so far, indexed by robot index
    std::vector<Path> planned_paths(n);

    const auto budget_start = std::chrono::steady_clock::now();

    for (std::size_t order_pos = 0; order_pos < order.size(); ++order_pos) {
        const int robot_idx = order[order_pos];
        auto &robot_inst = problem_->robot(robot_idx);
        int ndof = robot_inst.model->numJoints();
        const size_t resolution = problem_->resolution();

        std::size_t min_safe_arrival_ts = 0;
        double longest_prior_duration_sec = 0.0;
        if (persist_at_goal_) {
            for (int j = 0; j < n; ++j) {
                if (planned_paths[j].empty())
                    continue;

                longest_prior_duration_sec = std::max(
                    longest_prior_duration_sec,
                    pathDurationSeconds(planned_paths[j], resolution));

                const auto constraint =
                    problem_->collisionChecker().computeGoalHoldConstraint(
                        *robot_inst.model, robot_inst.goal,
                        *problem_->robot(j).model, planned_paths[j]);
                if (constraint.permanently_blocked) {
                    finalizePlannerStats();
                    return ob::PlannerStatus::INVALID_GOAL;
                }
                min_safe_arrival_ts = std::max(
                    min_safe_arrival_ts, constraint.min_safe_arrival_timestep);
            }
        } else {
            for (int j = 0; j < n; ++j) {
                if (planned_paths[j].empty())
                    continue;
                longest_prior_duration_sec = std::max(
                    longest_prior_duration_sec,
                    pathDurationSeconds(planned_paths[j], resolution));
            }
        }

        auto vectorSpace = problem_->createStateSpace(robot_idx);
        const auto state_sampler_seed = prioritizedStrrtComponentSeed(
            planning_seed_, kPlanningSeedDomainStrrtStateSampler, robot_idx);
        vectorSpace->setStateSamplerAllocator(
            [state_sampler_seed](const ob::StateSpace *sampler_space) {
                return std::make_shared<detail::SeededRealVectorStateSampler>(
                    sampler_space, state_sampler_seed);
            });
        const double vmax = problem_->vmax();
        auto space =
            std::make_shared<ob::SpaceTimeStateSpace>(vectorSpace, vmax);
        const auto time_sampler_seed = prioritizedStrrtComponentSeed(
            planning_seed_, kPlanningSeedDomainStrrtTimeSampler, robot_idx);
        space->getTimeComponent()->setStateSamplerAllocator(
            [time_sampler_seed](const ob::StateSpace *sampler_space) {
                return std::make_shared<detail::SeededTimeStateSampler>(
                    sampler_space, time_sampler_seed);
            });
        const auto planner_local_seed = prioritizedStrrtComponentSeed(
            planning_seed_, kPlanningSeedDomainStrrtPlanner, robot_idx);
        const auto conditional_sampler_seed = prioritizedStrrtComponentSeed(
            planning_seed_, kPlanningSeedDomainStrrtConditionalSampler,
            robot_idx);
        const auto simplifier_local_seed = prioritizedStrrtComponentSeed(
            planning_seed_, kPlanningSeedDomainStrrtSimplifier, robot_idx);
        state_sampler_seeds.push_back(state_sampler_seed);
        time_sampler_seeds.push_back(time_sampler_seed);
        planner_local_seeds.push_back(planner_local_seed);
        conditional_sampler_seeds.push_back(conditional_sampler_seed);
        simplifier_local_seeds.push_back(simplifier_local_seed);
        const double min_safe_arrival_time =
            static_cast<double>(min_safe_arrival_ts) /
            static_cast<double>(resolution);
        double effective_t_ub = 0.0;
        if (!use_unbounded_time_) {
            effective_t_ub = std::max(
                std::max(space_time_upper_bound_sec_, 1e-9),
                (persist_at_goal_ && longest_prior_duration_sec > 0.0)
                    ? (2.0 * longest_prior_duration_sec)
                    : 0.0);
            if (min_safe_arrival_time > effective_t_ub) {
                finalizePlannerStats();
                return ob::PlannerStatus::INVALID_GOAL;
            }
            space->setTimeBounds(0.0, effective_t_ub);
        }

        auto si = std::make_shared<ob::SpaceInformation>(space);

        // State validity checker: obstacle + self-collision + dynamic obstacles
        const auto *model = robot_inst.model.get();
        const auto *cc = &problem_->collisionChecker();
        auto planned_copy =
            std::make_shared<std::vector<std::pair<int, Path>>>();
        for (int j = 0; j < n; ++j) {
            if (!planned_paths[j].empty())
                planned_copy->push_back({j, planned_paths[j]});
        }

        auto robots_ptr = std::make_shared<std::vector<const RobotModel *>>(
            problem_->robotModelPtrs());

        si->setStateValidityChecker(
            [model, cc, ndof, planned_copy, robots_ptr,
             robot_idx, persist = persist_at_goal_, resolution](
                const ob::State *state) -> bool {
                auto *cs = state->as<ob::CompoundState>();
                auto *rv =
                    cs->as<ob::RealVectorStateSpace::StateType>(0);
                double t_sec =
                    cs->as<ob::TimeStateSpace::StateType>(1)->position;

                std::vector<double> config(ndof);
                for (int i = 0; i < ndof; ++i)
                    config[i] = rv->values[i];

                // Check obstacle and self-collision
                if (!cc->isValidSingleFull(*model, config))
                    return false;

                // Check against dynamic obstacles (previously planned paths).
                // OMPL time t_sec is in seconds; convert to global timesteps.
                int timestep = static_cast<int>(std::round(t_sec * static_cast<double>(resolution)));
                for (auto &[other_idx, other_path] : *planned_copy) {
                    if (!persist && timestep >= static_cast<int>(other_path.size()))
                        continue;  // Prior robot finished; skip (no padding)
                    int clamped_t = std::min(
                        timestep,
                        static_cast<int>(other_path.size()) - 1);
                    clamped_t = std::max(0, clamped_t);
                    const auto &other_config = other_path[clamped_t];
                    if (!cc->isValidPair(*model, config,
                                         *(*robots_ptr)[other_idx],
                                         other_config))
                        return false;
                }
                return true;
            });

        si->setMotionValidator(
            std::make_shared<STMotionValidator>(si, vmax, model, cc, planned_copy,
                                                robots_ptr, resolution,
                                                persist_at_goal_));
        si->setup();

        og::SimpleSetup setup(si);
        ob::ScopedState<> start(space);
        for (int d = 0; d < ndof; ++d)
            start[d] = robot_inst.start[d];
        start.get()
            ->as<ob::CompoundState>()
            ->as<ob::TimeStateSpace::StateType>(1)
            ->position = 0.0;

        ob::ScopedState<> goal(space);
        for (int d = 0; d < ndof; ++d)
            goal[d] = robot_inst.goal[d];
        goal.get()
            ->as<ob::CompoundState>()
            ->as<ob::TimeStateSpace::StateType>(1)
            ->position = 0.0;

        const auto batch_inflation = detail::computeStrrtBatchInflation(
            space->timeToCoverDistance(start.get(), goal.get()),
            min_safe_arrival_time, strrt_initial_batch_size_,
            strrt_initial_time_bound_factor_,
            strrt_time_bound_factor_increase_,
            strrt_max_inflated_batch_multiplier_);
        const bool apply_inflated_batch =
            use_unbounded_time_ &&
            inflate_initial_batch_from_min_goal_time_ && persist_at_goal_ &&
            batch_inflation.virtual_expansions > 0;
        const unsigned int planner_batch_size =
            apply_inflated_batch ? batch_inflation.inflated_batch_size
                                 : batch_inflation.base_batch_size;

        auto planner = std::make_shared<LimitedSTRRTstar>(si);
        planner->setLocalSeeds(planner_local_seed,
                               conditional_sampler_seed);
        planner->setRange(vmax);
        planner->setBatchSize(static_cast<int>(std::min<unsigned int>(
            planner_batch_size,
            static_cast<unsigned int>(std::numeric_limits<int>::max()))));
        planner->setInitialTimeBoundFactor(strrt_initial_time_bound_factor_);
        planner->setTimeBoundFactorIncrease(strrt_time_bound_factor_increase_);
        planner->setSampleUniformForUnboundedTime(true);
        planner->setMinimumGoalTime(min_safe_arrival_time);
        switch (strrt_rewiring_) {
        case StrrtRewiring::Off:
            planner->setRewiringToOff();
            break;
        case StrrtRewiring::Radius:
            planner->setRewiringToRadius();
            break;
        case StrrtRewiring::KNearest:
            planner->setRewiringToKNearest();
            break;
        }
        planner->setReturnFirstSolution(return_first_solution_);
        setup.setPlanner(planner);

        setup.setStartAndGoalStates(start, goal);

        const double elapsed =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now() - budget_start)
                .count();
        const double remaining_wall = timeLimit - elapsed;
        if (remaining_wall <= 0.0) {
            finalizePlannerStats();
            return ob::PlannerStatus::TIMEOUT;
        }
        const std::size_t remaining_robots = order.size() - order_pos;
        double solve_seconds = remaining_wall;
        if (!return_first_solution_) {
            solve_seconds =
                (per_robot_fraction_ > 0.0)
                    ? timeLimit * per_robot_fraction_
                    : remaining_wall / static_cast<double>(remaining_robots);
            solve_seconds = std::min(solve_seconds, remaining_wall);
        }

        const auto solve_start = std::chrono::steady_clock::now();
        auto time_ptc = ob::timedPlannerTerminationCondition(solve_seconds);
        ob::PlannerTerminationCondition ptc(
            [this, planner, time_ptc,
             max_iterations = strrt_max_iterations_]() {
                return time_ptc || cancellationRequested() ||
                       (max_iterations > 0 &&
                        planner->iterationCount() >= max_iterations);
            });
        auto status = setup.solve(ptc);
        last_robot_solve_times_seconds_.push_back(
            std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                          solve_start)
                .count());
        if (status != ob::PlannerStatus::EXACT_SOLUTION) {
            finalizePlannerStats();
            return status;
        }

        if (simplify_after_plan_) {
            auto simplifier = std::make_shared<detail::SeededPathSimplifier>(
                si, simplifier_local_seed, setup.getGoal(),
                setup.getOptimizationObjective());
            detail::simplifyPathBounded(setup.getSolutionPath(), simplifier,
                                        simplification_options_);
        }
        auto &gpath = setup.getSolutionPath();
        gpath.interpolate();

        // Extract (config, time) pairs from the space-time path
        struct TimedConfig {
            double time;
            std::vector<double> config;
        };
        std::vector<TimedConfig> timed_path;
        for (size_t s = 0; s < gpath.getStateCount(); ++s) {
            auto *cs = gpath.getState(s)->as<ob::CompoundState>();
            auto *rv = cs->as<ob::RealVectorStateSpace::StateType>(0);
            double time_val =
                cs->as<ob::TimeStateSpace::StateType>(1)->position;
            std::vector<double> cfg(ndof);
            for (int d = 0; d < ndof; ++d)
                cfg[d] = rv->values[d];
            timed_path.push_back({time_val, std::move(cfg)});
        }

        // Resample at global timesteps (resolution timesteps per second).
        // OMPL time is in seconds; output uses waypoint_timesteps_[k] = k
        // so index = timestep, matching global path convention.
        double t_max_sec = timed_path.back().time;
        const size_t num_steps = std::max(size_t{1},
            static_cast<size_t>(std::ceil(t_max_sec * static_cast<double>(resolution))));
        Path robot_path;
        robot_path.reserve(num_steps + 1);
        for (size_t k = 0; k <= num_steps; ++k) {
            double target_t_sec = static_cast<double>(k) / static_cast<double>(resolution);
            size_t seg = 0;
            for (size_t i = 1; i < timed_path.size(); ++i) {
                if (timed_path[i].time >= target_t_sec) {
                    seg = i - 1;
                    break;
                }
                seg = i - 1;
            }
            if (seg + 1 >= timed_path.size()) {
                robot_path.push_back(timed_path.back().config);
            } else {
                double t0 = timed_path[seg].time;
                double t1 = timed_path[seg + 1].time;
                double alpha = (t1 > t0)
                    ? (target_t_sec - t0) / (t1 - t0)
                    : 0.0;
                alpha = std::clamp(alpha, 0.0, 1.0);
                robot_path.push_back(interpolateConfig(
                    timed_path[seg].config,
                    timed_path[seg + 1].config,
                    alpha));
            }
        }
        robot_path.markDenseTimestepsImplicit();

        planned_paths[robot_idx] = std::move(robot_path);
    }

    // Collect in order
    solution_paths_ = std::move(planned_paths);
    setSolutionMetricsFromPaths(solution_paths_);
    if (equalize_paths_)
        equalizePaths(solution_paths_);
    finalizePlannerStats();

    return ob::PlannerStatus::EXACT_SOLUTION;
}

std::vector<Path> PrioritizedSTRRT::getSolutionPaths() const {
    return solution_paths_;
}

} // namespace comotion
