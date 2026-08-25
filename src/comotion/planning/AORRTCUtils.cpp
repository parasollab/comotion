#include "comotion/planning/AORRTCUtils.h"

#include "comotion/planning/MakespanInformedSampler.h"
#include "comotion/planning/MakespanCompositeStateSpace.h"
#include "comotion/planning/PathSimplification.h"
#include "comotion/planning/PlanningSeed.h"
#include "comotion/planning/detail/PlannerInvariantUtils.h"
#include "comotion/planning/detail/SeededOmpl.h"

#include <ompl/base/ScopedState.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/samplers/informed/PathLengthDirectInfSampler.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/AORRTC.h>
#include <ompl/geometric/planners/rrt/AOXRRTConnect.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <sstream>
#include <stdexcept>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace comotion {
namespace aorrtc {
namespace {

using Clock = std::chrono::steady_clock;

class CappedAOXRRTConnect : public og::AOXRRTConnect {
public:
    explicit CappedAOXRRTConnect(const ob::SpaceInformationPtr &si)
        : og::AOXRRTConnect(si) {}

    void setMaxInternalSamples(std::size_t value) {
        if (value == 0)
            return;
        maxInternalSamples = value;
        maxInternalSamplesIncrement = value;
    }

    void setMaxInternalVertices(std::size_t value) {
        if (value == 0)
            return;
        maxInternalVertices = value;
        maxInternalVerticesIncrement = value;
    }
};

class SeededPathLengthObjective final
    : public ob::PathLengthOptimizationObjective {
public:
    SeededPathLengthObjective(const ob::SpaceInformationPtr &si,
                              std::uint_fast32_t sampler_seed,
                              std::uint_fast32_t base_sampler_seed,
                              std::uint_fast32_t uninformed_sampler_seed)
        : ob::PathLengthOptimizationObjective(si),
          sampler_seed_(sampler_seed),
          base_sampler_seed_(base_sampler_seed),
          uninformed_sampler_seed_(uninformed_sampler_seed) {}

    ob::InformedSamplerPtr allocInformedStateSampler(
        const ob::ProblemDefinitionPtr &prob_def,
        unsigned int max_number_calls) const override {
        auto sampler = std::make_shared<ob::PathLengthDirectInfSampler>(
            prob_def, max_number_calls);
        sampler->setLocalSeeds(sampler_seed_, base_sampler_seed_,
                               uninformed_sampler_seed_);
        return sampler;
    }

private:
    std::uint_fast32_t sampler_seed_;
    std::uint_fast32_t base_sampler_seed_;
    std::uint_fast32_t uninformed_sampler_seed_;
};

class SeededAORRTC final : public og::AORRTC {
public:
    SeededAORRTC(const ob::SpaceInformationPtr &si,
                 std::uint_fast32_t outer_seed,
                 std::uint_fast32_t inner_seed,
                 std::uint_fast32_t simplifier_seed)
        : og::AORRTC(si), outer_seed_(outer_seed), inner_seed_(inner_seed),
          simplifier_seed_(simplifier_seed) {
        rng_.setLocalSeed(outer_seed_);
    }

    void setup() override {
        og::AORRTC::setup();
        rng_.setLocalSeed(outer_seed_);
        aox_planner->setLocalSeed(inner_seed_);
        if (psk_)
            psk_->setLocalSeed(simplifier_seed_);
    }

private:
    std::uint_fast32_t outer_seed_;
    std::uint_fast32_t inner_seed_;
    std::uint_fast32_t simplifier_seed_;
};

struct SolveLocalSeeds {
    std::uint_fast32_t planner = 0;
    std::uint_fast32_t simplifier = 0;
    std::uint_fast32_t informed_sampler = 0;
    std::uint_fast32_t informed_base_sampler = 0;
    std::uint_fast32_t informed_uninformed_sampler = 0;
    std::uint_fast32_t outer_planner = 0;
    std::uint_fast32_t state_sampler = 0;
};

SolveLocalSeeds localSeeds(std::uint32_t planning_seed,
                           std::uint64_t solve_domain) {
    return {
        aorrtcComponentSeed(planning_seed, solve_domain, 1),
        aorrtcComponentSeed(planning_seed, solve_domain, 2),
        aorrtcComponentSeed(planning_seed, solve_domain, 3),
        aorrtcComponentSeed(planning_seed, solve_domain, 4),
        aorrtcComponentSeed(planning_seed, solve_domain, 5),
        aorrtcComponentSeed(planning_seed, solve_domain, 6),
        aorrtcComponentSeed(planning_seed, solve_domain, 7),
    };
}

void installStateSampler(const ob::StateSpacePtr &space,
                         std::uint_fast32_t seed) {
    space->setStateSamplerAllocator(
        [seed](const ob::StateSpace *sampler_space) {
            return std::make_shared<detail::SeededRealVectorStateSampler>(
                sampler_space, seed);
        });
}

struct RobotBlock {
    int robot_index = -1;
    int offset = 0;
    int ndof = 0;
};

std::vector<int> defaultRobotIndices(const MultiRobotProblem &problem,
                                     std::vector<int> robot_indices) {
    if (!robot_indices.empty())
        return robot_indices;
    robot_indices.reserve(static_cast<std::size_t>(problem.numRobots()));
    for (int i = 0; i < problem.numRobots(); ++i)
        robot_indices.push_back(i);
    return robot_indices;
}

std::vector<RobotBlock> makeBlocks(const MultiRobotProblem &problem,
                                   const std::vector<int> &robot_indices) {
    std::vector<RobotBlock> blocks;
    blocks.reserve(robot_indices.size());
    int offset = 0;
    for (const int idx : robot_indices) {
        const int ndof = problem.robot(idx).model->numJoints();
        blocks.push_back({idx, offset, ndof});
        offset += ndof;
    }
    return blocks;
}

std::vector<double> configFromState(const ob::State *state, int offset,
                                    int ndof) {
    const auto *rv = state->as<ob::RealVectorStateSpace::StateType>();
    std::vector<double> config(static_cast<std::size_t>(ndof));
    for (int d = 0; d < ndof; ++d)
        config[static_cast<std::size_t>(d)] = rv->values[offset + d];
    return config;
}

int denseCompositeSegmentChecks(const MultiRobotProblem &problem,
                                const std::vector<RobotBlock> &blocks,
                                const ob::State *s1, const ob::State *s2,
                                int fallback_checks) {
    int checks = std::max(1, fallback_checks);
    if (problem.vmax() <= 0.0 || problem.resolution() == 0)
        return checks;

    double max_robot_dist = 0.0;
    for (const auto &block : blocks) {
        const auto from = configFromState(s1, block.offset, block.ndof);
        const auto to = configFromState(s2, block.offset, block.ndof);
        double dist_sq = 0.0;
        for (std::size_t d = 0; d < from.size(); ++d) {
            const double diff = to[d] - from[d];
            dist_sq += diff * diff;
        }
        max_robot_dist = std::max(max_robot_dist, std::sqrt(dist_sq));
    }

    const auto timestep_checks = static_cast<int>(std::ceil(
        max_robot_dist * static_cast<double>(problem.resolution()) /
        problem.vmax()));
    return std::max(checks, timestep_checks);
}

std::optional<CompositeConflict> firstCompositePathConflict(
    const MultiRobotProblem &problem, const std::vector<int> &robot_indices,
    const std::vector<Path> &paths) {
    std::vector<const RobotModel *> robots;
    robots.reserve(robot_indices.size());
    for (const int idx : robot_indices)
        robots.push_back(problem.robot(idx).model.get());

    CompositePathValidationOptions options;
    options.check_environment = true;
    return problem.collisionChecker().findFirstCompositePathConflict(
        paths, robots, options);
}

bool compositePathsAreValid(const MultiRobotProblem &problem,
                            const std::vector<int> &robot_indices,
                            const std::vector<Path> &paths) {
    return !firstCompositePathConflict(problem, robot_indices, paths).has_value();
}

void setCompositeState(const MultiRobotProblem &problem,
                       const std::vector<RobotBlock> &blocks,
                       bool use_goal, ob::State *state) {
    auto *rv = state->as<ob::RealVectorStateSpace::StateType>();
    for (const auto &block : blocks) {
        const auto &robot = problem.robot(block.robot_index);
        const auto &config = use_goal ? robot.goal : robot.start;
        for (int d = 0; d < block.ndof; ++d)
            rv->values[block.offset + d] = config[static_cast<std::size_t>(d)];
    }
}

double maxAbsConfigDiff(const std::vector<double> &a,
                        const std::vector<double> &b) {
    if (a.size() != b.size())
        return std::numeric_limits<double>::infinity();
    double diff = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i)
        diff = std::max(diff, std::abs(a[i] - b[i]));
    return diff;
}

Path pathFromSingleGeometric(const og::PathGeometric &gpath,
                             const RobotInstance &robot,
                             std::size_t resolution, double vmax) {
    const int ndof = robot.model->numJoints();
    Path out;
    out.reserve(gpath.getStateCount());
    for (std::size_t s = 0; s < gpath.getStateCount(); ++s) {
        const auto *rv =
            gpath.getState(s)->as<ob::RealVectorStateSpace::StateType>();
        std::vector<double> cfg(static_cast<std::size_t>(ndof));
        for (int d = 0; d < ndof; ++d)
            cfg[static_cast<std::size_t>(d)] = rv->values[d];
        out.push_back(std::move(cfg));
    }
    if (out.size() >= 2) {
        const double forward_error =
            maxAbsConfigDiff(out.front(), robot.start) +
            maxAbsConfigDiff(out.back(), robot.goal);
        const double reversed_error =
            maxAbsConfigDiff(out.front(), robot.goal) +
            maxAbsConfigDiff(out.back(), robot.start);
        if (reversed_error < forward_error)
            std::reverse(out.begin(), out.end());
    }
    out.computeTimestepsFromDistance(resolution, vmax);
    out.interpolate_to_timesteps(resolution, vmax);
    return out;
}

void normalizeCompositeStateOrder(const MultiRobotProblem &problem,
                                  const std::vector<RobotBlock> &blocks,
                                  std::vector<const ob::State *> &states) {
    if (states.size() < 2)
        return;

    double forward_error = 0.0;
    double reversed_error = 0.0;
    for (const auto &block : blocks) {
        const auto first =
            configFromState(states.front(), block.offset, block.ndof);
        const auto last =
            configFromState(states.back(), block.offset, block.ndof);
        const auto &robot = problem.robot(block.robot_index);
        forward_error += maxAbsConfigDiff(first, robot.start) +
                         maxAbsConfigDiff(last, robot.goal);
        reversed_error += maxAbsConfigDiff(first, robot.goal) +
                          maxAbsConfigDiff(last, robot.start);
    }

    if (reversed_error < forward_error)
        std::reverse(states.begin(), states.end());
}

bool pathMatchesRobotEndpoints(const Path &path, const RobotInstance &robot) {
    constexpr double kEndpointTolerance = 1e-6;
    return !path.empty() &&
           maxAbsConfigDiff(path.front(), robot.start) <= kEndpointTolerance &&
           maxAbsConfigDiff(path.back(), robot.goal) <= kEndpointTolerance;
}

bool pathsMatchRobotEndpoints(const MultiRobotProblem &problem,
                              const std::vector<int> &robot_indices,
                              const std::vector<Path> &paths) {
    if (paths.size() != robot_indices.size())
        return false;
    for (std::size_t i = 0; i < paths.size(); ++i) {
        if (!pathMatchesRobotEndpoints(paths[i],
                                       problem.robot(robot_indices[i])))
            return false;
    }
    return true;
}

Path directSinglePath(const RobotInstance &robot, std::size_t resolution,
                      double vmax) {
    Path path;
    path.push_back(robot.start);
    path.push_back(robot.goal);
    path.computeTimestepsFromDistance(resolution, vmax);
    path.interpolate_to_timesteps(resolution, vmax);
    return path;
}

std::vector<Path> splitCompositeStates(const MultiRobotProblem &problem,
                                       const std::vector<RobotBlock> &blocks,
                                       std::vector<const ob::State *> states) {
    normalizeCompositeStateOrder(problem, blocks, states);
    std::vector<Path> paths(blocks.size());
    for (std::size_t s = 0; s < states.size(); ++s) {
        for (std::size_t bi = 0; bi < blocks.size(); ++bi) {
            const auto &block = blocks[bi];
            paths[bi].push_back(configFromState(states[s], block.offset,
                                                block.ndof));
        }
    }

    std::vector<double> segment_times_sec;
    if (!states.empty() && states.size() >= 2) {
        segment_times_sec.reserve(states.size() - 1);
        for (std::size_t seg = 0; seg + 1 < states.size(); ++seg) {
            double max_robot_dist = 0.0;
            for (const auto &block : blocks) {
                const auto a = configFromState(states[seg], block.offset,
                                               block.ndof);
                const auto b = configFromState(states[seg + 1], block.offset,
                                               block.ndof);
                double dist_sq = 0.0;
                for (std::size_t d = 0; d < a.size(); ++d) {
                    const double diff = b[d] - a[d];
                    dist_sq += diff * diff;
                }
                max_robot_dist = std::max(max_robot_dist, std::sqrt(dist_sq));
            }
            segment_times_sec.push_back(max_robot_dist / problem.vmax());
        }
    }

    for (auto &path : paths) {
        if (!segment_times_sec.empty())
            path.setTimestepsFromSegmentTimes(segment_times_sec,
                                              problem.resolution());
        path.interpolate_to_timesteps(problem.resolution(), problem.vmax());
    }
    return paths;
}

std::vector<Path> splitCompositeGeometric(const MultiRobotProblem &problem,
                                          const std::vector<RobotBlock> &blocks,
                                          const og::PathGeometric &gpath) {
    std::vector<const ob::State *> states;
    states.reserve(gpath.getStateCount());
    for (std::size_t s = 0; s < gpath.getStateCount(); ++s)
        states.push_back(gpath.getState(s));
    return splitCompositeStates(problem, blocks, states);
}

std::pair<std::uint64_t, std::uint64_t>
metricsFromPaths(const std::vector<Path> &paths) {
    std::uint64_t sum = 0;
    std::uint64_t makespan = 0;
    for (const auto &path : paths) {
        const auto arrival =
            static_cast<std::uint64_t>(path.arrival_timestep());
        sum += arrival;
        makespan = std::max(makespan, arrival);
    }
    return {sum, makespan};
}

SolutionEvent makeEvent(const MultiRobotProblem &problem,
                        const Clock::time_point &solve_start,
                        const std::vector<Path> &paths, double ompl_cost,
                        std::string kind) {
    const auto [sum, makespan] = metricsFromPaths(paths);
    SolutionEvent event;
    event.elapsed_seconds =
        std::chrono::duration<double>(Clock::now() - solve_start).count();
    event.ompl_cost = ompl_cost;
    event.sum_of_cost_timesteps = sum;
    event.makespan_timesteps = makespan;
    event.kind = std::move(kind);
    (void)problem;
    return event;
}

void appendEvent(SolveResult &result, const SolveOptions &options,
                 const SolutionEvent &event) {
    result.solution_events.push_back(event);
    if (options.solution_event_callback)
        options.solution_event_callback(event);
}

ob::SpaceInformationPtr makeCompositeSpaceInfo(
    const MultiRobotProblem &problem, const std::vector<int> &robot_indices,
    bool use_makespan_metric) {
    if (!use_makespan_metric)
        return problem.createCompositeSpaceInfo(robot_indices);

    auto default_space = problem.createCompositeStateSpace(robot_indices);
    std::vector<unsigned int> block_dims;
    block_dims.reserve(robot_indices.size());
    for (const int idx : robot_indices)
        block_dims.push_back(static_cast<unsigned int>(
            problem.robot(idx).model->numJoints()));

    auto space = std::make_shared<MakespanCompositeStateSpace>(block_dims);
    space->setBounds(default_space->getBounds());
    auto si = std::make_shared<ob::SpaceInformation>(space);

    const auto blocks = std::make_shared<std::vector<RobotBlock>>(
        makeBlocks(problem, robot_indices));
    const auto checker = &problem.collisionChecker();

    si->setStateValidityChecker(
        [blocks, checker, &problem](const ob::State *state) -> bool {
            std::vector<const RobotModel *> robots;
            std::vector<std::vector<double>> configs;
            robots.reserve(blocks->size());
            configs.reserve(blocks->size());
            for (const auto &block : *blocks) {
                robots.push_back(problem.robot(block.robot_index).model.get());
                configs.push_back(
                    configFromState(state, block.offset, block.ndof));
            }
            return checker->isValidComposite(robots, configs);
        });

    class CompositeMotionValidator final : public ob::MotionValidator {
    public:
        CompositeMotionValidator(
            const ob::SpaceInformationPtr &si,
            std::shared_ptr<std::vector<RobotBlock>> blocks,
            const MultiRobotProblem *problem)
            : ob::MotionValidator(si), blocks_(std::move(blocks)),
              problem_(problem) {}

        bool checkMotion(const ob::State *s1,
                         const ob::State *s2) const override {
            CompositePathValidationOptions options;
            options.check_environment = true;
            options.discrete_num_checks_hint = std::max(
                1, static_cast<int>(
                       si_->getStateSpace()->validSegmentCount(s1, s2)));
            options.discrete_num_checks_hint = denseCompositeSegmentChecks(
                *problem_, *blocks_, s1, s2, options.discrete_num_checks_hint);

            std::vector<const RobotModel *> robots;
            std::vector<std::vector<double>> from;
            std::vector<std::vector<double>> to;
            robots.reserve(blocks_->size());
            from.reserve(blocks_->size());
            to.reserve(blocks_->size());
            for (const auto &block : *blocks_) {
                robots.push_back(problem_->robot(block.robot_index).model.get());
                from.push_back(configFromState(s1, block.offset, block.ndof));
                to.push_back(configFromState(s2, block.offset, block.ndof));
            }
            return problem_->collisionChecker().isCompositeMotionValid(
                robots, from, to, options);
        }

        bool checkMotion(const ob::State *s1, const ob::State *s2,
                         std::pair<ob::State *, double> &lastValid)
            const override {
            lastValid.first = nullptr;
            lastValid.second = 0.0;
            return checkMotion(s1, s2);
        }

    private:
        std::shared_ptr<std::vector<RobotBlock>> blocks_;
        const MultiRobotProblem *problem_ = nullptr;
    };

    if (!robot_indices.empty()) {
        auto single_space = problem.createStateSpace(robot_indices[0]);
        const double single_extent = single_space->getMaximumExtent();
        const double composite_extent = space->getMaximumExtent();
        if (composite_extent > 1e-10) {
            si->setStateValidityCheckingResolution(
                0.01 * single_extent / composite_extent);
        }
    }
    si->setMotionValidator(
        std::make_shared<CompositeMotionValidator>(si, blocks, &problem));
    si->setup();
    return si;
}

void setCompositeOptimizationObjective(og::SimpleSetup &setup,
                                       const ob::SpaceInformationPtr &si,
                                       bool use_makespan_metric,
                                       const std::optional<SolveLocalSeeds>
                                           &local_seeds) {
    if (use_makespan_metric) {
        std::optional<std::uint_fast32_t> sampler_seed;
        if (local_seeds)
            sampler_seed = local_seeds->informed_sampler;
        setup.getProblemDefinition()->setOptimizationObjective(
            std::make_shared<MakespanPathLengthObjective>(si, sampler_seed));
        return;
    }

    if (local_seeds) {
        setup.getProblemDefinition()->setOptimizationObjective(
            std::make_shared<SeededPathLengthObjective>(
                si, local_seeds->informed_sampler,
                local_seeds->informed_base_sampler,
                local_seeds->informed_uninformed_sampler));
    }
}

void requireExactEndpoints(const MultiRobotProblem &problem,
                           const std::vector<int> &robot_indices,
                           const std::vector<Path> &paths,
                           const char *planner_name) {
    constexpr double kEndpointTolerance = 1e-6;
    if (paths.size() != robot_indices.size())
        throw std::runtime_error(std::string(planner_name) +
                                 " returned wrong path count");
    for (std::size_t local_idx = 0; local_idx < paths.size(); ++local_idx) {
        const int robot_idx = robot_indices[local_idx];
        const auto &path = paths[local_idx];
        if (path.empty()) {
            std::ostringstream msg;
            msg << planner_name << " exact solution path missing for robot "
                << robot_idx;
            throw std::runtime_error(msg.str());
        }
        std::ostringstream start_context;
        start_context << planner_name << " exact solution start mismatch for robot "
                      << robot_idx;
        comotion::detail::requireConfigNear(path.front(),
                                        problem.robot(robot_idx).start,
                                        kEndpointTolerance,
                                        start_context.str());

        std::ostringstream goal_context;
        goal_context << planner_name << " exact solution goal mismatch for robot "
                     << robot_idx;
        comotion::detail::requireConfigNear(path.back(),
                                        problem.robot(robot_idx).goal,
                                        kEndpointTolerance,
                                        goal_context.str());
    }
}

} // namespace

double timestepsToOmplCost(std::uint64_t timesteps,
                           const MultiRobotProblem &problem) {
    if (problem.resolution() == 0)
        return std::numeric_limits<double>::infinity();
    return static_cast<double>(timesteps) * problem.vmax() /
           static_cast<double>(problem.resolution());
}

std::uint64_t omplCostToTimesteps(double cost,
                                  const MultiRobotProblem &problem) {
    if (problem.vmax() <= 0.0)
        return 0;
    return static_cast<std::uint64_t>(std::llround(
        cost * static_cast<double>(problem.resolution()) / problem.vmax()));
}

SolveResult solveSingleRobotBounded(const MultiRobotProblem &problem,
                                    int robot_index, double time_limit,
                                    const SolveOptions &options) {
    SolveResult result;
    if (!options.cost_bound_timesteps)
        throw std::invalid_argument(
            "solveSingleRobotBounded requires cost_bound_timesteps");

    auto si = problem.createSpaceInfo(robot_index);
    auto space = si->getStateSpace();
    std::optional<SolveLocalSeeds> local_seeds;
    if (options.planning_seed) {
        local_seeds = localSeeds(
            *options.planning_seed,
            kPlanningSeedDomainAorrtcSingleBounded);
        installStateSampler(space, local_seeds->state_sampler);
    }
    og::SimpleSetup setup(si);
    setCompositeOptimizationObjective(setup, si, false, local_seeds);
    auto planner = std::make_shared<CappedAOXRRTConnect>(si);
    if (local_seeds)
        planner->setLocalSeed(local_seeds->planner);
    if (options.max_internal_samples)
        planner->setMaxInternalSamples(*options.max_internal_samples);
    if (options.max_internal_vertices)
        planner->setMaxInternalVertices(*options.max_internal_vertices);
    setup.setPlanner(planner);

    const auto &robot = problem.robot(robot_index);
    const int ndof = robot.model->numJoints();
    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);
    for (int d = 0; d < ndof; ++d) {
        start->as<ob::RealVectorStateSpace::StateType>()->values[d] =
            robot.start[static_cast<std::size_t>(d)];
        goal->as<ob::RealVectorStateSpace::StateType>()->values[d] =
            robot.goal[static_cast<std::size_t>(d)];
    }
    setup.setStartAndGoalStates(start, goal);
    setup.setup();
    if (local_seeds && setup.getPathSimplifier())
        setup.getPathSimplifier()->setLocalSeed(local_seeds->simplifier);
    const double bound_cost =
        timestepsToOmplCost(*options.cost_bound_timesteps, problem);
    planner->setPathCost(bound_cost);

    const auto solve_start = Clock::now();
    result.status = setup.solve(time_limit);
    if (result.status == ob::PlannerStatus::EXACT_SOLUTION) {
        if (options.simplify_solution) {
            const auto simplify_start = Clock::now();
            detail::simplifySolutionBounded(setup,
                                            options.simplification_options);
            result.simplify_seconds =
                std::chrono::duration<double>(Clock::now() - simplify_start)
                    .count();
        }
        auto &gpath = setup.getSolutionPath();
        gpath.interpolate();
        result.paths.push_back(pathFromSingleGeometric(
            gpath, robot, problem.resolution(), problem.vmax()));
        if (!pathMatchesRobotEndpoints(result.paths.back(), robot)) {
            if (!si->checkMotion(start.get(), goal.get())) {
                result.status = ob::PlannerStatus::TIMEOUT;
                result.paths.clear();
                result.best_ompl_cost =
                    std::numeric_limits<double>::infinity();
                return result;
            }
            result.paths.back() =
                directSinglePath(robot, problem.resolution(), problem.vmax());
        }
        result.best_ompl_cost = result.paths.back().path_cost();
        if (result.best_ompl_cost > bound_cost + 1e-9) {
            result.status = ob::PlannerStatus::TIMEOUT;
            result.paths.clear();
            result.best_ompl_cost = std::numeric_limits<double>::infinity();
            return result;
        }
        const auto event =
            makeEvent(problem, solve_start, result.paths, result.best_ompl_cost,
                      "bounded_single_solution");
        appendEvent(result, options, event);
    }
    return result;
}

SolveResult solveCompositeBounded(const MultiRobotProblem &problem,
                                  const std::vector<int> &input_robot_indices,
                                  double time_limit,
                                  const SolveOptions &options) {
    SolveResult result;
    if (!options.cost_bound_timesteps)
        throw std::invalid_argument(
            "solveCompositeBounded requires cost_bound_timesteps");
    const auto robot_indices =
        defaultRobotIndices(problem, input_robot_indices);
    const auto blocks = makeBlocks(problem, robot_indices);
    auto si = makeCompositeSpaceInfo(problem, robot_indices,
                                    options.use_makespan_metric);
    auto space = si->getStateSpace();
    std::optional<SolveLocalSeeds> local_seeds;
    if (options.planning_seed) {
        local_seeds = localSeeds(
            *options.planning_seed,
            kPlanningSeedDomainAorrtcCompositeBounded);
        installStateSampler(space, local_seeds->state_sampler);
    }
    og::SimpleSetup setup(si);
    auto planner = std::make_shared<CappedAOXRRTConnect>(si);
    if (local_seeds)
        planner->setLocalSeed(local_seeds->planner);
    if (options.max_internal_samples)
        planner->setMaxInternalSamples(*options.max_internal_samples);
    if (options.max_internal_vertices)
        planner->setMaxInternalVertices(*options.max_internal_vertices);
    setup.setPlanner(planner);

    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);
    setCompositeState(problem, blocks, false, start.get());
    setCompositeState(problem, blocks, true, goal.get());
    setup.setStartAndGoalStates(start, goal);
    setCompositeOptimizationObjective(setup, si, options.use_makespan_metric,
                                      local_seeds);
    setup.setup();
    if (local_seeds && setup.getPathSimplifier())
        setup.getPathSimplifier()->setLocalSeed(local_seeds->simplifier);
    const double bound_cost =
        timestepsToOmplCost(*options.cost_bound_timesteps, problem);
    planner->setPathCost(bound_cost);

    const auto solve_start = Clock::now();
    result.status = setup.solve(time_limit);
    if (result.status == ob::PlannerStatus::EXACT_SOLUTION) {
        if (options.simplify_solution) {
            const auto simplify_start = Clock::now();
            detail::simplifySolutionBounded(setup,
                                            options.simplification_options);
            result.simplify_seconds =
                std::chrono::duration<double>(Clock::now() - simplify_start)
                    .count();
        }
        auto &gpath = setup.getSolutionPath();
        gpath.interpolate();
        result.paths = splitCompositeGeometric(problem, blocks, gpath);
        if (!pathsMatchRobotEndpoints(problem, robot_indices, result.paths)) {
            if (!si->checkMotion(start.get(), goal.get())) {
                result.status = ob::PlannerStatus::TIMEOUT;
                result.paths.clear();
                result.best_ompl_cost =
                    std::numeric_limits<double>::infinity();
                return result;
            }
            std::vector<const ob::State *> direct_states{start.get(),
                                                        goal.get()};
            result.paths = splitCompositeStates(problem, blocks, direct_states);
        }
        if (!compositePathsAreValid(problem, robot_indices, result.paths)) {
            result.status = ob::PlannerStatus::TIMEOUT;
            result.paths.clear();
            result.best_ompl_cost = std::numeric_limits<double>::infinity();
            return result;
        }
        requireExactEndpoints(problem, robot_indices, result.paths,
                              "CompositeAOXRRTConnect");
        const auto [_, makespan] = metricsFromPaths(result.paths);
        (void)_;
        result.best_ompl_cost = timestepsToOmplCost(makespan, problem);
        if (result.best_ompl_cost > bound_cost + 1e-9) {
            result.status = ob::PlannerStatus::TIMEOUT;
            result.paths.clear();
            result.best_ompl_cost = std::numeric_limits<double>::infinity();
            return result;
        }
        const auto event =
            makeEvent(problem, solve_start, result.paths, result.best_ompl_cost,
                      "bounded_composite_solution");
        appendEvent(result, options, event);
    }
    return result;
}

SolveResult solveCompositeAnytime(const MultiRobotProblem &problem,
                                  const std::vector<int> &input_robot_indices,
                                  double time_limit,
                                  const SolveOptions &options) {
    SolveResult result;
    const auto robot_indices =
        defaultRobotIndices(problem, input_robot_indices);
    const auto blocks = makeBlocks(problem, robot_indices);
    auto si = makeCompositeSpaceInfo(problem, robot_indices,
                                    options.use_makespan_metric);
    auto space = si->getStateSpace();
    std::optional<SolveLocalSeeds> local_seeds;
    if (options.planning_seed) {
        local_seeds = localSeeds(
            *options.planning_seed,
            kPlanningSeedDomainAorrtcCompositeAnytime);
        installStateSampler(space, local_seeds->state_sampler);
    }
    og::SimpleSetup setup(si);
    std::shared_ptr<og::AORRTC> planner;
    if (local_seeds) {
        planner = std::make_shared<SeededAORRTC>(
            si, local_seeds->outer_planner, local_seeds->planner,
            local_seeds->simplifier);
    } else {
        planner = std::make_shared<og::AORRTC>(si);
    }
    if (options.max_internal_samples)
        planner->setMaxInternalSamples(*options.max_internal_samples);
    if (options.max_internal_vertices)
        planner->setMaxInternalVertices(*options.max_internal_vertices);
    setup.setPlanner(planner);

    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);
    setCompositeState(problem, blocks, false, start.get());
    setCompositeState(problem, blocks, true, goal.get());
    setup.setStartAndGoalStates(start, goal);
    setCompositeOptimizationObjective(setup, si, options.use_makespan_metric,
                                      local_seeds);

    const auto solve_start = Clock::now();
    double best_reported_ompl_cost = std::numeric_limits<double>::infinity();
    const auto append_incumbent_event = [&](const SolutionEvent &event,
                                            const std::vector<Path> &) {
        if (event.ompl_cost + 1e-9 >= best_reported_ompl_cost)
            return;

        best_reported_ompl_cost = event.ompl_cost;
        appendEvent(result, options, event);
    };

    setup.getProblemDefinition()->setIntermediateSolutionCallback(
        [&](const ob::Planner *, const std::vector<const ob::State *> &states,
            const ob::Cost cost) {
            auto paths = splitCompositeStates(problem, blocks, states);
            if (!pathsMatchRobotEndpoints(problem, robot_indices, paths) ||
                !compositePathsAreValid(problem, robot_indices, paths)) {
                return;
            }
            const auto event = makeEvent(problem, solve_start, paths,
                                         cost.value(),
                                         "composite_aorrtc_improvement");
            append_incumbent_event(event, paths);
        });

    result.status = setup.solve(time_limit);
    if (setup.getProblemDefinition()->hasExactSolution()) {
        auto &gpath = setup.getSolutionPath();
        gpath.interpolate();
        result.paths = splitCompositeGeometric(problem, blocks, gpath);
        if (!compositePathsAreValid(problem, robot_indices, result.paths)) {
            result.status = ob::PlannerStatus::TIMEOUT;
            result.paths.clear();
            result.best_ompl_cost = std::numeric_limits<double>::infinity();
            return result;
        }
        requireExactEndpoints(problem, robot_indices, result.paths,
                              "CompositeAORRTC");
        result.best_ompl_cost = gpath.length();
        if (result.solution_events.empty()) {
            const auto event = makeEvent(problem, solve_start, result.paths,
                                         result.best_ompl_cost,
                                         "composite_aorrtc_solution");
            append_incumbent_event(event, result.paths);
        }
        result.status = ob::PlannerStatus::EXACT_SOLUTION;
    }
    return result;
}

} // namespace aorrtc
} // namespace comotion
