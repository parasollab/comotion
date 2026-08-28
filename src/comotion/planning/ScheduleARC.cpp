#include "comotion/planning/ScheduleARC.h"

#include "comotion/collision/CollisionChecker.h"
#include "comotion/planning/CompositeRRT.h"
#include "comotion/planning/PlanningSeed.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <nlohmann/json.hpp>
#include <ompl/base/PlannerStatus.h>
#include <set>
#include <sstream>
#include <stdexcept>

namespace {

using Clock = std::chrono::steady_clock;

double elapsedSeconds(const Clock::time_point &start) {
    return std::chrono::duration<double>(Clock::now() - start).count();
}

std::vector<double> interpolateAtPathTimestep(const comotion::Path &path,
                                              double timestep) {
    if (path.empty())
        return {};
    if (path.size() == 1 || timestep <= path.timestep_at(0))
        return path.front();
    const double final_t = static_cast<double>(path.arrival_timestep());
    if (timestep >= final_t)
        return path.back();

    std::size_t hi = 1;
    while (hi < path.size() &&
           static_cast<double>(path.timestep_at(hi)) < timestep) {
        ++hi;
    }
    hi = std::min<std::size_t>(hi, path.size() - 1);
    const auto lo = hi - 1;
    const double lo_t = static_cast<double>(path.timestep_at(lo));
    const double hi_t = static_cast<double>(path.timestep_at(hi));
    const double alpha = hi_t > lo_t ? (timestep - lo_t) / (hi_t - lo_t) : 0.0;
    return comotion::interpolateConfig(path[lo], path[hi], alpha);
}

} // namespace

namespace comotion {

std::string toString(ScheduleArcConflictType type) {
    switch (type) {
    case ScheduleArcConflictType::MovingCollision:
        return "moving_collision";
    case ScheduleArcConflictType::StationaryCollision:
        return "stationary_collision";
    case ScheduleArcConflictType::EnvironmentCollision:
        return "environment_collision";
    }
    return "unknown";
}

std::string describeScheduleArcConflict(
    const ScheduleArcConflict &conflict) {
    std::ostringstream out;
    out << toString(conflict.type) << " at t=" << conflict.timestep;
    if (conflict.type == ScheduleArcConflictType::MovingCollision) {
        out << " between motion " << conflict.motion_i << " ("
            << conflict.label_i << ") and motion " << conflict.motion_j
            << " (" << conflict.label_j << ")";
    } else if (conflict.type == ScheduleArcConflictType::StationaryCollision) {
        out << " between motion " << conflict.motion_i << " ("
            << conflict.label_i << ") and stationary entity "
            << conflict.stationary_entity;
    } else {
        out << " for motion " << conflict.motion_i << " ("
            << conflict.label_i << ") against the problem environment";
    }
    return out.str();
}

void ScheduleARC::setMotions(std::vector<ScheduleArcMotion> motions) {
    motions_ = std::move(motions);
    for (auto &motion : motions_) {
        if (!motion.path.empty()) {
            if (motion.start.empty())
                motion.start = motion.path.front();
            if (motion.goal.empty())
                motion.goal = motion.path.back();
            normalizeMotionPath(motion);
        }
        if (motion.model == nullptr) {
            throw std::runtime_error(
                "ScheduleARC motion is missing a RobotModel");
        }
    }
}

void ScheduleARC::setStationaryEntities(
    std::vector<ScheduleArcStationaryEntity> entities) {
    stationary_entities_ = std::move(entities);
}

bool ScheduleARC::intervalContains(const ScheduleArcMotion &motion,
                                   std::size_t timestep) {
    return motion.start_t < timestep && timestep <= motion.end_t;
}

bool ScheduleARC::intervalOverlaps(std::size_t a_begin, std::size_t a_end,
                                   std::size_t b_begin, std::size_t b_end) {
    return a_begin <= b_end && b_begin <= a_end;
}

std::vector<double>
ScheduleARC::configAt(const ScheduleArcMotion &motion, std::size_t timestep) {
    if (motion.path.empty())
        throw std::runtime_error("ScheduleARC motion path is empty");
    if (timestep <= motion.start_t)
        return motion.path.front();
    return motion.path.config_at_timestep(timestep - motion.start_t);
}

Path ScheduleARC::resamplePath(const Path &path, std::size_t waypoint_count) {
    Path out;
    if (path.empty() || waypoint_count == 0)
        return out;
    out.reserve(waypoint_count);
    if (waypoint_count == 1) {
        out.push_back(path.front());
        out.markDenseTimestepsImplicit();
        return out;
    }

    const double first_t = static_cast<double>(path.timestep_at(0));
    const double span = static_cast<double>(path.arrival_timestep()) - first_t;
    const double scale = span /
                         static_cast<double>(waypoint_count - 1);
    for (std::size_t i = 0; i < waypoint_count; ++i) {
        out.push_back(interpolateAtPathTimestep(
            path, first_t + static_cast<double>(i) * scale));
    }
    out.markDenseTimestepsImplicit();
    return out;
}

void ScheduleARC::normalizeMotionPath(ScheduleArcMotion &motion) {
    if (motion.path.empty())
        return;
    const auto arrival = motion.path.arrival_timestep();
    if (motion.end_t < motion.start_t ||
        (motion.end_t == motion.start_t && arrival > 0)) {
        motion.end_t = motion.start_t + arrival;
    }
    const auto duration = motion.end_t - motion.start_t;
    motion.path = resamplePath(motion.path, duration + 1);
    motion.start = motion.path.front();
    motion.goal = motion.path.back();
}

bool ScheduleARC::hasEntity(const std::vector<std::string> &entities,
                            const std::string &entity) {
    return std::find(entities.begin(), entities.end(), entity) !=
           entities.end();
}

bool ScheduleARC::shouldSkipMovingPair(const ScheduleArcMotion &lhs,
                                       const ScheduleArcMotion &rhs) const {
    return motionsAreCoupled(lhs, rhs);
}

bool ScheduleARC::motionsAreCoupled(const ScheduleArcMotion &lhs,
                                    const ScheduleArcMotion &rhs) const {
    const auto unset = std::numeric_limits<std::size_t>::max();
    if (lhs.coupling_id != unset && lhs.coupling_id == rhs.coupling_id)
        return true;

    for (const auto &entity : lhs.moving_entities) {
        if (hasEntity(rhs.moving_entities, entity))
            return true;
    }
    return false;
}

bool ScheduleARC::shouldSkipStationaryEntity(
    const ScheduleArcMotion &motion, const std::string &entity) const {
    return hasEntity(motion.moving_entities, entity) ||
           hasEntity(motion.ignored_stationary_entities, entity);
}

std::size_t ScheduleARC::makespan() const {
    std::size_t out = 0;
    for (const auto &motion : motions_)
        out = std::max(out, motion.end_t);
    for (const auto &entity : stationary_entities_)
        out = std::max(out, entity.end_t);
    return out;
}

std::optional<ScheduleArcConflict> ScheduleARC::findFirstConflict() const {
    return findFirstConflict({}, nullptr);
}

std::optional<ScheduleArcConflict> ScheduleARC::findFirstConflict(
    const std::function<bool()> &stop_requested, bool *stopped) const {
    if (!problem_)
        throw std::runtime_error("ScheduleARC requires a MultiRobotProblem");
    if (stopped)
        *stopped = false;

    CollisionChecker checker(problem_->collisionChecker());
    const auto horizon = makespan();
    std::optional<ScheduleArcConflict> earliest_moving_conflict;
    for (std::size_t i = 0; i < motions_.size(); ++i) {
        const auto &lhs = motions_[i];
        for (std::size_t j = i + 1; j < motions_.size(); ++j) {
            if (stop_requested && stop_requested()) {
                if (stopped)
                    *stopped = true;
                return std::nullopt;
            }
            const auto &rhs = motions_[j];
            if (shouldSkipMovingPair(lhs, rhs))
                continue;
            const auto overlap_begin =
                std::max(lhs.start_t, rhs.start_t) + 1;
            const auto overlap_end = std::min(lhs.end_t, rhs.end_t);
            if (overlap_begin > overlap_end)
                continue;

            Path lhs_overlap;
            Path rhs_overlap;
            const auto overlap_count = overlap_end - overlap_begin + 1;
            lhs_overlap.reserve(overlap_count);
            rhs_overlap.reserve(overlap_count);
            for (std::size_t t = overlap_begin; t <= overlap_end; ++t) {
                lhs_overlap.push_back(configAt(lhs, t));
                rhs_overlap.push_back(configAt(rhs, t));
            }
            lhs_overlap.markDenseTimestepsImplicit();
            rhs_overlap.markDenseTimestepsImplicit();
            const auto pair_conflict = checker.findFirstPairPathConflict(
                *lhs.model, lhs_overlap, *rhs.model, rhs_overlap, 0,
                overlap_count);
            if (!pair_conflict)
                continue;

            const auto conflict_t = overlap_begin + pair_conflict->timestep;
            if (earliest_moving_conflict &&
                conflict_t >= earliest_moving_conflict->timestep) {
                continue;
            }
            ScheduleArcConflict conflict;
            conflict.type = ScheduleArcConflictType::MovingCollision;
            conflict.timestep = conflict_t;
            conflict.motion_i = i;
            conflict.motion_j = j;
            conflict.label_i = lhs.label;
            conflict.label_j = rhs.label;
            conflict.moving_entity_i = lhs.robot_name;
            conflict.moving_entity_j = rhs.robot_name;
            earliest_moving_conflict = std::move(conflict);
        }
    }

    for (std::size_t t = 0; t <= horizon; ++t) {
        if (stop_requested && stop_requested()) {
            if (stopped)
                *stopped = true;
            return std::nullopt;
        }
        std::vector<std::size_t> active;
        for (std::size_t i = 0; i < motions_.size(); ++i) {
            if (intervalContains(motions_[i], t))
                active.push_back(i);
        }

        if (earliest_moving_conflict &&
            earliest_moving_conflict->timestep == t)
            return earliest_moving_conflict;

        for (const auto motion_index : active) {
            const auto &motion = motions_[motion_index];
            if (!checker.isValidSingleFull(*motion.model,
                                           configAt(motion, t))) {
                ScheduleArcConflict conflict;
                conflict.type = ScheduleArcConflictType::EnvironmentCollision;
                conflict.timestep = t;
                conflict.motion_i = motion_index;
                conflict.stationary_entity = "problem_environment";
                conflict.label_i = motion.label;
                conflict.moving_entity_i = motion.robot_name;
                return conflict;
            }
            for (const auto &entity : stationary_entities_) {
                if (!intervalOverlaps(t, t, entity.start_t, entity.end_t))
                    continue;
                if (shouldSkipStationaryEntity(motion, entity.name))
                    continue;
                CollisionChecker entity_checker(
                    problem_->collisionChecker().backend());
                entity_checker.setObstacles(entity.spheres);
                entity_checker.setCylinderObstacles(entity.cylinders);
                if (!entity_checker.isValidSingle(
                        *motion.model, configAt(motion, t))) {
                    ScheduleArcConflict conflict;
                    conflict.type =
                        ScheduleArcConflictType::StationaryCollision;
                    conflict.timestep = t;
                    conflict.motion_i = motion_index;
                    conflict.stationary_entity = entity.name;
                    conflict.label_i = motion.label;
                    conflict.moving_entity_i = motion.robot_name;
                    return conflict;
                }
            }
        }
    }
    return std::nullopt;
}

std::vector<std::size_t>
ScheduleARC::motionsForConflict(const ScheduleArcConflict &conflict) const {
    std::vector<std::size_t> out;
    if (conflict.motion_i < motions_.size())
        out.push_back(conflict.motion_i);
    if (conflict.type == ScheduleArcConflictType::MovingCollision &&
        conflict.motion_j < motions_.size() &&
        conflict.motion_j != conflict.motion_i) {
        out.push_back(conflict.motion_j);
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    bool changed = true;
    while (changed) {
        changed = false;
        for (std::size_t candidate = 0; candidate < motions_.size(); ++candidate) {
            if (std::find(out.begin(), out.end(), candidate) != out.end() ||
                !intervalContains(motions_[candidate], conflict.timestep)) {
                continue;
            }
            const bool related = std::any_of(
                out.begin(), out.end(), [&](std::size_t member) {
                    return motionsAreCoupled(motions_[member],
                                             motions_[candidate]);
                });
            if (related) {
                out.push_back(candidate);
                changed = true;
            }
        }
    }
    std::sort(out.begin(), out.end());
    return out;
}

std::vector<std::size_t> ScheduleARC::expandMotionTeamForWindow(
    std::vector<std::size_t> motion_indices, std::size_t conflict_timestep,
    std::size_t begin_t, std::size_t end_t) const {
    bool changed = true;
    while (changed) {
        changed = false;
        for (const auto &repair : applied_repairs_) {
            if (!intervalOverlaps(begin_t, end_t, repair.begin_t, repair.end_t))
                continue;
            const bool touches_team = std::any_of(
                repair.motions.begin(), repair.motions.end(),
                [&](std::size_t index) {
                    return std::find(motion_indices.begin(), motion_indices.end(),
                                     index) != motion_indices.end();
                });
            if (!touches_team)
                continue;
            for (const auto candidate : repair.motions) {
                if (candidate >= motions_.size() ||
                    !intervalContains(motions_[candidate], conflict_timestep) ||
                    std::find(motion_indices.begin(), motion_indices.end(),
                              candidate) != motion_indices.end()) {
                    continue;
                }
                motion_indices.push_back(candidate);
                changed = true;
            }
        }
    }
    std::sort(motion_indices.begin(), motion_indices.end());
    motion_indices.erase(
        std::unique(motion_indices.begin(), motion_indices.end()),
        motion_indices.end());
    return motion_indices;
}

std::optional<ScheduleARC::RepairWindow>
ScheduleARC::initialRepairWindow(
    const ScheduleArcConflict &conflict,
    const std::vector<std::size_t> &motion_indices) const {
    if (motion_indices.empty())
        return std::nullopt;

    std::size_t lower = 0;
    std::size_t upper = std::numeric_limits<std::size_t>::max();
    for (const auto motion_index : motion_indices) {
        const auto &motion = motions_.at(motion_index);
        lower = std::max(lower, motion.start_t);
        upper = std::min(upper, motion.end_t);
    }
    if (lower >= upper)
        return std::nullopt;

    const auto begin = conflict.timestep > initial_window_
                           ? conflict.timestep - initial_window_
                           : std::size_t{0};
    RepairWindow window;
    window.begin_t = std::max(lower, begin);
    window.end_t = std::min(upper, conflict.timestep + initial_window_);
    if (window.begin_t >= window.end_t) {
        if (conflict.timestep > lower)
            window.begin_t = conflict.timestep - 1;
        else
            window.begin_t = lower;
        window.end_t = std::min(upper, window.begin_t + 1);
    }
    if (window.begin_t >= window.end_t)
        return std::nullopt;
    return window;
}

std::vector<ObstacleSphere>
ScheduleARC::stationarySpheresForWindow(
    std::size_t begin_t, std::size_t end_t,
    const std::vector<std::size_t> &motion_indices) const {
    std::set<std::string> moving_entities;
    std::map<std::string, std::size_t> ignored_entity_counts;
    for (const auto motion_index : motion_indices) {
        const auto &motion = motions_.at(motion_index);
        moving_entities.insert(motion.moving_entities.begin(),
                               motion.moving_entities.end());
        const std::set<std::string> unique_ignored(
            motion.ignored_stationary_entities.begin(),
            motion.ignored_stationary_entities.end());
        for (const auto &entity : unique_ignored)
            ++ignored_entity_counts[entity];
    }

    std::vector<ObstacleSphere> out;
    for (const auto &entity : stationary_entities_) {
        if (!intervalOverlaps(begin_t, end_t, entity.start_t, entity.end_t))
            continue;
        if (moving_entities.count(entity.name) != 0 ||
            ignored_entity_counts[entity.name] == motion_indices.size()) {
            continue;
        }
        out.insert(out.end(), entity.spheres.begin(), entity.spheres.end());
    }
    return out;
}

std::vector<ObstacleCylinder>
ScheduleARC::stationaryCylindersForWindow(
    std::size_t begin_t, std::size_t end_t,
    const std::vector<std::size_t> &motion_indices) const {
    std::set<std::string> moving_entities;
    std::map<std::string, std::size_t> ignored_entity_counts;
    for (const auto motion_index : motion_indices) {
        const auto &motion = motions_.at(motion_index);
        moving_entities.insert(motion.moving_entities.begin(),
                               motion.moving_entities.end());
        const std::set<std::string> unique_ignored(
            motion.ignored_stationary_entities.begin(),
            motion.ignored_stationary_entities.end());
        for (const auto &entity : unique_ignored)
            ++ignored_entity_counts[entity];
    }

    std::vector<ObstacleCylinder> out;
    for (const auto &entity : stationary_entities_) {
        if (!intervalOverlaps(begin_t, end_t, entity.start_t, entity.end_t))
            continue;
        if (moving_entities.count(entity.name) != 0 ||
            ignored_entity_counts[entity.name] == motion_indices.size()) {
            continue;
        }
        out.insert(out.end(), entity.cylinders.begin(), entity.cylinders.end());
    }
    return out;
}

bool ScheduleARC::planMissingMotionPath(ScheduleArcMotion &motion,
                                        double time_limit,
                                        std::uint32_t motion_seed) {
    if (!motion.path.empty())
        return true;
    if (motion.start.empty() || motion.goal.empty())
        return false;

    auto local_problem = std::make_shared<MultiRobotProblem>(
        problem_->collisionChecker().backend());
    local_problem->setResolution(problem_->resolution());
    local_problem->setVmax(problem_->vmax());
    local_problem->setObstacles(problem_->collisionChecker().obstacles());
    local_problem->setCylinderObstacles(problem_->collisionChecker().cylinders());
    local_problem->collisionChecker().setVampValidationStrategy(
        problem_->collisionChecker().vampValidationStrategy());
    local_problem->addRobot(motion.model, motion.start, motion.goal);

    CompositeRRT planner;
    planner.setProblem(local_problem);
    planner.setPlanningSeed(motion_seed);
    planner.setSimplifySolution(simplify_initial_solutions_);
    planner.setPathSimplificationOptions(simplification_options_);
    if (local_composite_rrt_range_)
        planner.setRange(*local_composite_rrt_range_);
    planner.setUseMakespanMetric(local_composite_rrt_use_makespan_metric_);
    planner.setMaxRrtConnectIterations(local_composite_rrt_max_samples_);
    planner.setCancellationCallback(cancel_requested_);
    const auto status = planner.solve(time_limit);
    if (status != ompl::base::PlannerStatus::EXACT_SOLUTION)
        return false;
    auto paths = planner.getSolutionPaths();
    if (paths.empty() || paths.front().empty())
        return false;
    motion.path = std::move(paths.front());
    normalizeMotionPath(motion);
    return true;
}

bool ScheduleARC::solveWindow(
    const std::vector<std::size_t> &motion_indices,
    const RepairWindow &window, double time_limit,
    std::uint32_t attempt_seed, std::vector<Path> &local_paths,
    bool &start_valid, bool &goal_valid, nlohmann::json &local_stats) const {
    auto local_problem = std::make_shared<MultiRobotProblem>(
        problem_->collisionChecker().backend());
    local_problem->setResolution(problem_->resolution());
    local_problem->setVmax(problem_->vmax());
    local_problem->collisionChecker().setVampValidationStrategy(
        problem_->collisionChecker().vampValidationStrategy());

    auto spheres = problem_->collisionChecker().obstacles();
    auto window_spheres =
        stationarySpheresForWindow(window.begin_t, window.end_t,
                                   motion_indices);
    spheres.insert(spheres.end(), window_spheres.begin(),
                   window_spheres.end());
    local_problem->setObstacles(spheres);

    auto cylinders = problem_->collisionChecker().cylinders();
    auto window_cylinders =
        stationaryCylindersForWindow(window.begin_t, window.end_t,
                                     motion_indices);
    cylinders.insert(cylinders.end(), window_cylinders.begin(),
                     window_cylinders.end());
    local_problem->setCylinderObstacles(cylinders);

    for (const auto motion_index : motion_indices) {
        const auto &motion = motions_.at(motion_index);
        local_problem->addRobot(motion.model, configAt(motion, window.begin_t),
                                configAt(motion, window.end_t));
    }

    const auto models = local_problem->robotModelPtrs();
    std::vector<std::vector<double>> starts;
    std::vector<std::vector<double>> goals;
    starts.reserve(motion_indices.size());
    goals.reserve(motion_indices.size());
    for (int i = 0; i < local_problem->numRobots(); ++i) {
        starts.push_back(local_problem->robot(i).start);
        goals.push_back(local_problem->robot(i).goal);
    }
    start_valid = local_problem->collisionChecker().isValidComposite(models,
                                                                      starts);
    goal_valid = local_problem->collisionChecker().isValidComposite(models,
                                                                     goals);
    if (!start_valid || !goal_valid)
        return false;

    ARC planner;
    planner.setProblem(local_problem);
    planner.setPlanningSeed(attempt_seed);
    planner.setCancellationCallback(cancel_requested_);
    planner.setInitialWindow(initial_window_);
    planner.setExpansionStep(expansion_step_);
    planner.setExpansionPolicy(expansion_policy_);
    planner.setCustomExpansionMultipliers(custom_expansion_multipliers_);
    if (initial_valid_window_expansion_step_)
        planner.setInitialValidWindowExpansionStep(
            *initial_valid_window_expansion_step_);
    if (initial_valid_window_expansion_policy_)
        planner.setInitialValidWindowExpansionPolicy(
            *initial_valid_window_expansion_policy_);
    if (initial_valid_window_expansion_multipliers_)
        planner.setInitialValidWindowExpansionMultipliers(
            *initial_valid_window_expansion_multipliers_);
    planner.setInitialValidWindowExpansionSymmetric(
        initial_valid_window_expansion_symmetric_);
    planner.setLocalCompositeRrtMaxSamples(local_composite_rrt_max_samples_);
    if (local_composite_rrt_range_)
        planner.setLocalCompositeRrtRange(*local_composite_rrt_range_);
    planner.setLocalCompositeRrtUseMakespanMetric(
        local_composite_rrt_use_makespan_metric_);
    planner.setLocalSolverMode(local_solver_mode_);
    planner.setLocalPrioritizedStrrtMaxIterations(
        local_prioritized_strrt_max_iterations_);
    planner.setLocalPrioritizedStrrtReturnFirstSolution(
        local_prioritized_strrt_return_first_solution_);
    planner.setLocalPrioritizedStrrtRewiring(
        local_prioritized_strrt_rewiring_);
    planner.setLocalPrioritizedStrrtPersistAtGoal(
        local_prioritized_strrt_persist_at_goal_);
    planner.setUseCspaceBounds(use_cspace_bounds_);
    planner.setCspaceBoundMargin(cspace_bound_margin_);
    planner.setMinCspaceBoundRange(min_cspace_bound_range_);
    planner.setStrrtSpaceTimeSpanFactor(strrt_space_time_span_factor_);
    const auto repair_simplification_options =
        conflict_simplification_options_.value_or(simplification_options_);
    planner.setPathSimplificationOptions(repair_simplification_options);
    planner.setConflictPathSimplificationOptions(
        repair_simplification_options);
    planner.setSimplifyInitialSolutions(simplify_conflict_solutions_);
    planner.setSimplifyConflictSolutions(simplify_conflict_solutions_);
    const auto status = planner.solve(time_limit);
    local_stats = planner.plannerStatsJson();
    if (status != ompl::base::PlannerStatus::EXACT_SOLUTION)
        return false;
    local_paths = planner.getSolutionPaths();
    return local_paths.size() == motion_indices.size();
}

void ScheduleARC::spliceWindow(
    const std::vector<std::size_t> &motion_indices,
    const RepairWindow &window, const std::vector<Path> &local_paths) {
    for (std::size_t i = 0; i < motion_indices.size(); ++i) {
        auto &motion = motions_.at(motion_indices[i]);
        auto &path = motion.path;
        if (path.empty())
            continue;
        const auto local_begin = window.begin_t - motion.start_t;
        const auto local_end = window.end_t - motion.start_t;
        if (local_begin >= path.size() || local_end >= path.size() ||
            local_begin > local_end) {
            throw std::runtime_error(
                "ScheduleARC splice window outside motion path");
        }
        const auto required_count = local_end - local_begin + 1;
        const auto replacement = resamplePath(local_paths.at(i), required_count);
        if (replacement.size() != required_count)
            throw std::runtime_error("ScheduleARC replacement path size mismatch");
        for (std::size_t k = 0; k < required_count; ++k)
            path[local_begin + k] = replacement[k];
        path.markDenseTimestepsImplicit();
        motion.start = path.front();
        motion.goal = path.back();
    }
}

bool ScheduleARC::repairConflict(
    const ScheduleArcConflict &conflict, const Clock::time_point &start,
    double time_limit) {
    auto motion_indices = motionsForConflict(conflict);
    auto maybe_window = initialRepairWindow(conflict, motion_indices);
    if (!maybe_window)
        return false;

    RepairWindow window = *maybe_window;
    ExpansionScheduleState expansion_state;
    std::uint64_t attempt_index = 0;
    for (;;) {
        if (cancellationRequested())
            return false;
        const double remaining = time_limit - elapsedSeconds(start);
        if (remaining <= 0.0)
            return false;

        const auto expanded_team = expandMotionTeamForWindow(
            motion_indices, conflict.timestep, window.begin_t, window.end_t);
        if (expanded_team != motion_indices) {
            motion_indices = expanded_team;
            expansion_state = ExpansionScheduleState{};
        }
        max_repair_team_size_ =
            std::max(max_repair_team_size_, motion_indices.size());

        std::size_t lower = 0;
        std::size_t upper = std::numeric_limits<std::size_t>::max();
        for (const auto motion_index : motion_indices) {
            const auto &motion = motions_.at(motion_index);
            lower = std::max(lower, motion.start_t);
            upper = std::min(upper, motion.end_t);
        }
        if (lower >= upper || conflict.timestep < lower ||
            conflict.timestep > upper) {
            return false;
        }
        window.begin_t = std::max(window.begin_t, lower);
        window.end_t = std::min(window.end_t, upper);
        if (window.begin_t >= window.end_t)
            return false;

        ++repair_attempts_;

        std::vector<Path> local_paths;
        const double local_budget = local_solve_time_limit_ > 0.0
                                        ? std::min(remaining,
                                                   local_solve_time_limit_)
                                        : remaining;
        const auto attempt_seed = scheduleArcRepairPlanningSeed(
            planning_seed_, conflicts_seen_, attempt_index);
        bool start_valid = false;
        bool goal_valid = false;
        nlohmann::json local_stats = nlohmann::json::object();
        const bool solved = solveWindow(
            motion_indices, window, local_budget, attempt_seed, local_paths,
            start_valid, goal_valid, local_stats);

        nlohmann::json event = {
            {"repair_index", conflicts_seen_},
            {"attempt_index", attempt_index},
            {"planning_seed", attempt_seed},
            {"motions", motion_indices},
            {"window_begin_t", window.begin_t},
            {"window_end_t", window.end_t},
            {"start_valid", start_valid},
            {"goal_valid", goal_valid},
            {"solved", solved},
        };
        if (!local_stats.empty())
            event["arc"] = std::move(local_stats);
        schedule_repair_attempt_events_.push_back(std::move(event));
        ++attempt_index;

        if (solved) {
            std::vector<int> visualization_motions;
            visualization_motions.reserve(motion_indices.size());
            for (const auto index : motion_indices)
                visualization_motions.push_back(static_cast<int>(index));
            appendVisualizationRepair(0, visualization_motions,
                                      static_cast<int>(window.begin_t),
                                      static_cast<int>(window.end_t),
                                      local_paths);
            spliceWindow(motion_indices, window, local_paths);
            applied_repairs_.push_back(
                AppliedRepair{motion_indices, window.begin_t, window.end_t});
            ++repair_successes_;
            return true;
        }

        const auto prev_begin = window.begin_t;
        const auto prev_end = window.end_t;
        const auto local_begin = static_cast<int>(window.begin_t - lower);
        const auto local_end = static_cast<int>(window.end_t - lower);
        const auto local_max = upper - lower;
        const auto next = nextExpansionWindowAfterAttempt(
            local_begin, local_end, local_max, start_valid, goal_valid,
            expansion_state);
        window.begin_t = lower + static_cast<std::size_t>(std::max(0, next.first));
        window.end_t = lower + static_cast<std::size_t>(std::max(0, next.second));
        window.begin_t = std::max(window.begin_t, lower);
        window.end_t = std::min(window.end_t, upper);
        if (window.begin_t == prev_begin && window.end_t == prev_end)
            return false;
        ++temporal_expansions_;
        if (expansion_state.last_expansion_used_initial_valid_schedule)
            ++initial_valid_temporal_expansions_;
        else
            ++main_temporal_expansions_;
    }
}

ompl::base::PlannerStatus ScheduleARC::solve(double timeLimit) {
    resetArcSolveState();
    conflicts_seen_ = 0;
    moving_conflicts_seen_ = 0;
    stationary_conflicts_seen_ = 0;
    environment_conflicts_seen_ = 0;
    repair_attempts_ = 0;
    repair_successes_ = 0;
    temporal_expansions_ = 0;
    initial_valid_temporal_expansions_ = 0;
    main_temporal_expansions_ = 0;
    max_repair_team_size_ = 0;
    applied_repairs_.clear();
    schedule_repair_attempt_events_ = nlohmann::json::array();

    if (!problem_) {
        throw std::runtime_error("ScheduleARC requires a MultiRobotProblem");
    }

    const auto start = Clock::now();
    for (std::size_t i = 0; i < motions_.size(); ++i) {
        auto &motion = motions_[i];
        const double remaining = timeLimit - elapsedSeconds(start);
        if (remaining <= 0.0 || cancellationRequested() ||
            !planMissingMotionPath(
                motion, remaining,
                scheduleArcInitialPlanningSeed(planning_seed_, i))) {
            updateStats(false);
            return ompl::base::PlannerStatus::TIMEOUT;
        }
    }

    if (global_makespan_bound_timesteps_ &&
        makespan() > *global_makespan_bound_timesteps_) {
        updateStats(false);
        return ompl::base::PlannerStatus::TIMEOUT;
    }

    while (elapsedSeconds(start) < timeLimit) {
        if (cancellationRequested()) {
            updateStats(false);
            return ompl::base::PlannerStatus::TIMEOUT;
        }
        startVisualizationIteration(getSolutionPaths());
        const auto conflict_detection_start = Clock::now();
        bool conflict_scan_stopped = false;
        const auto conflict = findFirstConflict(
            [&]() {
                return cancellationRequested() ||
                       elapsedSeconds(start) >= timeLimit;
            },
            &conflict_scan_stopped);
        conflict_detection_times_seconds_.push_back(
            elapsedSeconds(conflict_detection_start));
        if (conflict_scan_stopped) {
            updateStats(false);
            return ompl::base::PlannerStatus::TIMEOUT;
        }
        if (!conflict) {
            setVisualizationConflicts({});
            updateMetrics();
            updateStats(true);
            return ompl::base::PlannerStatus::EXACT_SOLUTION;
        }

        ++conflicts_seen_;
        if (conflict->type == ScheduleArcConflictType::MovingCollision)
            ++moving_conflicts_seen_;
        else if (conflict->type ==
                 ScheduleArcConflictType::StationaryCollision)
            ++stationary_conflicts_seen_;
        else
            ++environment_conflicts_seen_;

        SubproblemConflict visualization_conflict;
        visualization_conflict.conflict_timestep =
            static_cast<int>(conflict->timestep);
        visualization_conflict.window_begin_t = static_cast<int>(
            conflict->timestep > static_cast<std::size_t>(initial_window_)
                ? conflict->timestep - static_cast<std::size_t>(initial_window_)
                : 0);
        visualization_conflict.window_end_t = static_cast<int>(
            std::min(makespan(), conflict->timestep +
                                     static_cast<std::size_t>(initial_window_)));
        visualization_conflict.seed_robot_i =
            static_cast<int>(conflict->motion_i);
        if (conflict->type == ScheduleArcConflictType::MovingCollision)
            visualization_conflict.seed_robot_j =
                static_cast<int>(conflict->motion_j);
        const auto visualization_team = motionsForConflict(*conflict);
        for (const auto index : visualization_team)
            visualization_conflict.robots.push_back(static_cast<int>(index));
        setVisualizationConflicts({visualization_conflict});

        const auto repair_start = Clock::now();
        if (!repairConflict(*conflict, start, timeLimit)) {
            conflict_resolution_times_seconds_.push_back(
                elapsedSeconds(repair_start));
            if (failure_callback_)
                failure_callback_(*conflict);
            updateStats(false);
            return ompl::base::PlannerStatus::TIMEOUT;
        }
        conflict_resolution_times_seconds_.push_back(
            elapsedSeconds(repair_start));
    }

    updateStats(false);
    return ompl::base::PlannerStatus::TIMEOUT;
}

std::vector<Path> ScheduleARC::getSolutionPaths() const {
    std::vector<Path> out;
    out.reserve(motions_.size());
    for (const auto &motion : motions_)
        out.push_back(motion.path);
    return out;
}

void ScheduleARC::updateMetrics() {
    std::uint64_t sum = 0;
    for (const auto &motion : motions_) {
        if (motion.end_t >= motion.start_t)
            sum += static_cast<std::uint64_t>(motion.end_t - motion.start_t);
    }
    setSolutionMetrics(sum, static_cast<std::uint64_t>(makespan()));
}

void ScheduleARC::updateStats(bool solved) {
    nlohmann::json stats = nlohmann::json::object();
    stats["solved"] = solved;
    stats["num_motions"] = motions_.size();
    stats["num_stationary_entities"] = stationary_entities_.size();
    stats["num_conflicts"] = conflicts_seen_;
    stats["num_moving_conflicts"] = moving_conflicts_seen_;
    stats["num_stationary_conflicts"] = stationary_conflicts_seen_;
    stats["num_environment_conflicts"] = environment_conflicts_seen_;
    stats["num_repair_attempts"] = repair_attempts_;
    stats["num_repair_successes"] = repair_successes_;
    stats["num_temporal_expansions"] = temporal_expansions_;
    stats["num_initial_valid_temporal_expansions"] =
        initial_valid_temporal_expansions_;
    stats["num_main_temporal_expansions"] = main_temporal_expansions_;
    stats["max_repair_team_size"] = max_repair_team_size_;
    stats["num_applied_repairs"] = applied_repairs_.size();
    stats["repair_attempt_events"] = schedule_repair_attempt_events_;
    stats["planning_seed"] = planning_seed_;
    stats["local_solver_mode"] = static_cast<int>(local_solver_mode_);
    stats["expansion_policy"] = static_cast<int>(expansion_policy_);
    stats["local_solve_time_limit_seconds"] = local_solve_time_limit_;
    stats["conflict_detection_times_seconds"] =
        conflict_detection_times_seconds_;
    stats["conflict_resolution_times_seconds"] =
        conflict_resolution_times_seconds_;
    stats["initial_window"] = initial_window_;
    stats["expansion_step"] = expansion_step_;
    setPlannerStatsJson(std::move(stats));
}

} // namespace comotion
