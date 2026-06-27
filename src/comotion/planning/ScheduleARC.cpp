#include "comotion/planning/ScheduleARC.h"

#include "comotion/collision/CollisionChecker.h"
#include "comotion/planning/CompositeRRT.h"
#include "comotion/planning/PlanningSeed.h"

#include <algorithm>
#include <chrono>
#include <cmath>
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

std::vector<double> interpolateByIndex(const comotion::Path &path,
                                       double index) {
    if (path.empty())
        return {};
    if (path.size() == 1 || index <= 0.0)
        return path.front();
    const double max_index = static_cast<double>(path.size() - 1);
    if (index >= max_index)
        return path.back();

    const auto lo = static_cast<std::size_t>(std::floor(index));
    const auto hi = std::min<std::size_t>(lo + 1, path.size() - 1);
    const double alpha = index - static_cast<double>(lo);
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
    } else {
        out << " between motion " << conflict.motion_i << " ("
            << conflict.label_i << ") and stationary entity "
            << conflict.stationary_entity;
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
            if (motion.end_t < motion.start_t)
                motion.end_t = motion.start_t + motion.path.size() - 1;
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

const std::vector<double> &
ScheduleARC::configAt(const ScheduleArcMotion &motion, std::size_t timestep) {
    if (motion.path.empty())
        throw std::runtime_error("ScheduleARC motion path is empty");
    if (timestep <= motion.start_t)
        return motion.path.front();
    const auto local = std::min<std::size_t>(
        timestep - motion.start_t, motion.path.size() - 1);
    return motion.path[local];
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

    const double scale = static_cast<double>(path.size() - 1) /
                         static_cast<double>(waypoint_count - 1);
    for (std::size_t i = 0; i < waypoint_count; ++i) {
        out.push_back(interpolateByIndex(path, static_cast<double>(i) * scale));
    }
    out.markDenseTimestepsImplicit();
    return out;
}

bool ScheduleARC::hasEntity(const std::vector<std::string> &entities,
                            const std::string &entity) {
    return std::find(entities.begin(), entities.end(), entity) !=
           entities.end();
}

bool ScheduleARC::shouldSkipMovingPair(const ScheduleArcMotion &lhs,
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
    if (!problem_)
        throw std::runtime_error("ScheduleARC requires a MultiRobotProblem");

    CollisionChecker checker(problem_->collisionChecker().backend());
    const auto horizon = makespan();
    for (std::size_t t = 0; t <= horizon; ++t) {
        std::vector<std::size_t> active;
        for (std::size_t i = 0; i < motions_.size(); ++i) {
            if (intervalContains(motions_[i], t))
                active.push_back(i);
        }

        for (std::size_t ai = 0; ai < active.size(); ++ai) {
            const auto i = active[ai];
            const auto &lhs = motions_[i];
            for (std::size_t aj = ai + 1; aj < active.size(); ++aj) {
                const auto j = active[aj];
                const auto &rhs = motions_[j];
                if (shouldSkipMovingPair(lhs, rhs))
                    continue;
                if (!checker.isValidPair(*lhs.model, configAt(lhs, t),
                                         *rhs.model, configAt(rhs, t))) {
                    ScheduleArcConflict conflict;
                    conflict.type = ScheduleArcConflictType::MovingCollision;
                    conflict.timestep = t;
                    conflict.motion_i = i;
                    conflict.motion_j = j;
                    conflict.label_i = lhs.label;
                    conflict.label_j = rhs.label;
                    conflict.moving_entity_i = lhs.robot_name;
                    conflict.moving_entity_j = rhs.robot_name;
                    return conflict;
                }
            }
        }

        for (const auto motion_index : active) {
            const auto &motion = motions_[motion_index];
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
    return out;
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
    std::set<std::string> ignored_entities;
    for (const auto motion_index : motion_indices) {
        const auto &motion = motions_.at(motion_index);
        moving_entities.insert(motion.moving_entities.begin(),
                               motion.moving_entities.end());
        ignored_entities.insert(motion.ignored_stationary_entities.begin(),
                                motion.ignored_stationary_entities.end());
    }

    std::vector<ObstacleSphere> out;
    for (const auto &entity : stationary_entities_) {
        if (!intervalOverlaps(begin_t, end_t, entity.start_t, entity.end_t))
            continue;
        if (moving_entities.count(entity.name) != 0 ||
            ignored_entities.count(entity.name) != 0) {
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
    std::set<std::string> ignored_entities;
    for (const auto motion_index : motion_indices) {
        const auto &motion = motions_.at(motion_index);
        moving_entities.insert(motion.moving_entities.begin(),
                               motion.moving_entities.end());
        ignored_entities.insert(motion.ignored_stationary_entities.begin(),
                                motion.ignored_stationary_entities.end());
    }

    std::vector<ObstacleCylinder> out;
    for (const auto &entity : stationary_entities_) {
        if (!intervalOverlaps(begin_t, end_t, entity.start_t, entity.end_t))
            continue;
        if (moving_entities.count(entity.name) != 0 ||
            ignored_entities.count(entity.name) != 0) {
            continue;
        }
        out.insert(out.end(), entity.cylinders.begin(), entity.cylinders.end());
    }
    return out;
}

bool ScheduleARC::planMissingMotionPath(ScheduleArcMotion &motion,
                                        double time_limit) {
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
    local_problem->addRobot(motion.model, motion.start, motion.goal);

    CompositeRRT planner;
    planner.setProblem(local_problem);
    planner.setPlanningSeed(planning_seed_);
    planner.setSimplifySolution(true);
    const auto status = planner.solve(time_limit);
    if (status != ompl::base::PlannerStatus::EXACT_SOLUTION)
        return false;
    auto paths = planner.getSolutionPaths();
    if (paths.empty() || paths.front().empty())
        return false;
    motion.path = std::move(paths.front());
    if (motion.end_t < motion.start_t)
        motion.end_t = motion.start_t + motion.path.size() - 1;
    return true;
}

bool ScheduleARC::solveWindow(
    const std::vector<std::size_t> &motion_indices,
    const RepairWindow &window, double time_limit,
    std::vector<Path> &local_paths) const {
    auto local_problem = std::make_shared<MultiRobotProblem>(
        problem_->collisionChecker().backend());
    local_problem->setResolution(problem_->resolution());
    local_problem->setVmax(problem_->vmax());

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

    CompositeRRT planner;
    planner.setProblem(local_problem);
    planner.setPlanningSeed(planning_seed_);
    planner.setSimplifySolution(true);
    planner.setCancellationCallback(cancel_requested_);
    const auto status = planner.solve(time_limit);
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
    const auto motion_indices = motionsForConflict(conflict);
    auto maybe_window = initialRepairWindow(conflict, motion_indices);
    if (!maybe_window)
        return false;

    RepairWindow window = *maybe_window;
    for (;;) {
        if (cancellationRequested())
            return false;
        const double remaining = time_limit - elapsedSeconds(start);
        if (remaining <= 0.0)
            return false;
        ++repair_attempts_;

        std::vector<Path> local_paths;
        const double local_budget =
            std::min(remaining, std::max(0.01, local_solve_time_limit_));
        if (solveWindow(motion_indices, window, local_budget, local_paths)) {
            spliceWindow(motion_indices, window, local_paths);
            ++repair_successes_;
            return true;
        }

        std::size_t lower = 0;
        std::size_t upper = std::numeric_limits<std::size_t>::max();
        for (const auto motion_index : motion_indices) {
            const auto &motion = motions_.at(motion_index);
            lower = std::max(lower, motion.start_t);
            upper = std::min(upper, motion.end_t);
        }
        const auto prev_begin = window.begin_t;
        const auto prev_end = window.end_t;
        window.begin_t =
            window.begin_t > expansion_step_
                ? std::max(lower, window.begin_t - expansion_step_)
                : lower;
        window.end_t = std::min(upper, window.end_t + expansion_step_);
        if (window.begin_t == prev_begin && window.end_t == prev_end)
            return false;
        ++temporal_expansions_;
    }
}

ompl::base::PlannerStatus ScheduleARC::solve(double timeLimit) {
    resetPlannerRunMetrics();
    conflicts_seen_ = 0;
    moving_conflicts_seen_ = 0;
    stationary_conflicts_seen_ = 0;
    repair_attempts_ = 0;
    repair_successes_ = 0;
    temporal_expansions_ = 0;

    if (!problem_) {
        throw std::runtime_error("ScheduleARC requires a MultiRobotProblem");
    }

    const auto start = Clock::now();
    for (auto &motion : motions_) {
        const double remaining = timeLimit - elapsedSeconds(start);
        if (remaining <= 0.0 || !planMissingMotionPath(motion, remaining)) {
            updateStats(false);
            return ompl::base::PlannerStatus::TIMEOUT;
        }
    }

    while (elapsedSeconds(start) < timeLimit) {
        const auto conflict = findFirstConflict();
        if (!conflict) {
            updateMetrics();
            updateStats(true);
            return ompl::base::PlannerStatus::EXACT_SOLUTION;
        }

        ++conflicts_seen_;
        if (conflict->type == ScheduleArcConflictType::MovingCollision)
            ++moving_conflicts_seen_;
        else
            ++stationary_conflicts_seen_;

        if (!repairConflict(*conflict, start, timeLimit)) {
            if (failure_callback_)
                failure_callback_(*conflict);
            updateStats(false);
            return ompl::base::PlannerStatus::TIMEOUT;
        }
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
    stats["num_repair_attempts"] = repair_attempts_;
    stats["num_repair_successes"] = repair_successes_;
    stats["num_temporal_expansions"] = temporal_expansions_;
    stats["initial_window"] = initial_window_;
    stats["expansion_step"] = expansion_step_;
    setPlannerStatsJson(std::move(stats));
}

} // namespace comotion
