#pragma once

#include "comotion/collision/ObstacleShapes.h"
#include "comotion/planning/ARC.h"
#include "comotion/planning/Path.h"
#include "comotion/robot/RobotModel.h"

#include <algorithm>
#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace comotion {

enum class ScheduleArcConflictType {
    MovingCollision,
    StationaryCollision,
    EnvironmentCollision,
};

struct ScheduleArcMotion {
    std::size_t step_index = 0;
    std::size_t coupling_id = std::numeric_limits<std::size_t>::max();
    std::string label;
    std::string robot_name;
    std::shared_ptr<RobotModel> model;
    Path path;
    std::vector<double> start;
    std::vector<double> goal;
    std::size_t start_t = 0;
    std::size_t end_t = 0;
    std::vector<std::string> moving_entities;
    std::vector<std::string> ignored_stationary_entities;
};

struct ScheduleArcStationaryEntity {
    std::string name;
    std::size_t start_t = 0;
    std::size_t end_t = 0;
    std::vector<ObstacleSphere> spheres;
    std::vector<ObstacleCylinder> cylinders;
};

struct ScheduleArcConflict {
    ScheduleArcConflictType type = ScheduleArcConflictType::MovingCollision;
    std::size_t timestep = 0;
    std::size_t motion_i = std::numeric_limits<std::size_t>::max();
    std::size_t motion_j = std::numeric_limits<std::size_t>::max();
    std::string moving_entity_i;
    std::string moving_entity_j;
    std::string stationary_entity;
    std::string label_i;
    std::string label_j;
};

std::string toString(ScheduleArcConflictType type);
std::string describeScheduleArcConflict(const ScheduleArcConflict &conflict);

/// Schedule-aware ARC variant for optimistic task-motion schedules.
///
/// Each motion has a local path and a global schedule interval. Conflict checks
/// only compare motions that are active at a timestep and skip motions from the
/// same coupling, assuming those local coupled plans are already internally
/// valid. Stationary conflicts are repaired by adding the stationary entity as
/// a static obstacle to the local subproblem. Supplied sparse paths are sampled
/// using Path timestep metadata and normalized to the fixed schedule interval.
/// Repairs retain coupled teams and use the current ARC repair engine and its
/// inherited expansion, local-solver, bounds, simplification, tracing, seed,
/// and cancellation settings.
class ScheduleARC : public ARC {
public:
    using FailureCallback =
        std::function<void(const ScheduleArcConflict &)>;

    std::string name() const override { return "ScheduleARC"; }
    ompl::base::PlannerStatus solve(double timeLimit) override;
    std::vector<Path> getSolutionPaths() const override;

    void setMotions(std::vector<ScheduleArcMotion> motions);
    const std::vector<ScheduleArcMotion> &motions() const { return motions_; }

    void setStationaryEntities(
        std::vector<ScheduleArcStationaryEntity> entities);
    const std::vector<ScheduleArcStationaryEntity> &stationaryEntities() const {
        return stationary_entities_;
    }

    void setFailureCallback(FailureCallback callback) {
        failure_callback_ = std::move(callback);
    }

    using ARC::setExpansionStep;
    using ARC::setInitialWindow;
    void setInitialWindow(std::size_t window) {
        ARC::setInitialWindow(static_cast<int>(std::min<std::size_t>(
            window, static_cast<std::size_t>(std::numeric_limits<int>::max()))));
    }
    /// Optional compatibility cap for an individual schedule-window attempt.
    /// A non-positive value (the default) gives the current ARC engine the full
    /// remaining global budget.
    void setLocalSolveTimeLimit(double seconds) {
        local_solve_time_limit_ = seconds > 0.0 ? seconds : 0.0;
    }

    std::optional<ScheduleArcConflict> findFirstConflict() const;

private:
    struct RepairWindow {
        std::size_t begin_t = 0;
        std::size_t end_t = 0;
    };

    static bool intervalContains(const ScheduleArcMotion &motion,
                                 std::size_t timestep);
    static bool intervalOverlaps(std::size_t a_begin, std::size_t a_end,
                                 std::size_t b_begin, std::size_t b_end);
    static std::vector<double>
    configAt(const ScheduleArcMotion &motion, std::size_t timestep);
    static Path resamplePath(const Path &path, std::size_t waypoint_count);
    static void normalizeMotionPath(ScheduleArcMotion &motion);
    static bool hasEntity(const std::vector<std::string> &entities,
                          const std::string &entity);
    std::optional<ScheduleArcConflict> findFirstConflict(
        const std::function<bool()> &stop_requested, bool *stopped) const;

    bool shouldSkipMovingPair(const ScheduleArcMotion &lhs,
                              const ScheduleArcMotion &rhs) const;
    bool shouldSkipStationaryEntity(const ScheduleArcMotion &motion,
                                    const std::string &entity) const;
    std::vector<std::size_t>
    motionsForConflict(const ScheduleArcConflict &conflict) const;
    std::vector<std::size_t> expandMotionTeamForWindow(
        std::vector<std::size_t> motion_indices, std::size_t conflict_timestep,
        std::size_t begin_t, std::size_t end_t) const;
    bool motionsAreCoupled(const ScheduleArcMotion &lhs,
                           const ScheduleArcMotion &rhs) const;
    std::optional<RepairWindow>
    initialRepairWindow(const ScheduleArcConflict &conflict,
                        const std::vector<std::size_t> &motion_indices) const;
    std::vector<ObstacleSphere>
    stationarySpheresForWindow(std::size_t begin_t, std::size_t end_t,
                               const std::vector<std::size_t> &motion_indices)
        const;
    std::vector<ObstacleCylinder>
    stationaryCylindersForWindow(std::size_t begin_t, std::size_t end_t,
                                 const std::vector<std::size_t> &motion_indices)
        const;

    bool planMissingMotionPath(ScheduleArcMotion &motion, double time_limit,
                               std::uint32_t motion_seed);
    bool repairConflict(const ScheduleArcConflict &conflict,
                        const std::chrono::steady_clock::time_point &start,
                        double time_limit);
    bool solveWindow(const std::vector<std::size_t> &motion_indices,
                     const RepairWindow &window, double time_limit,
                     std::uint32_t attempt_seed,
                     std::vector<Path> &local_paths, bool &start_valid,
                     bool &goal_valid, nlohmann::json &local_stats) const;
    void spliceWindow(const std::vector<std::size_t> &motion_indices,
                      const RepairWindow &window,
                      const std::vector<Path> &local_paths);
    std::size_t makespan() const;
    void updateMetrics();
    void updateStats(bool solved);

    struct AppliedRepair {
        std::vector<std::size_t> motions;
        std::size_t begin_t = 0;
        std::size_t end_t = 0;
    };

    std::vector<ScheduleArcMotion> motions_;
    std::vector<ScheduleArcStationaryEntity> stationary_entities_;
    FailureCallback failure_callback_;
    double local_solve_time_limit_ = 0.0;
    std::uint64_t conflicts_seen_ = 0;
    std::uint64_t moving_conflicts_seen_ = 0;
    std::uint64_t stationary_conflicts_seen_ = 0;
    std::uint64_t environment_conflicts_seen_ = 0;
    std::uint64_t repair_attempts_ = 0;
    std::uint64_t repair_successes_ = 0;
    std::uint64_t temporal_expansions_ = 0;
    std::uint64_t initial_valid_temporal_expansions_ = 0;
    std::uint64_t main_temporal_expansions_ = 0;
    std::size_t max_repair_team_size_ = 0;
    std::vector<AppliedRepair> applied_repairs_;
    nlohmann::json schedule_repair_attempt_events_ = nlohmann::json::array();
};

} // namespace comotion
