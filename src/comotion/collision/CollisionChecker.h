#pragma once

#include "comotion/collision/ObstacleShapes.h"
#include "comotion/collision/ValidationTypes.h"
#include "comotion/planning/Path.h"
#include "comotion/robot/RobotModel.h"

#include <Eigen/Core>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

namespace comotion {

class CollisionChecker {
public:
    enum class Backend { Spheres, Fcl, Vamp };

    CollisionChecker();
    explicit CollisionChecker(Backend backend);
    ~CollisionChecker();

    CollisionChecker(const CollisionChecker &other);
    CollisionChecker &operator=(const CollisionChecker &other);
    CollisionChecker(CollisionChecker &&other) noexcept;
    CollisionChecker &operator=(CollisionChecker &&other) noexcept;

    Backend backend() const { return backend_; }

    void setObstacles(const std::vector<ObstacleSphere> &obstacles);
    void setCylinderObstacles(const std::vector<ObstacleCylinder> &cylinders);
    void setVampValidationStrategy(const VampValidationStrategy &strategy);
    VampValidationStrategy vampValidationStrategy() const;
    ValidationWorkStats lastValidationWorkStats() const;
    static void resetValidationTimingStats();
    static ValidationTimingStats validationTimingStats();

    bool isValidSingle(const RobotModel &robot,
                       const std::vector<double> &config) const;

    bool isSelfCollisionFree(const RobotModel &robot,
                             const std::vector<double> &config) const;

    bool isValidSingleFull(const RobotModel &robot,
                           const std::vector<double> &config) const;

    bool isValidPair(const RobotModel &robot_a,
                     const std::vector<double> &config_a,
                     const RobotModel &robot_b,
                     const std::vector<double> &config_b) const;

    bool isValidComposite(
        const std::vector<const RobotModel *> &robots,
        const std::vector<std::vector<double>> &configs) const;

    bool isMotionValid(const RobotModel &robot,
                       const std::vector<double> &from,
                       const std::vector<double> &to,
                       int num_checks = 10) const;

    bool isRobotPathValid(const RobotModel &robot, const Path &path) const;

    bool isPairPathValid(
        const RobotModel &robot_a, const Path &path_a,
        const RobotModel &robot_b, const Path &path_b,
        std::size_t t_begin = 0,
        std::size_t t_end = std::numeric_limits<std::size_t>::max()) const;

    std::optional<PairPathConflict> findFirstPairPathConflict(
        const RobotModel &robot_a, const Path &path_a,
        const RobotModel &robot_b, const Path &path_b,
        std::size_t t_begin = 0,
        std::size_t t_end = std::numeric_limits<std::size_t>::max()) const;

    GoalHoldConstraint computeGoalHoldConstraint(
        const RobotModel &goal_robot,
        const std::vector<double> &goal_config,
        const RobotModel &prior_robot,
        const Path &prior_path) const;

    bool isCompositeMotionValid(
        const std::vector<const RobotModel *> &robots,
        const std::vector<std::vector<double>> &from,
        const std::vector<std::vector<double>> &to,
        const CompositePathValidationOptions &options = {}) const;

    std::optional<CompositeConflict> findFirstCompositeMotionConflict(
        const std::vector<const RobotModel *> &robots,
        const std::vector<std::vector<double>> &from,
        const std::vector<std::vector<double>> &to,
        const CompositePathValidationOptions &options = {}) const;

    bool validateCompositePaths(
        const std::vector<Path> &paths,
        const std::vector<const RobotModel *> &robots,
        const CompositePathValidationOptions &options = {}) const;

    std::optional<CompositeConflict> findFirstCompositePathConflict(
        const std::vector<Path> &paths,
        const std::vector<const RobotModel *> &robots,
        const CompositePathValidationOptions &options = {},
        std::vector<std::size_t> *next_t_begin_by_robot_out = nullptr) const;

    std::vector<CompositeConflict> findInterRobotPathConflictsCompositeScan(
        const std::vector<Path> &paths,
        const std::vector<const RobotModel *> &robots,
        const CompositePathValidationOptions &options,
        std::size_t max_conflicts, bool unique,
        const InterRobotConflictCallback &on_conflict = {},
        std::vector<std::size_t> *next_t_begin_by_robot_out = nullptr,
        std::vector<std::size_t> *next_t_begin_by_pair_out = nullptr) const;

    const std::vector<ObstacleSphere> &obstacles() const { return obstacles_; }
    const std::vector<ObstacleCylinder> &cylinders() const { return cylinders_; }

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    Backend backend_;
    std::vector<ObstacleSphere> obstacles_;
    std::vector<ObstacleCylinder> cylinders_;
};

} // namespace comotion
