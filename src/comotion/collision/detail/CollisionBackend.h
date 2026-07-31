#pragma once

#include "comotion/collision/ObstacleShapes.h"
#include "comotion/collision/ValidationTypes.h"
#include "comotion/planning/Path.h"
#include "comotion/robot/RobotModel.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace comotion {
namespace detail {

inline std::uint64_t choose2(std::size_t n) {
    if (n < 2)
        return 0;
    return static_cast<std::uint64_t>(n) *
           static_cast<std::uint64_t>(n - 1) / 2;
}

inline std::vector<std::size_t> effectivePathStarts(
    std::size_t path_count, const CompositePathValidationOptions &options) {
    if (!options.per_path_t_begin.empty() &&
        options.per_path_t_begin.size() != path_count) {
        throw std::invalid_argument(
            "CompositePathValidationOptions.per_path_t_begin size must "
            "match path count");
    }

    std::vector<std::size_t> starts(path_count, options.t_begin);
    if (options.per_path_t_begin.empty())
        return starts;

    for (std::size_t i = 0; i < path_count; ++i) {
        starts[i] = std::max(options.t_begin, options.per_path_t_begin[i]);
    }
    return starts;
}

inline std::vector<std::size_t> effectivePairStarts(
    std::size_t path_count, const CompositePathValidationOptions &options,
    const std::vector<std::size_t> &path_starts) {
    if (path_starts.size() != path_count) {
        throw std::invalid_argument(
            "effectivePairStarts path start size must match path count");
    }
    const std::size_t pair_count = pairFrontierSize(path_count);
    if (!options.per_pair_t_begin.empty() &&
        options.per_pair_t_begin.size() != pair_count) {
        throw std::invalid_argument(
            "CompositePathValidationOptions.per_pair_t_begin size must "
            "match path pair count");
    }

    std::vector<std::size_t> starts(pair_count, options.t_begin);
    for (std::size_t i = 0; i < path_count; ++i) {
        for (std::size_t j = i + 1; j < path_count; ++j) {
            const std::size_t pair_index =
                pairFrontierIndex(i, j, path_count);
            std::size_t start = std::max(path_starts[i], path_starts[j]);
            if (!options.per_pair_t_begin.empty()) {
                start = std::max(start, options.per_pair_t_begin[pair_index]);
            }
            starts[pair_index] = start;
        }
    }
    return starts;
}

inline void initializeNextPairStarts(std::vector<std::size_t> *next_out,
                                     const std::vector<std::size_t> &starts) {
    if (next_out)
        *next_out = starts;
}

inline void initializeNextPathStarts(std::vector<std::size_t> *next_out,
                                     const std::vector<std::size_t> &starts) {
    if (next_out)
        *next_out = starts;
}

inline void assignActivePathStarts(std::vector<std::size_t> *next_out,
                                   const std::vector<std::size_t> &active,
                                   std::size_t value) {
    if (!next_out)
        return;
    for (const std::size_t robot : active)
        (*next_out)[robot] = value;
}

struct AcceptedInterRobotConflictClaim {
    std::size_t timestep = 0;
    std::vector<int> robots;
};

inline std::vector<int>
defaultConflictClaimRobots(const CompositeConflict &conflict) {
    std::vector<int> robots;
    if (conflict.robot_i >= 0)
        robots.push_back(conflict.robot_i);
    if (conflict.robot_j >= 0)
        robots.push_back(conflict.robot_j);
    std::sort(robots.begin(), robots.end());
    robots.erase(std::unique(robots.begin(), robots.end()), robots.end());
    return robots;
}

inline bool acceptInterRobotConflictCandidate(
    const CompositeConflict &conflict,
    const InterRobotConflictCallback &on_conflict, bool unique,
    std::vector<char> &robot_used, std::vector<CompositeConflict> &out,
    std::vector<AcceptedInterRobotConflictClaim> &accepted_claims,
    InterRobotConflictBatchMode batch_mode =
        InterRobotConflictBatchMode::OptimisticIndependent) {
    InterRobotConflictDecision decision;
    if (on_conflict)
        decision = on_conflict(conflict);
    else
        decision.robots_to_claim = defaultConflictClaimRobots(conflict);

    if (!decision.accept)
        return false;

    if (decision.robots_to_claim.empty())
        decision.robots_to_claim = defaultConflictClaimRobots(conflict);

    std::sort(decision.robots_to_claim.begin(),
              decision.robots_to_claim.end());
    decision.robots_to_claim.erase(
        std::unique(decision.robots_to_claim.begin(),
                    decision.robots_to_claim.end()),
        decision.robots_to_claim.end());

    bool overlaps_used_robot = false;
    for (const int robot : decision.robots_to_claim) {
        if (robot < 0 ||
            static_cast<std::size_t>(robot) >= robot_used.size()) {
            throw std::runtime_error(
                "Inter-robot conflict claimed robot index out of range");
        }
        if (unique && robot_used[static_cast<std::size_t>(robot)])
            overlaps_used_robot = true;
    }

    if (unique && overlaps_used_robot) {
        if (batch_mode == InterRobotConflictBatchMode::IndependentOnly) {
            for (const int robot : decision.robots_to_claim)
                robot_used[static_cast<std::size_t>(robot)] = 1;
        }
        return false;
    }

    out.push_back(conflict);
    accepted_claims.push_back(
        AcceptedInterRobotConflictClaim{conflict.timestep,
                                        decision.robots_to_claim});
    if (unique) {
        for (const int robot : decision.robots_to_claim)
            robot_used[static_cast<std::size_t>(robot)] = 1;
    }
    return true;
}

inline void assignUniqueConflictFrontier(
    std::vector<std::size_t> *next_out,
    const std::vector<std::size_t> &starts,
    const std::vector<AcceptedInterRobotConflictClaim> &accepted_claims) {
    if (!next_out)
        return;
    *next_out = starts;
    for (const auto &claim : accepted_claims) {
        for (const int robot : claim.robots) {
            if (robot < 0 ||
                static_cast<std::size_t>(robot) >= next_out->size()) {
                throw std::runtime_error(
                    "Inter-robot conflict frontier robot index out of range");
            }
            (*next_out)[static_cast<std::size_t>(robot)] = claim.timestep;
        }
    }
}

class ActiveRobotSchedule {
public:
    explicit ActiveRobotSchedule(const std::vector<std::size_t> &starts)
        : starts_(starts), order_(starts.size(), 0), cursor_(0) {
        for (std::size_t i = 0; i < order_.size(); ++i)
            order_[i] = i;
        std::sort(order_.begin(), order_.end(), [&](std::size_t lhs,
                                                    std::size_t rhs) {
            if (starts_[lhs] != starts_[rhs])
                return starts_[lhs] < starts_[rhs];
            return lhs < rhs;
        });
    }

    void activateThrough(std::size_t timestep, std::vector<std::size_t> &active) {
        while (cursor_ < order_.size() &&
               starts_[order_[cursor_]] <= timestep) {
            active.push_back(order_[cursor_]);
            ++cursor_;
        }
    }

private:
    const std::vector<std::size_t> &starts_;
    std::vector<std::size_t> order_;
    std::size_t cursor_;
};

struct CollisionBackend {
    virtual ~CollisionBackend() = default;

    virtual std::unique_ptr<CollisionBackend> clone() const = 0;

    virtual void setVampValidationStrategy(
        const VampValidationStrategy &) {}

    virtual VampValidationStrategy vampValidationStrategy() const {
        return {};
    }

    virtual void onEnvironmentChanged(
        const std::vector<ObstacleSphere> &spheres,
        const std::vector<ObstacleCylinder> &cylinders) = 0;

    virtual bool isValidSingle(
        const RobotModel &robot, const std::vector<double> &config,
        const std::vector<ObstacleSphere> &obstacles,
        const std::vector<ObstacleCylinder> &cylinders) const = 0;

    virtual bool isSelfCollisionFree(
        const RobotModel &robot, const std::vector<double> &config) const = 0;

    virtual bool isValidPair(
        const RobotModel &robot_a, const std::vector<double> &config_a,
        const RobotModel &robot_b, const std::vector<double> &config_b) const = 0;

    virtual bool isMotionValid(
        const RobotModel &robot, const std::vector<double> &from,
        const std::vector<double> &to, int num_checks,
        const std::vector<ObstacleSphere> &obstacles,
        const std::vector<ObstacleCylinder> &cylinders) const = 0;

    virtual bool isRobotPathValid(
        const RobotModel &robot, const Path &path,
        const std::vector<ObstacleSphere> &obstacles,
        const std::vector<ObstacleCylinder> &cylinders) const = 0;

    virtual bool isPairPathValid(
        const RobotModel &robot_a, const Path &path_a,
        const RobotModel &robot_b, const Path &path_b,
        std::size_t t_begin, std::size_t t_end) const = 0;

    virtual std::optional<PairPathConflict> findFirstPairPathConflict(
        const RobotModel &robot_a, const Path &path_a,
        const RobotModel &robot_b, const Path &path_b,
        std::size_t t_begin, std::size_t t_end) const = 0;

    virtual GoalHoldConstraint computeGoalHoldConstraint(
        const RobotModel &goal_robot, const std::vector<double> &goal_config,
        const RobotModel &prior_robot, const Path &prior_path) const = 0;

    virtual bool isCompositeMotionValid(
        const std::vector<const RobotModel *> &robots,
        const std::vector<std::vector<double>> &from,
        const std::vector<std::vector<double>> &to,
        const CompositePathValidationOptions &options,
        const std::vector<ObstacleSphere> &obstacles,
        const std::vector<ObstacleCylinder> &cylinders) const = 0;

    virtual std::optional<CompositeConflict> findFirstCompositeMotionConflict(
        const std::vector<const RobotModel *> &robots,
        const std::vector<std::vector<double>> &from,
        const std::vector<std::vector<double>> &to,
        const CompositePathValidationOptions &options,
        const std::vector<ObstacleSphere> &obstacles,
        const std::vector<ObstacleCylinder> &cylinders) const = 0;

    virtual bool validateCompositePaths(
        const std::vector<Path> &paths,
        const std::vector<const RobotModel *> &robots,
        const CompositePathValidationOptions &options,
        const std::vector<ObstacleSphere> &obstacles,
        const std::vector<ObstacleCylinder> &cylinders) const = 0;

    virtual std::optional<CompositeConflict> findFirstCompositePathConflict(
        const std::vector<Path> &paths,
        const std::vector<const RobotModel *> &robots,
        const CompositePathValidationOptions &options,
        const std::vector<ObstacleSphere> &obstacles,
        const std::vector<ObstacleCylinder> &cylinders,
        std::vector<std::size_t> *next_t_begin_by_robot_out) const = 0;

    // Inter-robot vertex conflicts only, in composite timestep order (same as
    // findFirstCompositePathConflict inter-robot loop). Ignores environment.
    virtual std::vector<CompositeConflict>
    findInterRobotPathConflictsCompositeScan(
        const std::vector<Path> &paths,
        const std::vector<const RobotModel *> &robots,
        const CompositePathValidationOptions &options,
        std::size_t max_conflicts, bool unique,
        const InterRobotConflictCallback &on_conflict,
        const std::vector<ObstacleSphere> &obstacles,
        const std::vector<ObstacleCylinder> &cylinders,
        std::vector<std::size_t> *next_t_begin_by_robot_out,
        std::vector<std::size_t> *next_t_begin_by_pair_out) const = 0;

};

std::unique_ptr<CollisionBackend> makeSphereBackend();
std::unique_ptr<CollisionBackend> makeFclBackend();
std::unique_ptr<CollisionBackend> makeVampBackend();

} // namespace detail
} // namespace comotion
