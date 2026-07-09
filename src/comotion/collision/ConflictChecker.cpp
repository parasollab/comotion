#include "comotion/collision/ConflictChecker.h"
#include "comotion/collision/detail/CollisionBackend.h"

#include <algorithm>
#include <stdexcept>

namespace {

void normalizeRobots(std::vector<int> &robots) {
    std::sort(robots.begin(), robots.end());
    robots.erase(std::unique(robots.begin(), robots.end()), robots.end());
}

} // namespace

namespace comotion {

std::vector<double> ConflictChecker::configAt(const Path &path,
                                              std::size_t t) {
    return path.config_at_timestep(t);
}

SubproblemConflict
ConflictChecker::defaultExpandedConflict(const Conflict &conflict) {
    SubproblemConflict expanded;
    expanded.robots = {conflict.robot_i, conflict.robot_j};
    normalizeRobots(expanded.robots);
    expanded.conflict_timestep = conflict.timestep;
    expanded.window_begin_t = conflict.timestep;
    expanded.window_end_t = conflict.timestep;
    expanded.seed_robot_i = conflict.robot_i;
    expanded.seed_robot_j = conflict.robot_j;
    expanded.alpha = conflict.alpha;
    expanded.kind = conflict.kind;
    expanded.config_i = conflict.config_i;
    expanded.config_j = conflict.config_j;
    return expanded;
}

bool ConflictChecker::isConflict(const Path &path_a,
                                 const RobotModel &robot_a, const Path &path_b,
                                 const RobotModel &robot_b) const {
    return !cc_.isPairPathValid(robot_a, path_a, robot_b, path_b);
}

bool ConflictChecker::isConflict(
    const std::vector<Path> &paths,
    const std::vector<const RobotModel *> &robots) const {
    if (paths.size() != robots.size())
        return true;
    int n = static_cast<int>(paths.size());
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            if (isConflict(paths[i], *robots[i], paths[j], *robots[j]))
                return true;
        }
    }
    return false;
}

bool ConflictChecker::validateCompositePaths(
    const std::vector<Path> &paths,
    const std::vector<const RobotModel *> &robots,
    const CompositePathValidationOptions &options) const {
    return cc_.validateCompositePaths(paths, robots, options);
}

std::optional<Conflict> ConflictChecker::findConflict(
    const std::vector<Path> &paths,
    const std::vector<const RobotModel *> &robots, int min_timestep,
    std::vector<std::size_t> *next_t_begin_by_robot_out) const {
    CompositePathValidationOptions options;
    options.check_environment = false;
    return findConflict(paths, robots, options, min_timestep,
                        next_t_begin_by_robot_out);
}

std::optional<Conflict> ConflictChecker::findConflict(
    const std::vector<Path> &paths,
    const std::vector<const RobotModel *> &robots,
    const CompositePathValidationOptions &options, int min_timestep,
    std::vector<std::size_t> *next_t_begin_by_robot_out) const {
    CompositePathValidationOptions scoped = options;
    if (min_timestep > 0) {
        scoped.t_begin =
            std::max(scoped.t_begin, static_cast<std::size_t>(min_timestep));
    }
    auto conflict = cc_.findFirstCompositePathConflict(
        paths, robots, scoped, next_t_begin_by_robot_out);
    if (!conflict)
        return std::nullopt;

    return Conflict{conflict->robot_i, conflict->robot_j,
                    static_cast<int>(conflict->timestep), conflict->alpha,
                    conflict->kind, std::move(conflict->config_i),
                    std::move(conflict->config_j)};
}

std::vector<SubproblemConflict> ConflictChecker::findConflicts(
    const std::vector<Path> &paths,
    const std::vector<const RobotModel *> &robots, int min_timestep,
    std::size_t max_conflicts, bool unique, ConflictExpansionFn expand_conflict,
    std::vector<std::size_t> *next_t_begin_by_robot_out,
    std::vector<std::size_t> *next_t_begin_by_pair_out) const {
    CompositePathValidationOptions options;
    options.check_environment = false;
    return findConflicts(paths, robots, options, min_timestep, max_conflicts,
                         unique, std::move(expand_conflict),
                         next_t_begin_by_robot_out,
                         next_t_begin_by_pair_out);
}

std::vector<SubproblemConflict> ConflictChecker::findConflicts(
    const std::vector<Path> &paths, const std::vector<const RobotModel *> &robots,
    const CompositePathValidationOptions &options, int min_timestep,
    std::size_t max_conflicts, bool unique, ConflictExpansionFn expand_conflict,
    std::vector<std::size_t> *next_t_begin_by_robot_out,
    std::vector<std::size_t> *next_t_begin_by_pair_out) const {
    std::vector<SubproblemConflict> conflicts;
    if (paths.size() != robots.size())
        return conflicts;

    CompositePathValidationOptions scoped = options;
    scoped.check_environment = false;
    if (min_timestep > 0) {
        scoped.t_begin =
            std::max(scoped.t_begin, static_cast<std::size_t>(min_timestep));
    }

    if (!expand_conflict)
        expand_conflict = defaultExpandedConflict;

    conflicts.reserve(max_conflicts > 0 ? max_conflicts : paths.size());

    const auto makeExpanded = [&](const CompositeConflict &raw) {
        auto expanded = expand_conflict(
            Conflict{raw.robot_i, raw.robot_j, static_cast<int>(raw.timestep),
                     raw.alpha, raw.kind, raw.config_i, raw.config_j});
        normalizeRobots(expanded.robots);
        if (expanded.window_end_t < expanded.window_begin_t) {
            throw std::runtime_error(
                "Expanded conflict window end precedes window begin");
        }
        for (const int robot : expanded.robots) {
            if (robot < 0 ||
                static_cast<std::size_t>(robot) >= paths.size()) {
                throw std::runtime_error(
                    "Expanded conflict robot index out of range");
            }
        }
        return expanded;
    };

    const InterRobotConflictCallback on_conflict =
        [&](const CompositeConflict &raw) {
            if (raw.robot_i < 0 || raw.robot_j < 0)
                return InterRobotConflictDecision{false, {}};
            auto expanded = makeExpanded(raw);
            if (expanded.robots.empty())
                return InterRobotConflictDecision{false, {}};
            return InterRobotConflictDecision{true, expanded.robots};
        };

    std::vector<std::size_t> backend_next_t_begin_by_robot;
    std::vector<std::size_t> backend_next_t_begin_by_pair;
    const auto raw_conflicts = cc_.findInterRobotPathConflictsCompositeScan(
        paths, robots, scoped, max_conflicts, unique, on_conflict,
        &backend_next_t_begin_by_robot, &backend_next_t_begin_by_pair);

    for (const auto &raw : raw_conflicts) {
        if (raw.robot_i < 0 || raw.robot_j < 0)
            continue;

        auto expanded = makeExpanded(raw);
        if (expanded.robots.empty())
            continue;

        conflicts.push_back(std::move(expanded));
    }

    if (next_t_begin_by_robot_out)
        *next_t_begin_by_robot_out = std::move(backend_next_t_begin_by_robot);
    if (next_t_begin_by_pair_out)
        *next_t_begin_by_pair_out = std::move(backend_next_t_begin_by_pair);
    return conflicts;
}

} // namespace comotion
