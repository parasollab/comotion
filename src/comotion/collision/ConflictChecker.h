#pragma once

#include "comotion/collision/CollisionChecker.h"
#include "comotion/collision/ValidationTypes.h"
#include "comotion/planning/Path.h"
#include "comotion/robot/RobotModel.h"
#include <functional>
#include <optional>
#include <vector>

namespace comotion {

struct Conflict {
    int robot_i;
    int robot_j;
    int timestep;
    double alpha = 0.0;
    ConflictKind kind = ConflictKind::Vertex;
    std::vector<double> config_i;
    std::vector<double> config_j;
};

struct SubproblemConflict {
    struct ExpansionTraceStep {
        int from_robot = -1;
        int added_robot = -1;
        int window_robot_a = -1;
        int window_robot_b = -1;
        int window_start_t = 0;
        int window_end_t = 0;
        std::vector<int> history_event_ids;

        template <class Archive>
        void serialize(Archive &ar, const unsigned int /*version*/) {
            ar & from_robot;
            ar & added_robot;
            ar & window_robot_a;
            ar & window_robot_b;
            ar & window_start_t;
            ar & window_end_t;
            ar & history_event_ids;
        }
    };

    std::vector<int> robots;
    int conflict_timestep = 0;
    int window_begin_t = 0;
    int window_end_t = 0;
    int seed_robot_i = -1;
    int seed_robot_j = -1;
    double alpha = 0.0;
    ConflictKind kind = ConflictKind::Vertex;
    std::vector<double> config_i;
    std::vector<double> config_j;
    std::vector<ExpansionTraceStep> expansion_trace;

    template <class Archive>
    void serialize(Archive &ar, const unsigned int /*version*/) {
        ar & robots;
        ar & conflict_timestep;
        ar & window_begin_t;
        ar & window_end_t;
        ar & seed_robot_i;
        ar & seed_robot_j;
        ar & alpha;
        ar & kind;
        ar & config_i;
        ar & config_j;
        ar & expansion_trace;
    }
};

class ConflictChecker {
public:
    using ConflictExpansionFn =
        std::function<SubproblemConflict(const Conflict &)>;

    explicit ConflictChecker(const CollisionChecker &cc) : cc_(cc) {}

    // Check if two paths conflict (pairwise). Returns true on first collision.
    bool isConflict(const Path &path_a, const RobotModel &robot_a,
                    const Path &path_b, const RobotModel &robot_b) const;

    // Check if any pair of paths conflicts.
    bool isConflict(const std::vector<Path> &paths,
                    const std::vector<const RobotModel *> &robots) const;

    bool validateCompositePaths(
        const std::vector<Path> &paths,
        const std::vector<const RobotModel *> &robots,
        const CompositePathValidationOptions &options = {}) const;

    // Find the first conflict across all paths, iterating native timestep/path
    // indices across all paths simultaneously. Shorter paths are treated as
    // holding their final configuration via configAt().
    // min_timestep: skip checking t < min_timestep.
    std::optional<Conflict> findConflict(
        const std::vector<Path> &paths,
        const std::vector<const RobotModel *> &robots,
        int min_timestep = 0,
        std::vector<std::size_t> *next_t_begin_by_robot_out = nullptr) const;

    std::optional<Conflict> findConflict(
        const std::vector<Path> &paths,
        const std::vector<const RobotModel *> &robots,
        const CompositePathValidationOptions &options,
        int min_timestep = 0,
        std::vector<std::size_t> *next_t_begin_by_robot_out = nullptr) const;

    // Robot-robot vertex conflicts only (no environment), in composite
    // timestep order. Each discovered pair conflict is expanded into a
    // subproblem conflict before optional uniqueness filtering.
    //
    // unique=false:
    //   return expanded conflicts in discovery order, up to max_conflicts.
    // unique=true:
    //   accept the earliest expanded conflicts whose robot sets are disjoint
    //   from robots claimed by previously accepted conflicts in the same scan.
    //   Later conflicts that touch any claimed robot are discarded. Uniqueness
    //   is therefore based on earliest accepted conflict timesteps, not on
    //   later same-round window merging.
    std::vector<SubproblemConflict> findConflicts(
        const std::vector<Path> &paths,
        const std::vector<const RobotModel *> &robots,
        int min_timestep = 0,
        std::size_t max_conflicts = 0,
        bool unique = false,
        ConflictExpansionFn expand_conflict = {},
        std::vector<std::size_t> *next_t_begin_by_robot_out = nullptr,
        std::vector<std::size_t> *next_t_begin_by_pair_out = nullptr) const;

    std::vector<SubproblemConflict> findConflicts(
        const std::vector<Path> &paths,
        const std::vector<const RobotModel *> &robots,
        const CompositePathValidationOptions &options,
        int min_timestep = 0,
        std::size_t max_conflicts = 0,
        bool unique = false,
        ConflictExpansionFn expand_conflict = {},
        std::vector<std::size_t> *next_t_begin_by_robot_out = nullptr,
        std::vector<std::size_t> *next_t_begin_by_pair_out = nullptr) const;

private:
    const CollisionChecker &cc_;

    // Get config at timestep t, clamping to last config if path is shorter
    static std::vector<double> configAt(const Path &path, std::size_t t);
    static SubproblemConflict
    defaultExpandedConflict(const Conflict &conflict);
};

} // namespace comotion
