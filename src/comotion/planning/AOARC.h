#pragma once

#include "comotion/planning/ARC.h"
#include "comotion/planning/AORRTCUtils.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace comotion {

/// Stable planner API.
///
/// Anytime-optimization wrapper around ARC that repeatedly tightens the current
/// makespan bound.
class AOARC : public ARC {
public:
    ompl::base::PlannerStatus solve(double timeLimit) override;
    std::string name() const override { return "AOARC"; }

    /// Reuse incumbent paths that satisfy the next strict reuse threshold.
    /// Replanned paths retain Bounded ARC's global bound. Enabled by default.
    void setSelectiveBoundedReplanning(bool enabled) {
        selective_bounded_replanning_ = enabled;
    }
    bool selectiveBoundedReplanning() const {
        return selective_bounded_replanning_;
    }

    /// On the first conflict scan of a bounded attempt, inspect only pairs
    /// containing a robot whose individual path was replanned. Enabled by
    /// default. If all paths are replanned, every pair naturally starts at 0.
    void setSelectiveInitialConflictScan(bool enabled) {
        selective_initial_conflict_scan_ = enabled;
    }
    bool selectiveInitialConflictScan() const {
        return selective_initial_conflict_scan_;
    }

    /// Also replan robots connected to an original bound violator by at most
    /// `depth` accepted repair-history edges. Zero disables expansion, one
    /// preserves the original direct-partner behavior, and larger values
    /// expand one breadth-first layer at a time. Defaults to zero.
    void setRepairHistoryReplanningDepth(std::size_t depth) {
        repair_history_replanning_depth_ = depth;
    }
    std::size_t repairHistoryReplanningDepth() const {
        return repair_history_replanning_depth_;
    }

    /// Backward-compatible spelling for the former one-hop boolean option.
    void setExpandReplanningFromRepairHistory(bool enabled) {
        setRepairHistoryReplanningDepth(enabled ? 1 : 0);
    }
    bool expandReplanningFromRepairHistory() const {
        return repair_history_replanning_depth_ != 0;
    }

    /// Probability that a selective bounded attempt discards every incumbent
    /// path and performs a full restart. The decision is replayable from the
    /// planning seed and bounded-attempt index. Zero disables random restarts.
    void setRandomFullRestartProbability(double probability);
    double randomFullRestartProbability() const {
        return random_full_restart_probability_;
    }

protected:
    void configureArcAttempt(ARC &planner) const;
    static std::uint64_t boundedInitialPathReuseBoundTimesteps(
        std::uint64_t incumbent_makespan, std::uint64_t epsilon_timesteps);
    static std::vector<int> repairPartnerRobotsWithinDepth(
        const std::vector<int> &original_bound_violators,
        const std::vector<AppliedRepairHistoryEvent> &repair_history,
        std::size_t depth);
    static std::vector<int> oneHopRepairPartnerRobots(
        const std::vector<int> &original_bound_violators,
        const std::vector<AppliedRepairHistoryEvent> &repair_history) {
        return repairPartnerRobotsWithinDepth(original_bound_violators,
                                              repair_history, 1);
    }
    static bool randomFullRestartForAttempt(std::uint32_t planning_seed,
                                            std::uint64_t attempt_index,
                                            double probability);
    static void incorporateAcceptedRepairHistory(
        std::vector<AppliedRepairHistoryEvent> &incumbent_history,
        const std::vector<AppliedRepairHistoryEvent> &accepted_history,
        bool incumbent_paths_retained);

private:
    static nlohmann::json
    solutionEventsJson(const std::vector<aorrtc::SolutionEvent> &events);

    bool selective_bounded_replanning_ = true;
    bool selective_initial_conflict_scan_ = true;
    std::size_t repair_history_replanning_depth_ = 0;
    double random_full_restart_probability_ = 0.0;
};

} // namespace comotion
