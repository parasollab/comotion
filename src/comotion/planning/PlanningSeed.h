#pragma once

#include <cstdint>
#include <limits>

namespace comotion {

/// Matches benchmark app handling of `std::numeric_limits<uint32_t>::max()`.
inline constexpr std::uint32_t kPlanningSeedPassthroughOmpl =
    std::numeric_limits<std::uint32_t>::max();

/// Root seed for `ompl::RNG::setSeed` (same +1 convention as the benchmark apps).
inline std::uint_fast32_t omplRootSeedFromUserPlanningSeed(std::uint32_t user_seed) {
    if (user_seed == kPlanningSeedPassthroughOmpl)
        return user_seed;
    return static_cast<std::uint_fast32_t>(user_seed) + 1u;
}

/// Per-agent deterministic seed for `ompl::RNG` instances (local / per-tree RNG).
inline std::uint_fast32_t omplLocalSeedFromUserPlanningSeed(std::uint32_t user_seed,
                                                            int agent_index) {
    std::uint64_t x = static_cast<std::uint64_t>(user_seed) +
                      0x9e3779b97f4a7c15ULL *
                          static_cast<std::uint64_t>(agent_index + 1);
    x ^= x >> 33u;
    x *= 0xff51afd7ed558ccdULL;
    x ^= x >> 33u;
    return static_cast<std::uint_fast32_t>(x);
}

/// Stable 64-bit finalizer used to derive independent planning-seed domains.
/// This is intentionally local and deterministic; it never touches OMPL's
/// process-global seed generator.
inline std::uint64_t mixPlanningSeedWord(std::uint64_t value) {
    value ^= value >> 30u;
    value *= 0xbf58476d1ce4e5b9ULL;
    value ^= value >> 27u;
    value *= 0x94d049bb133111ebULL;
    value ^= value >> 31u;
    return value;
}

/// Derives a replayable 32-bit child seed from a parent seed, a domain tag,
/// and two full-width identifiers. Keeping the identifiers separate avoids the
/// collisions and overflow risks of packing repair/attempt indices into an int
/// salt.
inline std::uint32_t derivePlanningSeed(std::uint32_t parent_seed,
                                        std::uint64_t domain,
                                        std::uint64_t component_a = 0,
                                        std::uint64_t component_b = 0) {
    std::uint64_t state = mixPlanningSeedWord(
        0x6a09e667f3bcc909ULL ^ static_cast<std::uint64_t>(parent_seed));
    state = mixPlanningSeedWord(
        state ^ mixPlanningSeedWord(domain + 0x9e3779b97f4a7c15ULL));
    state = mixPlanningSeedWord(
        state ^ mixPlanningSeedWord(component_a + 0x3c6ef372fe94f82bULL));
    state = mixPlanningSeedWord(
        state ^ mixPlanningSeedWord(component_b + 0xa54ff53a5f1d36f1ULL));
    return static_cast<std::uint32_t>(state ^ (state >> 32u));
}

inline constexpr std::uint64_t kPlanningSeedDomainArcRepairAttempt =
    0x4152435f4154544dULL; // "ARC_ATTM"
inline constexpr std::uint64_t kPlanningSeedDomainArcPrioritizedSolver =
    0x4152435f5052494fULL; // "ARC_PRIO"
inline constexpr std::uint64_t kPlanningSeedDomainArcCompositeSolver =
    0x4152435f434f4d50ULL; // "ARC_COMP"
inline constexpr std::uint64_t kPlanningSeedDomainCompositeRrtSampler =
    0x435252545f53414dULL; // "CRRT_SAM"
inline constexpr std::uint64_t kPlanningSeedDomainCompositeRrtPlanner =
    0x435252545f504c4eULL; // "CRRT_PLN"
inline constexpr std::uint64_t kPlanningSeedDomainCompositeRrtSimplifier =
    0x435252545f53494dULL; // "CRRT_SIM"
inline constexpr std::uint64_t kPlanningSeedDomainParallelArcRepair =
    0x504152435f525052ULL; // "PARC_RPR"
inline constexpr std::uint64_t kPlanningSeedDomainParallelArcAttempt =
    0x504152435f415454ULL; // "PARC_ATT"
inline constexpr std::uint64_t kPlanningSeedDomainStrrtStateSampler =
    0x53545252545f5354ULL; // "STRRT_ST"
inline constexpr std::uint64_t kPlanningSeedDomainStrrtTimeSampler =
    0x53545252545f544dULL; // "STRRT_TM"
inline constexpr std::uint64_t kPlanningSeedDomainStrrtPlanner =
    0x53545252545f504cULL; // "STRRT_PL"
inline constexpr std::uint64_t kPlanningSeedDomainStrrtConditionalSampler =
    0x53545252545f434eULL; // "STRRT_CN"
inline constexpr std::uint64_t kPlanningSeedDomainStrrtSimplifier =
    0x53545252545f5349ULL; // "STRRT_SI"
inline constexpr std::uint64_t kPlanningSeedDomainAorrtcSingleBounded =
    0x414f525f53424e44ULL; // "AOR_SBND"
inline constexpr std::uint64_t kPlanningSeedDomainAorrtcCompositeBounded =
    0x414f525f43424e44ULL; // "AOR_CBND"
inline constexpr std::uint64_t kPlanningSeedDomainAorrtcCompositeAnytime =
    0x414f525f43414e59ULL; // "AOR_CANY"

inline std::uint32_t arcRepairAttemptPlanningSeed(
    std::uint32_t parent_seed, std::uint64_t repair_id,
    std::uint64_t attempt_index) {
    return derivePlanningSeed(parent_seed, kPlanningSeedDomainArcRepairAttempt,
                              repair_id, attempt_index);
}

inline std::uint32_t arcRepairPrioritizedPlanningSeed(
    std::uint32_t attempt_root_seed) {
    return derivePlanningSeed(attempt_root_seed,
                              kPlanningSeedDomainArcPrioritizedSolver);
}

inline std::uint32_t arcRepairCompositePlanningSeed(
    std::uint32_t attempt_root_seed) {
    return derivePlanningSeed(attempt_root_seed,
                              kPlanningSeedDomainArcCompositeSolver);
}

/// Seed for one logical P-ARC repair attempt. The batch/task hierarchy avoids
/// structural collisions from integer salt packing. The worker slot is
/// intentionally absent: scheduling the same logical attempt on a different
/// process must not change the result.
inline std::uint32_t parallelArcRepairAttemptPlanningSeed(
    std::uint32_t parent_seed, std::uint64_t batch_index,
    std::uint64_t task_index, std::uint64_t attempt_index) {
    const auto repair_seed = derivePlanningSeed(
        parent_seed, kPlanningSeedDomainParallelArcRepair, batch_index,
        task_index);
    return derivePlanningSeed(repair_seed,
                              kPlanningSeedDomainParallelArcAttempt,
                              attempt_index);
}

inline std::uint_fast32_t compositeRrtStateSamplerSeed(
    std::uint32_t planning_seed) {
    return derivePlanningSeed(planning_seed,
                              kPlanningSeedDomainCompositeRrtSampler);
}

inline std::uint_fast32_t compositeRrtPlannerSeed(
    std::uint32_t planning_seed) {
    return derivePlanningSeed(planning_seed,
                              kPlanningSeedDomainCompositeRrtPlanner);
}

inline std::uint_fast32_t compositeRrtPathSimplifierSeed(
    std::uint32_t planning_seed) {
    return derivePlanningSeed(planning_seed,
                              kPlanningSeedDomainCompositeRrtSimplifier);
}

inline std::uint_fast32_t prioritizedStrrtComponentSeed(
    std::uint32_t planning_seed, std::uint64_t domain, int robot_index) {
    return derivePlanningSeed(planning_seed, domain,
                              static_cast<std::uint64_t>(robot_index));
}

inline std::uint_fast32_t aorrtcComponentSeed(
    std::uint32_t planning_seed, std::uint64_t solve_domain,
    std::uint64_t component) {
    return derivePlanningSeed(planning_seed, solve_domain, component);
}

/// Salt for `omplLocalSeedFromUserPlanningSeed` reserved for MRdRRT phase-2 (tensor) sampling.
inline constexpr int kOmplLocalSaltMrDrrtTensorPhase = 1000000001;

inline std::uint_fast32_t omplLocalSeedForMrDrrtTensorPhase(std::uint32_t user_seed) {
    return omplLocalSeedFromUserPlanningSeed(user_seed, kOmplLocalSaltMrDrrtTensorPhase);
}

/// Salt base for process-level OR-parallel planner worker seeds.
inline constexpr int kPlanningSeedSaltOrParallelWorkerBase = 1001000000;

/// Deterministic per-worker planner seed for process-level OR-parallel replicas.
inline std::uint32_t orParallelWorkerPlanningSeed(std::uint32_t user_seed,
                                                  int worker_index) {
    return static_cast<std::uint32_t>(omplLocalSeedFromUserPlanningSeed(
        user_seed, kPlanningSeedSaltOrParallelWorkerBase + worker_index));
}

/// Seed for `std::mt19937` in planners that keep their own C++ RNG.
inline std::uint32_t stdRngSeedFromUserPlanningSeed(std::uint32_t user_seed) {
    return user_seed;
}

} // namespace comotion
