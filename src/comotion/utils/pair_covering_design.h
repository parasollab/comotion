#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace comotion {

inline constexpr std::array<int, 7> kParallelArcPaperRobotCounts = {{
    4, 8, 16, 32, 64, 128, 256,
}};

inline constexpr std::array<int, 4> kParallelArcPaperWorkerCounts = {{
    2, 4, 8, 16,
}};

namespace detail {

inline constexpr std::array<std::uint16_t, 1> kPairCoverB2Cycle = {{
    0x0001,
}};

// Balanced period for buckets 0..3. One full cycle has bucket loads 3,3,3,3.
inline constexpr std::array<std::uint16_t, 5> kPairCoverB4Cycle = {{
    0x0003, 0x0005, 0x000e, 0x0009, 0x000e,
}};

// Heuristic-balanced intersecting mask family for buckets 0..7. One full cycle
// has bucket loads 10,10,10,10,10,10,10,10.
inline constexpr std::array<std::uint16_t, 24> kPairCoverB8Cycle = {{
    0x001c, 0x0049, 0x0091, 0x00c4, 0x008a, 0x0027,
    0x0072, 0x00a8, 0x0027, 0x001c, 0x0091, 0x0049,
    0x00c4, 0x0072, 0x008a, 0x0027, 0x001c, 0x0091,
    0x0049, 0x00c4, 0x0072, 0x00a8, 0x0027, 0x0072,
}};

// Cyclic projective-plane order-3 line masks, with a prefix order chosen to keep
// powers-of-two requests balanced. Buckets 13..15 are intentionally unused.
inline constexpr std::array<std::uint16_t, 13> kPairCoverB16Cycle = {{
    0x020b, 0x0416, 0x082c, 0x00b1, 0x02c4, 0x0c41, 0x1058,
    0x1620, 0x1882, 0x0162, 0x0588, 0x0b10, 0x1105,
}};

template <std::size_t CycleSize>
inline std::vector<std::vector<int>>
materializePairCoverCycle(int n, int bucket_count,
                          const std::array<std::uint16_t, CycleSize> &cycle) {
    std::vector<std::vector<int>> buckets(
        static_cast<std::size_t>(bucket_count));
    for (int item = 0; item < n; ++item) {
        const std::uint16_t mask =
            cycle[static_cast<std::size_t>(item) % CycleSize];
        const int mask_bucket_count = bucket_count < 16 ? bucket_count : 16;
        for (int bucket = 0; bucket < mask_bucket_count; ++bucket) {
            if ((mask & (std::uint16_t{1} << bucket)) != 0)
                buckets[static_cast<std::size_t>(bucket)].push_back(item);
        }
    }
    return buckets;
}

inline std::vector<std::vector<int>> materializeStarPairCover(int n,
                                                              int bucket_count) {
    std::vector<std::vector<int>> buckets(
        static_cast<std::size_t>(bucket_count));
    buckets.front().reserve(static_cast<std::size_t>(n));
    for (int item = 0; item < n; ++item)
        buckets.front().push_back(item);
    return buckets;
}

} // namespace detail

/// Returns buckets that cover every unordered pair of integers in [0, n).
///
/// The requested powers-of-two table entries for P-ARC paper team sizes
/// n in {4, 8, 16, 32, 64, 128, 256} and bucket counts 2, 4, 8, and 16 are
/// represented by the fixed mask cycles above.
/// The cycles are validated heuristic covers rather than certified optima. Other
/// positive inputs use the same deterministic cycles when available, or a valid
/// star cover.
inline std::vector<std::vector<int>> pairCoveringDesign(int n, int bucket_count) {
    if (n <= 1)
        throw std::invalid_argument("pairCoveringDesign requires n > 1");
    if (bucket_count <= 0)
        throw std::invalid_argument(
            "pairCoveringDesign requires bucket_count > 0");

    switch (bucket_count) {
    case 2:
        return detail::materializePairCoverCycle(n, bucket_count,
                                                 detail::kPairCoverB2Cycle);
    case 4:
        return detail::materializePairCoverCycle(n, bucket_count,
                                                 detail::kPairCoverB4Cycle);
    case 8:
        return detail::materializePairCoverCycle(n, bucket_count,
                                                 detail::kPairCoverB8Cycle);
    case 16:
        return detail::materializePairCoverCycle(n, bucket_count,
                                                 detail::kPairCoverB16Cycle);
    default:
        return detail::materializeStarPairCover(n, bucket_count);
    }
}

} // namespace comotion
