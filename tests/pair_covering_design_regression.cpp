#include "comotion/utils/pair_covering_design.h"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

bool expectTrue(const std::string &label, bool condition) {
    if (!condition) {
        std::cerr << "pair_covering_design_regression: " << label << "\n";
        return false;
    }
    return true;
}

bool validateCover(int n, int bucket_count) {
    const auto buckets = comotion::pairCoveringDesign(n, bucket_count);
    if (!expectTrue("bucket count for n=" + std::to_string(n) +
                        " b=" + std::to_string(bucket_count),
                    static_cast<int>(buckets.size()) == bucket_count))
        return false;

    std::vector<std::vector<char>> covered(
        static_cast<std::size_t>(n),
        std::vector<char>(static_cast<std::size_t>(n), 0));

    for (int bucket = 0; bucket < bucket_count; ++bucket) {
        std::vector<char> seen(static_cast<std::size_t>(n), 0);
        const auto &items = buckets[static_cast<std::size_t>(bucket)];
        for (int item : items) {
            if (!expectTrue("item range for n=" + std::to_string(n) +
                                " b=" + std::to_string(bucket_count),
                            item >= 0 && item < n))
                return false;
            if (!expectTrue("duplicate item in bucket for n=" +
                                std::to_string(n) + " b=" +
                                std::to_string(bucket_count),
                            seen[static_cast<std::size_t>(item)] == 0))
                return false;
            seen[static_cast<std::size_t>(item)] = 1;
        }

        for (std::size_t i = 0; i < items.size(); ++i) {
            for (std::size_t j = i + 1; j < items.size(); ++j) {
                const int a = items[i];
                const int b = items[j];
                covered[static_cast<std::size_t>(a)]
                       [static_cast<std::size_t>(b)] = 1;
                covered[static_cast<std::size_t>(b)]
                       [static_cast<std::size_t>(a)] = 1;
            }
        }
    }

    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            if (!expectTrue("uncovered pair (" + std::to_string(i) + "," +
                                std::to_string(j) + ") for n=" +
                                std::to_string(n) + " b=" +
                                std::to_string(bucket_count),
                            covered[static_cast<std::size_t>(i)]
                                   [static_cast<std::size_t>(j)] != 0))
                return false;
        }
    }

    return true;
}

bool expectInvalid(int n, int bucket_count) {
    try {
        (void)comotion::pairCoveringDesign(n, bucket_count);
    } catch (const std::invalid_argument &) {
        return true;
    } catch (const std::exception &ex) {
        std::cerr << "pair_covering_design_regression: unexpected exception: "
                  << ex.what() << "\n";
        return false;
    }

    std::cerr << "pair_covering_design_regression: expected invalid_argument"
              << " for n=" << n << " b=" << bucket_count << "\n";
    return false;
}

bool validateCanonicalLightestAssignment(int n, int bucket_count,
                                         int max_bucket_size_bound,
                                         int max_pair_load_bound) {
    const auto buckets = comotion::pairCoveringDesign(n, bucket_count);
    std::vector<std::uint64_t> robot_bucket_masks(static_cast<std::size_t>(n),
                                                  0);
    for (int bucket = 0; bucket < bucket_count; ++bucket) {
        const auto &items = buckets[static_cast<std::size_t>(bucket)];
        if (!expectTrue("bucket size bound for n=" + std::to_string(n) +
                            " b=" + std::to_string(bucket_count),
                        static_cast<int>(items.size()) <=
                            max_bucket_size_bound)) {
            return false;
        }
        for (int item : items) {
            robot_bucket_masks[static_cast<std::size_t>(item)] |=
                std::uint64_t{1} << bucket;
        }
    }

    std::vector<int> pair_loads(static_cast<std::size_t>(bucket_count), 0);
    std::vector<std::vector<char>> assigned(
        static_cast<std::size_t>(n),
        std::vector<char>(static_cast<std::size_t>(n), 0));
    int total_assigned = 0;
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            const std::uint64_t covering_buckets =
                robot_bucket_masks[static_cast<std::size_t>(i)] &
                robot_bucket_masks[static_cast<std::size_t>(j)];
            if (!expectTrue("canonical assignment cover exists",
                            covering_buckets != 0)) {
                return false;
            }

            int best_bucket = -1;
            int best_load = n * n;
            for (int bucket = 0; bucket < bucket_count; ++bucket) {
                if ((covering_buckets & (std::uint64_t{1} << bucket)) == 0)
                    continue;
                if (pair_loads[static_cast<std::size_t>(bucket)] <
                    best_load) {
                    best_bucket = bucket;
                    best_load =
                        pair_loads[static_cast<std::size_t>(bucket)];
                }
            }
            if (!expectTrue("canonical assignment selected bucket",
                            best_bucket >= 0)) {
                return false;
            }
            ++pair_loads[static_cast<std::size_t>(best_bucket)];
            ++assigned[static_cast<std::size_t>(i)]
                      [static_cast<std::size_t>(j)];
            ++total_assigned;
        }
    }

    if (!expectTrue("canonical assignment total pairs",
                    total_assigned == n * (n - 1) / 2)) {
        return false;
    }
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            if (!expectTrue("canonical assignment covers pair exactly once",
                            assigned[static_cast<std::size_t>(i)]
                                    [static_cast<std::size_t>(j)] == 1)) {
                return false;
            }
        }
    }
    for (int load : pair_loads) {
        if (!expectTrue("canonical assignment pair load bound",
                        load <= max_pair_load_bound)) {
            return false;
        }
    }
    return true;
}

} // namespace

int main() {
    constexpr auto cached_buckets = comotion::kParallelArcPaperWorkerCounts;

    for (int n : comotion::kParallelArcPaperRobotCounts) {
        for (int bucket_count : cached_buckets) {
            if (!validateCover(n, bucket_count))
                return 1;
        }
    }
    if (!validateCover(10, 8))
        return 1;
    if (!validateCover(17, 16))
        return 1;
    if (!validateCover(9, 5))
        return 1;

    if (!validateCanonicalLightestAssignment(128, 4, 77, 2035))
        return 1;
    if (!validateCanonicalLightestAssignment(128, 8, 54, 1048))
        return 1;

    if (!expectInvalid(1, 8))
        return 1;
    if (!expectInvalid(4, 0))
        return 1;

    std::cout << "pair_covering_design_regression: OK\n";
    return 0;
}
