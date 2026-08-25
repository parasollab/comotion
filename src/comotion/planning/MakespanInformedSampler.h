#pragma once

#include "comotion/planning/MakespanCompositeStateSpace.h"

#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/samplers/InformedStateSampler.h>
#include <ompl/base/samplers/informed/RejectionInfSampler.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/util/RandomNumbers.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <vector>

namespace comotion {
namespace aorrtc {

class MakespanDirectInfSampler final : public ompl::base::InformedSampler {
public:
    MakespanDirectInfSampler(const ompl::base::ProblemDefinitionPtr &probDefn,
                             unsigned int maxNumberCalls,
                             std::optional<std::uint_fast32_t> local_seed =
                                 std::nullopt)
        : ompl::base::InformedSampler(probDefn, maxNumberCalls),
          fallback_(std::make_shared<ompl::base::RejectionInfSampler>(
              probDefn, maxNumberCalls)) {
        if (local_seed)
            rng_.setLocalSeed(*local_seed);
        const auto *makespan_space =
            dynamic_cast<const MakespanCompositeStateSpace *>(space_.get());
        const auto *real_space =
            dynamic_cast<const ompl::base::RealVectorStateSpace *>(
                space_.get());
        if (makespan_space == nullptr || real_space == nullptr)
            return;

        block_dims_ = makespan_space->blockDimensions();
        dimension_ = real_space->getDimension();
        const unsigned int block_total =
            std::accumulate(block_dims_.begin(), block_dims_.end(), 0u);
        if (dimension_ == 0 || block_total != dimension_ ||
            probDefn_->getStartStateCount() != 1u)
            return;
        if (!initializeSamplingDimensions(real_space->getBounds()))
            return;

        if (!copyStateValues(probDefn_->getStartState(0u), start_values_))
            return;

        if (const auto *goal_state =
                dynamic_cast<const ompl::base::GoalState *>(
                    probDefn_->getGoal().get())) {
            supported_ = copyStateValues(goal_state->getState(), goal_values_);
        } else if (const auto *goal_sampleable =
                       dynamic_cast<const ompl::base::GoalSampleableRegion *>(
                           probDefn_->getGoal().get());
                   goal_sampleable != nullptr &&
                   goal_sampleable->maxSampleCount() > 0u) {
            auto *goal = space_->allocState();
            goal_sampleable->sampleGoal(goal);
            supported_ = copyStateValues(goal, goal_values_);
            space_->freeState(goal);
        }

        if (supported_)
            direct_lower_bound_ =
                makespanDistance(start_values_, goal_values_);
    }

    bool sampleUniform(ompl::base::State *statePtr,
                       const ompl::base::Cost &maxCost) override {
        if (isImpossibleBound(maxCost))
            return false;
        if (!canSampleDirect(maxCost))
            return fallback_->sampleUniform(statePtr, maxCost);
        return sampleDirect(statePtr, nullptr, maxCost);
    }

    bool sampleUniform(ompl::base::State *statePtr,
                       const ompl::base::Cost &minCost,
                       const ompl::base::Cost &maxCost) override {
        if (isImpossibleBound(maxCost))
            return false;
        if (!canSampleDirect(maxCost))
            return fallback_->sampleUniform(statePtr, minCost, maxCost);
        return sampleDirect(statePtr, &minCost, maxCost);
    }

    bool hasInformedMeasure() const override { return false; }

    double getInformedMeasure(
        const ompl::base::Cost &currentCost) const override {
        return fallback_->getInformedMeasure(currentCost);
    }

    double getInformedMeasure(
        const ompl::base::Cost &minCost,
        const ompl::base::Cost &maxCost) const override {
        return fallback_->getInformedMeasure(minCost, maxCost);
    }

private:
    bool copyStateValues(const ompl::base::State *state,
                         std::vector<double> &values) const {
        if (state == nullptr || dimension_ == 0)
            return false;
        const auto *rv =
            state->as<ompl::base::RealVectorStateSpace::StateType>();
        values.assign(rv->values, rv->values + dimension_);
        return std::all_of(values.begin(), values.end(),
                           [](double value) { return std::isfinite(value); });
    }

    bool initializeSamplingDimensions(
        const ompl::base::RealVectorBounds &bounds) {
        constexpr double kFixedSpanTolerance = 1e-12;
        fixed_values_.assign(dimension_, 0.0);
        active_dims_by_block_.clear();
        active_dims_by_block_.reserve(block_dims_.size());

        unsigned int offset = 0;
        for (const unsigned int dim : block_dims_) {
            std::vector<unsigned int> active_dims;
            active_dims.reserve(dim);
            for (unsigned int d = 0; d < dim; ++d) {
                const unsigned int absolute_dim = offset + d;
                const double low = bounds.low[absolute_dim];
                const double high = bounds.high[absolute_dim];
                if (!std::isfinite(low) || !std::isfinite(high) || low > high)
                    return false;
                if (high - low <= kFixedSpanTolerance) {
                    fixed_values_[absolute_dim] = 0.5 * (low + high);
                } else {
                    active_dims.push_back(d);
                }
            }
            active_dims_by_block_.push_back(std::move(active_dims));
            offset += dim;
        }
        return true;
    }

    bool canSampleDirect(const ompl::base::Cost &maxCost) const {
        const double bound = maxCost.value();
        return supported_ && std::isfinite(bound) && bound > 0.0 &&
               bound > direct_lower_bound_;
    }

    bool isImpossibleBound(const ompl::base::Cost &maxCost) const {
        const double bound = maxCost.value();
        return supported_ && std::isfinite(bound) &&
               bound <= direct_lower_bound_;
    }

    bool sampleDirect(ompl::base::State *statePtr,
                      const ompl::base::Cost *minCost,
                      const ompl::base::Cost &maxCost) {
        for (unsigned int attempt = 0; attempt < numIters_; ++attempt) {
            if (!sampleCompositeLens(statePtr, maxCost.value()))
                continue;
            if (!space_->satisfiesBounds(statePtr))
                continue;

            const auto sampled_cost = heuristicSolnCost(statePtr);
            if (!opt_->isCostBetterThan(sampled_cost, maxCost))
                continue;
            if (minCost != nullptr &&
                !(opt_->isCostEquivalentTo(*minCost, sampled_cost) ||
                  opt_->isCostBetterThan(*minCost, sampled_cost)))
                continue;
            return true;
        }
        return false;
    }

    bool sampleCompositeLens(ompl::base::State *statePtr, double bound) {
        const double tau = rng_.uniformReal(0.0, bound);
        auto *rv = statePtr->as<ompl::base::RealVectorStateSpace::StateType>();
        unsigned int offset = 0;
        for (std::size_t block = 0; block < block_dims_.size(); ++block) {
            if (!sampleBlockLens(rv->values, offset, block, tau, bound - tau))
                return false;
            offset += block_dims_[block];
        }
        return true;
    }

    bool sampleBlockLens(double *out, unsigned int offset, std::size_t block,
                         double start_radius, double goal_radius) {
        if (start_radius < 0.0 || goal_radius < 0.0)
            return false;

        const unsigned int dim = block_dims_[block];
        const auto &active_dims = active_dims_by_block_[block];
        const bool sample_from_start = start_radius <= goal_radius;
        const auto &primary_center =
            sample_from_start ? start_values_ : goal_values_;
        const auto &secondary_center =
            sample_from_start ? goal_values_ : start_values_;
        const double primary_radius =
            sample_from_start ? start_radius : goal_radius;
        const double secondary_radius =
            sample_from_start ? goal_radius : start_radius;
        constexpr double kEpsilon = 1e-12;
        const unsigned int attempts = std::max(1u, std::min(64u, numIters_));
        std::vector<double> delta(active_dims.size(), 0.0);

        for (unsigned int attempt = 0; attempt < attempts; ++attempt) {
            for (unsigned int d = 0; d < dim; ++d)
                out[offset + d] = fixed_values_[offset + d];
            if (active_dims.empty()) {
                if (blockDistance(out, secondary_center, offset, dim) <=
                    secondary_radius + kEpsilon)
                    return true;
                continue;
            }

            if (primary_radius <= kEpsilon) {
                for (const unsigned int d : active_dims)
                    out[offset + d] = primary_center[offset + d];
            } else {
                rng_.uniformInBall(primary_radius, delta);
                for (std::size_t i = 0; i < active_dims.size(); ++i) {
                    const unsigned int d = active_dims[i];
                    out[offset + d] = primary_center[offset + d] + delta[i];
                }
            }

            if (blockDistance(out, secondary_center, offset, dim) <=
                secondary_radius + kEpsilon)
                return true;
        }
        return false;
    }

    double blockDistance(const double *values,
                         const std::vector<double> &other,
                         unsigned int offset, unsigned int dim) const {
        double sq = 0.0;
        for (unsigned int d = 0; d < dim; ++d) {
            if (!std::isfinite(values[offset + d]))
                return std::numeric_limits<double>::infinity();
            const double diff = values[offset + d] - other[offset + d];
            sq += diff * diff;
        }
        return std::sqrt(sq);
    }

    double makespanDistance(const std::vector<double> &a,
                            const std::vector<double> &b) const {
        unsigned int offset = 0;
        double out = 0.0;
        for (const unsigned int dim : block_dims_) {
            double sq = 0.0;
            for (unsigned int d = 0; d < dim; ++d) {
                const double diff = a[offset + d] - b[offset + d];
                sq += diff * diff;
            }
            out = std::max(out, std::sqrt(sq));
            offset += dim;
        }
        return out;
    }

    ompl::base::InformedSamplerPtr fallback_;
    ompl::RNG rng_;
    std::vector<unsigned int> block_dims_;
    std::vector<std::vector<unsigned int>> active_dims_by_block_;
    std::vector<double> fixed_values_;
    std::vector<double> start_values_;
    std::vector<double> goal_values_;
    unsigned int dimension_ = 0;
    double direct_lower_bound_ = std::numeric_limits<double>::infinity();
    bool supported_ = false;
};

class MakespanPathLengthObjective final
    : public ompl::base::PathLengthOptimizationObjective {
public:
    explicit MakespanPathLengthObjective(
        const ompl::base::SpaceInformationPtr &si,
        std::optional<std::uint_fast32_t> sampler_seed = std::nullopt)
        : ompl::base::PathLengthOptimizationObjective(si),
          sampler_seed_(sampler_seed) {
        description_ = "Path Length (makespan direct informed sampling)";
    }

    ompl::base::InformedSamplerPtr allocInformedStateSampler(
        const ompl::base::ProblemDefinitionPtr &probDefn,
        unsigned int maxNumberCalls) const override {
        return std::make_shared<MakespanDirectInfSampler>(
            probDefn, maxNumberCalls, sampler_seed_);
    }

private:
    std::optional<std::uint_fast32_t> sampler_seed_;
};

} // namespace aorrtc
} // namespace comotion
