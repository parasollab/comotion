#pragma once

#include <boost/serialization/access.hpp>
#include <boost/serialization/vector.hpp>

#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <optional>
#include <utility>

namespace comotion {

// Interpolate linearly between two configs
inline std::vector<double> interpolateConfig(const std::vector<double> &a,
                                              const std::vector<double> &b,
                                              double t) {
    std::vector<double> result(a.size());
    for (size_t i = 0; i < a.size(); ++i)
        result[i] = a[i] + t * (b[i] - a[i]);
    return result;
}

// Path is a sequence of configurations with timestep metadata.
// Time is ALWAYS stored as timesteps (integer indices), never seconds.
// Index k corresponds to timestep k (physical time k/resolution seconds).
// UST-RRT* / ST-CBS: planner τ (time axis) is in seconds; use
// set_waypoint_timesteps_from_tau before interpolate_to_timesteps — do not
// store raw τ counts as k.
class Path : public std::vector<std::vector<double>> {
public:
    using Base = std::vector<std::vector<double>>;
    using Base::Base;

    /// Timestep at each waypoint. Size must match this->size() when non-empty.
    std::vector<size_t> waypoint_timesteps_;

    bool has_explicit_timesteps() const noexcept {
        return waypoint_timesteps_.size() == size() && !waypoint_timesteps_.empty();
    }

    bool has_implicit_dense_timesteps() const noexcept {
        return implicit_dense_timesteps_ && !empty();
    }

    bool has_timesteps() const noexcept {
        return has_explicit_timesteps() || has_implicit_dense_timesteps();
    }

    size_t timestep_at(size_t i, size_t /*resolution*/ = 128) const noexcept {
        if (has_explicit_timesteps() && i < waypoint_timesteps_.size())
            return waypoint_timesteps_[i];
        return i;  // fallback: index = timestep when no metadata
    }

    void clearWaypointTimesteps() noexcept {
        waypoint_timesteps_.clear();
        implicit_dense_timesteps_ = false;
    }

    void clearImplicitDenseTimesteps() noexcept {
        implicit_dense_timesteps_ = false;
    }

    std::vector<size_t> &mutable_waypoint_timesteps() noexcept {
        implicit_dense_timesteps_ = false;
        return waypoint_timesteps_;
    }

    void markDenseTimestepsImplicit() {
        waypoint_timesteps_.clear();
        implicit_dense_timesteps_ = !empty();
    }

    void set_waypoint_timesteps(const std::vector<size_t> &timesteps) {
        implicit_dense_timesteps_ = false;
        if (timesteps.size() == size()) {
            waypoint_timesteps_ = timesteps;
            ensureStrictlyIncreasingWaypointTimesteps();
            canonicalizeDenseTimesteps();
        } else {
            clearWaypointTimesteps();
        }
    }

    /// Map explicit planner times τ (seconds) to Path timestep indices k where
    /// physical time is k/resolution. UST-RRT* uses τ′ = τ + 1 per layer with
    /// spatial step ≤ vmax ⇒ default seconds_per_tau_unit = 1.
    void set_waypoint_timesteps_from_tau(const std::vector<double> &tau_seconds,
                                         size_t resolution,
                                         double seconds_per_tau_unit = 1.0) {
        if (tau_seconds.size() != size() || size() < 1 || resolution < 1 ||
            seconds_per_tau_unit <= 0.0) {
            clearWaypointTimesteps();
            return;
        }
        implicit_dense_timesteps_ = false;
        waypoint_timesteps_.resize(size());
        for (size_t i = 0; i < size(); ++i) {
            const double t_sec = tau_seconds[i] * seconds_per_tau_unit;
            waypoint_timesteps_[i] = static_cast<size_t>(
                std::llround(t_sec * static_cast<double>(resolution)));
        }
        for (size_t i = 1; i < waypoint_timesteps_.size(); ++i) {
            if (waypoint_timesteps_[i] < waypoint_timesteps_[i - 1])
                waypoint_timesteps_[i] = waypoint_timesteps_[i - 1];
        }
        ensureStrictlyIncreasingWaypointTimesteps();
        canonicalizeDenseTimesteps();
    }

    /// Compute waypoint_timesteps_ from path geometry assuming max velocity along each edge.
    /// resolution: timesteps per second. vmax: max velocity.
    void computeTimestepsFromDistance(size_t resolution, double vmax) {
        if (size() < 2 || vmax <= 0.0 || resolution < 1) {
            clearWaypointTimesteps();
            return;
        }
        implicit_dense_timesteps_ = false;
        waypoint_timesteps_.resize(size());
        waypoint_timesteps_[0] = 0;
        double cumul_time_sec = 0.0;
        for (size_t i = 0; i + 1 < size(); ++i) {
            const auto &a = (*this)[i];
            const auto &b = (*this)[i + 1];
            double dist = 0.0;
            for (size_t d = 0; d < a.size(); ++d) {
                double dd = b[d] - a[d];
                dist += dd * dd;
            }
            cumul_time_sec += std::sqrt(dist) / vmax;
            waypoint_timesteps_[i + 1] = static_cast<size_t>(std::round(cumul_time_sec * static_cast<double>(resolution)));
        }
        ensureStrictlyIncreasingWaypointTimesteps();
        canonicalizeDenseTimesteps();
    }

    /// Set waypoint_timesteps_ from given segment times (seconds per segment).
    /// segment_times_sec.size() must equal size() - 1.
    void setTimestepsFromSegmentTimes(const std::vector<double> &segment_times_sec,
                                      size_t resolution) {
        if (segment_times_sec.size() != size() - 1 || size() < 1 || resolution < 1) {
            clearWaypointTimesteps();
            return;
        }
        implicit_dense_timesteps_ = false;
        waypoint_timesteps_.resize(size());
        waypoint_timesteps_[0] = 0;
        double cumul_sec = 0.0;
        for (size_t i = 0; i < segment_times_sec.size(); ++i) {
            cumul_sec += segment_times_sec[i];
            waypoint_timesteps_[i + 1] = static_cast<size_t>(std::round(cumul_sec * static_cast<double>(resolution)));
        }
        ensureStrictlyIncreasingWaypointTimesteps();
        canonicalizeDenseTimesteps();
    }

    /// Total config-space distance along the path.
    double path_cost() const {
        if (size() < 2) return 0.0;
        double total = 0.0;
        for (size_t i = 0; i + 1 < size(); ++i) {
            const auto &a = (*this)[i];
            const auto &b = (*this)[i + 1];
            double dist = 0.0;
            for (size_t d = 0; d < a.size(); ++d) {
                double dd = b[d] - a[d];
                dist += dd * dd;
            }
            total += std::sqrt(dist);
        }
        return total;
    }

    /// Terminal arrival timestep for this path.
    /// When waypoint timesteps are available, use the final explicit timestep;
    /// otherwise fall back to index semantics so cost is still measured in
    /// timesteps rather than sample count.
    std::size_t arrival_timestep() const noexcept {
        if (empty())
            return 0;
        if (has_explicit_timesteps())
            return waypoint_timesteps_.back();
        return size() - 1;
    }

    void config_at_timestep(std::size_t timestep,
                            std::vector<double> &out) const {
        if (empty()) {
            out.clear();
            return;
        }
        if (!has_explicit_timesteps()) {
            out = (*this)[std::min(timestep, size() - 1)];
            return;
        }
        if (timestep <= waypoint_timesteps_.front()) {
            out = front();
            return;
        }
        if (timestep >= waypoint_timesteps_.back()) {
            out = back();
            return;
        }
        const auto upper =
            std::upper_bound(waypoint_timesteps_.begin(),
                             waypoint_timesteps_.end(), timestep);
        const std::size_t next =
            static_cast<std::size_t>(upper - waypoint_timesteps_.begin());
        const std::size_t prev = next > 0 ? next - 1 : 0;
        if (waypoint_timesteps_[prev] == timestep) {
            out = (*this)[prev];
            return;
        }
        if (next >= size()) {
            out = back();
            return;
        }
        const double t0 = static_cast<double>(waypoint_timesteps_[prev]);
        const double t1 = static_cast<double>(waypoint_timesteps_[next]);
        const double alpha =
            (t1 > t0) ? ((static_cast<double>(timestep) - t0) / (t1 - t0))
                      : 0.0;
        out = interpolateConfig((*this)[prev], (*this)[next], alpha);
    }

    std::vector<double> config_at_timestep(std::size_t timestep) const {
        std::vector<double> out;
        config_at_timestep(timestep, out);
        return out;
    }

    /// Interpolate path so that path index k corresponds to timestep k.
    /// waypoint_timesteps_ are always in timestep units (never seconds).
    /// If !has_timesteps(), computes them from path geometry (max velocity per edge).
    /// Output path uses dense timestep semantics: timestep_at(k) = k.
    void interpolate_to_timesteps(size_t resolution, double vmax) {
        if (size() < 2 || vmax <= 0.0 || resolution < 1)
            return;

        if (has_implicit_dense_timesteps())
            return;

        // Ensure we always have timesteps (computed from distance if missing)
        if (!has_timesteps()) {
            computeTimestepsFromDistance(resolution, vmax);
            if (!has_timesteps())
                return;
        }
        if (has_implicit_dense_timesteps())
            return;
        if (!has_explicit_timesteps())
            return;

        bool already_synchronized = true;
        for (size_t i = 0; i < waypoint_timesteps_.size(); ++i) {
            if (waypoint_timesteps_[i] != i) {
                already_synchronized = false;
                break;
            }
        }
        if (already_synchronized) {
            markDenseTimestepsImplicit();
            return;
        }

        const size_t arrival_timesteps = waypoint_timesteps_.back();
        if (arrival_timesteps < 1)
            return;

        Path new_path;
        new_path.reserve(arrival_timesteps + 1);

        size_t seg = 0;
        for (size_t ts = 0; ts <= arrival_timesteps; ++ts) {
            if (ts >= waypoint_timesteps_.back()) {
                new_path.push_back(back());
                continue;
            }

            while (seg + 1 < waypoint_timesteps_.size() &&
                   waypoint_timesteps_[seg + 1] <= ts)
                ++seg;

            if (seg + 1 >= size()) {
                new_path.push_back(back());
                continue;
            }

            if (waypoint_timesteps_[seg] == ts) {
                new_path.push_back((*this)[seg]);
                continue;
            }

            const double t0 = static_cast<double>(waypoint_timesteps_[seg]);
            const double t1 = static_cast<double>(waypoint_timesteps_[seg + 1]);
            const double alpha =
                (t1 > t0) ? ((static_cast<double>(ts) - t0) / (t1 - t0)) : 0.0;
            new_path.push_back(
                interpolateConfig((*this)[seg], (*this)[seg + 1], alpha));
        }

        // Path inherits std::vector; vector::swap only exchanges the sequence of
        // configs, not waypoint_timesteps_. Move both so has_timesteps() stays valid.
        static_cast<Base &>(*this) = std::move(static_cast<Base &>(new_path));
        markDenseTimestepsImplicit();
    }

private:
    friend class boost::serialization::access;

    template <class Archive>
    void serialize(Archive &ar, const unsigned int /*version*/) {
        Base &base = static_cast<Base &>(*this);
        ar & base;
        ar & waypoint_timesteps_;
        ar & implicit_dense_timesteps_;
    }

    bool implicit_dense_timesteps_ = false;

    void canonicalizeDenseTimesteps() {
        if (!has_explicit_timesteps())
            return;
        for (size_t i = 0; i < waypoint_timesteps_.size(); ++i) {
            if (waypoint_timesteps_[i] != i)
                return;
        }
        markDenseTimestepsImplicit();
    }

    /// Cumulative rounding can assign the same discrete timestep to consecutive
    /// vertices; sampling (interpolate_to_timesteps, STCBS-style scans) then
    /// treats target_t==0 as the last vertex on that plateau, not path[0].
    /// Enforce strict increase so each edge spans at least one tick when another
    /// waypoint follows (may lengthen arrival_timesteps slightly vs raw round).
    void ensureStrictlyIncreasingWaypointTimesteps() {
        if (waypoint_timesteps_.size() != size() || waypoint_timesteps_.empty())
            return;
        for (size_t i = 1; i < waypoint_timesteps_.size(); ++i) {
            if (waypoint_timesteps_[i] <= waypoint_timesteps_[i - 1])
                waypoint_timesteps_[i] = waypoint_timesteps_[i - 1] + 1;
        }
    }
};

// Pad shorter paths to the same length by repeating the last config.
// This is a planner utility for rectangular outputs; collision/path validation
// works directly on ragged paths by clamping to the last config as needed.
// When the path already has synced waypoint_timesteps_ (has_timesteps()), each
// padded row advances global discrete time by one to preserve hold-at-goal time.
inline void equalizePaths(std::vector<Path> &paths) {
    size_t max_len = 0;
    for (auto &p : paths)
        max_len = std::max(max_len, p.size());
    for (auto &p : paths) {
        while (p.size() < max_len) {
            const bool had_explicit_timesteps = p.has_explicit_timesteps();
            p.push_back(p.back());
            if (had_explicit_timesteps) {
                auto &timesteps = p.mutable_waypoint_timesteps();
                timesteps.push_back(timesteps.back() + 1);
            }
        }
    }
}

} // namespace comotion
