#include "comotion/collision/CollisionChecker.h"
#include "comotion/collision/detail/CollisionBackend.h"

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <utility>

namespace comotion {

bool validationInstrumentationEnabled() {
    static const bool enabled = [] {
        const char *value = std::getenv("COMOTION_VALIDATION_INSTRUMENTATION");
        if (!value)
            return false;
        return std::strcmp(value, "1") == 0 ||
               std::strcmp(value, "true") == 0 ||
               std::strcmp(value, "TRUE") == 0 ||
               std::strcmp(value, "yes") == 0 ||
               std::strcmp(value, "YES") == 0 ||
               std::strcmp(value, "on") == 0 ||
               std::strcmp(value, "ON") == 0;
    }();
    return enabled;
}

namespace {

using TraceClock = std::chrono::steady_clock;

enum class ValidationTimingOp {
    CompositeState,
    PairPath,
    PairPathConflict,
    GoalHoldConstraint,
    CompositeMotion,
    CompositeMotionConflict,
    CompositePaths,
    CompositePathConflict,
    InterRobotPathConflictsScan,
};

std::mutex g_validation_timing_mutex;
ValidationTimingStats g_validation_timing_stats;

void addValidationTiming(ValidationTimingOp op, double seconds,
                         const ValidationWorkStats &work) {
    std::lock_guard<std::mutex> lock(g_validation_timing_mutex);
    g_validation_timing_stats.total_validation_time_seconds += seconds;
    ++g_validation_timing_stats.total_validation_calls;
    g_validation_timing_stats.work += work;

    switch (op) {
    case ValidationTimingOp::CompositeState:
        g_validation_timing_stats.composite_state_seconds += seconds;
        ++g_validation_timing_stats.composite_state_calls;
        break;
    case ValidationTimingOp::PairPath:
        g_validation_timing_stats.pair_path_seconds += seconds;
        ++g_validation_timing_stats.pair_path_calls;
        break;
    case ValidationTimingOp::PairPathConflict:
        g_validation_timing_stats.pair_path_conflict_seconds += seconds;
        ++g_validation_timing_stats.pair_path_conflict_calls;
        break;
    case ValidationTimingOp::GoalHoldConstraint:
        g_validation_timing_stats.goal_hold_constraint_seconds += seconds;
        ++g_validation_timing_stats.goal_hold_constraint_calls;
        break;
    case ValidationTimingOp::CompositeMotion:
        g_validation_timing_stats.composite_motion_seconds += seconds;
        ++g_validation_timing_stats.composite_motion_calls;
        break;
    case ValidationTimingOp::CompositeMotionConflict:
        g_validation_timing_stats.composite_motion_conflict_seconds += seconds;
        ++g_validation_timing_stats.composite_motion_conflict_calls;
        break;
    case ValidationTimingOp::CompositePaths:
        g_validation_timing_stats.composite_paths_seconds += seconds;
        ++g_validation_timing_stats.composite_paths_calls;
        break;
    case ValidationTimingOp::CompositePathConflict:
        g_validation_timing_stats.composite_path_conflict_seconds += seconds;
        ++g_validation_timing_stats.composite_path_conflict_calls;
        break;
    case ValidationTimingOp::InterRobotPathConflictsScan:
        g_validation_timing_stats.inter_robot_path_conflicts_scan_seconds +=
            seconds;
        ++g_validation_timing_stats.inter_robot_path_conflicts_scan_calls;
        break;
    }
}

class ScopedValidationTiming {
public:
    ScopedValidationTiming(ValidationTimingOp op,
                           const detail::CollisionBackend &backend)
        : op_(op), backend_(&backend),
          enabled_(validationInstrumentationEnabled()) {
        if (enabled_)
            start_ = TraceClock::now();
    }

    ~ScopedValidationTiming() {
        if (!enabled_)
            return;
        ValidationWorkStats work;
        if (backend_)
            work = backend_->lastValidationWorkStats();
        const double seconds =
            std::chrono::duration<double>(TraceClock::now() - start_).count();
        addValidationTiming(op_, seconds, work);
    }

private:
    ValidationTimingOp op_;
    const detail::CollisionBackend *backend_ = nullptr;
    bool enabled_ = false;
    TraceClock::time_point start_;
};

} // namespace

struct CollisionChecker::Impl {
    std::unique_ptr<detail::CollisionBackend> backend;

    explicit Impl(std::unique_ptr<detail::CollisionBackend> b)
        : backend(std::move(b)) {}
};

static std::unique_ptr<detail::CollisionBackend> makeBackend(CollisionChecker::Backend b) {
    switch (b) {
    case CollisionChecker::Backend::Spheres:
        return detail::makeSphereBackend();
    case CollisionChecker::Backend::Fcl:
        return detail::makeFclBackend();
    case CollisionChecker::Backend::Vamp:
        return detail::makeVampBackend();
    }
    return detail::makeSphereBackend();
}

CollisionChecker::CollisionChecker()
    : CollisionChecker(Backend::Spheres) {}

CollisionChecker::CollisionChecker(Backend backend)
    : impl_(std::make_unique<Impl>(makeBackend(backend))), backend_(backend) {
    impl_->backend->onEnvironmentChanged(obstacles_, cylinders_);
}

CollisionChecker::~CollisionChecker() = default;

CollisionChecker::CollisionChecker(const CollisionChecker &other)
    : impl_(std::make_unique<Impl>(other.impl_->backend->clone())),
      backend_(other.backend_), obstacles_(other.obstacles_),
      cylinders_(other.cylinders_) {
    impl_->backend->onEnvironmentChanged(obstacles_, cylinders_);
}

CollisionChecker &CollisionChecker::operator=(const CollisionChecker &other) {
    if (this == &other)
        return *this;
    backend_ = other.backend_;
    obstacles_ = other.obstacles_;
    cylinders_ = other.cylinders_;
    impl_ = std::make_unique<Impl>(other.impl_->backend->clone());
    impl_->backend->onEnvironmentChanged(obstacles_, cylinders_);
    return *this;
}

CollisionChecker::CollisionChecker(CollisionChecker &&other) noexcept = default;

CollisionChecker &CollisionChecker::operator=(CollisionChecker &&other) noexcept =
    default;

void CollisionChecker::setObstacles(
    const std::vector<ObstacleSphere> &obstacles) {
    obstacles_ = obstacles;
    impl_->backend->onEnvironmentChanged(obstacles_, cylinders_);
}

void CollisionChecker::setCylinderObstacles(
    const std::vector<ObstacleCylinder> &cylinders) {
    cylinders_ = cylinders;
    impl_->backend->onEnvironmentChanged(obstacles_, cylinders_);
}

void CollisionChecker::setVampValidationStrategy(
    const VampValidationStrategy &strategy) {
    impl_->backend->setVampValidationStrategy(strategy);
}

VampValidationStrategy CollisionChecker::vampValidationStrategy() const {
    return impl_->backend->vampValidationStrategy();
}

ValidationWorkStats CollisionChecker::lastValidationWorkStats() const {
    return impl_->backend->lastValidationWorkStats();
}

void CollisionChecker::resetValidationTimingStats() {
    std::lock_guard<std::mutex> lock(g_validation_timing_mutex);
    g_validation_timing_stats = ValidationTimingStats {};
}

ValidationTimingStats CollisionChecker::validationTimingStats() {
    std::lock_guard<std::mutex> lock(g_validation_timing_mutex);
    return g_validation_timing_stats;
}

bool CollisionChecker::isValidSingle(const RobotModel &robot,
                                     const std::vector<double> &config) const {
    return impl_->backend->isValidSingle(robot, config, obstacles_, cylinders_);
}

bool CollisionChecker::isSelfCollisionFree(
    const RobotModel &robot, const std::vector<double> &config) const {
    return impl_->backend->isSelfCollisionFree(robot, config);
}

bool CollisionChecker::isValidSingleFull(
    const RobotModel &robot, const std::vector<double> &config) const {
    return isValidSingle(robot, config) && isSelfCollisionFree(robot, config);
}

bool CollisionChecker::isValidPair(const RobotModel &robot_a,
                                   const std::vector<double> &config_a,
                                   const RobotModel &robot_b,
                                   const std::vector<double> &config_b) const {
    return impl_->backend->isValidPair(robot_a, config_a, robot_b, config_b);
}

bool CollisionChecker::isValidComposite(
    const std::vector<const RobotModel *> &robots,
    const std::vector<std::vector<double>> &configs) const {
    const ScopedValidationTiming timer(ValidationTimingOp::CompositeState,
                                       *impl_->backend);
    int n = static_cast<int>(robots.size());
    for (int i = 0; i < n; ++i) {
        if (!isValidSingleFull(*robots[i], configs[i]))
            return false;
    }
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            if (!isValidPair(*robots[i], configs[i], *robots[j], configs[j]))
                return false;
        }
    }
    return true;
}

bool CollisionChecker::isMotionValid(const RobotModel &robot,
                                     const std::vector<double> &from,
                                     const std::vector<double> &to,
                                     int num_checks) const {
    return impl_->backend->isMotionValid(robot, from, to, num_checks,
                                         obstacles_, cylinders_);
}

bool CollisionChecker::isRobotPathValid(const RobotModel &robot,
                                        const Path &path) const {
    return impl_->backend->isRobotPathValid(robot, path, obstacles_,
                                            cylinders_);
}

bool CollisionChecker::isPairPathValid(
    const RobotModel &robot_a, const Path &path_a,
    const RobotModel &robot_b, const Path &path_b,
    std::size_t t_begin, std::size_t t_end) const {
    const ScopedValidationTiming timer(ValidationTimingOp::PairPath,
                                       *impl_->backend);
    return impl_->backend->isPairPathValid(robot_a, path_a, robot_b, path_b,
                                           t_begin, t_end);
}

std::optional<PairPathConflict> CollisionChecker::findFirstPairPathConflict(
    const RobotModel &robot_a, const Path &path_a,
    const RobotModel &robot_b, const Path &path_b,
    std::size_t t_begin, std::size_t t_end) const {
    const ScopedValidationTiming timer(ValidationTimingOp::PairPathConflict,
                                       *impl_->backend);
    return impl_->backend->findFirstPairPathConflict(
        robot_a, path_a, robot_b, path_b, t_begin, t_end);
}

GoalHoldConstraint CollisionChecker::computeGoalHoldConstraint(
    const RobotModel &goal_robot,
    const std::vector<double> &goal_config,
    const RobotModel &prior_robot,
    const Path &prior_path) const {
    const ScopedValidationTiming timer(ValidationTimingOp::GoalHoldConstraint,
                                       *impl_->backend);
    return impl_->backend->computeGoalHoldConstraint(
        goal_robot, goal_config, prior_robot, prior_path);
}

bool CollisionChecker::isCompositeMotionValid(
    const std::vector<const RobotModel *> &robots,
    const std::vector<std::vector<double>> &from,
    const std::vector<std::vector<double>> &to,
    const CompositePathValidationOptions &options) const {
    const ScopedValidationTiming timer(ValidationTimingOp::CompositeMotion,
                                       *impl_->backend);
    return impl_->backend->isCompositeMotionValid(robots, from, to, options,
                                                  obstacles_, cylinders_);
}

std::optional<CompositeConflict>
CollisionChecker::findFirstCompositeMotionConflict(
    const std::vector<const RobotModel *> &robots,
    const std::vector<std::vector<double>> &from,
    const std::vector<std::vector<double>> &to,
    const CompositePathValidationOptions &options) const {
    const ScopedValidationTiming timer(
        ValidationTimingOp::CompositeMotionConflict, *impl_->backend);
    return impl_->backend->findFirstCompositeMotionConflict(
        robots, from, to, options, obstacles_, cylinders_);
}

bool CollisionChecker::validateCompositePaths(
    const std::vector<Path> &paths,
    const std::vector<const RobotModel *> &robots,
    const CompositePathValidationOptions &options) const {
    const ScopedValidationTiming timer(ValidationTimingOp::CompositePaths,
                                       *impl_->backend);
    return impl_->backend->validateCompositePaths(paths, robots, options,
                                                  obstacles_, cylinders_);
}

std::optional<CompositeConflict>
CollisionChecker::findFirstCompositePathConflict(
    const std::vector<Path> &paths,
    const std::vector<const RobotModel *> &robots,
    const CompositePathValidationOptions &options,
    std::vector<std::size_t> *next_t_begin_by_robot_out) const {
    const ScopedValidationTiming timer(ValidationTimingOp::CompositePathConflict,
                                       *impl_->backend);
    return impl_->backend->findFirstCompositePathConflict(
        paths, robots, options, obstacles_, cylinders_,
        next_t_begin_by_robot_out);
}

std::vector<CompositeConflict>
CollisionChecker::findInterRobotPathConflictsCompositeScan(
    const std::vector<Path> &paths,
    const std::vector<const RobotModel *> &robots,
    const CompositePathValidationOptions &options,
    std::size_t max_conflicts, bool unique,
    const InterRobotConflictCallback &on_conflict,
    std::vector<std::size_t> *next_t_begin_by_robot_out,
    std::vector<std::size_t> *next_t_begin_by_pair_out) const {
    const ScopedValidationTiming timer(
        ValidationTimingOp::InterRobotPathConflictsScan, *impl_->backend);
    return impl_->backend->findInterRobotPathConflictsCompositeScan(
        paths, robots, options, max_conflicts, unique, on_conflict, obstacles_,
        cylinders_, next_t_begin_by_robot_out, next_t_begin_by_pair_out);
}

} // namespace comotion
