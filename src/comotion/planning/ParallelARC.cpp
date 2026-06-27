#include "comotion/planning/ParallelARC.h"

#include "comotion/collision/ConflictChecker.h"
#include "comotion/planning/PlanningRng.h"

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/vector.hpp>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <climits>
#include <cstdint>
#include <cstddef>
#include <ctime>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <limits>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#if !defined(_WIN32)
#include <poll.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace {

using Clock = std::chrono::steady_clock;

std::uint64_t elapsedNanoseconds(const Clock::time_point &start) {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() -
                                                             start)
            .count());
}

double processCpuSeconds() {
#if defined(CLOCK_PROCESS_CPUTIME_ID)
    timespec ts {};
    if (::clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts) == 0) {
        return static_cast<double>(ts.tv_sec) +
               static_cast<double>(ts.tv_nsec) * 1e-9;
    }
#endif
    return static_cast<double>(std::clock()) /
           static_cast<double>(CLOCKS_PER_SEC);
}

double elapsedProcessCpuSeconds(double start) {
    const double elapsed = processCpuSeconds() - start;
    return elapsed < 0.0 ? 0.0 : elapsed;
}

std::uint64_t processCpuElapsedNanoseconds(double start) {
    return static_cast<std::uint64_t>(elapsedProcessCpuSeconds(start) * 1e9);
}

std::string formatRobotList(const std::vector<int> &robots) {
    std::ostringstream os;
    os << "[";
    for (std::size_t i = 0; i < robots.size(); ++i)
        os << (i ? ", " : "") << robots[i];
    os << "]";
    return os.str();
}

std::vector<int> vectorDifference(const std::vector<int> &lhs,
                                  const std::vector<int> &rhs) {
    std::set<int> rhs_set(rhs.begin(), rhs.end());
    std::vector<int> diff;
    diff.reserve(lhs.size());
    for (const int robot : lhs) {
        if (!rhs_set.count(robot))
            diff.push_back(robot);
    }
    return diff;
}

std::uint32_t workerPlanningSeed(std::uint32_t planner_seed, int batch_index,
                                 int task_index, int slot_index,
                                 int attempt_index) {
    const int salt = 1000000 + batch_index * 100000 + task_index * 4096 +
                     attempt_index * 64 + slot_index;
    return static_cast<std::uint32_t>(
        comotion::omplLocalSeedFromUserPlanningSeed(planner_seed, salt));
}

std::uint32_t initialIndividualPlanningSeed(std::uint32_t planner_seed,
                                            int robot_index,
                                            int attempt_index) {
    const int salt = 1003000000 + robot_index * 4096 + attempt_index;
    return static_cast<std::uint32_t>(
        comotion::omplLocalSeedFromUserPlanningSeed(planner_seed, salt));
}

const char *parallelArcConflictFindModeStr(
    comotion::ParallelArcConflictFindMode mode) {
    switch (mode) {
    case comotion::ParallelArcConflictFindMode::Sequential:
        return "sequential";
    case comotion::ParallelArcConflictFindMode::SegmentParallel:
        return "segment_parallel";
    }
    return "unknown";
}

struct WorkerResult {
    struct RepairStats {
        std::uint64_t subproblem_attempts = 0;
        std::uint64_t temporal_expansions = 0;
        double conflict_resolution_times_seconds_wall_clock = 0.0;
        double conflict_resolution_times_seconds_total = 0.0;
        double conflict_resolution_times_seconds_cpu = 0.0;

        template <class Archive>
        void serialize(Archive &ar, const unsigned int /*version*/) {
            ar & subproblem_attempts;
            ar & temporal_expansions;
            ar & conflict_resolution_times_seconds_wall_clock;
            ar & conflict_resolution_times_seconds_total;
            ar & conflict_resolution_times_seconds_cpu;
        }
    };

    struct PathPatch {
        int robot_id = -1;
        std::uint64_t base_version = 0;
        int window_start_t = 0;
        int window_end_t = 0;
        comotion::Path local_path;

        template <class Archive>
        void serialize(Archive &ar, const unsigned int /*version*/) {
            ar & robot_id;
            ar & base_version;
            ar & window_start_t;
            ar & window_end_t;
            ar & local_path;
        }
    };

    bool success = false;
    int window_start_t = 0;
    int window_end_t = 0;
    std::vector<int> final_involved_robots;
    std::vector<PathPatch> patches;
    RepairStats repair_stats;
    std::uint64_t patch_payload_bytes = 0;
    std::uint64_t stats_payload_bytes = 0;
    std::string error_message;

    template <class Archive>
    void serialize(Archive &ar, const unsigned int /*version*/) {
        ar & success;
        ar & window_start_t;
        ar & window_end_t;
        ar & final_involved_robots;
        ar & patches;
        ar & repair_stats;
        ar & patch_payload_bytes;
        ar & stats_payload_bytes;
        ar & error_message;
    }
};

struct InitialPlanCommand {
    bool quit = false;
    int robot_id = -1;
    int attempt_index = 0;
    std::uint32_t planning_seed = 0;
    double time_budget_seconds = 0.0;

    template <class Archive>
    void serialize(Archive &ar, const unsigned int /*version*/) {
        ar & quit;
        ar & robot_id;
        ar & attempt_index;
        ar & planning_seed;
        ar & time_budget_seconds;
    }
};

struct InitialPlanResult {
    int worker_index = -1;
    int robot_id = -1;
    int attempt_index = 0;
    bool success = false;
    std::uint64_t worker_wall_ns = 0;
    std::uint64_t solve_ns = 0;
    std::uint64_t simplify_ns = 0;
    double cpu_seconds = 0.0;
    comotion::Path path;
    std::string error_message;

    template <class Archive>
    void serialize(Archive &ar, const unsigned int /*version*/) {
        ar & worker_index;
        ar & robot_id;
        ar & attempt_index;
        ar & success;
        ar & worker_wall_ns;
        ar & solve_ns;
        ar & simplify_ns;
        ar & cpu_seconds;
        ar & path;
        ar & error_message;
    }
};

struct RepairWorkerCommand {
    bool quit = false;
    int slot_index = -1;
    int task_index = -1;
    int attempt_index = -1;
    std::uint32_t planning_seed = 0;

    template <class Archive>
    void serialize(Archive &ar, const unsigned int /*version*/) {
        ar & quit;
        ar & slot_index;
        ar & task_index;
        ar & attempt_index;
        ar & planning_seed;
    }
};

struct RepairWorkerResultHeader {
    int slot_index = -1;
    int task_index = -1;
    int attempt_index = -1;
    bool cancelled = false;
    bool has_payload = false;
    bool worker_error = false;
    std::uint64_t worker_wall_ns = 0;
    std::uint64_t serialize_ns = 0;
    std::uint64_t payload_bytes = 0;
    std::string error_message;

    template <class Archive>
    void serialize(Archive &ar, const unsigned int /*version*/) {
        ar & slot_index;
        ar & task_index;
        ar & attempt_index;
        ar & cancelled;
        ar & has_payload;
        ar & worker_error;
        ar & worker_wall_ns;
        ar & serialize_ns;
        ar & payload_bytes;
        ar & error_message;
    }
};

struct RepairWorkerResultFooter {
    std::uint64_t pipe_write_ns = 0;
    std::uint64_t bytes_written = 0;

    template <class Archive>
    void serialize(Archive &ar, const unsigned int /*version*/) {
        ar & pipe_write_ns;
        ar & bytes_written;
    }
};

#if !defined(_WIN32)

template <typename T>
bool serializeBinary(const T &value, std::string &payload,
                     std::uint64_t *serialize_ns = nullptr);

template <typename T>
bool serializeBinary(const T &value, std::string &payload,
                     std::uint64_t *serialize_ns) {
    const auto serialize_start = Clock::now();
    try {
        std::ostringstream stream(std::ios::binary);
        boost::archive::binary_oarchive archive(stream);
        archive << value;
        payload = stream.str();
    } catch (...) {
        return false;
    }
    if (serialize_ns)
        *serialize_ns = elapsedNanoseconds(serialize_start);
    return true;
}

template <typename T>
bool deserializeBinary(const std::string &payload, T &value,
                       std::uint64_t *deserialize_ns = nullptr) {
    const auto deserialize_start = Clock::now();
    try {
        std::istringstream stream(payload, std::ios::binary);
        boost::archive::binary_iarchive archive(stream);
        archive >> value;
    } catch (...) {
        return false;
    }
    if (deserialize_ns)
        *deserialize_ns = elapsedNanoseconds(deserialize_start);
    return true;
}

template <typename T>
std::uint64_t serializedByteSize(const T &value) {
    std::string payload;
    if (!serializeBinary(value, payload))
        return 0;
    return static_cast<std::uint64_t>(payload.size());
}

bool writeBytesToFile(const std::string &path, const std::string &payload,
                      std::uint64_t *file_write_ns = nullptr) {
    const auto write_start = Clock::now();
    std::ofstream stream(path, std::ios::binary | std::ios::trunc);
    if (!stream.good())
        return false;
    stream.write(payload.data(),
                 static_cast<std::streamsize>(payload.size()));
    stream.flush();
    if (file_write_ns)
        *file_write_ns = elapsedNanoseconds(write_start);
    return stream.good();
}

bool readBytesFromFile(const std::string &path, std::string &payload,
                       std::uint64_t *bytes_read = nullptr,
                       std::uint64_t *file_read_ns = nullptr) {
    const auto read_start = Clock::now();
    std::ifstream stream(path, std::ios::binary | std::ios::ate);
    if (!stream.good())
        return false;
    const auto end_pos = stream.tellg();
    if (end_pos < 0)
        return false;
    payload.resize(static_cast<std::size_t>(end_pos));
    if (bytes_read) {
        *bytes_read = static_cast<std::uint64_t>(end_pos);
    }
    stream.seekg(0, std::ios::beg);
    if (!payload.empty()) {
        stream.read(payload.data(),
                    static_cast<std::streamsize>(payload.size()));
    }
    if (file_read_ns)
        *file_read_ns = elapsedNanoseconds(read_start);
    return stream.good() || stream.eof();
}

bool writeAll(int fd, const void *data, std::size_t size) {
    const auto *ptr = static_cast<const char *>(data);
    std::size_t written = 0;
    while (written < size) {
        const ssize_t n =
            ::write(fd, ptr + written, static_cast<size_t>(size - written));
        if (n < 0) {
            if (errno == EINTR)
                continue;
            return false;
        }
        if (n == 0)
            return false;
        written += static_cast<std::size_t>(n);
    }
    return true;
}

bool readAll(int fd, void *data, std::size_t size) {
    auto *ptr = static_cast<char *>(data);
    std::size_t read_count = 0;
    while (read_count < size) {
        const ssize_t n =
            ::read(fd, ptr + read_count, static_cast<size_t>(size - read_count));
        if (n < 0) {
            if (errno == EINTR)
                continue;
            return false;
        }
        if (n == 0)
            return false;
        read_count += static_cast<std::size_t>(n);
    }
    return true;
}

template <typename T>
bool writeFramedBinary(int fd, const T &value,
                       std::uint64_t *bytes_written = nullptr) {
    std::string payload;
    if (!serializeBinary(value, payload))
        return false;
    const std::uint64_t size = static_cast<std::uint64_t>(payload.size());
    if (!writeAll(fd, &size, sizeof(size)))
        return false;
    if (!payload.empty() && !writeAll(fd, payload.data(), payload.size()))
        return false;
    if (bytes_written)
        *bytes_written += sizeof(size) + size;
    return true;
}

template <typename T>
bool readFramedBinary(int fd, T &value, std::uint64_t *bytes_read = nullptr) {
    std::uint64_t size = 0;
    if (!readAll(fd, &size, sizeof(size)))
        return false;
    if (size > (1ULL << 34))
        return false;
    std::string payload;
    payload.resize(static_cast<std::size_t>(size));
    if (!payload.empty() && !readAll(fd, payload.data(), payload.size()))
        return false;
    if (bytes_read)
        *bytes_read += sizeof(size) + size;
    return deserializeBinary(payload, value);
}

volatile std::sig_atomic_t g_repair_worker_cancel_requested = 0;

void repairWorkerCancelHandler(int /*signum*/) {
    g_repair_worker_cancel_requested = 1;
}

void resetRepairWorkerCancelFlag() {
    g_repair_worker_cancel_requested = 0;
}

bool repairWorkerCancelRequested() {
    return g_repair_worker_cancel_requested != 0;
}

void installRepairWorkerCancelHandler() {
    struct sigaction action {};
    action.sa_handler = repairWorkerCancelHandler;
    sigemptyset(&action.sa_mask);
    action.sa_flags = 0;
    ::sigaction(SIGUSR1, &action, nullptr);
}

#endif

} // namespace

namespace comotion {

std::vector<ParallelARC::BatchConflictTask>
ParallelARC::selectConflictBatch(
    const std::vector<SubproblemConflict> &conflicts) const {
    switch (conflict_selection_strategy_) {
    case ParallelArcConflictSelectionStrategy::Greedy:
        break;
    case ParallelArcConflictSelectionStrategy::SpatialDistribution:
        throw std::logic_error(
            "ParallelARC spatial conflict selection is not yet supported");
    }

    std::vector<BatchConflictTask> ordered;
    ordered.reserve(conflicts.size());
    for (const auto &conflict : conflicts) {
        BatchConflictTask task;
        task.conflict = conflict;
        ordered.push_back(std::move(task));
    }

    std::sort(ordered.begin(), ordered.end(),
              [](const BatchConflictTask &lhs, const BatchConflictTask &rhs) {
                  if (lhs.conflict.window_begin_t != rhs.conflict.window_begin_t)
                      return lhs.conflict.window_begin_t <
                             rhs.conflict.window_begin_t;
                  if (lhs.conflict.conflict_timestep !=
                      rhs.conflict.conflict_timestep) {
                      return lhs.conflict.conflict_timestep <
                             rhs.conflict.conflict_timestep;
                  }
                  if (lhs.conflict.seed_robot_i != rhs.conflict.seed_robot_i)
                      return lhs.conflict.seed_robot_i <
                             rhs.conflict.seed_robot_i;
                  return lhs.conflict.seed_robot_j <
                         rhs.conflict.seed_robot_j;
              });
    return ordered;
}

void ParallelARC::resetConflictRoundStats() {
    conflict_round_stats_.clear();
}

void ParallelARC::appendConflictRoundStats(ConflictRoundStats round_stats) {
    if (!round_stats.entries.empty())
        conflict_round_stats_.push_back(std::move(round_stats));
}

void ParallelARC::printConflictRoundStats(std::ostream &os) const {
    for (std::size_t round_index = 0; round_index < conflict_round_stats_.size();
         ++round_index) {
        const auto &round = conflict_round_stats_[round_index];
        os << "Round " << (round_index + 1) << ": " << round.entries.size()
           << " conflicts\n";
        for (std::size_t entry_index = 0; entry_index < round.entries.size();
             ++entry_index) {
            const auto &entry = round.entries[entry_index];
            const std::vector<int> direct_pair{entry.seed_robot_i,
                                               entry.seed_robot_j};
            const auto expanded_added =
                vectorDifference(entry.expanded_team, direct_pair);
            const auto final_added = vectorDifference(entry.final_team,
                                                      entry.expanded_team);
            os << (round_index + 1) << "." << (entry_index + 1) << " Robots "
               << entry.seed_robot_i << " and " << entry.seed_robot_j
               << ", conflict timestep " << entry.conflict_timestep
               << ", expanded team " << formatRobotList(entry.expanded_team)
               << ", expanded added " << formatRobotList(expanded_added)
               << ", final team " << formatRobotList(entry.final_team)
               << ", geometric added " << formatRobotList(final_added)
               << ", initial subproblem window " << entry.window_begin_t << ", "
               << entry.window_end_t << "\n";
        }
    }
}

bool ParallelARC::planInitialIndividualPathsWithWorkers(
    const Clock::time_point &solve_start, double timeLimit,
    unsigned worker_count, std::vector<Path> &working_paths) {
    const int robot_count = problem_->numRobots();
    if (robot_count <= 0)
        return false;
    const unsigned effective_workers = std::max(1u, worker_count);
    if (effective_workers <= 1)
        return planIndividualPaths(solve_start, timeLimit, working_paths);

#if defined(_WIN32)
    (void)effective_workers;
    throw std::runtime_error(
        "ParallelARC initial individual planning requires POSIX fork support");
#else
    struct SignalGuard {
        using Handler = void (*)(int);
        Handler previous = nullptr;
        SignalGuard() : previous(std::signal(SIGPIPE, SIG_IGN)) {}
        ~SignalGuard() {
            if (previous != SIG_ERR)
                std::signal(SIGPIPE, previous);
        }
    } signal_guard;

    struct InitialWorkerState {
        int worker_index = -1;
        std::array<int, 2> command_pipe{{-1, -1}};
        std::array<int, 2> result_pipe{{-1, -1}};
        pid_t pid = -1;
        bool busy = false;
        int robot_id = -1;
        int attempt_index = 0;
    };

    const auto closeFd = [](int &fd) {
        if (fd >= 0) {
            ::close(fd);
            fd = -1;
        }
    };

    working_paths.clear();
    working_paths.resize(static_cast<std::size_t>(robot_count));
    true_arrival_timesteps_.assign(static_cast<std::size_t>(robot_count), 0);
    initial_individual_worker_processes_used_ = effective_workers;
    initial_individual_result_bytes_read_ = 0;
    initial_individual_command_bytes_written_ = 0;
    initial_individual_process_launch_ns_ = 0;
    initial_individual_process_shutdown_ns_ = 0;
    initial_individual_command_write_ns_ = 0;
    initial_individual_result_read_ns_ = 0;
    initial_individual_parent_wait_ns_ = 0;
    initial_individual_duplicate_attempts_ = 0;
    initial_individual_worker_stats_.assign(effective_workers, {});

    const auto initial_wall_start = Clock::now();
    double worker_cpu_seconds_total = 0.0;
    const auto finishInitialTiming = [&]() {
        initial_solution_times_seconds_wall_clock_ +=
            std::chrono::duration<double>(Clock::now() - initial_wall_start)
                .count();
        initial_solution_times_seconds_cpu_ += worker_cpu_seconds_total;
    };

    const auto launch_start = Clock::now();
    std::vector<InitialWorkerState> states(effective_workers);
    for (unsigned i = 0; i < effective_workers; ++i) {
        states[i].worker_index = static_cast<int>(i);
        initial_individual_worker_stats_[i].worker_index = static_cast<int>(i);
        if (::pipe(states[i].command_pipe.data()) != 0 ||
            ::pipe(states[i].result_pipe.data()) != 0) {
            for (auto &state : states) {
                closeFd(state.command_pipe[0]);
                closeFd(state.command_pipe[1]);
                closeFd(state.result_pipe[0]);
                closeFd(state.result_pipe[1]);
            }
            throw std::runtime_error(
                "ParallelARC failed to create initial-planning worker pipes");
        }
    }

    bool launch_failed = false;
    for (unsigned i = 0; i < effective_workers; ++i) {
        const pid_t pid = ::fork();
        if (pid < 0) {
            launch_failed = true;
            break;
        }
        if (pid == 0) {
            for (unsigned j = 0; j < effective_workers; ++j) {
                if (j == i) {
                    closeFd(states[j].command_pipe[1]);
                    closeFd(states[j].result_pipe[0]);
                } else {
                    closeFd(states[j].command_pipe[0]);
                    closeFd(states[j].command_pipe[1]);
                    closeFd(states[j].result_pipe[0]);
                    closeFd(states[j].result_pipe[1]);
                }
            }

            while (true) {
                InitialPlanCommand command;
                if (!readFramedBinary(states[i].command_pipe[0], command))
                    _exit(2);
                if (command.quit)
                    _exit(0);

                InitialPlanResult result;
                result.worker_index = static_cast<int>(i);
                result.robot_id = command.robot_id;
                result.attempt_index = command.attempt_index;
                try {
                    const auto local_seed =
                        command.planning_seed != 0
                            ? command.planning_seed
                            : initialIndividualPlanningSeed(
                                  planning_seed_, command.robot_id,
                                  command.attempt_index);
                    setPlanningSeed(local_seed);
                    const auto worker_wall_start = Clock::now();
                    const auto plan_result = planIndividualPath(
                        command.robot_id, command.time_budget_seconds,
                        local_seed);
                    result.worker_wall_ns = elapsedNanoseconds(worker_wall_start);
                    result.success = plan_result.success;
                    result.solve_ns = plan_result.solve_ns;
                    result.simplify_ns = plan_result.simplify_ns;
                    result.cpu_seconds = plan_result.cpu_seconds;
                    result.path = plan_result.path;
                    result.error_message = plan_result.error_message.empty()
                                               ? plan_result.status_message
                                               : plan_result.error_message;
                } catch (const std::exception &ex) {
                    result.success = false;
                    result.error_message = ex.what();
                } catch (...) {
                    result.success = false;
                    result.error_message = "unknown initial-plan worker exception";
                }
                if (!writeFramedBinary(states[i].result_pipe[1], result))
                    _exit(2);
            }
        }
        states[i].pid = pid;
    }

    for (auto &state : states) {
        closeFd(state.command_pipe[0]);
        closeFd(state.result_pipe[1]);
    }
    initial_individual_process_launch_ns_ = elapsedNanoseconds(launch_start);

    const auto shutdownWorkers = [&](bool terminate) {
        const auto shutdown_start = Clock::now();
        for (auto &state : states) {
            if (terminate && state.pid > 0)
                ::kill(state.pid, SIGTERM);
            if (!terminate && state.command_pipe[1] >= 0) {
                InitialPlanCommand quit;
                quit.quit = true;
                (void)writeFramedBinary(state.command_pipe[1], quit);
            }
            closeFd(state.command_pipe[1]);
        }
        bool ok = true;
        for (auto &state : states) {
            if (state.pid > 0) {
                int status = 0;
                while (::waitpid(state.pid, &status, 0) < 0) {
                    if (errno == EINTR)
                        continue;
                    ok = false;
                    break;
                }
                if (!terminate &&
                    (!WIFEXITED(status) || WEXITSTATUS(status) != 0)) {
                    ok = false;
                }
                state.pid = -1;
            }
            closeFd(state.result_pipe[0]);
        }
        initial_individual_process_shutdown_ns_ +=
            elapsedNanoseconds(shutdown_start);
        return ok;
    };

    if (launch_failed) {
        shutdownWorkers(true);
        throw std::runtime_error(
            "ParallelARC failed to launch initial-planning worker processes");
    }

    std::vector<char> robot_completed(static_cast<std::size_t>(robot_count), 0);
    std::vector<unsigned> robot_live_attempts(
        static_cast<std::size_t>(robot_count), 0);
    std::vector<unsigned> robot_attempts_launched(
        static_cast<std::size_t>(robot_count), 0);

    auto assignRobot = [&](InitialWorkerState &state, int robot_id) {
        if (robot_id < 0 || robot_id >= robot_count)
            return false;
        const auto robot_index = static_cast<std::size_t>(robot_id);
        const double elapsed_s =
            std::chrono::duration<double>(Clock::now() - solve_start).count();
        const double remaining = timeLimit - elapsed_s;
        if (remaining <= 0.0)
            return false;
        const unsigned attempt_index = robot_attempts_launched[robot_index]++;
        InitialPlanCommand command;
        command.robot_id = robot_id;
        command.attempt_index = static_cast<int>(attempt_index);
        command.planning_seed = initialIndividualPlanningSeed(
            planning_seed_, robot_id, static_cast<int>(attempt_index));
        command.time_budget_seconds = remaining;
        std::uint64_t bytes_written = 0;
        const auto write_start = Clock::now();
        const bool write_ok =
            writeFramedBinary(state.command_pipe[1], command, &bytes_written);
        const auto write_ns = elapsedNanoseconds(write_start);
        initial_individual_command_write_ns_ += write_ns;
        if (state.worker_index >= 0 &&
            static_cast<std::size_t>(state.worker_index) <
                initial_individual_worker_stats_.size()) {
            auto &stats = initial_individual_worker_stats_[
                static_cast<std::size_t>(state.worker_index)];
            stats.command_write_ns += write_ns;
            stats.command_bytes_written += bytes_written;
        }
        if (!write_ok)
            return false;
        initial_individual_command_bytes_written_ += bytes_written;
        state.busy = true;
        state.robot_id = robot_id;
        state.attempt_index = static_cast<int>(attempt_index);
        ++robot_live_attempts[robot_index];
        if (attempt_index > 0)
            ++initial_individual_duplicate_attempts_;
        return true;
    };

    int next_robot = 0;
    int duplicate_cursor = 0;
    int completed = 0;
    bool failed = false;

    auto nextDuplicateRobot = [&]() -> std::optional<int> {
        if (!initial_solution_or_ || robot_count <= 0)
            return std::nullopt;
        for (int offset = 0; offset < robot_count; ++offset) {
            const int robot_id = (duplicate_cursor + offset) % robot_count;
            const auto robot_index = static_cast<std::size_t>(robot_id);
            if (robot_completed[robot_index] ||
                robot_live_attempts[robot_index] == 0) {
                continue;
            }
            duplicate_cursor = (robot_id + 1) % robot_count;
            return robot_id;
        }
        return std::nullopt;
    };

    auto assignNextWork = [&](InitialWorkerState &state) {
        if (next_robot < robot_count)
            return assignRobot(state, next_robot++);
        const auto duplicate_robot = nextDuplicateRobot();
        if (duplicate_robot.has_value())
            return assignRobot(state, *duplicate_robot);
        return true;
    };

    for (auto &state : states) {
        if (!assignNextWork(state)) {
            failed = true;
            break;
        }
    }

    while (!failed && completed < robot_count) {
        std::vector<pollfd> poll_fds;
        std::vector<std::size_t> poll_state_indices;
        for (std::size_t i = 0; i < states.size(); ++i) {
            if (!states[i].busy)
                continue;
            pollfd pfd {};
            pfd.fd = states[i].result_pipe[0];
            pfd.events = POLLIN | POLLHUP | POLLERR;
            poll_fds.push_back(pfd);
            poll_state_indices.push_back(i);
        }
        if (poll_fds.empty()) {
            failed = true;
            break;
        }

        const double elapsed_s =
            std::chrono::duration<double>(Clock::now() - solve_start).count();
        const double remaining = timeLimit - elapsed_s;
        if (remaining <= 0.0) {
            failed = true;
            break;
        }
        const int timeout_ms =
            std::max(0, static_cast<int>(std::ceil(remaining * 1000.0)));
        int ready = 0;
        const auto poll_start = Clock::now();
        do {
            ready = ::poll(poll_fds.data(),
                           static_cast<nfds_t>(poll_fds.size()), timeout_ms);
        } while (ready < 0 && errno == EINTR);
        initial_individual_parent_wait_ns_ += elapsedNanoseconds(poll_start);
        if (ready <= 0) {
            failed = true;
            break;
        }

        for (std::size_t i = 0; i < poll_fds.size(); ++i) {
            if (!(poll_fds[i].revents & (POLLIN | POLLHUP | POLLERR)))
                continue;
            auto &state = states[poll_state_indices[i]];
            InitialPlanResult result;
            std::uint64_t bytes_read = 0;
            const auto read_start = Clock::now();
            const bool read_ok =
                readFramedBinary(state.result_pipe[0], result, &bytes_read);
            const auto read_ns = elapsedNanoseconds(read_start);
            initial_individual_result_read_ns_ += read_ns;
            if (state.worker_index >= 0 &&
                static_cast<std::size_t>(state.worker_index) <
                    initial_individual_worker_stats_.size()) {
                auto &stats = initial_individual_worker_stats_[
                    static_cast<std::size_t>(state.worker_index)];
                stats.result_read_ns += read_ns;
                stats.result_bytes_read += bytes_read;
                if (result.robot_id >= 0)
                    stats.robots.push_back(result.robot_id);
                stats.worker_wall_ns += result.worker_wall_ns;
                stats.solve_ns += result.solve_ns;
                stats.simplify_ns += result.simplify_ns;
                stats.cpu_seconds += result.cpu_seconds;
            }
            if (!read_ok) {
                failed = true;
                break;
            }
            initial_individual_result_bytes_read_ += bytes_read;
            const int expected_robot_id = state.robot_id;
            const int expected_attempt_index = state.attempt_index;
            state.busy = false;
            state.robot_id = -1;
            state.attempt_index = 0;

            IndividualPlanResult stats_result;
            stats_result.solve_ns = result.solve_ns;
            stats_result.simplify_ns = result.simplify_ns;
            recordInitialIndividualPlanStats(stats_result);
            worker_cpu_seconds_total += result.cpu_seconds;

            if (expected_robot_id < 0 || expected_robot_id >= robot_count ||
                result.robot_id != expected_robot_id ||
                result.attempt_index != expected_attempt_index) {
                failed = true;
                break;
            }

            const auto robot_index = static_cast<std::size_t>(expected_robot_id);
            if (robot_live_attempts[robot_index] > 0)
                --robot_live_attempts[robot_index];

            if (!robot_completed[robot_index]) {
                if (result.success && !result.path.empty()) {
                    const auto arrival_timestep = static_cast<std::uint64_t>(
                        result.path.arrival_timestep());
                    working_paths[robot_index] = std::move(result.path);
                    true_arrival_timesteps_[robot_index] = arrival_timestep;
                    robot_completed[robot_index] = 1;
                    ++completed;
                } else if (robot_live_attempts[robot_index] == 0) {
                    failed = true;
                    break;
                }
            }

            if (completed >= robot_count)
                break;
            if (!assignNextWork(state)) {
                failed = true;
                break;
            }
        }
    }

    if (failed) {
        shutdownWorkers(true);
        finishInitialTiming();
        return false;
    }

    const bool workers_busy =
        std::any_of(states.begin(), states.end(),
                    [](const InitialWorkerState &state) {
                        return state.busy;
                    });
    const bool shutdown_ok = shutdownWorkers(workers_busy);
    if (!shutdown_ok) {
        finishInitialTiming();
        return false;
    }
    finishInitialIndividualPaths(working_paths);
    finishInitialTiming();
    return true;
#endif
}

ompl::base::PlannerStatus ParallelARC::solve(double timeLimit) {
    resetArcSolveState();
    resetConflictRoundStats();
    initial_individual_worker_processes_used_ = 0;
    initial_individual_result_bytes_read_ = 0;
    initial_individual_command_bytes_written_ = 0;
    initial_individual_process_launch_ns_ = 0;
    initial_individual_process_shutdown_ns_ = 0;
    initial_individual_command_write_ns_ = 0;
    initial_individual_result_read_ns_ = 0;
    initial_individual_parent_wait_ns_ = 0;
    initial_individual_duplicate_attempts_ = 0;
    initial_individual_worker_stats_.clear();
    const auto solve_start = Clock::now();
    const unsigned effective_workers = std::max(1u, worker_processes_);
    ArcPlannerStatsSummary planner_stats_summary;
    planner_stats_summary.local_solver_mode = local_solver_mode_;
    planner_stats_summary.local_prioritized_strrt_max_iterations =
        local_prioritized_strrt_max_iterations_;
    const auto finalizePlannerStats = [&]() {
        auto stats = plannerStatsJsonFromSummary(
            planner_stats_summary);
        stats["parallel_arc_result_transport"] = "pipes";
        stats["parallel_arc_commit_strategy"] = "parent_splice";
        stats["parallel_arc_parallel_initial_individual_plans"] =
            parallelize_initial_individual_plans_;
        stats["parallel_arc_initial_individual_workers"] =
            initial_individual_worker_processes_used_;
        stats["parallel_arc_initial_solution_or"] = initial_solution_or_;
        stats["parallel_arc_initial_individual_duplicate_attempts_enabled"] =
            initial_solution_or_;
        stats["parallel_arc_initial_individual_or_parallelism"] =
            initial_individual_duplicate_attempts_ > 0;
        stats["parallel_arc_initial_individual_duplicate_attempts"] =
            initial_individual_duplicate_attempts_;
        stats["parallel_arc_initial_individual_duplicate_attempt_count"] =
            initial_individual_duplicate_attempts_;
        stats["parallel_arc_initial_individual_process_lifecycle"] =
            initial_individual_worker_processes_used_ > 0 ? "persistent_pool"
                                                          : "";
        stats["parallel_arc_initial_individual_assignment_strategy"] =
            initial_individual_worker_processes_used_ > 0
                ? (initial_solution_or_ ? "dynamic_robot_queue_with_or"
                                        : "dynamic_robot_queue")
                : "";
        stats["parallel_arc_initial_individual_ipc"] =
            initial_individual_worker_processes_used_ > 0 ? "pipes" : "";
        stats["parallel_arc_initial_individual_payload"] =
            initial_individual_worker_processes_used_ > 0 ? "path_timings" : "";
        stats["parallel_arc_initial_individual_command_bytes_written"] =
            initial_individual_command_bytes_written_;
        stats["parallel_arc_initial_individual_result_bytes_read"] =
            initial_individual_result_bytes_read_;
        stats["parallel_arc_initial_individual_process_launch_ns"] =
            initial_individual_process_launch_ns_;
        stats["parallel_arc_initial_individual_process_shutdown_ns"] =
            initial_individual_process_shutdown_ns_;
        stats["parallel_arc_initial_individual_command_write_ns"] =
            initial_individual_command_write_ns_;
        stats["parallel_arc_initial_individual_result_read_ns"] =
            initial_individual_result_read_ns_;
        stats["parallel_arc_initial_individual_parent_wait_ns"] =
            initial_individual_parent_wait_ns_;
        stats["parallel_arc_initial_individual_process_launch_ms"] =
            static_cast<double>(initial_individual_process_launch_ns_) * 1e-6;
        stats["parallel_arc_initial_individual_process_shutdown_ms"] =
            static_cast<double>(initial_individual_process_shutdown_ns_) * 1e-6;
        stats["parallel_arc_initial_individual_command_write_ms"] =
            static_cast<double>(initial_individual_command_write_ns_) * 1e-6;
        stats["parallel_arc_initial_individual_result_read_ms"] =
            static_cast<double>(initial_individual_result_read_ns_) * 1e-6;
        stats["parallel_arc_initial_individual_parent_wait_ms"] =
            static_cast<double>(initial_individual_parent_wait_ns_) * 1e-6;
        nlohmann::json initial_worker_stats = nlohmann::json::array();
        for (const auto &worker_stats : initial_individual_worker_stats_) {
            initial_worker_stats.push_back({
                {"worker_index", worker_stats.worker_index},
                {"robots", worker_stats.robots},
                {"robot_count", worker_stats.robots.size()},
                {"command_write_ns", worker_stats.command_write_ns},
                {"result_read_ns", worker_stats.result_read_ns},
                {"command_bytes_written",
                 worker_stats.command_bytes_written},
                {"result_bytes_read", worker_stats.result_bytes_read},
                {"worker_wall_ns", worker_stats.worker_wall_ns},
                {"solve_ns", worker_stats.solve_ns},
                {"simplify_ns", worker_stats.simplify_ns},
                {"cpu_seconds", worker_stats.cpu_seconds},
                {"command_write_ms",
                 static_cast<double>(worker_stats.command_write_ns) * 1e-6},
                {"result_read_ms",
                 static_cast<double>(worker_stats.result_read_ns) * 1e-6},
                {"worker_wall_ms",
                 static_cast<double>(worker_stats.worker_wall_ns) * 1e-6},
                {"solve_ms",
                 static_cast<double>(worker_stats.solve_ns) * 1e-6},
                {"simplify_ms",
                 static_cast<double>(worker_stats.simplify_ns) * 1e-6},
            });
        }
        stats["parallel_arc_initial_individual_worker_stats"] =
            std::move(initial_worker_stats);
        stats["parallel_arc_conflict_find_mode"] =
            parallelArcConflictFindModeStr(conflict_find_mode_);
        stats["parallel_arc_conflict_find_horizon"] =
            conflict_find_horizon_;
        stats["parallel_arc_conflict_find_workers"] =
            conflict_find_mode_ == ParallelArcConflictFindMode::SegmentParallel
                ? worker_processes_
                : 1;
        stats["parallel_arc_conflict_find_ipc"] =
            conflict_find_mode_ == ParallelArcConflictFindMode::SegmentParallel
                ? "pipes"
                : "";
        stats["parallel_arc_conflict_find_process_lifecycle"] =
            conflict_find_mode_ == ParallelArcConflictFindMode::SegmentParallel
                ? "per_find_call"
                : "";
        stats["parallel_arc_conflict_find_assignment_strategy"] =
            conflict_find_mode_ == ParallelArcConflictFindMode::SegmentParallel
                ? (worker_processes_ == 4 || worker_processes_ == 8 ||
                           worker_processes_ == 16
                       ? "pair_cover_buckets"
                       : "round_robin_pairs")
                : "";
        stats["parallel_arc_conflict_find_logical_bucket_count"] =
            conflict_find_mode_ == ParallelArcConflictFindMode::SegmentParallel
                ? worker_processes_
                : 0;
        stats["parallel_arc_repair_parallelism"] = worker_processes_ > 1;
        stats["parallel_arc_repair_or_parallelism"] =
            worker_processes_ > 1 && repair_duplicate_attempts_;
        stats["parallel_arc_repair_duplicate_attempts"] =
            repair_duplicate_attempts_;
        stats["parallel_arc_repair_assignment_strategy"] =
            worker_processes_ > 1
                ? (repair_duplicate_attempts_
                       ? "round_robin_active_subproblems"
                       : "round_robin_one_live_attempt_per_subproblem")
                : "";
        stats["parallel_arc_repair_process_lifecycle"] =
            worker_processes_ > 1 ? "persistent_batch_pool" : "";
        stats["parallel_arc_repair_ipc"] =
            worker_processes_ > 1 ? "pipes" : "";
        stats["parallel_arc_repair_cancellation"] =
            worker_processes_ > 1
                ? "cooperative_signal_with_terminate_fallback"
                : "";
        setPlannerStatsJson(std::move(stats));
    };

    const auto robot_count_for_initial =
        static_cast<unsigned>(std::max(0, problem_->numRobots()));
    const unsigned initial_workers =
        robot_count_for_initial == 0
            ? 0
            : (initial_solution_or_
                   ? effective_workers
                   : std::min(effective_workers, robot_count_for_initial));
    const bool use_parallel_initial =
        parallelize_initial_individual_plans_ && initial_workers > 1;
    const bool initial_plans_ok =
        use_parallel_initial
            ? planInitialIndividualPathsWithWorkers(
                  solve_start, timeLimit, initial_workers, solution_paths_)
            : planIndividualPaths(solve_start, timeLimit, solution_paths_);
    if (!initial_plans_ok) {
        planner_stats_summary.initial_solution_times_seconds_wall_clock =
            initial_solution_times_seconds_wall_clock_;
        planner_stats_summary.initial_solution_times_seconds_cpu =
            initial_solution_times_seconds_cpu_;
        finalizePlannerStats();
        return ompl::base::PlannerStatus::TIMEOUT;
    }
    planner_stats_summary.initial_solution_times_seconds_wall_clock =
        initial_solution_times_seconds_wall_clock_;
    planner_stats_summary.initial_solution_times_seconds_cpu =
        initial_solution_times_seconds_cpu_;
    initializeConflictScanStarts(solution_paths_.size());
    std::vector<std::uint64_t> path_versions(solution_paths_.size(), 0);

#if defined(_WIN32)
    if (effective_workers > 1) {
        throw std::runtime_error(
            "ParallelARC with worker_processes > 1 requires POSIX fork support");
    }
#endif

    auto ptrs = problem_->robotModelPtrs();
    int batch_index = 0;
    auto returnExactSolution = [&]() {
        setSolutionMetricsFromPaths(solution_paths_);
        finalizePlannerStats();
        return ompl::base::PlannerStatus::EXACT_SOLUTION;
    };
    while (true) {
        const double elapsed_s =
            std::chrono::duration<double>(Clock::now() - solve_start).count();
        if (elapsed_s >= timeLimit)
            break;

        switch (parallel_strategy_) {
        case ParallelArcParallelStrategy::Synchronous:
            break;
        case ParallelArcParallelStrategy::Asynchronous:
            throw std::logic_error(
                "ParallelARC asynchronous scheduling is not yet supported");
        }

        CompositePathValidationOptions options = conflictScanOptions();
        if (conflict_find_mode_ ==
            ParallelArcConflictFindMode::SegmentParallel) {
            options.conflict_find_parallel_workers = effective_workers;
            options.conflict_find_parallel_horizon = conflict_find_horizon_;
        }
        ConflictChecker conflict_checker(problem_->collisionChecker());
        std::vector<std::size_t> next_t_begin_by_pair;
        std::vector<SubproblemConflict> conflicts = conflict_checker.findConflicts(
            solution_paths_, ptrs, options, 0, effective_workers, true,
            [this](const Conflict &conflict) {
                return expandConflictForSubproblem(conflict);
            },
            nullptr, &next_t_begin_by_pair);
        if (conflicts.empty())
            return returnExactSolution();
        planner_stats_summary.num_conflicts +=
            static_cast<std::uint64_t>(conflicts.size());

        auto tasks = selectConflictBatch(conflicts);
        for (auto &task : tasks) {
            task.base_versions.clear();
            task.base_versions.reserve(task.conflict.robots.size());
            for (const int robot : task.conflict.robots) {
                const auto robot_index = static_cast<std::size_t>(robot);
                task.base_versions.push_back(
                    robot_index < path_versions.size()
                        ? path_versions[robot_index]
                        : 0);
            }
        }
        if (tasks.empty())
            return returnExactSolution();
        ++planner_stats_summary.subproblem_batches;

        enum class ApplyResult { Applied, Discarded, Error };

        auto baseVersionForRobot = [](const BatchConflictTask &task,
                                      int robot) -> std::uint64_t {
            for (std::size_t i = 0; i < task.conflict.robots.size(); ++i) {
                if (task.conflict.robots[i] == robot &&
                    i < task.base_versions.size()) {
                    return task.base_versions[i];
                }
            }
            return 0;
        };

        auto appendOutcomePatches =
            [&](const BatchConflictTask &task, const ARC::RepairOutcome &outcome,
                WorkerResult &result) {
                if (outcome.final_involved_robots.size() !=
                    outcome.local_patch_paths.size()) {
                    throw std::runtime_error(
                        "ParallelARC worker repair did not produce one local "
                        "patch per involved robot");
                }
                for (std::size_t i = 0;
                     i < outcome.final_involved_robots.size(); ++i) {
                    WorkerResult::PathPatch patch;
                    patch.robot_id = outcome.final_involved_robots[i];
                    patch.base_version =
                        baseVersionForRobot(task, patch.robot_id);
                    patch.window_start_t = outcome.window_start_t;
                    patch.window_end_t = outcome.window_end_t;
                    patch.local_path = outcome.local_patch_paths[i];
                    result.patches.push_back(std::move(patch));
                }
            };

        auto applyResult = [&](const WorkerResult &result,
                               std::set<int> &patched_robots) {
            if (result.final_involved_robots.size() != result.patches.size())
                return ApplyResult::Error;

            std::vector<int> patch_robots;
            std::vector<const Path *> local_paths;
            patch_robots.reserve(result.patches.size());
            local_paths.reserve(result.patches.size());
            for (const auto &patch : result.patches) {
                const int robot = patch.robot_id;
                if (patch_robots.size() >= result.final_involved_robots.size() ||
                    result.final_involved_robots[patch_robots.size()] != robot)
                    return ApplyResult::Error;
                if (robot < 0 ||
                    static_cast<std::size_t>(robot) >= solution_paths_.size())
                    return ApplyResult::Error;
                if (patch.window_start_t != result.window_start_t ||
                    patch.window_end_t != result.window_end_t)
                    return ApplyResult::Error;
                if (patch.local_path.empty())
                    return ApplyResult::Error;
                if (patched_robots.count(robot)) {
                    return ApplyResult::Discarded;
                }
                if (patch.base_version !=
                    path_versions[static_cast<std::size_t>(robot)]) {
                    return ApplyResult::Discarded;
                }
                patch_robots.push_back(robot);
                local_paths.push_back(&patch.local_path);
            }

            try {
                spliceSolutionIntoPaths(patch_robots, result.window_start_t,
                                        result.window_end_t, local_paths,
                                        solution_paths_);
            } catch (...) {
                return ApplyResult::Error;
            }
            for (const int robot : patch_robots) {
                patched_robots.insert(robot);
                ++path_versions[static_cast<std::size_t>(robot)];
            }
            recordAppliedRepairHistory(patch_robots, result.window_start_t,
                                       result.window_end_t);
            return ApplyResult::Applied;
        };

        const auto batch_resolution_start = Clock::now();

        if (effective_workers <= 1) {
            std::set<int> patched_robots;
            std::vector<int> patched_window_starts(
                solution_paths_.size(), std::numeric_limits<int>::max());
            ConflictRoundStats round_stats;
            for (const auto &task : tasks) {
                const auto stats_before = currentArcPlannerStatsSummary();
                auto candidate_paths = solution_paths_;
                const auto repair_start = Clock::now();
                const double repair_cpu_start = processCpuSeconds();
                const auto outcome = resolveConflictOnPaths(
                    task.conflict, solve_start, timeLimit, candidate_paths,
                    false);
                const double repair_elapsed_seconds =
                    std::chrono::duration<double>(Clock::now() - repair_start)
                        .count();
                const double repair_cpu_seconds =
                    elapsedProcessCpuSeconds(repair_cpu_start);
                const auto stats_after = currentArcPlannerStatsSummary();
                planner_stats_summary.subproblem_attempts +=
                    stats_after.subproblem_attempts -
                    stats_before.subproblem_attempts;
                planner_stats_summary.temporal_expansions +=
                    stats_after.temporal_expansions -
                    stats_before.temporal_expansions;
                planner_stats_summary.conflict_resolution_times_seconds_total +=
                    repair_elapsed_seconds;
                planner_stats_summary.conflict_resolution_times_seconds_cpu +=
                    repair_cpu_seconds;
                if (!outcome.resolved) {
                    planner_stats_summary
                        .conflict_resolution_times_seconds_wall_clock +=
                        std::chrono::duration<double>(Clock::now() -
                                                      batch_resolution_start)
                            .count();
                    finalizePlannerStats();
                    return ompl::base::PlannerStatus::TIMEOUT;
                }

                WorkerResult result;
                result.success = true;
                result.window_start_t = outcome.window_start_t;
                result.window_end_t = outcome.window_end_t;
                result.final_involved_robots = outcome.final_involved_robots;
                appendOutcomePatches(task, outcome, result);

                const auto apply_status =
                    applyResult(result, patched_robots);
                if (apply_status == ApplyResult::Error) {
                    planner_stats_summary
                        .conflict_resolution_times_seconds_wall_clock +=
                        std::chrono::duration<double>(Clock::now() -
                                                      batch_resolution_start)
                            .count();
                    finalizePlannerStats();
                    return ompl::base::PlannerStatus::TIMEOUT;
                }
                if (apply_status == ApplyResult::Applied) {
                    round_stats.entries.push_back(ConflictRoundEntry{
                        task.conflict.seed_robot_i,
                        task.conflict.seed_robot_j,
                        task.conflict.conflict_timestep,
                        task.conflict.robots,
                        outcome.final_involved_robots,
                        task.conflict.window_begin_t,
                        outcome.window_end_t,
                    });
                    for (const int robot : outcome.final_involved_robots) {
                        patched_window_starts[static_cast<std::size_t>(robot)] =
                            outcome.window_start_t;
                    }
                }
            }

            planner_stats_summary.conflict_resolution_times_seconds_wall_clock +=
                std::chrono::duration<double>(Clock::now() -
                                              batch_resolution_start)
                    .count();
            applyConflictScanProgress(next_t_begin_by_pair);
            for (std::size_t robot = 0; robot < patched_window_starts.size();
                 ++robot) {
                if (patched_window_starts[robot] !=
                    std::numeric_limits<int>::max()) {
                    resetConflictScanStartsForRobots(
                        {static_cast<int>(robot)}, patched_window_starts[robot]);
                }
            }
            appendConflictRoundStats(std::move(round_stats));
            ++batch_index;
            continue;
        }

#if !defined(_WIN32)
        struct RuntimeTaskState {
            BatchConflictTask task;
            std::size_t task_index = 0;
            std::size_t live_attempts = 0;
            std::size_t attempts_launched = 0;
            bool solved = false;
            bool failed = false;
            bool terminal_failure_seen = false;
        };

        struct FinalizedAttempt {
            int slot_index = -1;
            std::size_t task_index = 0;
            std::size_t attempt_index = 0;
            bool cancelled = false;
            bool exit_failed = false;
            bool has_result = false;
            WorkerResult result;
        };

        struct RepairWorkerSlot {
            int slot_index = -1;
            std::array<int, 2> command_pipe{{-1, -1}};
            std::array<int, 2> result_pipe{{-1, -1}};
            pid_t pid = -1;
            std::size_t task_index = 0;
            std::size_t attempt_index = 0;
            bool active = false;
            bool cancel_requested = false;
            std::optional<Clock::time_point> cancel_signal_time;
        };

        std::vector<RuntimeTaskState> task_states;
        task_states.reserve(tasks.size());
        for (std::size_t i = 0; i < tasks.size(); ++i) {
            RuntimeTaskState task_state;
            task_state.task = tasks[i];
            task_state.task_index = i;
            task_states.push_back(std::move(task_state));
        }

        std::vector<RepairWorkerSlot> slots(effective_workers);
        for (std::size_t i = 0; i < slots.size(); ++i)
            slots[i].slot_index = static_cast<int>(i);

        bool batch_failed = false;
        bool launch_failed = false;
        std::size_t active_attempts = 0;
        std::size_t unresolved_tasks = task_states.size();
        std::size_t next_task_cursor = 0;
        std::set<int> patched_robots;
        std::vector<int> patched_window_starts(
            solution_paths_.size(), std::numeric_limits<int>::max());
        ConflictRoundStats round_stats;
        const std::vector<Path> batch_solution_paths = solution_paths_;
        constexpr auto kCancelGrace = std::chrono::milliseconds(250);

        struct SignalGuard {
            using Handler = void (*)(int);
            Handler previous = nullptr;
            SignalGuard() : previous(std::signal(SIGPIPE, SIG_IGN)) {}
            ~SignalGuard() {
                if (previous != SIG_ERR)
                    std::signal(SIGPIPE, previous);
            }
        } signal_guard;

        const auto closeFd = [](int &fd) {
            if (fd >= 0) {
                ::close(fd);
                fd = -1;
            }
        };

        auto taskAssignable = [&](std::size_t task_index) {
            const auto &task_state = task_states[task_index];
            if (task_state.solved || task_state.failed ||
                task_state.terminal_failure_seen) {
                return false;
            }
            return repair_duplicate_attempts_ || task_state.live_attempts == 0;
        };

        auto nextAssignableTask = [&]() -> std::optional<std::size_t> {
            if (task_states.empty())
                return std::nullopt;
            for (std::size_t offset = 0; offset < task_states.size(); ++offset) {
                const std::size_t task_index =
                    (next_task_cursor + offset) % task_states.size();
                if (taskAssignable(task_index)) {
                    next_task_cursor =
                        (task_index + 1) % task_states.size();
                    return task_index;
                }
            }
            return std::nullopt;
        };

        auto resetWorkerArcAttemptState = [&]() {
            num_conflicts_ = 0;
            num_subproblem_attempts_ = 0;
            num_temporal_expansions_ = 0;
            initial_solution_times_seconds_wall_clock_ = 0.0;
            initial_solution_times_seconds_cpu_ = 0.0;
            conflict_resolution_times_seconds_.clear();
            conflict_resolution_times_cpu_seconds_.clear();
        };

        auto writeRepairWorkerResponse =
            [&](int fd, RepairWorkerResultHeader header,
                const std::string &payload) {
                std::uint64_t bytes_written = 0;
                const auto write_start = Clock::now();
                bool ok = writeFramedBinary(fd, header, &bytes_written);
                if (ok && !payload.empty()) {
                    ok = writeAll(fd, payload.data(), payload.size());
                    bytes_written +=
                        static_cast<std::uint64_t>(payload.size());
                }
                const auto pipe_write_ns = elapsedNanoseconds(write_start);
                RepairWorkerResultFooter footer;
                footer.pipe_write_ns = pipe_write_ns;
                footer.bytes_written = bytes_written;
                ok = ok && writeFramedBinary(fd, footer, &bytes_written);
                return ok;
            };

        auto spawnRepairWorker = [&](RepairWorkerSlot &slot) -> bool {
            closeFd(slot.command_pipe[0]);
            closeFd(slot.command_pipe[1]);
            closeFd(slot.result_pipe[0]);
            closeFd(slot.result_pipe[1]);
            slot.pid = -1;
            if (::pipe(slot.command_pipe.data()) != 0)
                return false;
            if (::pipe(slot.result_pipe.data()) != 0) {
                closeFd(slot.command_pipe[0]);
                closeFd(slot.command_pipe[1]);
                return false;
            }

            const auto launch_start = Clock::now();
            const pid_t pid = ::fork();
            if (pid < 0) {
                closeFd(slot.command_pipe[0]);
                closeFd(slot.command_pipe[1]);
                closeFd(slot.result_pipe[0]);
                closeFd(slot.result_pipe[1]);
                return false;
            }

            if (pid == 0) {
                for (std::size_t j = 0; j < slots.size(); ++j) {
                    if (slots[j].slot_index == slot.slot_index) {
                        closeFd(slots[j].command_pipe[1]);
                        closeFd(slots[j].result_pipe[0]);
                    } else {
                        closeFd(slots[j].command_pipe[0]);
                        closeFd(slots[j].command_pipe[1]);
                        closeFd(slots[j].result_pipe[0]);
                        closeFd(slots[j].result_pipe[1]);
                    }
                }

                installRepairWorkerCancelHandler();
                solution_paths_ = batch_solution_paths;

                while (true) {
                    RepairWorkerCommand command;
                    if (!readFramedBinary(slot.command_pipe[0], command))
                        _exit(2);
                    if (command.quit)
                        _exit(0);
                    if (command.task_index < 0 ||
                        static_cast<std::size_t>(command.task_index) >=
                            tasks.size()) {
                        _exit(2);
                    }

                    resetRepairWorkerCancelFlag();
                    resetWorkerArcAttemptState();
                    solution_paths_ = batch_solution_paths;

                    const auto &task =
                        tasks[static_cast<std::size_t>(command.task_index)];
                    RepairWorkerResultHeader header;
                    header.slot_index = command.slot_index;
                    header.task_index = command.task_index;
                    header.attempt_index = command.attempt_index;
                    WorkerResult result;
                    std::string payload;
                    bool serialize_ok = false;

                    const auto worker_start = Clock::now();
                    try {
                        setPlanningSeed(command.planning_seed);

                        const auto repair_start = Clock::now();
                        const double repair_cpu_start = processCpuSeconds();
                        const auto outcome = resolveConflictOnPaths(
                            task.conflict, solve_start, timeLimit,
                            solution_paths_, false,
                            []() { return repairWorkerCancelRequested(); });
                        const double repair_elapsed_seconds =
                            std::chrono::duration<double>(Clock::now() -
                                                          repair_start)
                                .count();
                        const double repair_cpu_seconds =
                            elapsedProcessCpuSeconds(repair_cpu_start);
                        const auto repair_summary =
                            currentArcPlannerStatsSummary();
                        result.success = outcome.resolved;
                        result.window_start_t = outcome.window_start_t;
                        result.window_end_t = outcome.window_end_t;
                        result.final_involved_robots =
                            outcome.final_involved_robots;
                        result.repair_stats.subproblem_attempts =
                            repair_summary.subproblem_attempts;
                        result.repair_stats.temporal_expansions =
                            repair_summary.temporal_expansions;
                        result.repair_stats
                            .conflict_resolution_times_seconds_wall_clock =
                            repair_elapsed_seconds;
                        result.repair_stats
                            .conflict_resolution_times_seconds_total =
                            repair_elapsed_seconds;
                        result.repair_stats
                            .conflict_resolution_times_seconds_cpu =
                            repair_cpu_seconds;
                        if (outcome.resolved)
                            appendOutcomePatches(task, outcome, result);
                        result.patch_payload_bytes =
                            serializedByteSize(result.patches);
                    } catch (const std::exception &ex) {
                        result.success = false;
                        result.error_message = ex.what();
                    } catch (...) {
                        result.success = false;
                        result.error_message = "unknown worker exception";
                    }
                    header.worker_wall_ns = elapsedNanoseconds(worker_start);
                    header.cancelled =
                        repairWorkerCancelRequested() && !result.success;
                    if (!header.cancelled) {
                        serialize_ok =
                            serializeBinary(result, payload, &header.serialize_ns);
                        header.has_payload = serialize_ok;
                        header.worker_error = !serialize_ok;
                        if (!serialize_ok)
                            header.error_message =
                                "repair worker failed to serialize result";
                        else
                            header.payload_bytes =
                                static_cast<std::uint64_t>(payload.size());
                    }

                    if (!writeRepairWorkerResponse(slot.result_pipe[1], header,
                                                   payload)) {
                        _exit(2);
                    }
                }
            }

            slot.pid = pid;
            closeFd(slot.command_pipe[0]);
            closeFd(slot.result_pipe[1]);
            return true;
        };

        auto shutdownWorkers = [&](bool terminate) {
            for (auto &slot : slots) {
                if (slot.pid <= 0)
                    continue;
                if (terminate) {
                    ::kill(slot.pid, SIGTERM);
                } else if (slot.command_pipe[1] >= 0) {
                    RepairWorkerCommand quit;
                    quit.quit = true;
                    (void)writeFramedBinary(slot.command_pipe[1], quit);
                }
                closeFd(slot.command_pipe[1]);
            }
            for (auto &slot : slots) {
                if (slot.pid > 0) {
                    int status = 0;
                    const auto wait_start = Clock::now();
                    while (::waitpid(slot.pid, &status, 0) < 0) {
                        if (errno == EINTR)
                            continue;
                        break;
                    }
                    slot.pid = -1;
                }
                closeFd(slot.command_pipe[0]);
                closeFd(slot.command_pipe[1]);
                closeFd(slot.result_pipe[0]);
                closeFd(slot.result_pipe[1]);
            }
        };

        auto clearActiveSlot = [&](RepairWorkerSlot &slot) {
            if (slot.active) {
                if (active_attempts > 0)
                    --active_attempts;
                auto &runtime_task = task_states[slot.task_index];
                if (runtime_task.live_attempts > 0)
                    --runtime_task.live_attempts;
            }
            slot.active = false;
            slot.cancel_requested = false;
            slot.cancel_signal_time.reset();
        };

        auto finalizeWorkerResult =
            [&](RepairWorkerSlot &slot,
                std::uint64_t wait_ns) -> FinalizedAttempt {
                const auto finalize_start = Clock::now();
                FinalizedAttempt finalized;
                finalized.slot_index = slot.slot_index;
                finalized.task_index = slot.task_index;
                finalized.attempt_index = slot.attempt_index;

                RepairWorkerResultHeader header;
                RepairWorkerResultFooter footer;
                std::uint64_t bytes_read = 0;
                const auto pipe_read_start = Clock::now();
                bool read_ok =
                    readFramedBinary(slot.result_pipe[0], header, &bytes_read);
                std::string payload;
                if (read_ok && header.payload_bytes > 0) {
                    payload.resize(
                        static_cast<std::size_t>(header.payload_bytes));
                    read_ok = readAll(slot.result_pipe[0], payload.data(),
                                      payload.size());
                    bytes_read += header.payload_bytes;
                }
                if (read_ok)
                    read_ok =
                        readFramedBinary(slot.result_pipe[0], footer, &bytes_read);
                const auto pipe_read_ns = elapsedNanoseconds(pipe_read_start);

                finalized.cancelled =
                    slot.cancel_requested || (read_ok && header.cancelled);
                finalized.exit_failed =
                    !read_ok || (read_ok && header.worker_error);

                std::uint64_t deserialize_ns = 0;
                if (!finalized.cancelled && !finalized.exit_failed &&
                    header.has_payload) {
                    if (!deserializeBinary(payload, finalized.result,
                                           &deserialize_ns)) {
                        finalized.exit_failed = true;
                    }
                }
                finalized.has_result =
                    !finalized.cancelled && !finalized.exit_failed &&
                    header.has_payload;
                clearActiveSlot(slot);
                return finalized;
            };

        auto finalizeForcedTermination =
            [&](RepairWorkerSlot &slot,
                std::uint64_t wait_ns) -> FinalizedAttempt {
                const auto finalize_start = Clock::now();
                FinalizedAttempt finalized;
                finalized.slot_index = slot.slot_index;
                finalized.task_index = slot.task_index;
                finalized.attempt_index = slot.attempt_index;
                finalized.cancelled = true;
                clearActiveSlot(slot);
                return finalized;
            };

        auto launchAttempt = [&](RepairWorkerSlot &slot,
                                 std::size_t task_index) -> bool {
            if (slot.active)
                return false;
            auto &task_state = task_states[task_index];
            const std::size_t attempt_index = task_state.attempts_launched;
            if (slot.pid <= 0 && !spawnRepairWorker(slot))
                return false;

            slot.task_index = task_index;
            slot.attempt_index = attempt_index;
            slot.active = true;
            slot.cancel_requested = false;
            slot.cancel_signal_time.reset();
            const std::uint32_t worker_seed =
                workerPlanningSeed(planning_seed_, batch_index,
                                   static_cast<int>(task_index),
                                   slot.slot_index,
                                   static_cast<int>(attempt_index));
            RepairWorkerCommand command;
            command.slot_index = slot.slot_index;
            command.task_index = static_cast<int>(task_index);
            command.attempt_index = static_cast<int>(attempt_index);
            command.planning_seed = worker_seed;
            const bool write_ok =
                writeFramedBinary(slot.command_pipe[1], command);
            if (!write_ok) {
                slot.active = false;
                return false;
            }
            ++task_state.live_attempts;
            ++task_state.attempts_launched;
            ++active_attempts;
            return true;
        };

        auto assignSlotToNextTask = [&](RepairWorkerSlot &slot) {
            const auto task_index = nextAssignableTask();
            if (!task_index)
                return true;
            return launchAttempt(slot, *task_index);
        };

        auto signalCancelActiveAttempts =
            [&](std::optional<std::size_t> task_index,
                std::optional<int> keep_slot_index) {
                std::vector<std::size_t> cancelled_slots;
                for (std::size_t i = 0; i < slots.size(); ++i) {
                    auto &slot = slots[i];
                    if (!slot.active)
                        continue;
                    if (keep_slot_index && slot.slot_index == *keep_slot_index)
                        continue;
                    if (task_index && slot.task_index != *task_index)
                        continue;
                    if (!slot.cancel_requested) {
                        slot.cancel_requested = true;
                        slot.cancel_signal_time = Clock::now();
                        if (slot.pid > 0)
                            ::kill(slot.pid, SIGUSR1);
                    }
                    cancelled_slots.push_back(i);
                }
                return cancelled_slots;
            };

        std::function<void(FinalizedAttempt)> finishCancelledAttempt;

        auto forceExpiredCancelledSlots = [&]() {
            const auto now = Clock::now();
            for (std::size_t i = 0; i < slots.size(); ++i) {
                auto &slot = slots[i];
                if (!slot.active || !slot.cancel_requested ||
                    !slot.cancel_signal_time) {
                    continue;
                }
                if (now - *slot.cancel_signal_time < kCancelGrace)
                    continue;
                if (slot.pid > 0)
                    ::kill(slot.pid, SIGTERM);
                const auto wait_start = Clock::now();
                int status = 0;
                while (slot.pid > 0 && ::waitpid(slot.pid, &status, 0) < 0) {
                    if (errno == EINTR)
                        continue;
                    break;
                }
                const std::uint64_t wait_ns = elapsedNanoseconds(wait_start);
                slot.pid = -1;
                auto attempt = finalizeForcedTermination(slot, wait_ns);
                finishCancelledAttempt(std::move(attempt));
                closeFd(slot.command_pipe[0]);
                closeFd(slot.command_pipe[1]);
                closeFd(slot.result_pipe[0]);
                closeFd(slot.result_pipe[1]);
                if (unresolved_tasks > 0) {
                    if (!spawnRepairWorker(slot)) {
                        batch_failed = true;
                        launch_failed = true;
                    } else if (!assignSlotToNextTask(slot)) {
                        batch_failed = true;
                        launch_failed = true;
                    }
                }
            }
        };

        auto mergeAttemptAccounting = [&](FinalizedAttempt &attempt,
                                          bool) {
            if (!attempt.has_result)
                return;
            planner_stats_summary.subproblem_attempts +=
                attempt.result.repair_stats.subproblem_attempts;
            planner_stats_summary.temporal_expansions +=
                attempt.result.repair_stats.temporal_expansions;
            planner_stats_summary.conflict_resolution_times_seconds_total +=
                attempt.result.repair_stats
                    .conflict_resolution_times_seconds_total;
            planner_stats_summary.conflict_resolution_times_seconds_cpu +=
                attempt.result.repair_stats
                    .conflict_resolution_times_seconds_cpu;
        };

        finishCancelledAttempt = [&](FinalizedAttempt attempt) {
            (void)attempt;
        };

        auto drainCancelledSlots = [&](const std::vector<std::size_t> &slot_ids) {
            for (const std::size_t slot_id : slot_ids) {
                if (slot_id >= slots.size())
                    continue;
                auto &slot = slots[slot_id];
                while (!batch_failed && slot.active &&
                       slot.cancel_requested) {
                    const auto wait_start = Clock::now();
                    pollfd pfd {};
                    pfd.fd = slot.result_pipe[0];
                    pfd.events = POLLIN | POLLHUP | POLLERR;
                    int poll_rc = -1;
                    do {
                        poll_rc = ::poll(&pfd, 1, 10);
                    } while (poll_rc < 0 && errno == EINTR);
                    const std::uint64_t wait_ns =
                        elapsedNanoseconds(wait_start);
                    if (poll_rc < 0) {
                        batch_failed = true;
                        break;
                    }
                    if (poll_rc > 0 && pfd.revents != 0) {
                        auto cancelled_attempt =
                            finalizeWorkerResult(slot, wait_ns);
                        finishCancelledAttempt(
                            std::move(cancelled_attempt));
                        if (!assignSlotToNextTask(slot)) {
                            launch_failed = true;
                            batch_failed = true;
                        }
                        break;
                    }
                    forceExpiredCancelledSlots();
                    if (!slot.active || !slot.cancel_requested)
                        break;
                }
            }
        };

        auto finishFailedAttempt = [&](FinalizedAttempt attempt) {
            mergeAttemptAccounting(attempt, false);
        };

        auto applySavedAttempt = [&](FinalizedAttempt attempt) {
            mergeAttemptAccounting(attempt, true);
            const auto apply_status =
                applyResult(attempt.result, patched_robots);
            if (apply_status == ApplyResult::Error) {
                attempt.exit_failed = true;
                batch_failed = true;
            }
            if (apply_status == ApplyResult::Applied) {
                const auto &task =
                    task_states[attempt.task_index].task;
                round_stats.entries.push_back(ConflictRoundEntry{
                    task.conflict.seed_robot_i,
                    task.conflict.seed_robot_j,
                    task.conflict.conflict_timestep,
                    task.conflict.robots,
                    attempt.result.final_involved_robots,
                    task.conflict.window_begin_t,
                    attempt.result.window_end_t,
                });
                for (const int robot : attempt.result.final_involved_robots) {
                    patched_window_starts[static_cast<std::size_t>(robot)] =
                        attempt.result.window_start_t;
                }
            }
        };

        for (auto &slot : slots) {
            if (!spawnRepairWorker(slot)) {
                launch_failed = true;
                break;
            }
        }
        if (launch_failed) {
            shutdownWorkers(true);
            throw std::runtime_error(
                "ParallelARC failed to launch repair worker pool");
        }

        for (auto &slot : slots) {
            const auto task_index = nextAssignableTask();
            if (!task_index)
                break;
            if (!launchAttempt(slot, *task_index)) {
                launch_failed = true;
                break;
            }
        }

        if (launch_failed || active_attempts == 0) {
            shutdownWorkers(true);
            throw std::runtime_error(
                "ParallelARC failed to assign one or more repair worker tasks");
        }

        while (!batch_failed && unresolved_tasks > 0) {
            if (active_attempts == 0) {
                bool assigned = false;
                for (auto &slot : slots) {
                    if (slot.active)
                        continue;
                    const auto task_index = nextAssignableTask();
                    if (!task_index)
                        continue;
                    if (!launchAttempt(slot, *task_index)) {
                        batch_failed = true;
                        launch_failed = true;
                        break;
                    }
                    assigned = true;
                }
                if (batch_failed || !assigned) {
                    batch_failed = true;
                    break;
                }
            }
            const auto wait_start = Clock::now();
            std::vector<pollfd> poll_fds;
            std::vector<std::size_t> poll_slots;
            for (std::size_t i = 0; i < slots.size(); ++i) {
                if (!slots[i].active || slots[i].result_pipe[0] < 0)
                    continue;
                pollfd pfd {};
                pfd.fd = slots[i].result_pipe[0];
                pfd.events = POLLIN | POLLHUP | POLLERR;
                poll_fds.push_back(pfd);
                poll_slots.push_back(i);
            }
            if (poll_fds.empty()) {
                batch_failed = true;
                break;
            }
            int poll_rc = -1;
            do {
                poll_rc = ::poll(poll_fds.data(),
                                 static_cast<nfds_t>(poll_fds.size()), 50);
            } while (poll_rc < 0 && errno == EINTR);
            const std::uint64_t wait_ns = elapsedNanoseconds(wait_start);
            if (poll_rc < 0) {
                batch_failed = true;
                break;
            }

            if (poll_rc == 0) {
                forceExpiredCancelledSlots();
                continue;
            }

            std::vector<std::size_t> ready_slots;
            for (std::size_t i = 0; i < poll_fds.size(); ++i) {
                if (poll_fds[i].revents != 0)
                    ready_slots.push_back(poll_slots[i]);
            }

            std::set<std::size_t> handled_ready_slots;
            for (const std::size_t slot_index : ready_slots) {
                if (handled_ready_slots.count(slot_index) != 0)
                    continue;
                handled_ready_slots.insert(slot_index);
                if (batch_failed || !slots[slot_index].active)
                    continue;
                auto attempt =
                    finalizeWorkerResult(slots[slot_index], wait_ns);
                if (attempt.cancelled) {
                    finishCancelledAttempt(std::move(attempt));
                    if (!assignSlotToNextTask(slots[slot_index])) {
                        launch_failed = true;
                        batch_failed = true;
                    }
                    continue;
                }

                auto &runtime_task = task_states[attempt.task_index];
                if (!attempt.has_result || !attempt.result.success) {
                    runtime_task.terminal_failure_seen = true;
                    if (runtime_task.live_attempts == 0) {
                        runtime_task.failed = true;
                        batch_failed = true;
                    }
                    if (!batch_failed) {
                        if (!assignSlotToNextTask(slots[slot_index])) {
                            launch_failed = true;
                            batch_failed = true;
                        }
                    }
                    finishFailedAttempt(std::move(attempt));
                    continue;
                }

                runtime_task.solved = true;
                if (unresolved_tasks > 0)
                    --unresolved_tasks;

                const auto cancelled_slots =
                    signalCancelActiveAttempts(attempt.task_index,
                                               slots[slot_index].slot_index);
                if (!assignSlotToNextTask(slots[slot_index])) {
                    launch_failed = true;
                    batch_failed = true;
                }
                for (const auto cancelled_slot : cancelled_slots)
                    handled_ready_slots.insert(cancelled_slot);
                drainCancelledSlots(cancelled_slots);
                applySavedAttempt(std::move(attempt));
            }
            forceExpiredCancelledSlots();
        }

        if (!batch_failed) {
            signalCancelActiveAttempts(std::nullopt, std::nullopt);
            while (active_attempts > 0 && !batch_failed) {
                const auto wait_start = Clock::now();
                std::vector<pollfd> poll_fds;
                std::vector<std::size_t> poll_slots;
                for (std::size_t i = 0; i < slots.size(); ++i) {
                    if (!slots[i].active || slots[i].result_pipe[0] < 0)
                        continue;
                    pollfd pfd {};
                    pfd.fd = slots[i].result_pipe[0];
                    pfd.events = POLLIN | POLLHUP | POLLERR;
                    poll_fds.push_back(pfd);
                    poll_slots.push_back(i);
                }
                if (poll_fds.empty())
                    break;
                int poll_rc = -1;
                do {
                    poll_rc = ::poll(poll_fds.data(),
                                     static_cast<nfds_t>(poll_fds.size()), 50);
                } while (poll_rc < 0 && errno == EINTR);
                const std::uint64_t wait_ns = elapsedNanoseconds(wait_start);
                if (poll_rc < 0) {
                    batch_failed = true;
                    break;
                }
                if (poll_rc > 0) {
                    for (std::size_t i = 0; i < poll_fds.size(); ++i) {
                        if (poll_fds[i].revents == 0)
                            continue;
                        auto attempt = finalizeWorkerResult(
                            slots[poll_slots[i]], wait_ns);
                        finishCancelledAttempt(std::move(attempt));
                    }
                }
                forceExpiredCancelledSlots();
            }
        }

        shutdownWorkers(batch_failed);

        if (batch_failed) {
            planner_stats_summary.conflict_resolution_times_seconds_wall_clock +=
                std::chrono::duration<double>(Clock::now() -
                                              batch_resolution_start)
                    .count();
            finalizePlannerStats();
            return ompl::base::PlannerStatus::TIMEOUT;
        }
        planner_stats_summary.conflict_resolution_times_seconds_wall_clock +=
            std::chrono::duration<double>(Clock::now() -
                                          batch_resolution_start)
                .count();
        applyConflictScanProgress(next_t_begin_by_pair);
        for (std::size_t robot = 0; robot < patched_window_starts.size();
             ++robot) {
            if (patched_window_starts[robot] !=
                std::numeric_limits<int>::max()) {
                resetConflictScanStartsForRobots(
                    {static_cast<int>(robot)}, patched_window_starts[robot]);
            }
        }
        appendConflictRoundStats(std::move(round_stats));
#endif

        ++batch_index;
    }

    finalizePlannerStats();
    return ompl::base::PlannerStatus::TIMEOUT;
}

} // namespace comotion
