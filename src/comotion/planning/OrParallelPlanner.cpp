#include "comotion/planning/OrParallelPlanner.h"

#include "comotion/planning/detail/PosixProcess.h"
#include "comotion/planning/PlanningRng.h"
#include "comotion/planning/PlanningSeed.h"

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/vector.hpp>

#include <nlohmann/json.hpp>

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#if !defined(_WIN32)
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace {

using Clock = std::chrono::steady_clock;

template <typename T>
bool serializeBinary(const T &value, std::string &payload) {
    try {
        std::ostringstream stream(std::ios::binary);
        boost::archive::binary_oarchive archive(stream);
        archive << value;
        payload = stream.str();
    } catch (...) {
        return false;
    }
    return true;
}

template <typename T>
bool deserializeBinary(const std::string &payload, T &value) {
    try {
        std::istringstream stream(payload, std::ios::binary);
        boost::archive::binary_iarchive archive(stream);
        archive >> value;
    } catch (...) {
        return false;
    }
    return true;
}

bool writeBytesToFile(const std::string &path, const std::string &payload) {
    std::ofstream stream(path, std::ios::binary | std::ios::trunc);
    if (!stream.good())
        return false;
    stream.write(payload.data(),
                 static_cast<std::streamsize>(payload.size()));
    stream.flush();
    return stream.good();
}

bool readBytesFromFile(const std::string &path, std::string &payload) {
    std::ifstream stream(path, std::ios::binary | std::ios::ate);
    if (!stream.good())
        return false;
    const auto end_pos = stream.tellg();
    if (end_pos < 0)
        return false;
    payload.resize(static_cast<std::size_t>(end_pos));
    stream.seekg(0, std::ios::beg);
    if (!payload.empty()) {
        stream.read(payload.data(),
                    static_cast<std::streamsize>(payload.size()));
    }
    return stream.good() || stream.eof();
}

struct WorkerResult {
    int worker_index = -1;
    std::uint32_t worker_seed = 0;
    int status_type =
        static_cast<int>(ompl::base::PlannerStatus::UNKNOWN);
    double elapsed_seconds = 0.0;
    std::vector<comotion::Path> solution_paths;
    std::string planner_stats_json;
    std::string error_message;

    template <class Archive>
    void serialize(Archive &ar, const unsigned int /*version*/) {
        ar & worker_index;
        ar & worker_seed;
        ar & status_type;
        ar & elapsed_seconds;
        ar & solution_paths;
        ar & planner_stats_json;
        ar & error_message;
    }
};

nlohmann::json parsePlannerStatsJson(const std::string &payload) {
    if (payload.empty())
        return nlohmann::json::object();
    try {
        const auto parsed = nlohmann::json::parse(payload);
        return parsed.is_object() ? parsed : nlohmann::json::object();
    } catch (...) {
        return nlohmann::json::object();
    }
}

nlohmann::json withOrParallelStats(nlohmann::json stats,
                                   const std::string &base_planner_name,
                                   unsigned worker_processes,
                                   int winner_index,
                                   std::uint32_t winner_seed,
                                   double winner_elapsed_seconds,
                                   bool killed_slower_workers) {
    if (!stats.is_object())
        stats = nlohmann::json::object();
    stats["or_parallel"] = {
        {"base_planner", base_planner_name},
        {"worker_processes", worker_processes},
        {"winner_index", winner_index},
        {"winner_seed", winner_seed},
        {"winner_elapsed_seconds", winner_elapsed_seconds},
        {"killed_slower_workers", killed_slower_workers},
    };
    return stats;
}

#if !defined(_WIN32)

void cleanupResultFile(std::string &path) {
    if (!path.empty()) {
        ::unlink(path.c_str());
        path.clear();
    }
}

#endif

} // namespace

namespace comotion {

std::string OrParallelPlanner::name() const {
    return "OrParallel(" + base_planner_name_ + ")";
}

ompl::base::PlannerStatus OrParallelPlanner::solve(double timeLimit) {
    resetPlannerRunMetrics();
    solution_paths_.clear();

    if (!problem_) {
        throw std::runtime_error(
            "OrParallelPlanner requires setProblem() before solve()");
    }
    if (!planner_factory_) {
        throw std::runtime_error(
            "OrParallelPlanner requires setPlannerFactory() before solve()");
    }

    if (worker_processes_ <= 1) {
        auto planner = planner_factory_();
        if (!planner) {
            throw std::runtime_error(
                "OrParallelPlanner factory returned null planner");
        }
        planner->setProblem(problem_);
        planner->setPlanningSeed(planning_seed_);
        const auto start = Clock::now();
        const auto status = planner->solve(timeLimit);
        const double elapsed_seconds =
            std::chrono::duration<double>(Clock::now() - start).count();
        setPlannerStatsJson(withOrParallelStats(
            planner->plannerStatsJson(), base_planner_name_, worker_processes_,
            0, planning_seed_, elapsed_seconds, false));
        if (status == ompl::base::PlannerStatus::EXACT_SOLUTION ||
            status == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION) {
            solution_paths_ = planner->getSolutionPaths();
            setSolutionMetricsFromPaths(solution_paths_);
        }
        return status;
    }

#if defined(_WIN32)
    throw std::runtime_error(
        "OrParallelPlanner with worker_processes > 1 requires POSIX fork support");
#else
    struct WorkerState {
        int worker_index = -1;
        std::uint32_t worker_seed = 0;
        pid_t pid = -1;
        std::string result_path;
        bool reaped = false;
    };

    std::vector<WorkerState> states;
    states.reserve(worker_processes_);
    std::unordered_map<pid_t, std::size_t> state_index_by_pid;
    bool launch_failed = false;

    for (unsigned int i = 0; i < worker_processes_; ++i) {
        char result_template[] = "/tmp/comotion-or-parallel-XXXXXX";
        const int tmp_fd = ::mkstemp(result_template);
        if (tmp_fd < 0) {
            launch_failed = true;
            break;
        }
        ::close(tmp_fd);

        const std::uint32_t worker_seed =
            orParallelWorkerPlanningSeed(planning_seed_, static_cast<int>(i));
        const pid_t parent_pid = ::getpid();
        const pid_t pid = ::fork();
        if (pid < 0) {
            ::unlink(result_template);
            launch_failed = true;
            break;
        }

        if (pid == 0) {
            if (!comotion::detail::armParentDeathSignal(parent_pid))
                _exit(3);

            WorkerResult result;
            result.worker_index = static_cast<int>(i);
            result.worker_seed = worker_seed;

            try {
                seedOmplGlobalFromUserPlanningSeed(worker_seed);

                auto planner = planner_factory_();
                if (!planner) {
                    throw std::runtime_error(
                        "OrParallelPlanner factory returned null planner");
                }
                planner->setProblem(problem_);
                planner->setPlanningSeed(worker_seed);

                const auto start = Clock::now();
                const auto status = planner->solve(timeLimit);
                result.elapsed_seconds =
                    std::chrono::duration<double>(Clock::now() - start).count();
                result.status_type =
                    static_cast<int>(
                        static_cast<ompl::base::PlannerStatus::StatusType>(
                            status));
                if (status == ompl::base::PlannerStatus::EXACT_SOLUTION ||
                    status == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION) {
                    result.solution_paths = planner->getSolutionPaths();
                }
                result.planner_stats_json = planner->plannerStatsJson().dump();
            } catch (const std::exception &ex) {
                result.status_type =
                    static_cast<int>(ompl::base::PlannerStatus::ABORT);
                result.error_message = ex.what();
            } catch (...) {
                result.status_type =
                    static_cast<int>(ompl::base::PlannerStatus::ABORT);
                result.error_message = "unknown worker exception";
            }

            std::string payload;
            const bool write_ok =
                serializeBinary(result, payload) &&
                writeBytesToFile(result_template, payload);
            _exit(write_ok ? 0 : 2);
        }

        WorkerState state;
        state.worker_index = static_cast<int>(i);
        state.worker_seed = worker_seed;
        state.pid = pid;
        state.result_path = result_template;
        state_index_by_pid.emplace(pid, states.size());
        states.push_back(std::move(state));
    }

    if (launch_failed || states.size() != worker_processes_) {
        for (auto &state : states) {
            if (state.pid > 0)
                ::kill(state.pid, SIGKILL);
        }
        for (auto &state : states) {
            if (state.pid > 0) {
                int wait_status = 0;
                while (::waitpid(state.pid, &wait_status, 0) < 0) {
                    if (errno != EINTR)
                        break;
                }
            }
            cleanupResultFile(state.result_path);
        }
        throw std::runtime_error(
            "OrParallelPlanner failed to launch one or more worker processes");
    }

    bool exact_winner_found = false;
    bool approximate_fallback_found = false;
    bool killed_slower_workers = false;
    WorkerResult exact_winner;
    WorkerResult approximate_fallback;
    nlohmann::json worker_outcomes = nlohmann::json::array();
    std::size_t remaining = states.size();

    while (remaining > 0) {
        int wait_status = 0;
        const pid_t pid = ::waitpid(-1, &wait_status, 0);
        if (pid < 0) {
            if (errno == EINTR)
                continue;
            break;
        }

        const auto it = state_index_by_pid.find(pid);
        if (it == state_index_by_pid.end())
            continue;

        WorkerState &state = states[it->second];
        state.reaped = true;
        --remaining;

        WorkerResult result;
        bool result_ok = false;
        std::size_t result_payload_bytes = 0;
        if (WIFEXITED(wait_status) && WEXITSTATUS(wait_status) == 0) {
            std::string payload;
            if (readBytesFromFile(state.result_path, payload)) {
                result_payload_bytes = payload.size();
            }
            if (!payload.empty() &&
                deserializeBinary(payload, result)) {
                result_ok = true;
            }
        }
        cleanupResultFile(state.result_path);

        nlohmann::json outcome = {
            {"worker_index", state.worker_index},
            {"worker_seed", state.worker_seed},
            {"result_ok", result_ok},
            {"result_payload_bytes", result_payload_bytes},
            {"exited", WIFEXITED(wait_status)},
            {"exit_code", WIFEXITED(wait_status)
                              ? nlohmann::json(WEXITSTATUS(wait_status))
                              : nlohmann::json(nullptr)},
            {"signaled", WIFSIGNALED(wait_status)},
            {"signal", WIFSIGNALED(wait_status)
                           ? nlohmann::json(WTERMSIG(wait_status))
                           : nlohmann::json(nullptr)},
        };
        if (result_ok) {
            outcome["status_type"] = result.status_type;
            outcome["elapsed_seconds"] = result.elapsed_seconds;
            outcome["error_message"] = result.error_message;
        }
        worker_outcomes.push_back(std::move(outcome));

        if (!result_ok)
            continue;

        const auto status =
            static_cast<ompl::base::PlannerStatus::StatusType>(
                result.status_type);
        if (!approximate_fallback_found &&
            status == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION) {
            approximate_fallback = result;
            approximate_fallback_found = true;
        }

        if (!exact_winner_found &&
            status == ompl::base::PlannerStatus::EXACT_SOLUTION) {
            exact_winner = result;
            exact_winner_found = true;
            for (auto &other : states) {
                if (!other.reaped && other.pid > 0) {
                    ::kill(other.pid, SIGKILL);
                    killed_slower_workers = true;
                }
            }
        }
    }

    for (auto &state : states)
        cleanupResultFile(state.result_path);

    if (exact_winner_found) {
        solution_paths_ = std::move(exact_winner.solution_paths);
        auto stats = withOrParallelStats(
            parsePlannerStatsJson(exact_winner.planner_stats_json),
            base_planner_name_, worker_processes_, exact_winner.worker_index,
            exact_winner.worker_seed, exact_winner.elapsed_seconds,
            killed_slower_workers);
        stats["or_parallel"]["worker_outcomes"] = worker_outcomes;
        setPlannerStatsJson(std::move(stats));
        setSolutionMetricsFromPaths(solution_paths_);
        return ompl::base::PlannerStatus::EXACT_SOLUTION;
    }

    if (return_approximate_if_no_exact_ && approximate_fallback_found) {
        solution_paths_ = std::move(approximate_fallback.solution_paths);
        auto stats = withOrParallelStats(
            parsePlannerStatsJson(approximate_fallback.planner_stats_json),
            base_planner_name_, worker_processes_,
            approximate_fallback.worker_index, approximate_fallback.worker_seed,
            approximate_fallback.elapsed_seconds, false);
        stats["or_parallel"]["worker_outcomes"] = worker_outcomes;
        setPlannerStatsJson(std::move(stats));
        setSolutionMetricsFromPaths(solution_paths_);
        return ompl::base::PlannerStatus::APPROXIMATE_SOLUTION;
    }

    auto stats = withOrParallelStats(nlohmann::json::object(),
                                     base_planner_name_, worker_processes_, -1,
                                     0, 0.0, killed_slower_workers);
    stats["or_parallel"]["worker_outcomes"] = worker_outcomes;
    setPlannerStatsJson(std::move(stats));
    return ompl::base::PlannerStatus::TIMEOUT;
#endif
}

} // namespace comotion
