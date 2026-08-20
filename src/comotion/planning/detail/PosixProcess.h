#pragma once

#if !defined(_WIN32)
#include <signal.h>
#include <sys/types.h>
#include <unistd.h>
#if defined(__linux__)
#include <sys/prctl.h>
#endif
#endif

namespace comotion::detail {

#if !defined(_WIN32)

// Ensure a nested worker cannot outlive the process that forked it. The
// parent identity check closes the race where the parent exits between fork()
// and prctl(). Other POSIX platforms retain the existing process-group
// cleanup behavior; Linux is the platform used by the benchmark campaign.
inline bool armParentDeathSignal(pid_t expected_parent) noexcept {
#if defined(__linux__)
    if (::prctl(PR_SET_PDEATHSIG, SIGKILL) != 0)
        return false;
    return ::getppid() == expected_parent;
#else
    (void)expected_parent;
    return true;
#endif
}

#endif

} // namespace comotion::detail
