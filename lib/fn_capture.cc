/* -*- c++ -*- */
/*
 * FN capture-path knobs and real-time helper; see fn_capture.h.
 */

#include "fn_capture.h"

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>

namespace osmosdr_fn {

long parse_env_long(const char *s, long def, long lo, long hi)
{
  if (s == NULL || *s == '\0')
    return def;
  char *end = NULL;
  errno = 0;
  long v = std::strtol(s, &end, 10);
  if (end == s || *end != '\0' || errno == ERANGE)
    return def;
  if (v < lo)
    v = lo;
  if (v > hi)
    v = hi;
  return v;
}

const capture_config &config()
{
  /* Function-local static: initialised once, thread-safe (C++11). */
  static const capture_config c = [] {
    capture_config r;
    r.rt_prio = (int)parse_env_long(std::getenv("FN_SDR_RT_PRIO"), 10, 0, 99);
    r.gapfill = parse_env_long(std::getenv("FN_SDR_GAPFILL"), 1, 0, 1) != 0;
    r.gapfill_max_ms = (double)parse_env_long(std::getenv("FN_SDR_GAPFILL_MAX_MS"), 250, 1, 2000);
    r.airspy_fifo = (size_t)parse_env_long(std::getenv("FN_AIRSPY_FIFO_SAMPLES"),
                                           5000000, 262144, 100000000);
    long ring = parse_env_long(std::getenv("FN_RTL_RING_BUFFERS"), 0, 0, 1024);
    r.rtl_ring = (ring > 0 && ring < 4) ? 4u : (unsigned)ring;
    return r;
  }();
  return c;
}

static int set_fifo(int prio)
{
  struct sched_param sp;
  std::memset(&sp, 0, sizeof(sp));
  sp.sched_priority = prio;
#if defined(__linux__) && defined(SCHED_RESET_ON_FORK)
  /* pid 0 = the calling thread; nothing it creates inherits real-time. */
  if (sched_setscheduler(0, SCHED_FIFO | SCHED_RESET_ON_FORK, &sp) == 0)
    return 0;
  return errno;
#else
  return pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
#endif
}

int rt_apply(int requested, int *prio, char *msg, size_t msglen)
{
  *prio = 0;

  if (requested <= 0) {
    std::snprintf(msg, msglen, "disabled (FN_SDR_RT_PRIO=0), running at normal priority");
    return RT_OFF;
  }

  const int lo = sched_get_priority_min(SCHED_FIFO);
  const int hi = sched_get_priority_max(SCHED_FIFO);
  if (lo > 0 && requested < lo)
    requested = lo;
  if (hi > 0 && requested > hi)
    requested = hi;

  const int err = set_fifo(requested);
  if (err == 0) {
    *prio = requested;
    std::snprintf(msg, msglen, "SCHED_FIFO prio %d", requested);
    return RT_FIFO;
  }

  struct rlimit rl;
  bool have_lim = false;
  unsigned long lim = 0;
  if (getrlimit(RLIMIT_RTPRIO, &rl) == 0) {
    have_lim = true;
    lim = (rl.rlim_cur == RLIM_INFINITY) ? 99UL : (unsigned long)rl.rlim_cur;
  }

  if (err == EPERM) {
    if (have_lim && lim > 0 && lim < (unsigned long)requested && set_fifo((int)lim) == 0) {
      *prio = (int)lim;
      std::snprintf(msg, msglen, "SCHED_FIFO prio %d (RLIMIT_RTPRIO=%lu caps the requested %d)",
                    (int)lim, lim, requested);
      return RT_FIFO;
    }
    if (have_lim && lim == 0)
      std::snprintf(msg, msglen, "not permitted (RLIMIT_RTPRIO=0), running at normal priority");
    else if (have_lim)
      std::snprintf(msg, msglen,
                    "not permitted (RLIMIT_RTPRIO=%lu; real-time cgroup budget?), running at normal priority",
                    lim);
    else
      std::snprintf(msg, msglen, "not permitted, running at normal priority");
    return RT_DENIED;
  }

  std::snprintf(msg, msglen, "SCHED_FIFO prio %d failed (%s), running at normal priority",
                requested, std::strerror(err));
  return RT_FAILED;
}

} // namespace osmosdr_fn
