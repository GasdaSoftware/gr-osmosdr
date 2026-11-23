/* -*- c++ -*- */
/*
 * Per-source sample-loss accounting, exported to the host application.
 *
 * A source keeps one drop_counter as a member.  The driver's receive
 * callback adds to it with relaxed atomics only (no allocation, locking or
 * I/O on that path); work() calls maybe_report(), which prints at most one
 * stderr line per source per minute and only when something was lost.
 *
 * The application reads the counters through two C symbols with default
 * visibility, so it can look them up with dlsym(RTLD_DEFAULT, ...) and run
 * unchanged against a gr-osmosdr that does not have them:
 *
 *   osmosdr_fn_drop_stats()   lib_dropped / fifo_dropped / delivered
 *   osmosdr_fn_stall_stats()  receive-callback timing (stalls, gaps)
 *   osmosdr_fn_capture_stats() gap fill and capture-thread scheduling (FN)
 *
 * All sample counts are complex samples at the source's output rate and
 * cumulative since the source was created.
 */

#ifndef OSMOSDR_FN_DROP_STATS_H
#define OSMOSDR_FN_DROP_STATS_H

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

#include "fn_gap_debt.h"

extern "C" {

struct osmosdr_fn_drop_stat {
  char     device[96];     /* device-args string the source was created with (truncated, NUL-terminated) */
  uint64_t lib_dropped;    /* samples the vendor library reported dropped before the callback (airspy only) */
  uint64_t fifo_dropped;   /* samples discarded because the block's own fifo/ring was full */
  uint64_t delivered;      /* samples handed to the flowgraph */
};

struct osmosdr_fn_stall_stat {
  char     device[96];     /* same string as osmosdr_fn_drop_stat.device */
  uint64_t buffers;        /* receive callbacks (buffers) seen */
  uint64_t short_buffers;  /* buffers shorter than the transfer size (rtl/hackrf); 0 elsewhere */
  uint64_t stalls;         /* callback gaps longer than max(4 x expected buffer period, 50 ms) */
  uint64_t stall_ns;       /* sum of those gaps, whole gap length, ns */
  uint64_t max_gap_ns;     /* longest callback gap since the source was created, ns */
  double   sample_rate;    /* rate the expected buffer period is computed from (0 = unknown) */
};

__attribute__((visibility("default")))
int osmosdr_fn_drop_stats(struct osmosdr_fn_drop_stat *out, int max);

__attribute__((visibility("default")))
int osmosdr_fn_stall_stats(struct osmosdr_fn_stall_stat *out, int max);

/*
 * Gap fill and capture scheduling.  Sources are in the same order as in the
 * two calls above (match on device as well).  Losses the block can count
 * (lib_dropped + fifo_dropped) are either zero-filled at the point of loss,
 * abandoned, or still owed:
 *
 *   lib_dropped + fifo_dropped == filled + unfilled + owed
 *
 * so the samples handed to the flowgraph are delivered + filled.  Losses the
 * device itself suffered (USB thread late) are invisible and never filled.
 *
 * stat_size is sizeof(struct osmosdr_fn_capture_stat) as the caller was
 * compiled; out is an array with that stride and each element receives at
 * most that many bytes, so fields can be appended later.  Returns the number
 * of sources (which may exceed max), or 0 on error.
 */
struct osmosdr_fn_capture_stat {
  char     device[96];     /* same string as osmosdr_fn_drop_stat.device */
  uint32_t size;           /* bytes of this struct the library filled */
  int32_t  rt_status;      /* capture thread: 0 n/a or not reported, 1 SCHED_FIFO, 2 off (FN_SDR_RT_PRIO=0), 3 not permitted, 4 failed */
  int32_t  rt_prio;        /* SCHED_FIFO priority obtained, 0 otherwise */
  int32_t  gapfill;        /* 1 = zero fill on for this source (FN_SDR_GAPFILL) */
  uint64_t filled;         /* zero samples inserted in place of counted losses */
  uint64_t unfilled;       /* counted losses not filled: gap fill off, or debt over the cap / not placeable */
  uint64_t owed;           /* counted losses waiting to be filled at the next buffer */
  uint64_t gap_cap;        /* largest debt still filled, samples */
  uint32_t queue_buffers;  /* libairspy transfer-to-consumer queue length (0 = unknown: not airspy or stock libairspy) */
  uint32_t queue_peak;     /* most buffers waiting in that queue at once since start */
  double   sample_rate;    /* rate gap_cap was computed from */
};

__attribute__((visibility("default")))
int osmosdr_fn_capture_stats(struct osmosdr_fn_capture_stat *out, int max, size_t stat_size);

}

namespace osmosdr_fn {

class drop_counter
{
public:
  explicit drop_counter(const std::string &device_args);
  ~drop_counter();

  drop_counter(const drop_counter &) = delete;
  drop_counter &operator=(const drop_counter &) = delete;

  /* Called by start(): sets the rate used for the stall threshold and
   * forgets the previous callback time, so a stop/start pause is not a stall. */
  void streaming_started(double sample_rate)
  {
    _rate.store(sample_rate, std::memory_order_relaxed);
    _last_cb_ns.store(0, std::memory_order_relaxed);
  }

  /* Receive-callback path: atomics only. */
  void lib_dropped(uint64_t n)  { if (n) _lib.fetch_add(n, std::memory_order_relaxed); }
  void fifo_dropped(uint64_t n) { if (n) _fifo.fetch_add(n, std::memory_order_relaxed); }
  void delivered(uint64_t n)    { if (n) _delivered.fetch_add(n, std::memory_order_relaxed); }
  void short_buffer()           { _short.fetch_add(1, std::memory_order_relaxed); }

  /* Call once per callback with the buffer's size in complex samples. */
  void buffer_arrived(uint64_t nsamples);

  /* work() path: prints one line per minute when something was lost. */
  void maybe_report();

  /* Capture-thread scheduling (any thread). */
  void set_rt(int status, int prio)
  {
    _rt_prio.store(prio, std::memory_order_relaxed);
    _rt_status.store(status, std::memory_order_relaxed);
  }
  /* libairspy queue (work() thread). */
  void set_lib_queue(uint32_t buffers, uint32_t peak)
  {
    _queue_buffers.store(buffers, std::memory_order_relaxed);
    _queue_peak.store(peak, std::memory_order_relaxed);
  }
  /* Publish the owner's gap debt (owner's thread; atomics only). */
  void publish(const gap_debt &d, double rate)
  {
    _gapfill.store(d.enabled() ? 1 : 0, std::memory_order_relaxed);
    _gap_cap.store(d.cap(), std::memory_order_relaxed);
    _gap_rate.store(rate, std::memory_order_relaxed);
    publish(d);
  }
  void publish(const gap_debt &d)
  {
    _filled.store(d.filled(), std::memory_order_relaxed);
    _unfilled.store(d.unfilled(), std::memory_order_relaxed);
    _owed.store(d.owed(), std::memory_order_relaxed);
  }

  void fill(osmosdr_fn_drop_stat &s) const;
  void fill(osmosdr_fn_stall_stat &s) const;
  void fill(osmosdr_fn_capture_stat &s) const;

private:
  char _device[96];

  std::atomic<uint64_t> _lib{0};
  std::atomic<uint64_t> _fifo{0};
  std::atomic<uint64_t> _delivered{0};

  std::atomic<double>   _rate{0.0};
  std::atomic<int64_t>  _last_cb_ns{0};
  std::atomic<uint64_t> _buffers{0};
  std::atomic<uint64_t> _short{0};
  std::atomic<uint64_t> _stalls{0};
  std::atomic<uint64_t> _stall_ns{0};
  std::atomic<uint64_t> _max_gap_ns{0};
  std::atomic<uint64_t> _win_max_gap_ns{0};

  std::atomic<int>      _rt_status{0};
  std::atomic<int>      _rt_prio{0};
  std::atomic<int>      _gapfill{0};
  std::atomic<uint64_t> _filled{0};
  std::atomic<uint64_t> _unfilled{0};
  std::atomic<uint64_t> _owed{0};
  std::atomic<uint64_t> _gap_cap{0};
  std::atomic<double>   _gap_rate{0.0};
  std::atomic<uint32_t> _queue_buffers{0};
  std::atomic<uint32_t> _queue_peak{0};

  /* report state, touched only by maybe_report() (the work() thread) */
  int64_t  _rep_t0_ns;
  uint64_t _rep_lib, _rep_fifo, _rep_delivered, _rep_stalls;
  uint64_t _rep_filled, _rep_unfilled;
};

} // namespace osmosdr_fn

#endif /* OSMOSDR_FN_DROP_STATS_H */
