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
 *
 * All sample counts are complex samples at the source's output rate and
 * cumulative since the source was created.
 */

#ifndef OSMOSDR_FN_DROP_STATS_H
#define OSMOSDR_FN_DROP_STATS_H

#include <atomic>
#include <cstdint>
#include <string>

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

  void fill(osmosdr_fn_drop_stat &s) const;
  void fill(osmosdr_fn_stall_stat &s) const;

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

  /* report state, touched only by maybe_report() (the work() thread) */
  int64_t  _rep_t0_ns;
  uint64_t _rep_lib, _rep_fifo, _rep_delivered, _rep_stalls;
};

} // namespace osmosdr_fn

#endif /* OSMOSDR_FN_DROP_STATS_H */
