/* -*- c++ -*- */
/*
 * FN capture-path knobs and the real-time helper for source threads that
 * gr-osmosdr owns (the RTL-SDR read thread).  The Airspy USB thread lives in
 * libairspy, which has its own copy of the same logic (FN libairspy fork).
 *
 * Environment, read once per process; every knob degrades to the upstream
 * behaviour when unset to its "off" value:
 *
 *   FN_SDR_RT_PRIO          SCHED_FIFO priority for capture threads
 *                           (default 10, 0 = do not try, 1..99)
 *   FN_SDR_GAPFILL          1 (default) = insert zeros for counted losses,
 *                           0 = splice the stream as upstream does
 *   FN_SDR_GAPFILL_MAX_MS   largest loss that is still filled (default 250,
 *                           1..2000 ms); larger debts are abandoned
 *   FN_AIRSPY_FIFO_SAMPLES  airspy block fifo, complex samples
 *                           (default 5000000 = upstream; 262144..100000000)
 *   FN_RTL_RING_BUFFERS     RTL ring slots (default 0 = the USB transfer
 *                           count, upstream; 4..1024).  Does not change the
 *                           number of USB transfers.
 */

#ifndef OSMOSDR_FN_CAPTURE_H
#define OSMOSDR_FN_CAPTURE_H

#include <cstddef>

namespace osmosdr_fn {

/* Same numbering as osmosdr_fn_capture_stat.rt_status and libairspy's
 * AIRSPY_FN_RT_*. */
enum rt_status {
  RT_NA = 0,      /* not started / not applicable / not reported */
  RT_FIFO = 1,    /* SCHED_FIFO at rt_prio */
  RT_OFF = 2,     /* FN_SDR_RT_PRIO=0 */
  RT_DENIED = 3,  /* not permitted */
  RT_FAILED = 4,  /* other error */
};

struct capture_config {
  int rt_prio;
  bool gapfill;
  double gapfill_max_ms;
  size_t airspy_fifo;
  unsigned rtl_ring;
};

/* Parsed once, on first use. */
const capture_config &config();

/* Exposed for the unit test: parse s as a decimal integer; def if unset or
 * malformed, otherwise clamped to [lo, hi]. */
long parse_env_long(const char *s, long def, long lo, long hi);

/* Put the calling thread in SCHED_FIFO at requested (<= 0: do not try),
 * falling back to RLIMIT_RTPRIO if lower and non-zero; never throws.
 * Returns rt_status, sets *prio, writes a description (no newline). */
int rt_apply(int requested, int *prio, char *msg, size_t msglen);

} // namespace osmosdr_fn

#endif /* OSMOSDR_FN_CAPTURE_H */
