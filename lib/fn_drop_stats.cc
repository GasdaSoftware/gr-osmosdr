/* -*- c++ -*- */
/*
 * Per-source sample-loss accounting; see fn_drop_stats.h.
 */

#include "fn_drop_stats.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <vector>

namespace osmosdr_fn {

namespace {

struct registry {
  std::mutex lock;
  std::vector<drop_counter *> sources;
};

/* Never destroyed, so a source torn down during static destruction (or a
 * reader on another thread at exit) still finds a valid registry. */
registry &reg()
{
  static registry *r = new registry;
  return *r;
}

int64_t now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::steady_clock::now().time_since_epoch()).count();
}

const int64_t REPORT_INTERVAL_NS = 60LL * 1000000000LL;
const int64_t MIN_STALL_NS = 50LL * 1000000LL;

void atomic_max(std::atomic<uint64_t> &a, uint64_t v)
{
  uint64_t cur = a.load(std::memory_order_relaxed);
  while (v > cur && !a.compare_exchange_weak(cur, v, std::memory_order_relaxed))
    ;
}

} // namespace

drop_counter::drop_counter(const std::string &device_args)
  : _rep_t0_ns(0), _rep_lib(0), _rep_fifo(0), _rep_delivered(0), _rep_stalls(0),
    _rep_filled(0), _rep_unfilled(0)
{
  std::snprintf(_device, sizeof(_device), "%s", device_args.c_str());

  registry &r = reg();
  std::lock_guard<std::mutex> g(r.lock);
  r.sources.push_back(this);
}

drop_counter::~drop_counter()
{
  registry &r = reg();
  std::lock_guard<std::mutex> g(r.lock);
  r.sources.erase(std::remove(r.sources.begin(), r.sources.end(), this),
                  r.sources.end());
}

void drop_counter::buffer_arrived(uint64_t nsamples)
{
  _buffers.fetch_add(1, std::memory_order_relaxed);

  const int64_t now = now_ns();
  const int64_t prev = _last_cb_ns.exchange(now, std::memory_order_relaxed);
  if (prev == 0 || now <= prev)
    return;

  const uint64_t gap = (uint64_t)(now - prev);
  atomic_max(_max_gap_ns, gap);
  atomic_max(_win_max_gap_ns, gap);

  const double rate = _rate.load(std::memory_order_relaxed);
  if (rate <= 0.0)
    return;

  int64_t threshold = (int64_t)(4.0e9 * (double)nsamples / rate);
  if (threshold < MIN_STALL_NS)
    threshold = MIN_STALL_NS;

  if ((int64_t)gap > threshold) {
    _stalls.fetch_add(1, std::memory_order_relaxed);
    _stall_ns.fetch_add(gap, std::memory_order_relaxed);
  }
}

void drop_counter::maybe_report()
{
  const int64_t now = now_ns();

  if (_rep_t0_ns == 0) {
    _rep_t0_ns = now;
    _rep_lib = _lib.load(std::memory_order_relaxed);
    _rep_fifo = _fifo.load(std::memory_order_relaxed);
    _rep_delivered = _delivered.load(std::memory_order_relaxed);
    _rep_stalls = _stalls.load(std::memory_order_relaxed);
    _rep_filled = _filled.load(std::memory_order_relaxed);
    _rep_unfilled = _unfilled.load(std::memory_order_relaxed);
    return;
  }

  if (now - _rep_t0_ns < REPORT_INTERVAL_NS)
    return;

  const uint64_t lib = _lib.load(std::memory_order_relaxed);
  const uint64_t fifo = _fifo.load(std::memory_order_relaxed);
  const uint64_t delivered = _delivered.load(std::memory_order_relaxed);
  const uint64_t stalls = _stalls.load(std::memory_order_relaxed);
  const uint64_t win_max_gap = _win_max_gap_ns.exchange(0, std::memory_order_relaxed);
  const uint64_t filled = _filled.load(std::memory_order_relaxed);
  const uint64_t unfilled = _unfilled.load(std::memory_order_relaxed);

  const uint64_t d_lib = lib - _rep_lib;
  const uint64_t d_fifo = fifo - _rep_fifo;
  const uint64_t d_total = d_lib + d_fifo + (delivered - _rep_delivered);
  const uint64_t d_stalls = stalls - _rep_stalls;

  if (d_lib || d_fifo || d_stalls) {
    char stall_part[96] = "";
    if (d_stalls)
      std::snprintf(stall_part, sizeof(stall_part),
                    "; callback stalls=%llu max_gap=%.0fms",
                    (unsigned long long)d_stalls, win_max_gap / 1e6);
    /* One fprintf so the line is not interleaved with "O" markers. */
    char fill_part[96] = "";
    if (_gapfill.load(std::memory_order_relaxed) && (d_lib || d_fifo))
      std::snprintf(fill_part, sizeof(fill_part), "; zero-filled=%llu unfilled=%llu",
                    (unsigned long long)(filled - _rep_filled),
                    (unsigned long long)(unfilled - _rep_unfilled));
    std::fprintf(stderr,
                 "\ngr-osmosdr: %s dropped lib=%llu fifo=%llu of %llu samples in last %.0fs%s%s\n",
                 _device, (unsigned long long)d_lib, (unsigned long long)d_fifo,
                 (unsigned long long)d_total, (now - _rep_t0_ns) / 1e9, stall_part, fill_part);
  }

  _rep_t0_ns = now;
  _rep_lib = lib;
  _rep_fifo = fifo;
  _rep_delivered = delivered;
  _rep_stalls = stalls;
  _rep_filled = filled;
  _rep_unfilled = unfilled;
}

void drop_counter::fill(osmosdr_fn_drop_stat &s) const
{
  std::memcpy(s.device, _device, sizeof(s.device));
  s.lib_dropped = _lib.load(std::memory_order_relaxed);
  s.fifo_dropped = _fifo.load(std::memory_order_relaxed);
  s.delivered = _delivered.load(std::memory_order_relaxed);
}

void drop_counter::fill(osmosdr_fn_stall_stat &s) const
{
  std::memcpy(s.device, _device, sizeof(s.device));
  s.buffers = _buffers.load(std::memory_order_relaxed);
  s.short_buffers = _short.load(std::memory_order_relaxed);
  s.stalls = _stalls.load(std::memory_order_relaxed);
  s.stall_ns = _stall_ns.load(std::memory_order_relaxed);
  s.max_gap_ns = _max_gap_ns.load(std::memory_order_relaxed);
  s.sample_rate = _rate.load(std::memory_order_relaxed);
}

void drop_counter::fill(osmosdr_fn_capture_stat &s) const
{
  std::memset(&s, 0, sizeof(s));
  std::memcpy(s.device, _device, sizeof(s.device));
  s.size = sizeof(s);
  s.rt_status = _rt_status.load(std::memory_order_relaxed);
  s.rt_prio = _rt_prio.load(std::memory_order_relaxed);
  s.gapfill = _gapfill.load(std::memory_order_relaxed);
  s.filled = _filled.load(std::memory_order_relaxed);
  s.unfilled = _unfilled.load(std::memory_order_relaxed);
  s.owed = _owed.load(std::memory_order_relaxed);
  s.gap_cap = _gap_cap.load(std::memory_order_relaxed);
  s.queue_buffers = _queue_buffers.load(std::memory_order_relaxed);
  s.queue_peak = _queue_peak.load(std::memory_order_relaxed);
  s.sample_rate = _gap_rate.load(std::memory_order_relaxed);
  s.clip_samples = _clip.load(std::memory_order_relaxed);
  s.clip_examined = _clip_examined.load(std::memory_order_relaxed);
}

template <typename T>
static int fill_all(T *out, int max)
{
  try {
    registry &r = reg();
    std::lock_guard<std::mutex> g(r.lock);
    const int n = (int)r.sources.size();
    if (out)
      for (int i = 0; i < n && i < max; i++)
        r.sources[i]->fill(out[i]);
    return n;
  } catch (...) {
    return 0;
  }
}

} // namespace osmosdr_fn

extern "C" int osmosdr_fn_drop_stats(struct osmosdr_fn_drop_stat *out, int max)
{
  return osmosdr_fn::fill_all(out, max);
}

extern "C" int osmosdr_fn_stall_stats(struct osmosdr_fn_stall_stat *out, int max)
{
  return osmosdr_fn::fill_all(out, max);
}

extern "C" int osmosdr_fn_capture_stats(struct osmosdr_fn_capture_stat *out, int max, size_t stat_size)
{
  try {
    if (out && (stat_size < sizeof(uint32_t) + sizeof(out->device)))
      return 0;
    osmosdr_fn::registry &r = osmosdr_fn::reg();
    std::lock_guard<std::mutex> g(r.lock);
    const int n = (int)r.sources.size();
    if (out) {
      const size_t copy = stat_size < sizeof(osmosdr_fn_capture_stat) ? stat_size : sizeof(osmosdr_fn_capture_stat);
      char *base = reinterpret_cast<char *>(out);
      for (int i = 0; i < n && i < max; i++) {
        osmosdr_fn_capture_stat s;
        r.sources[i]->fill(s);
        s.size = (uint32_t)copy;
        std::memcpy(base + (size_t)i * stat_size, &s, copy);
      }
    }
    return n;
  } catch (...) {
    return 0;
  }
}
