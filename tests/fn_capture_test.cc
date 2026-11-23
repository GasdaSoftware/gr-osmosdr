/*
 * Standalone tests for the FN capture additions (no SDR, no GNU Radio):
 * gap-debt logic, a sample-exact stream model of the airspy fifo and RTL ring
 * paths, knob parsing, the osmosdr_fn_capture_stats() export, and the
 * real-time helper.
 *
 *   g++ -std=c++17 -O1 -Wall -Wextra -I lib -o fn_capture_test \
 *       tests/fn_capture_test.cc lib/fn_capture.cc lib/fn_drop_stats.cc -lpthread
 *   ./fn_capture_test                       # RT part: reports whatever it got
 *   prlimit --rtprio=0 ./fn_capture_test denied
 *   prlimit --rtprio=10 ./fn_capture_test fifo:10
 */

#include "fn_capture.h"
#include "fn_drop_stats.h"
#include "fn_gap_debt.h"

#include <pthread.h>
#include <sched.h>

#include <cstdio>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

using osmosdr_fn::gap_debt;

static int fails;
#define CHECK(c) do { if (!(c)) { std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); fails++; } } while (0)

static void check_invariant(const gap_debt &d, uint64_t lost_total)
{
  CHECK(d.filled() + d.unfilled() + d.owed() == lost_total);
}

static void test_debt_basics()
{
  gap_debt d;
  d.configure(true, 1000);
  uint64_t lost = 0;

  /* library drop before a buffer, plenty of room: zeros then all data */
  d.lost(300); lost += 300;
  gap_debt::plan p = d.place(10000, 500);
  CHECK(p.zeros == 300 && p.data == 500);
  CHECK(d.owed() == 0 && d.filled() == 300);
  check_invariant(d, lost);

  /* fifo has room for only part of the buffer: tail becomes debt */
  p = d.place(200, 500); lost += 300;
  CHECK(p.zeros == 0 && p.data == 200);
  CHECK(d.owed() == 300);
  check_invariant(d, lost);

  /* next buffer: debt + buffer fit -> zeros first */
  p = d.place(5000, 500);
  CHECK(p.zeros == 300 && p.data == 500);
  CHECK(d.owed() == 0 && d.filled() == 600);
  check_invariant(d, lost);

  /* debt + buffer do not fit: the buffer joins the debt (exact time) */
  d.lost(400); lost += 400;
  p = d.place(600, 500); lost += 500;
  CHECK(p.zeros == 0 && p.data == 0);
  CHECK(d.owed() == 900);
  check_invariant(d, lost);
  /* backlog cleared: all of it goes in */
  p = d.place(2000, 500);
  CHECK(p.zeros == 900 && p.data == 500);
  CHECK(d.owed() == 0 && d.filled() == 1500);
  check_invariant(d, lost);

  /* over the cap: abandoned at once */
  d.lost(1001); lost += 1001;
  CHECK(d.owed() == 0 && d.unfilled() == 1001);
  check_invariant(d, lost);

  /* accumulating past the cap */
  d.lost(600); lost += 600;
  CHECK(d.owed() == 600);
  d.lost(600); lost += 600;
  CHECK(d.owed() == 0 && d.unfilled() == 2201);
  check_invariant(d, lost);

  /* exactly at the cap is still filled */
  d.lost(1000); lost += 1000;
  p = d.place(1500, 500);
  CHECK(p.zeros == 1000 && p.data == 500);
  check_invariant(d, lost);

  /* full fifo: buffers join the debt until the cap, then upstream */
  p = d.place(0, 500); lost += 500;
  CHECK(p.zeros == 0 && p.data == 0 && d.owed() == 500);
  p = d.place(0, 500); lost += 500;
  CHECK(p.data == 0 && d.owed() == 1000);
  p = d.place(0, 500); lost += 500;
  CHECK(p.data == 0 && d.owed() == 500 && d.unfilled() == 3201);
  check_invariant(d, lost);
  /* debt abandoned at the cap, then a partial fit: upstream min(room, n) */
  d.abandon();
  CHECK(d.unfilled() == 3701);
  d.lost(800); lost += 800;
  p = d.place(400, 500); lost += 100;   /* 800 + 500 > cap: abandon, upstream */
  CHECK(p.zeros == 0 && p.data == 400 && d.owed() == 100);
  CHECK(d.unfilled() == 4501);
  check_invariant(d, lost);
  p = d.place(400, 500); lost += 500;   /* 100 + 500 do not fit: joins the debt */
  CHECK(p.zeros == 0 && p.data == 0 && d.owed() == 600);
  check_invariant(d, lost);
  p = d.place(1100, 500);               /* 600 + 500 fit exactly */
  CHECK(p.zeros == 600 && p.data == 500 && d.owed() == 0);
  check_invariant(d, lost);

  /* restart forgets the debt */
  d.lost(10); lost += 10;
  d.abandon();
  CHECK(d.owed() == 0);
  check_invariant(d, lost);
}

static void test_debt_disabled()
{
  gap_debt d;
  d.configure(false, 1000);
  uint64_t lost = 0;
  d.lost(300); lost += 300;
  CHECK(d.owed() == 0 && d.unfilled() == 300);
  gap_debt::plan p = d.place(10000, 500);
  CHECK(p.zeros == 0 && p.data == 500);
  p = d.place(100, 500); lost += 400;
  CHECK(p.zeros == 0 && p.data == 100);   /* exactly the upstream min(room, n) */
  CHECK(d.filled() == 0 && d.owed() == 0);
  check_invariant(d, lost);

  gap_debt z;  /* cap 0 also means off */
  z.configure(true, 0);
  CHECK(!z.enabled());
}

/*
 * Stream model of the airspy path: the device produces samples numbered
 * 0, 1, 2, ...  Some buffers are dropped by "libairspy" (reported as
 * dropped_samples on the next buffer), and the block's fifo is drained at a
 * varying rate.  With gap fill, every real sample must come out at the
 * position equal to its number (time is continuous) until the first
 * abandoned debt; zeros appear exactly where samples were lost.
 */
static void airspy_stream_model(bool overload)
{
  const uint64_t BUF = 1000, CAP_FIFO = 5000, GAP_CAP = 5000;
  gap_debt d;
  d.configure(true, GAP_CAP);

  std::deque<int64_t> fifo;   /* -1 = zero sample */
  std::vector<int64_t> out;
  uint64_t produced = 0, lib_pending = 0, lost_total = 0;

  /* schedule: buffer k -> dropped by library? ; drain amount after it */
  for (int k = 0; k < 60; k++) {
    const bool lib_drop = (k == 5 || k == 6 || k == 20);
    const uint64_t first = produced;
    produced += BUF;
    if (lib_drop) {
      lib_pending += BUF;
      continue;
    }
    /* callback */
    d.lost(lib_pending);
    lost_total += lib_pending;
    lib_pending = 0;
    const uint64_t room = CAP_FIFO - fifo.size();
    gap_debt::plan p = d.place(room, BUF);
    lost_total += BUF - p.data;
    for (uint64_t i = 0; i < p.zeros; i++) fifo.push_back(-1);
    for (uint64_t i = 0; i < p.data; i++) fifo.push_back((int64_t)(first + i));
    /* work(): stalls for k in [30, 37) so the fifo overflows, then
     * catches up; with overload it never catches up (debt abandoned). */
    uint64_t drain = 1600;
    if (k >= 30 && k < 37)
      drain = 0;
    else if (k >= 37 && k < 45)
      drain = overload ? 900 : 4000;
    while (drain-- && !fifo.empty()) { out.push_back(fifo.front()); fifo.pop_front(); }
    check_invariant(d, lost_total);
  }
  while (!fifo.empty()) { out.push_back(fifo.front()); fifo.pop_front(); }

  size_t misplaced = 0;
  for (size_t i = 0; i < out.size(); i++)
    if (out[i] >= 0 && (uint64_t)out[i] != i) misplaced++;
  CHECK(d.filled() >= 3 * BUF);   /* the library drops are always filled */
  if (!overload) {
    CHECK(d.unfilled() == 0);     /* every loss in this schedule fits */
    CHECK(misplaced == 0);        /* every real sample at its own index */
  } else {
    CHECK(d.unfilled() > 0);      /* fifo never drains: debt abandoned */
  }
  /* handed downstream = produced - unfilled - owed, always */
  CHECK(out.size() + d.unfilled() + d.owed() == produced);
  std::printf("airspy model%s: produced=%llu out=%zu filled=%llu unfilled=%llu owed=%llu misplaced=%zu\n",
              overload ? " (overload)" : "",
              (unsigned long long)produced, out.size(), (unsigned long long)d.filled(),
              (unsigned long long)d.unfilled(), (unsigned long long)d.owed(), misplaced);
}

/*
 * Stream model of the RTL path with gap fill: drop-newest when the ring is
 * full, zeros per slot emitted by the reader before the slot's data.
 */
static void test_rtl_stream_model()
{
  const unsigned RING = 8;
  const uint64_t SLOT = 1000;
  gap_debt d;
  d.configure(true, 5 * SLOT);

  struct slot { int64_t first; uint64_t gap; };
  std::deque<slot> ring;
  std::vector<int64_t> out;
  uint64_t produced = 0, lost_total = 0;

  for (int k = 0; k < 80; k++) {
    const uint64_t first = produced;
    produced += SLOT;
    const uint64_t room = (RING - ring.size()) * SLOT;
    gap_debt::plan p = d.place(room, SLOT);
    if (p.data == SLOT) {
      ring.push_back({ (int64_t)first, p.zeros });
    } else {
      CHECK(p.data == 0);
      lost_total += SLOT;
    }
    check_invariant(d, lost_total);
    /* reader: one slot per buffer; stalls for k in [10, 20), then catches
     * up at two slots per buffer */
    const int reads = (k >= 10 && k < 20) ? 0 : (k < 40 && k >= 20) ? 2 : 1;
    for (int r = 0; r < reads && !ring.empty(); r++) {
      slot s = ring.front(); ring.pop_front();
      for (uint64_t i = 0; i < s.gap; i++) out.push_back(-1);
      for (uint64_t i = 0; i < SLOT; i++) out.push_back(s.first + (int64_t)i);
    }
  }
  while (!ring.empty()) {
    slot s = ring.front(); ring.pop_front();
    for (uint64_t i = 0; i < s.gap; i++) out.push_back(-1);
    for (uint64_t i = 0; i < SLOT; i++) out.push_back(s.first + (int64_t)i);
  }
  /* The stall overflows the ring and the debt grows while the reader
   * catches up (zeros count against the ring), but stays within the cap:
   * every real sample comes out at its own index. */
  size_t misplaced = 0;
  for (size_t i = 0; i < out.size(); i++)
    if (out[i] >= 0 && (uint64_t)out[i] != i) misplaced++;
  CHECK(d.unfilled() == 0);
  CHECK(misplaced == 0);
  CHECK(d.filled() == 5 * SLOT);
  CHECK(out.size() == produced);
  std::printf("rtl model: produced=%llu out=%zu filled=%llu unfilled=%llu misplaced=%zu\n",
              (unsigned long long)produced, out.size(), (unsigned long long)d.filled(),
              (unsigned long long)d.unfilled(), misplaced);
}

static void test_parse()
{
  using osmosdr_fn::parse_env_long;
  CHECK(parse_env_long(NULL, 10, 0, 99) == 10);
  CHECK(parse_env_long("", 10, 0, 99) == 10);
  CHECK(parse_env_long("0", 10, 0, 99) == 0);
  CHECK(parse_env_long("-4", 10, 0, 99) == 0);
  CHECK(parse_env_long("150", 10, 0, 99) == 99);
  CHECK(parse_env_long("12a", 10, 0, 99) == 10);
  CHECK(parse_env_long("99999999999999999999", 10, 0, 99) == 10);
}

static void test_export()
{
  osmosdr_fn::drop_counter a("airspy=0x1"), b("rtl=0");
  gap_debt d;
  d.configure(true, 100);
  d.lost(40);
  d.place(1000, 10);
  d.lost(500);   /* over the cap */
  d.lost(7);     /* owed */
  a.publish(d, 10e6);
  a.set_rt(osmosdr_fn::RT_FIFO, 10);
  a.set_lib_queue(128, 9);

  /* count only */
  CHECK(osmosdr_fn_capture_stats(NULL, 0, sizeof(osmosdr_fn_capture_stat)) == 2);

  osmosdr_fn_capture_stat s[2];
  std::memset(s, 0xff, sizeof(s));
  CHECK(osmosdr_fn_capture_stats(s, 2, sizeof(s[0])) == 2);
  CHECK(std::string(s[0].device) == "airspy=0x1");
  CHECK(s[0].size == sizeof(osmosdr_fn_capture_stat));
  CHECK(s[0].filled == 40 && s[0].unfilled == 500 && s[0].owed == 7);
  CHECK(s[0].gapfill == 1 && s[0].gap_cap == 100 && s[0].sample_rate == 10e6);
  CHECK(s[0].rt_status == 1 && s[0].rt_prio == 10);
  CHECK(s[0].queue_buffers == 128 && s[0].queue_peak == 9);
  CHECK(std::string(s[1].device) == "rtl=0");
  CHECK(s[1].rt_status == 0 && s[1].filled == 0 && s[1].gapfill == 0);

  /* an older, shorter caller struct: stride and size respected */
  struct short_stat { char device[96]; uint32_t size; int32_t rt_status; } ss[2];
  std::memset(ss, 0, sizeof(ss));
  CHECK(osmosdr_fn_capture_stats((osmosdr_fn_capture_stat *)ss, 2, sizeof(ss[0])) == 2);
  CHECK(ss[0].size == sizeof(ss[0]) && ss[0].rt_status == 1);
  CHECK(std::string(ss[1].device) == "rtl=0");

  /* nonsense size */
  CHECK(osmosdr_fn_capture_stats(s, 2, 3) == 0);

  /* the existing exports still work */
  osmosdr_fn_drop_stat ds[2];
  CHECK(osmosdr_fn_drop_stats(ds, 2) == 2);
}

struct rt_result { int status, prio, policy, child; char msg[160]; };

static void *rt_child(void *p)
{
  *(int *)p = sched_getscheduler(0) & ~SCHED_RESET_ON_FORK;
  return NULL;
}

static void *rt_worker(void *p)
{
  rt_result *r = (rt_result *)p;
  r->status = osmosdr_fn::rt_apply(osmosdr_fn::config().rt_prio, &r->prio, r->msg, sizeof(r->msg));
  r->policy = sched_getscheduler(0) & ~SCHED_RESET_ON_FORK;
  pthread_t t;
  r->child = -1;
  pthread_create(&t, NULL, rt_child, &r->child);
  pthread_join(t, NULL);
  return NULL;
}

static void test_rt(const char *expect)
{
  rt_result r;
  std::memset(&r, 0, sizeof(r));
  pthread_t t;
  pthread_create(&t, NULL, rt_worker, &r);
  pthread_join(t, NULL);
  std::printf("rt: status=%d prio=%d policy=%d child=%d msg=\"%s\"\n",
              r.status, r.prio, r.policy, r.child, r.msg);
  CHECK(r.child == SCHED_OTHER);
  CHECK((r.status == osmosdr_fn::RT_FIFO) == (r.policy == SCHED_FIFO));
  if (expect) {
    char got[32];
    if (r.status == osmosdr_fn::RT_FIFO)
      std::snprintf(got, sizeof(got), "fifo:%d", r.prio);
    else
      std::snprintf(got, sizeof(got), "%s",
                    r.status == osmosdr_fn::RT_OFF ? "off" :
                    r.status == osmosdr_fn::RT_DENIED ? "denied" : "failed");
    if (std::strcmp(got, expect) != 0) {
      std::fprintf(stderr, "FAIL rt expected %s got %s\n", expect, got);
      fails++;
    }
  }
}

int main(int argc, char **argv)
{
  test_debt_basics();
  test_debt_disabled();
  airspy_stream_model(false);
  airspy_stream_model(true);
  test_rtl_stream_model();
  test_parse();
  test_export();
  test_rt(argc > 1 ? argv[1] : NULL);
  std::printf("%s\n", fails ? "FAILED" : "PASS");
  return fails ? 1 : 0;
}
