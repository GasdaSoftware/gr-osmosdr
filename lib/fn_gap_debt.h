/* -*- c++ -*- */
/*
 * Gap debt: keep the sample count continuous across counted losses.
 *
 * When a source knows that N samples were lost at a given point of the
 * stream (the vendor library dropped whole buffers before this one, or the
 * block's own fifo had no room for the tail of the previous buffer), it
 * records the loss here and later inserts the same number of zero samples at
 * that point, so every downstream channel keeps its timing (P25 Phase 2 TDMA
 * slot and superframe sync in particular) instead of seeing the stream
 * spliced.
 *
 * Rules (pure logic, no locking; the owner calls it from one thread):
 *
 *  - lost(n): n samples are missing immediately before the next data that
 *    gets placed.  With gap fill off, or when the outstanding debt grows past
 *    the cap, the debt is abandoned and counted as unfilled: a long run of
 *    zeros into a flowgraph that is already behind only deepens the backlog,
 *    and decoder sync is lost anyway.
 *
 *  - place(room, n): the caller has n data samples and room free slots.
 *    With no debt this is the upstream rule: copy min(room, n), and the part
 *    that does not fit becomes the debt (it was lost right before the next
 *    buffer).  With a debt, the zeros and the WHOLE buffer go in only
 *    together; until they fit, each new buffer joins the debt instead (the
 *    gap simply grows, so time stays exact), and once the debt would pass
 *    the cap it is abandoned and the buffer is handled the upstream way.
 *    Zeros are therefore inserted only once the backlog has cleared; a
 *    flowgraph that stays behind never gets zeros instead of real samples,
 *    its loss just comes in cap-sized bursts instead of many small splices.
 *
 * Invariant: total lost == filled() + unfilled() + owed().
 */

#ifndef OSMOSDR_FN_GAP_DEBT_H
#define OSMOSDR_FN_GAP_DEBT_H

#include <cstdint>

namespace osmosdr_fn {

class gap_debt
{
public:
  struct plan {
    uint64_t zeros;  /* insert this many zero samples first */
    uint64_t data;   /* then this many of the caller's samples; the rest are lost */
  };

  gap_debt() : _enabled(false), _cap(0), _owed(0), _filled(0), _unfilled(0) {}

  /* cap: largest debt (samples) that is still filled. */
  void configure(bool enabled, uint64_t cap)
  {
    _enabled = enabled && cap > 0;
    _cap = cap;
    if (!_enabled)
      abandon();
  }

  bool enabled() const { return _enabled; }
  uint64_t cap() const { return _cap; }

  void lost(uint64_t n)
  {
    if (!n)
      return;
    _owed += n;
    if (!_enabled || _owed > _cap)
      abandon();
  }

  plan place(uint64_t room, uint64_t n)
  {
    plan p = { 0, 0 };

    if (_owed) {
      if (_owed <= room && n <= room - _owed) {
        /* The gap and this whole buffer fit: zeros first, then the data. */
        p.zeros = _owed;
        p.data = n;
        _filled += _owed;
        _owed = 0;
        return p;
      }
      if (n <= _cap && _owed <= _cap - n) {
        /* Still backed up: this buffer cannot go in front of the zeros it
         * follows, so it joins the debt (time stays exact). */
        _owed += n;
        return p;
      }
      /* The debt would pass the cap: give it up, back to upstream. */
      abandon();
    }

    p.data = n < room ? n : room;
    lost(n - p.data);
    return p;
  }

  /* Forget the outstanding debt (stream restarted, or it cannot be placed). */
  void abandon()
  {
    _unfilled += _owed;
    _owed = 0;
  }

  uint64_t owed() const { return _owed; }
  uint64_t filled() const { return _filled; }
  uint64_t unfilled() const { return _unfilled; }

private:
  bool _enabled;
  uint64_t _cap;
  uint64_t _owed;
  uint64_t _filled;
  uint64_t _unfilled;
};

} // namespace osmosdr_fn

#endif /* OSMOSDR_FN_GAP_DEBT_H */
