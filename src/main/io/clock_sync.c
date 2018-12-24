#include "io/clock_sync.h"

#define EPSILON 0.1f

// local_utime and peer_utime should correspond to measurements of the same event.  Returns the
// estimate of the peer_utime converted to local_utime.
// TODO: carefully test the wraparound cases
uint32_t update_clock_sync(clock_sync_t* clock_sync, uint32_t local_utime, uint32_t peer_utime) {
  uint32_t converted_peer_utime;

  if (clock_sync->has_last_utimes) {
    int32_t local_delta = local_utime - clock_sync->last_local_utime;
    int32_t peer_delta = peer_utime - clock_sync->last_peer_utime;

    float skew_rate = (float)peer_delta / (float)local_delta;

    if (clock_sync->has_estimated_skew_rate) {
      clock_sync->estimated_skew_rate = skew_rate * EPSILON + clock_sync->estimated_skew_rate * (1-EPSILON);
    } else {
      clock_sync->estimated_skew_rate = skew_rate;
      clock_sync->has_estimated_skew_rate = true;
    }

    uint32_t offset = peer_utime - local_utime;
    if (clock_sync->has_estimated_last_offset) {
      uint32_t expected_offset = clock_sync->estimated_last_offset + (int32_t)(clock_sync->estimated_skew_rate * local_delta);
      int32_t residual = offset - expected_offset;
      clock_sync->estimated_last_offset = expected_offset + residual * EPSILON;
    } else {
      clock_sync->estimated_last_offset = offset;
      clock_sync->has_estimated_last_offset = true;
    }

    converted_peer_utime = peer_utime - clock_sync->estimated_last_offset;
  } else {
    clock_sync->has_last_utimes = true;
    converted_peer_utime = local_utime;
  }

  clock_sync->last_local_utime = local_utime;
  clock_sync->last_peer_utime = peer_utime;

  return converted_peer_utime;
}
