#pragma once

#include <stdint.h>

typedef struct {
  bool has_estimated_skew_rate;
  float estimated_skew_rate;

  bool has_estimated_last_offset;
  uint32_t estimated_last_offset;

  bool has_last_utimes;
  uint32_t last_local_utime;
  uint32_t last_peer_utime;
} clock_sync_t;
