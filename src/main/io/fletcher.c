#include "io/fletcher.h"

#include <stddef.h>
#include <stdint.h>

// Note that the "mod 255" functionality is not perfect; it's possible for this function to return
// the value 255.  However I don't believe this harms the error detecting functionality of the
// Fletcher algorithm, so we accept it.
static inline int add_mod_255(int a, int b) {
  int sum = a + b;
  return (sum + (sum >> 8)) & 0xFF;
}

uint16_t compute_fletcher16(const uint8_t* data, const size_t size) {
  int c0 = 3;
  int c1 = 5;
  for (size_t i=0; i<size; i++) {
    c0 = add_mod_255(c0, data[i]);
    c1 = add_mod_255(c1, c0);
  }
  return (uint16_t)((c1 << 8) | c0);
}
