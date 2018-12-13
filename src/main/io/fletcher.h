#pragma once

#include <stddef.h>
#include <stdint.h>

// Computes a modified version of the Fletcher16 checksum algorithm on the specified data.
uint16_t compute_fletcher16(const uint8_t* data, const size_t size);
