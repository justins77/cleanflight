#include "io/debug_console.h"

#include <stdint.h>
#include "common/typeconversion.h"

#define DEBUG_BUFFER_SIZE 1024
#define DEBUG_BUFFER_SIZE_MASK 1023
uint8_t debug_buffer[DEBUG_BUFFER_SIZE];
int debug_head = 0;
int debug_tail = 0;

void addDebugChar(uint8_t ch) {
  debug_buffer[debug_head++] = ch;
  debug_head &= DEBUG_BUFFER_SIZE_MASK;
}

int getNextDebugChar() {
  if (debug_head == debug_tail) {
    return -1;
  }
  int ch = debug_buffer[debug_tail++];
  debug_tail &= DEBUG_BUFFER_SIZE_MASK;
  return ch;
}

void debugPrint(char* s) {
  while ((*s) != '\0') {
    addDebugChar(*s);
    s++;
  }
}

void debugPrinti(int x) {
  char buffer[80];
  i2a(x, buffer);
  debugPrint(buffer);
}

void debugPrintib(uint32_t x) {
  for (int i=31; i>=0; i--) {
    addDebugChar(((x >> i) & 1) ? '1' : '0');
  }
}

void debugPrintc(char ch) {
  addDebugChar(ch);
}

void debugPrintx(uint8_t x) {
  static const char* hex_chars = "0123456789ABCDEF";
  debugPrintc(hex_chars[x >> 4]);
  debugPrintc(hex_chars[x & 0xF]);
}

void debugPrintVar(char* s, int x) {
  debugPrint(s);
  debugPrinti(x);
  debugPrint("\r\n");
}

void debugPrintVarib(char* s, int x) {
  debugPrint(s);
  debugPrintib(x);
  debugPrint("\r\n");
}
