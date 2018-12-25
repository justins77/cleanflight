#pragma once

#include <stdint.h>

void debugPrint(const char* s);
void debugPrinti(int x);
void debugPrintu(unsigned int x);
void debugPrintib(uint32_t x);
void debugPrintc(char ch);
void debugPrintx(uint8_t x);
void debugPrintVar(const char*s, int x);
void debugPrintVarib(const char* s, int x);
int getNextDebugChar();
