#pragma once

#include <stdint.h>

void debugPrint(char* s);
void debugPrinti(int x);
void debugPrintc(char ch);
void debugPrintx(uint8_t x);
void debugPrintVar(char*s, int x);
int getNextDebugChar();
