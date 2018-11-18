#!/bin/bash

dfu-util --alt 0 -s 0x08000000:leave -D obj/cleanflight_2.4.1_SPRACINGF7DUAL.bin
