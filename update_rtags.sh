#!/bin/bash

make clean

# The sed command to remove the -save-temps arg is necessary because this arg seems to choke
# rtags command parsing.
make -nk | sed 's/-save-temps=obj//g' | rc -c -
