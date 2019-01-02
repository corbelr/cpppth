#!/bin/bash

# Builds pantilthat.o (interface to the Pimoroni Pan-Tilt HAT) and
# hansi_check (exploitation program) files.

# For this compilation to succeed, those packages must be installed :
# sudo apt-get install i2c-tools
# http://ftp.fr.debian.org/debian/pool/main/i/i2c-tools/libi2c0_4.1-1_armhf.deb
# sudo apt-get install libi2c-dev

# BEWARE ! The /usr/include/linux/i2c-dev.h file MUST be this one :
# https://github.com/torvalds/linux/blob/master/include/uapi/linux/i2c-dev.h

CFLAGS="-fpic -D_REENTRANT -O2 -Wall -Wextra -pedantic"
gcc ${CFLAGS} -c pantilthat.cpp -o pantilthat.o
g++ ${CFLAGS} check_pth.cpp -o check_pth pantilthat.o /usr/lib/arm-linux-gnueabihf/libi2c.so.0
strip check_pth

rm *.o

exit 0
