#!/bin/bash
PROJECT=Oneiroi
HARDWARE=1,0,0
#CONFIG=Debug
CONFIG=Release

make clean
make -j17 CONFIG=$CONFIG || exit 1
FirmwareSender -in Build/${PROJECT}.bin -flash `crc32 Build/${PROJECT}.bin` -save Build/${PROJECT}.syx || exit 1