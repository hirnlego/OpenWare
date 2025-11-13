#!/bin/bash
PROJECT=Iroi
#HARDWARE=3,0,0
HARDWARE=4,0,0 # When audio interface is connected
#CONFIG=Debug
CONFIG=Release

make clean
make -j17 CONFIG=$CONFIG || exit 1
/home/roberto/Programs/FirmwareSender/Builds/Linux/build/FirmwareSender -in Build/${PROJECT}.bin -flash `crc32 Build/${PROJECT}.bin` -save Build/${PROJECT}.syx || exit 1
echo "Entering bootloader mode"
amidi -p hw:${HARDWARE} -S f07d527ef7 || exit 1
sleep 3
echo "Sending firmware"
amidi -p hw:${HARDWARE} -s Build/${PROJECT}.syx || exit 1
sleep 1
echo "Restart"
amidi -p hw:${HARDWARE} -S f07d527df7 || exit 1
echo "Done!"