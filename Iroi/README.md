# Iroi firmware

Firmware for Befaco's Iroi. Developed by Roberto Noris (<https://github.com/hirnlego>).

## Building and flashing the firmware

The firmware is flashed via USB MIDI thanks to the bootloader.

To build and flash the Iroi firmware you can either run

`./programFirmware.sh`

to do everything via command line or

```bash
make clean
make -j17 CONFIG=Release|| exit 1
FirmwareSender -in Build/Iroi.bin -flash `crc32 Build/Iroi.bin` -save Build/Iroi.syx || exit 1
```

to build the firmware and then flash Iroi.syx using the web tool

<https://pingdynasty.github.io/OwlWebControl/firmware.html>

## About the bootloader

To build and flash the Iroi bootloader to the OWL enter `../MidiBoot3` and run

```bash
make clean
make PLATFORM=Iroi CONFIG=Release
make PLATFORM=Iroi upload
```

Note that you will need a programmer like the ST-LINK v2.
