# rp2040_camera_uvc

Pico#2: SPI slave receive from Pico#1 -> TinyUSB UVC MJPEG output to real robot PC

## Architecture

```
シミュPC ──USB CDC──> Pico#1 ──SPI──> [Pico#2] ──USB UVC──> 実機PC
                                      ^^^^^^^^              /dev/video0
                                      このファームウェア
```

Pico#2 is recognized as a standard USB UVC camera by the real robot PC.
It receives MJPEG frames from Pico#1 via SPI slave, and outputs them
through TinyUSB's UVC device class.

## Specifications

| Item | Value |
|------|-------|
| USB class | UVC (USB Video Class) MJPEG |
| Resolution | 640x480 |
| Max FPS | ~17fps (USB Full-Speed bandwidth limit) |
| Frame buffer | Double buffered, 64KB x 2 |
| SPI mode | Slave, 20MHz clock |

## Prerequisites

```bash
# Install pico-sdk
git clone https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk && git submodule update --init
export PICO_SDK_PATH=$(pwd)

# Install ARM toolchain
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential
```

## Build

```bash
cp $PICO_SDK_PATH/external/pico_sdk_import.cmake .
mkdir build && cd build
cmake .. -DPICO_SDK_PATH=$PICO_SDK_PATH
make -j$(nproc)
```

Output: `build/rp2040_camera_uvc.uf2`

## Flash

1. Hold BOOTSEL button on Pico
2. Connect USB to PC (appears as mass storage RPI-RP2)
3. Copy UF2:

```bash
cp build/rp2040_camera_uvc.uf2 /media/$USER/RPI-RP2/
```

After reboot, connect the Pico's USB to the **real robot PC** (not the simulation PC).

## Wiring (from Pico#1)

| Pico#1 Pin | Function      | Pico#2 Pin |
|------------|---------------|------------|
| GP19       | SPI0 MOSI(TX) | GP16 (RX)  |
| GP18       | SPI0 SCK      | GP18       |
| GP17       | SPI0 CSn      | GP17       |
| GP20       | READY signal  | GP20       |
| GND        | Ground        | GND        |

## Verify

On the real robot PC:

```bash
v4l2-ctl --list-devices     # Should show "HILS UVC Camera"
ffplay /dev/video0           # View camera output
```
