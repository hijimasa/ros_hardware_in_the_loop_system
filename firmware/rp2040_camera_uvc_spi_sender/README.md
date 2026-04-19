# rp2040_camera_uvc_spi_sender

Pico#1: USB-CDC receive from simulation PC -> SPI master transmit to Pico#2

## Role

Receives MJPEG frames from the ROS bridge node on the simulation PC via
USB CDC (`/dev/ttyACM0`), extracts JPEG payloads using the HILS framing
protocol, and forwards them to Pico#2 via SPI master.

## Build

```bash
cp $PICO_SDK_PATH/external/pico_sdk_import.cmake .
mkdir build && cd build
cmake .. -DPICO_SDK_PATH=$PICO_SDK_PATH
make -j$(nproc)
```

## Flash

Hold BOOTSEL, connect USB, then:

```bash
cp build/rp2040_camera_uvc_spi_sender.uf2 /media/$USER/RPI-RP2/
```

## Wiring to Pico#2

| Pico#1 Pin | Function      | Pico#2 Pin |
|------------|---------------|------------|
| GP19       | SPI0 MOSI(TX) | GP16 (RX)  |
| GP18       | SPI0 SCK      | GP18       |
| GP17       | SPI0 CSn      | GP17       |
| GP20       | READY signal  | GP20       |
| GND        | Ground        | GND        |
