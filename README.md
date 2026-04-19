**English** | [日本語](README-ja.md)

# ROS Hardware-in-the-Loop Simulation System

Affordable **Hardware-in-the-Loop Simulation (HILS)** for ROS 2 robots, using inexpensive microcontrollers and off-the-shelf USB devices to test the full communication stack including physical interfaces.

UVC camera verification (streaming simulation video from Isaac Sim)
![UVC camera HILS demo](./figs/uvc_camera_hils.gif)

Livox MID360 verification (transmitting ros2 bag data via the native Livox SDK2 protocol)
![Livox MID360 HILS demo](./figs/livox_mid360_hils.gif)

## Overview

Software simulators (Gazebo / Unity / Isaac Sim) publish directly to ROS topics, which means the actual sensor drivers and physical communication paths (UDP, UART, USB, I2C, etc.) are never tested. This project bridges that gap by converting simulator outputs into device-native protocols, so the real-hardware driver sees what it would see from a real sensor -- all for **under $30 in parts**.

## Repository Structure

```
ros_hardware_in_the_loop_system/
├── ros2_hils_bridge/          # ROS 2 packages (git submodule)
├── firmware/                  # RP2040 / ESP32 firmware
│   ├── common/                #   Shared headers (HILS frame protocol)
│   ├── rp2040_uvc_bridge/     #   UVC camera emulator
│   ├── rp2040_cdc_spi_sender/ #   CDC-SPI relay (for camera)
│   ├── rp2040_pwm_servo/      #   PWM servo + encoder output
│   ├── rp2040_i2c_slave/      #   I2C slave (MPU-6050)
│   ├── rp2040_encoder_emulator/ # Encoder output (standalone)
│   ├── rp2040_ethernet_bridge/  # Ethernet bridge (reference)
│   └── esp32_can_bridge/      #   CAN bridge (reference)
├── docs/                      # Architecture, BOM, verification guides
└── hardware/                  # Schematics, enclosures (planned)
```

### Submodule: ros2_hils_bridge

The ROS 2 packages live in `ros2_hils_bridge/` and can be cloned directly into `colcon_ws/src`. See [ros2_hils_bridge/README.md](ros2_hils_bridge/README.md) for details.

## Implementation Status

### Track A: PC-based developers (no microcontroller needed)

| Device | Method | ROS Package | Firmware | Status |
|--------|--------|-------------|----------|--------|
| Livox Mid-360 | USB-LAN + SW | hils_bridge_lidar_livox | N/A | Implemented, **verified** |
| Velodyne VLP-16 | USB-LAN + SW | hils_bridge_lidar_velodyne | N/A | Implemented, **verified** |
| Ouster OS1 | USB-LAN + SW | hils_bridge_lidar_ouster | N/A | Implemented, **verified**[^ouster-note] |

[^ouster-note]: For Ouster, the emulator exposes HTTP REST API on port 80, so Docker requires `sysctls: net.ipv4.ip_unprivileged_port_start=80`. See [docs/hils_verification_guide.md](docs/hils_verification_guide.md#13-ouster-os1) for details.
| GPS (NMEA) | FT234X x 2 | hils_bridge_serial_gps | N/A | Implemented, unverified |
| IMU (Witmotion) | FT234X x 2 | hils_bridge_serial_imu | N/A | Implemented, unverified |

### Track B: Microcontroller developers (RP2040 firmware)

| Device | Method | ROS Package | Firmware | Status |
|--------|--------|-------------|----------|--------|
| USB Camera | RP2040 UVC | hils_bridge_camera_uvc | rp2040_uvc_bridge + rp2040_cdc_spi_sender | Implemented, **verified** |
| PWM Servo | RP2040 PIO | hils_bridge_actuator_pwm | rp2040_pwm_servo | Implemented, unverified |
| Encoder | RP2040 PIO | (integrated with pwm) | rp2040_pwm_servo | Implemented, unverified |
| I2C IMU (MPU-6050) | RP2040 I2C slave | hils_bridge_sensor_i2c | rp2040_i2c_slave | Implemented, unverified |

## Quick Start

### Building the ROS 2 packages

```bash
cd ~/colcon_ws/src
git clone --recursive https://github.com/<your-org>/ros_hardware_in_the_loop_system.git
# Or clone only the ROS packages:
# git clone https://github.com/<your-org>/ros2_hils_bridge.git

cd ~/colcon_ws
colcon build
source install/setup.bash
```

### Building firmware (RP2040 only)

```bash
cd firmware/rp2040_uvc_bridge
mkdir build && cd build
cmake .. -DPICO_SDK_PATH=~/pico-sdk
make -j$(nproc)
# Copy build/rp2040_uvc_bridge.uf2 to Pico in BOOTSEL mode
```

## Documentation

- [Architecture Design](docs/ros_hils_architecture.md)
- [Bill of Materials (BOM)](docs/hils_verification_bom.md)
- [Expansion Roadmap](docs/hils_expansion_roadmap.md)
- [Verification Guide](docs/hils_verification_guide.md)

## License

MIT
