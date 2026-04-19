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
├── ros2_hils_bridge/                     # ROS 2 packages (git submodule)
├── firmware/                             # RP2040 / ESP32 firmware
│   ├── common/                           #   Shared headers (HILS frame protocol)
│   ├── rp2040_camera_uvc/                #   UVC camera (Pico#2)
│   ├── rp2040_camera_uvc_spi_sender/     #   USB-CDC -> SPI relay (Pico#1)
│   ├── rp2040_actuator_servo_pwm/        #   RC servo PWM output
│   ├── rp2040_encoder_quadrature/        #   Quadrature encoder A/B output
│   ├── rp2040_imu_invensense_mpu6050/    #   I2C slave MPU-6050 register map
│   ├── rp2040_ethernet_bridge/           #   Ethernet bridge (reference)
│   └── esp32_can_bridge/                 #   CAN bridge (reference)
├── docs/                                 # Architecture, BOM, verification guides
└── hardware/                             # Schematics, enclosures (planned)
```

### Submodule: ros2_hils_bridge

The ROS 2 packages live in `ros2_hils_bridge/` and can be cloned directly into `colcon_ws/src`. See [ros2_hils_bridge/README.md](ros2_hils_bridge/README.md) for details.

## Naming Convention

To keep the project consistent as new emulators are added, packages and firmware follow a **two-level naming pattern**:

```
ros2_hils_bridge/hils_bridge_<sensor_type>/hils_bridge_<sensor_type>_<protocol_or_vendor_series>/
firmware/rp2040_<sensor_type>_<protocol_or_vendor_series>/
```

- **`<sensor_type>`** — physical/functional category: `lidar`, `camera`, `gps`, `imu`, `actuator`, `encoder`, `can`
- **`<protocol_or_vendor_series>`** — chosen by the following rule:
  - **Industry-standard protocol** (UVC, NMEA0183, PWM, quadrature, …): use the protocol name itself, signaling that any vendor's device with that protocol works (e.g. `hils_bridge_camera_uvc`, `hils_bridge_gps_nmea0183`)
  - **Vendor-specific protocol**: use `<vendor>_<series>` so that the package's actual scope is unambiguous (e.g. `hils_bridge_lidar_livox_mid360`, `hils_bridge_imu_witmotion_wt901`, `hils_bridge_imu_invensense_mpu6050`). Even within one vendor, generations or series often break compatibility, so always include the series

When adding a new emulator, decide the sensor type, then pick the standard protocol name if the implementation truly works across vendors, otherwise the `<vendor>_<series>` form. Keep the ROS package name and the corresponding firmware directory in lockstep.

## Implementation Status

### Track A: PC-based developers (no microcontroller needed)

| Device | Method | ROS Package | Firmware | Status |
|--------|--------|-------------|----------|--------|
| Livox Mid-360 | USB-LAN + SW | hils_bridge_lidar_livox_mid360 | N/A | Implemented, **verified** |
| Velodyne VLP-16 | USB-LAN + SW | hils_bridge_lidar_velodyne_vlp16 | N/A | Implemented, **verified** |
| Ouster OS1 | USB-LAN + SW | hils_bridge_lidar_ouster_os1 | N/A | Implemented, **verified**[^ouster-note] |
| GPS (NMEA 0183) | FT234X x 2 | hils_bridge_gps_nmea0183 | N/A | Implemented, **verified** |
| IMU (Witmotion WT901) | FT234X x 2 | hils_bridge_imu_witmotion_wt901 | N/A | Implemented, **verified**[^witmotion-note] |

[^ouster-note]: For Ouster, the emulator exposes HTTP REST API on port 80, so Docker requires `sysctls: net.ipv4.ip_unprivileged_port_start=80`. See [docs/hils_verification_guide.md](docs/hils_verification_guide.md#13-ouster-os1) for details.
[^witmotion-note]: The IMU emulator emits the four standard WT901 packets (0x51 Accel / 0x52 Gyro / 0x53 Euler / 0x59 Quaternion) so it works with `witmotion_ros` (ElettraSciComp) at default `use_native_orientation: true`. Driver build needs `libqt5serialport5-dev` (already in the Dockerfile). See [docs/hils_verification_guide.md](docs/hils_verification_guide.md#22-シリアルimu-witmotion-wt901) for setup details.

### Track B: Microcontroller developers (RP2040 firmware)

| Device | Method | ROS Package | Firmware | Status |
|--------|--------|-------------|----------|--------|
| USB Camera | RP2040 UVC | hils_bridge_camera_uvc | rp2040_camera_uvc + rp2040_camera_uvc_spi_sender | Implemented, **verified** |
| RC Servo | RP2040 PIO | hils_bridge_actuator_servo_pwm | rp2040_actuator_servo_pwm | Implemented, unverified |
| Quadrature Encoder | RP2040 PIO | hils_bridge_encoder_quadrature | rp2040_encoder_quadrature | Implemented, unverified |
| I2C IMU (MPU-6050) | RP2040 I2C slave | hils_bridge_imu_invensense_mpu6050 | rp2040_imu_invensense_mpu6050 | Implemented, unverified |

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
cd firmware/rp2040_camera_uvc
mkdir build && cd build
cmake .. -DPICO_SDK_PATH=~/pico-sdk
make -j$(nproc)
# Copy build/rp2040_camera_uvc.uf2 to Pico in BOOTSEL mode
```

## Documentation

- [Architecture Design](docs/ros_hils_architecture.md)
- [Bill of Materials (BOM)](docs/hils_verification_bom.md)
- [Expansion Roadmap](docs/hils_expansion_roadmap.md)
- [Verification Guide](docs/hils_verification_guide.md)

## License

MIT
