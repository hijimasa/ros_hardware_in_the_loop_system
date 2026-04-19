# rp2040_imu_invensense_mpu6050

RP2040 I2C slave emulation of the InvenSense MPU-6050 register map.

Receives Imu data from the simulation PC via USB CDC and exposes it on the I2C bus as if it were a real MPU-6050. The real PC's I2C master code (typically reading registers 0x3B–0x48) sees the simulated values.

See [hils_bridge_imu_invensense_mpu6050](../../ros2_hils_bridge/hils_bridge_imu/hils_bridge_imu_invensense_mpu6050/) for the corresponding ROS 2 bridge.
