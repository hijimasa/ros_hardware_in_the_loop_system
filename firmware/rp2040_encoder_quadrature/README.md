# rp2040_encoder_quadrature

RP2040 PIO: Quadrature encoder pulse (A/B phase) generation.

This firmware emulates the feedback signal that a rotary encoder would produce, so the real PC's controller code (which reads the A/B pulses to estimate motor position) can be tested in a HILS loop.

See [hils_bridge_encoder_quadrature](../../ros2_hils_bridge/hils_bridge_encoder/hils_bridge_encoder_quadrature/) for the corresponding ROS 2 bridge.
