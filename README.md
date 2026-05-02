# MorphoNav

Repository scaffold for a robotics/navigation project split by system level and growth areas:

- `embedded-low-level/` for MCU firmware, low-level drivers, HAL, and BSP code.
- `high-level/` for Raspberry Pi applications and ROS 2 packages/workspaces.
- `sensors/` for sensor-specific workspaces (`lidar/` now, more later like `mpu/`, `gps/`).
- `features/` for cross-sensor capabilities (`slam/` now, more later).
- `shared/` for interfaces, messages, and reusable utilities.
- `docs/` for architecture and planning documentation.

This layout is intentionally modular so each subsystem and capability can evolve independently.
