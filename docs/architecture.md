# Project Architecture (Initial)

## Separation of concerns

- **Embedded/Low-Level** handles hardware-near logic and real-time control.
- **High-Level** handles application orchestration, ROS 2 graph behavior, and Pi services.
- **Sensors** isolates each sensor stack independently (LiDAR now, MPU/GPS later).
- **Features** isolates reusable capabilities (SLAM now, more features later).
- **Shared** centralizes reusable definitions to avoid duplication.

## Next recommended additions

1. Add per-section `.gitkeep` where empty folders must be retained.
2. Add CI workflows for linting/build sanity checks.
3. Add coding standards per subsystem.
4. Add ROS 2 workspace bootstrap scripts.
