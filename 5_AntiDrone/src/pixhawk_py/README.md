# pixhawk_py

Python migration of the PX4 offboard state machine from the C++ `pixhawk` package,
implemented as an independent `ament_python` package.

## Behavior-equivalent scope

This package keeps behavior aligned with the C++ source for:

- Topics:
  - Publishers: `/fmu/in/offboard_control_mode`, `/fmu/in/trajectory_setpoint`,
    `/fmu/in/vehicle_command`
  - Subscriptions: `/fmu/out/vehicle_status_v1`, `/fmu/out/vehicle_control_mode`,
    `/fmu/out/battery_status`
- QoS for subscriptions: `KEEP_LAST(depth=10)`, `BEST_EFFORT`,
  `TRANSIENT_LOCAL`
- Timer period: `100 ms`
- State machine flow and command timing:
  - `INIT`
  - `SENDING_CONTROL`
  - `REQUESTING_OFFBOARD`
  - `ARMING`
  - `WAITING_ARM_CONFIRM`
  - `OFFBOARD_ACTIVE`
  - `RETURN_REQUESTED`
  - `ERROR`
- Command publish behavior and timeout policy for OFFBOARD/ARM sequence
- RTL trigger and one-time log gate in return-flight flow

## Not covered as strict equivalence

- Log text is not required to be byte-for-byte identical to C++ logs.
- Error recovery method `handle_error_state()` is migrated but not enabled in the
  main `process()` loop, matching current C++ runtime behavior.

## Build and run

Run in workspace root `5_AntiDrone`:

```bash
colcon build --packages-select pixhawk_py
source install/setup.zsh
ros2 run pixhawk_py px4_node
```

## Coexistence with C++ package

- Run `pixhawk` (C++) when validating legacy behavior.
- Run `pixhawk_py` when validating Python migration.
- Do not run both nodes together on the same PX4 topics to avoid command
  conflicts.
