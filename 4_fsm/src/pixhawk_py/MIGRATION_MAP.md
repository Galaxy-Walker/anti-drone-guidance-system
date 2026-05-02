# Offboard C++ -> Python Migration Map

This table records behavior mapping from C++ `OffboardControl` to Python
`OffboardControl`.

## Topic and QoS contract

- Publishers
  - `/fmu/in/offboard_control_mode`
  - `/fmu/in/trajectory_setpoint`
  - `/fmu/in/vehicle_command`
- Subscribers
  - `/fmu/out/vehicle_status_v1`
  - `/fmu/out/vehicle_control_mode`
  - `/fmu/out/battery_status`
- Subscriber QoS
  - `history=KEEP_LAST`
  - `depth=10`
  - `reliability=BEST_EFFORT`
  - `durability=TRANSIENT_LOCAL`
- Timer
  - `100 ms`

## Method mapping

| C++ method | Python method | Input | Output | Side effects |
| --- | --- | --- | --- | --- |
| `initializePublishers` | `initializePublishers` | none | none | create 3 publishers |
| `initializeSubscribers` | `initializeSubscribers` | none | none | create 3 subscribers with PX4-compatible QoS |
| `process` | `process` | current nav/arming/offboard flags | state transitions | publish offboard/setpoint/cmd by state |
| `update_state` | `update_state` | `FlightState` | none | updates current state and state start time |
| `check_pre_arm_conditions` | `check_pre_arm_conditions` | current nav state | bool | none |
| `return_flight` | `return_flight` | nav state | none | send RTL command + anti-loss repeated setpoint stream |
| `publish_vehicle_command` | `publish_vehicle_command` | command + params | none | publish PX4 command with target/source fields |
| `publish_offboard_control_mode` | `publish_offboard_control_mode` | none | none | publish position-control mode with timestamp |
| `publish_trajectory_setpoint` | `publish_trajectory_setpoint` | x/y/z/yaw | none | publish hover setpoint with timestamp |
| `vehicle_status_callback` | `vehicle_status_callback` | `VehicleStatus` | none | update nav/arming/timestamp cache |
| `vehicle_control_mode_callback` | `vehicle_control_mode_callback` | `VehicleControlMode` | none | update offboard flags and transition logs |
| `battery_status_callback` | `battery_status_callback` | `BatteryStatus` | none | update battery connection/remaining/warning |
| `check_battery_safety` | `check_battery_safety` | cached battery fields | none | update safe_to_fly + state-change logs |
| `monitor_safety` | `monitor_safety` | nav state, arming state | none | warnings + optional state update to ERROR |
| `handle_error_state` | `handle_error_state` | nav state, arming state | none | disarm/manual-mode recovery attempts |

## State machine branch mapping

- `INIT`
  - reset setpoint counter
  - transition to `SENDING_CONTROL`
- `SENDING_CONTROL`
  - publish offboard mode + setpoint every 100 ms
  - after 10 cycles send `VEHICLE_CMD_DO_SET_MODE(1,6)`
  - transition to `REQUESTING_OFFBOARD`
- `REQUESTING_OFFBOARD`
  - keep publishing mode + setpoint
  - timeout 2 s -> `ERROR`
  - if nav is OFFBOARD and offboard enabled -> `ARMING`
- `ARMING`
  - keep publishing mode + setpoint
  - if pre-arm check passes send ARM command
  - transition to `WAITING_ARM_CONFIRM`
- `WAITING_ARM_CONFIRM`
  - keep publishing mode + setpoint
  - timeout 2 s -> `ERROR`
  - if armed -> `OFFBOARD_ACTIVE`
- `OFFBOARD_ACTIVE`
  - keep publishing mode + setpoint (hover)
- `RETURN_REQUESTED`
  - call return-flight logic
- `ERROR`
  - recovery function preserved but not enabled in the main loop (parity with
    current C++ runtime behavior)

## Shared-state handling strategy

- Python implementation uses ROS2 single-threaded callback semantics by default.
- A minimal lock protects cross-callback shared fields and state transitions.
- Goal is behavior consistency (state and command timing), not byte-level memory
  ordering equivalence.
