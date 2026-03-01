# STS3215 Register Notes

## PID Coefficients: Two Separate Control Loops

The STS3215 has **two independent PID loops**, one for each major operating mode:

| Registers | Addr | Mode | Purpose |
|---|---|---|---|
| P_Coefficient | 21 | Mode 0 (position) | Position tracking — proportional gain |
| D_Coefficient | 22 | Mode 0 (position) | Position tracking — derivative gain (damping) |
| I_Coefficient | 23 | Mode 0 (position) | Position tracking — integral gain (error accumulation) |
| Vel_P_Coeff | 37 | Mode 1 (speed, closed-loop) | Velocity regulation — proportional gain |
| Vel_I_Coeff | 39 | Mode 1 (speed, closed-loop) | Velocity regulation — integral gain |

- In position mode (mode=0), only P/D/I_Coefficient are active. Vel_P/I are dormant.
- In speed mode (mode=1, closed-loop), only Vel_P/I_Coeff are active. P/D/I are dormant.
- Factory defaults: P=16, D=32, I=0 (pure PD control), Vel_P=10, Vel_I=200.
- All are 1-byte EPROM registers.

Note: `Over_Current_Prot_Time` (addr 38) sits between Vel_P and Vel_I — there is no Vel_D register.

## Motion Profile Registers (Position Mode)

Three SRAM registers at addr 42-47 form a **motion command block** and are designed
to be written together (see Communication Protocol manual, Example 4):

| Register | Addr | Size | Description |
|---|---|---|---|
| Goal_Position | 42 | 2 bytes | Target position (sign-magnitude bit 15) |
| Goal_Time | 44 | 2 bytes | Desired move duration in ms (0 = ignored) |
| Goal_Velocity | 46 | 2 bytes | Max travel speed in steps/sec (sign-magnitude bit 15) |

### Priority / interaction logic

1. **Goal_Time > 0**: Time-based profiling. The servo calculates the required speed
   to reach Goal_Position in exactly N milliseconds.
2. **Goal_Time = 0, Goal_Velocity > 0**: Velocity-limited profiling. The servo moves
   toward Goal_Position at up to the specified speed.
3. **Goal_Time = 0, Goal_Velocity = 0**: Falls back to **Acceleration** (addr 41) and
   **Max_Velocity_Limit** (addr 84) to determine the motion profile.

### Acceleration fallback chain

When Goal_Time=0 and Goal_Velocity=0:
- **Acceleration** (addr 41, SRAM, 1 byte): Controls acceleration/deceleration ramp.
  When 0, uses Acceleration_Multiplier (addr 86) as fallback.
- **Acceleration_Multiplier** (addr 86, factory, 1 byte): Used when Acceleration=0.
  When also 0, the servo moves as fast as possible.
- **Max_Velocity_Limit** (addr 84, factory, 1 byte): Hard upper limit on velocity
  regardless of other settings. Factory default = 68.

## Operating Modes (addr 33)

| Value | Mode | Description |
|---|---|---|
| 0 | Position (servo) | Default. Absolute position control, 0-360 deg. Uses P/D/I PID. |
| 1 | Speed (closed-loop motor) | Constant velocity, load-compensated. Uses Vel_P/I PID. |
| 2 | Speed (open-loop motor) | Constant PWM drive. Speed drops under load. No PID. |
| 3 | Step | Relative position moves from current position. |

## Protocol Example (from Communication Protocol manual)

Writing position=2048, time=0, speed=1000 to servo ID 1:

```
Write 6 bytes starting at addr 0x2A (42):
  Position: 0x00 0x08  (2048)
  Time:     0x00 0x00  (0)
  Speed:    0xE8 0x03  (1000)

Frame: FF FF 01 09 03 2A 00 08 00 00 E8 03 D5
```

Note: STS series uses little-endian byte order (low byte first).

## Torque Limiting and Overload Protection

Three registers interact to control torque output and protect against sustained overload:

| Register | Addr | Size | Memory | Range | Description |
|---|---|---|---|---|---|
| Max_Torque_Limit | 16 | 2 | EPROM | 0-1000 | Hard ceiling (0-100%). Persists across power cycles. |
| Torque_Limit | 48 | 2 | SRAM | 0-1000 | Runtime limit. **Resets on power cycle — must be set during init.** |
| Overload_Torque | 36 | 1 | EPROM | 0-100 | Overload detection threshold (%). |
| Protective_Torque | 34 | 1 | EPROM | 0-100 | Torque level (%) the servo drops to when overload triggers. |
| Protection_Time | 35 | 1 | EPROM | | Duration the servo must exceed Overload_Torque before protection activates. |

### How overload protection works (from datasheet section 7-11)

When the servo's output torque exceeds `Overload_Torque`% of maximum for longer than
`Protection_Time`, the servo enters overload protection: torque drops to `Protective_Torque`%
and the overload flag is set. Re-issuing a position command clears the flag.

### Key rule: Overload_Torque must be > effective torque limit %

If SRAM `Torque_Limit` is set to N (0-1000), the effective maximum torque is N/10 %.
`Overload_Torque` (in %) must be **above** this value, otherwise the motor will trip
overload protection whenever it sustains high load — which is exactly the normal
operating condition for load-bearing joints like the shoulder.

### Current configuration

| Setting | Motors 1-5 | Motor 6 (gripper) | Notes |
|---|---|---|---|
| Max_Torque_Limit (EPROM) | 1000 (100%) | 500 (50%) | Factory-set |
| Torque_Limit (SRAM) | **900** (90%) | **900** (90%) | Set in robot_control.py connect() |
| Overload_Torque (EPROM) | **95** | 25 | Set via `read_all_params.py --write` |
| Protective_Torque (EPROM) | 20 | 20 | Factory default |

- `servo_test.py` still uses Torque_Limit=1000 (100%) for full-power interactive testing.
- `robot_control.py` (teleop) uses Torque_Limit=900 (90%) as a safety margin.
- Motor 6 (gripper) has Max_Torque_Limit=500 in EPROM, so it's hardware-capped at 50% regardless.

### Voltage matters

The STS3215 needs **6-7.4V** for meaningful torque. Below 6V the servos cannot
move the arm under load. Each motor can draw ~1.5A at stall (2.5A at 7.4V),
so a dedicated 6-7.4V PSU capable of several amps is required.
