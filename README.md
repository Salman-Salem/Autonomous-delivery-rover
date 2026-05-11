# Autonomous-delivery-rover

A combined 6-DOF robotic arm and 2-wheel drive vehicle built on ESP32. The system picks an object from a fixed location, drives 3–4 meters to a drop zone, places the object, then returns the arm to home — all autonomously in a single mission.

## Hardware

### ESP32 Microcontroller
- Single ESP32 board controls both the arm and vehicle
- MPU6050 gyroscope for straight-line driving correction
- I2C: SDA = GPIO21, SCL = GPIO22

### Robotic Arm — 6 Joints

| Joint | Function      | GPIO | Range     | Home |
|-------|---------------|------|-----------|------|
| J0    | Yaw (base)    | 13   | 0–180°    | 90°  |
| J1    | Shoulder      | 32   | 20–160°   | 90°  |
| J2    | Elbow         | 14   | 20–180°   | 90°  |
| J3    | Wrist         | 27   | 0–180°    | 90°  |
| J4    | Gripper Left  | 26   | 0–90°     | 45°  |
| J5    | Gripper Right | 25   | 0–90°     | 0°   |

> Gripper is mirrored — Left opens at 0°, Right opens at 45°.

### Vehicle — DC Motors

| Signal | GPIO |
|--------|------|
| IN1    | 18   |
| IN2    | 19   |
| ENA    | 4    |
| IN3    | 23   |
| IN4    | 5    |
| ENB    | 17   |

### Power
- 3S LiPo battery
- Servos should be powered independently from the ESP32 and motors

---


## How It Works

### Mission Phases

```
[STARTUP] Gyro calibration → Arm ramps to HOME
     ↓
[PHASE 1] Sequence A — Arm picks object from PICK position, swings to DROP hover
     ↓
[PAUSE]   1 second non-blocking wait
     ↓
[PHASE 2] Vehicle drives forward ~3–4 m (6 seconds, gyro-corrected)
     ↓
[PAUSE]   1 second non-blocking wait
     ↓
[PHASE 3] Sequence B — Arm places object at original PICK position, returns HOME
     ↓
[DONE]
```

### Arm Motion System

The arm uses a two-stage motion controller running at 50 Hz:

1. **Velocity profiler** — moves a waypoint `wpt` toward `tgt` at a limited speed (`VEL_LIMIT` °/s per joint)
2. **PID controller** — drives the actual servo position `cur` to track `wpt`

This produces smooth, controlled motion. A step in the sequence only advances when both the profiler and servo have reached the target (`allAtTarget()`).

### Vehicle Straight-Line Control

The MPU6050 gyroscope measures yaw drift. A PID controller corrects left/right motor speeds to keep the vehicle driving straight.


## Known Limitations

- No position feedback — servos are open-loop; physical position is assumed to match commanded position
- Hardcoded pick/drop coordinates — object must be placed precisely each run
- Gyro drift accumulates over long drives — 6 seconds is within acceptable range
