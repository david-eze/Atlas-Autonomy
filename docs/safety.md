# Deterministic Navigation Safety Layer

## Overview
The safety layer acts as a fail-safe supervisor operating independently of Nav2, SLAM, and AI perception nodes.

## Safety Monitor Architecture (`robot_safety/safety_monitor.py`)
- **Direct LiDAR Access**: Subscribes directly to raw `/scan` messages.
- **Direct Velocity Interception**: Monitors commanded forward velocity ($v_x > 0$).
- **Safety Rule**:
  $$\text{If } \min(d_{\text{obstacle}}) < 0.30\text{ m and } v_x > 0 \implies \text{Override } v_x = 0.0$$
- Operates at $20\text{ Hz}$ independently of planner or controller loop execution.

## Emergency Stop & Override Flow
```
Nav2 cmd_vel ----> Safety Monitor -------------> Robot Actuators
                         ^
LiDAR /scan -------------| (If distance < 0.30m -> ZERO VELOCITY)
```

If Nav2 fails to compute a collision-free path or experiences thread starvation, the safety monitor immediately halts motor output to prevent physical collisions.
