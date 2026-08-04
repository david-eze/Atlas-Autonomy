# AI Semantic Perception Layer

## Architecture & Responsibilities

```
Camera Sensor -> Object Detection Model Wrapper -> Bounding Box + Class ID
                                                         |
                                                         v
Nav Goal <- Semantic Location Lookup Table <- 3D World Model Marker
```

## Architectural Isolation: AI vs Deterministic Safety
A critical design decision in this software stack is the strict separation of AI perception from safety-critical motion control:

1. **AI Perception Domain**: Responsible for high-level semantic identification ("workstation", "charging_station", "pallet", "person"). Output is strictly restricted to landmark annotations and coordinate target suggestions.
2. **Deterministic Domain**: Nav2 costmaps, A* path planning, EKF state estimation, and the independent Safety Monitor execute all low-level motion commands (`/cmd_vel`).

**The AI perception system can NEVER directly publish velocity commands or override safety constraints.**

## Detected Object Pipeline
1. `detection_node` processes camera stream, detecting target classes (person, chair, table, box, doorway, charger).
2. Computes object 3D centroids in `map` frame using camera depth intrinsic parameters and TF lookup (`map -> camera_link`).
3. Registers landmark targets into `semantic_map` database for semantic navigation lookup.
