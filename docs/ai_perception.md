# AI Semantic Perception Layer

## Architecture & Responsibilities
```
Camera Sensor -> Object Detection Model Wrapper -> Bounding Box + Class ID
                                                       |
                                                       v
Nav Goal <- Semantic Location Lookup Table <- 3D World Model Marker
```
## Keeping AI Out of the Safety Path
One thing this stack is strict about: AI perception never touches motion control directly. These two things live in completely separate domains.

1. **AI Perception**: figures out what stuff is ("workstation", "charging_station", "pallet", "person"). It only ever hands off landmark annotations and suggested coordinates, nothing else.
2. **Deterministic Domain**: Nav2 costmaps, A* planning, EKF state estimation, and the Safety Monitor (running independently) are what actually own `/cmd_vel`.

**The perception model can't publish velocity commands or touch safety constraints.** No exceptions carved out for this.

## Detected Object Pipeline
1. `detection_node` reads the camera stream and picks out the classes it cares about: person, chair, table, box, doorway, charger.
2. For each detection, it computes a 3D centroid in the `map` frame using the depth intrinsics plus a TF lookup (`map -> camera_link`).
3. Landmarks get pushed into the `semantic_map` DB so nav can look them up later.
