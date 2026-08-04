# Nav2 Stack Configuration & Dynamic Replanning

## Costmap Architecture

### Global Costmap (`/global_costmap/costmap`)
- **Static Layer**: Ingests `/map` topic from SLAM Toolbox or AMCL map server.
- **Obstacle Layer**: Ingests live `/scan` LaserScan messages to mark dynamic obstacles.
- **Inflation Layer**: Expands obstacle boundaries with cost decay function ($r_{\text{inflation}} = 0.55\text{ m}$, cost scaling factor $= 3.0$).

### Local Costmap (`/local_costmap/costmap`)
- **Size**: $3.0\text{ m} \times 3.0\text{ m}$ rolling window centered on `base_link`.
- **Voxel/Obstacle Layer**: High frequency ($10\text{ Hz}$) live LiDAR observations for dynamic obstacle avoidance.
- **Inflation Layer**: Prevents controller trajectory collisions.

## Dynamic Obstacle Handling & Replanning
When an obstacle appears on the local costmap blocking the active global path:
1. Local costmap marks the obstacle cells as lethal ($254$).
2. Trajectory controller evaluates forward paths; detects collision on current global trajectory.
3. Controller requests global planner replan via Nav2 Behavior Tree.
4. Global planner calculates alternate collision-free route around the dynamic obstacle.

## Nav2 Behavior Tree Integration
```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <PipelineSequence name="NavigateWithReplanning">
      <RateController hz="1.0">
        <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridSearch"/>
      </RateController>
      <FollowPath path="{path}" controller_id="FollowPath"/>
    </PipelineSequence>
  </BehaviorTree>
</root>
```
