# learning_speed_rl

This ROS1 package owns the Learning Speed observation, policy and speed-adapter
boundary. It does **not** own waypoints, path planning, collision avoidance,
PX4 control, or any tower/swarm mission state.

The deployed v1 path is:

```text
EGO inflated occupancy + odometry + PositionCommand + local goal
  -> fixed observation contract
  -> MockSpeedPolicy
  -> clamp -> hysteresis -> slew limit -> low-pass filter
  -> learning_speed/v_max (std_msgs/Float64)
  -> EGO dynamic velocity-limit interface
```

The node only accepts `policy/mode: mock`. This is intentional: an untrained
CNN/MLP is not permitted to affect flight. `policy/network_contract.py`
defines the stable four-channel map, 22-element low-dimensional vector,
feature-fusion, and scalar-output contract that a future reviewed training and
inference backend must implement.

## Run the adapter

Inside a UAV namespace, the defaults are:

```bash
roslaunch learning_speed_rl speed_adapter.launch namespace:=uav1 static_max_vel:=0.20
rostopic pub -1 /uav1/learning_speed/mock_v_max std_msgs/Float64 "data: 0.08"
```

The EGO launch must separately opt into its dynamic interface. The audited
three-UAV launch does this when its `learning_speed_enabled` argument is true.
The adapter maximum is always overridden from that launch's static `max_vel`,
so the policy can reduce and restore the reviewed ceiling but cannot exceed it.

## ROS interface

Subscriptions (relative to the vehicle namespace):

- `Odometry` (`nav_msgs/Odometry`): pose; velocity and acceleration are
  filtered finite differences of pose after the first sample.
- `stage3/occupancy_inflate` (`sensor_msgs/PointCloud2`): current EGO inflated
  occupied geometry.
- `planning/pos_cmd` (`quadrotor_msgs/PositionCommand`): same-frame desired
  state and tracking-error reference.
- `move_base_simple/goal` (`geometry_msgs/PoseStamped`): local goal context
  only when its frame exactly matches odometry; cross-frame subtraction is
  rejected and reported rather than silently assuming an identity TF.
- `learning_speed/mock_v_max` (`std_msgs/Float64`): mock policy command.
- `learning_speed/applied_v_max` (`std_msgs/Float64`): EGO acknowledgement.

Publications:

- `learning_speed/raw_v_max`: raw policy request.
- `learning_speed/v_max`: filtered request sent only to EGO.
- `learning_speed/observation/low_dim`: normalized 22-element vector.
- `learning_speed/observation_ready`: strict readiness for future RL.
- `learning_speed/diagnostics`: source freshness, tensor contract, request and
  EGO acknowledgement.

There are deliberately no MAVROS, PX4-control, goal, trajectory, planning,
collision, or swarm-coordination publishers in this package.

## Observation contract

The map tensor is `float32 [4, Z, Y, X]`, centered on the UAV, with channel
order:

1. free
2. occupied
3. unknown
4. on_trajectory

Default crop size is 24 x 24 x 8 m at 0.5 m resolution, producing
`[4, 16, 48, 48]`. It is UAV-centered but remains aligned to the fixed EGO
planning-frame axes; it does not silently rotate with vehicle yaw. The current
EGO point-cloud export contains inflated
occupied cells only: it does not distinguish observed-free from unknown. The
voxelizer therefore marks every unproven cell unknown, never invents free
space, and keeps `map_semantics_complete=false`. One current command sample is
placed in the reserved trajectory channel, but this is not the paper's full
preplanned trajectory, so `trajectory_context_complete=false`. Consequently
`observation_ready` is correctly false even while the mock engineering chain
is healthy.

The low-dimensional vector order is fixed in `LOW_DIM_FIELDS`:

- position xyz
- velocity xyz
- acceleration xyz
- position tracking error xyz
- local-goal delta xyz
- desired velocity xyz
- desired acceleration xyz
- previous filtered `v_max`

Normalization scales are configured in `config/speed_adapter.yaml` and are
part of the future model version contract.

## Training and artifacts

`training/` contains only environment and artifact-boundary contracts;
`inference/` contains fail-closed reviewed-model loading. Formal flight launch
files never import training code.

All checkpoints, TensorBoard logs, CSV files, plots, episodes and evaluation
records must be written under:

```text
runtime_artifacts/learning_speed/
  checkpoints/
  logs/
  plots/
  evaluation/
```

Only a separately reviewed, selected inference model may later be copied to
`models/`. No model is shipped or selected by this integration.
