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

The node only accepts the explicit, mutually exclusive `policy/mode: mock` or
`policy/mode: fixed` sources.  It has no SAC/neural inference mode, so an
untrained CNN/MLP cannot affect flight. `policy/network_contract.py`
defines the stable four-channel map, 22-element low-dimensional vector,
feature-fusion, and scalar-output contract that a future reviewed training and
inference backend must implement.

Observation v1 and v2 are independent backends. The v1 implementation above
keeps its `[4,16,48,48]` categorical map contract. `observation/v2/` is a
separate read-only Mid360 surrogate prototype; it never imports a policy or
publishes `v_max`.

## Observation v2: lidar surrogate

Launch one explicitly namespaced instance only after FAST-LIO and the existing
filtered cloud are available:

```bash
roslaunch learning_speed_rl observation_v2_lidar_surrogate.launch \
  namespace:=uav1 \
  cloud_topic:=/uav1/stage3/cloud_registered_filtered \
  odom_topic:=/uav1/Odometry \
  world_frame:=uav1/camera_init \
  body_frame:=uav1/body \
  sensor_frame:=uav1/mid360_link
```

Single-vehicle non-prefixed deployments use `camera_init`, `body` and
`mid360_link`. Configuration is in
`config/observation_v2_lidar_surrogate.yaml`; reference defaults are five
frames, 10 Hz source rate, 0.05 m voxel size, 4.5 degree full-sphere bins,
3200 values and 10 m clipping.

The node matches every cloud's timestamp against a pose buffer. Exact matches
are used when available; otherwise translation is interpolated and orientation
uses quaternion SLERP between bracketing poses. It never substitutes the latest
pose. Each registered cloud is transformed `camera_init -> body(cloud_stamp)`
and fused at time `t` by:

```text
p_body(t) = inverse(T_world_body(t))
            * T_world_body(cloud_stamp)
            * p_body(cloud_stamp)
```

ROS time moving backward clears pose, pending-cloud and history buffers.
Missing, stale, frame-mismatched or unbracketed poses make `valid=false`.

The fixed flat order is elevation-major and azimuth-fast:

```text
flat_index = elevation_index * 80 + azimuth_index
azimuth:  [-180,180) deg, atan2(+Y_left,+X_forward)
elevation:[ -90, 90] deg, +Z_up
body axes: ROS FLU (+X forward,+Y left,+Z up)
```

Per-bin semantics are `0=unknown`, `1=observed_free`,
`2=known_obstacle`. Obstacle bins store nearest distance in `(0,10]`; observed
free bins store `10`; unknown bins store `20-observed_free_range`, disjoint in
`(10,20)`. Separate valid, unknown and semantic topics keep this auditable.

The unknown estimator is explicitly an engineering approximation to the
paper. It transforms calibrated historical Mid360 FoVs into the current body
frame, samples current-bin rays, and caps visibility at the nearest historical
return in the same angular cell. Filtered PointCloud2 has no per-ray miss
records, so this does not claim bit-for-bit reproduction of the unpublished
`20-d_unknown` algorithm.

Read-only outputs (relative to the UAV namespace) are:

- `learning_speed/observation_v2/surrogate`
- `learning_speed/observation_v2/stamped`
- `learning_speed/observation_v2/valid`
- `learning_speed/observation_v2/lidar_valid_mask`
- `learning_speed/observation_v2/unknown_mask`
- `learning_speed/observation_v2/semantic`
- `learning_speed/observation_v2/diagnostics`
- `learning_speed/observation_v2/aligned_history`
- `learning_speed/observation_v2/visualization`

No v2 output is consumed by EGO, MAVROS, PX4, the Speed Adapter or a policy.
The legacy array topics use standard ROS multi-arrays, which do not have headers;
their authoritative sample timestamp and body frame are published in the
diagnostic keys `observation_stamp_sec` and `output_frame`. The aligned cloud
and RViz markers carry the same timestamp/frame in their ROS headers. The
additive `LidarSurrogateStamped` mirror atomically carries the four policy-side
v2 arrays with that exact timestamp/body frame; Observation C consumes this
mirror and does not join latest-value multi-arrays by receipt time.

## Observation C: EGO future-trajectory fusion

Observation C is an independent, read-only feature contract. It subscribes to
the same namespaced `planning/bspline` (`traj_utils/Bspline`) that EGO publishes
to `traj_server`, evaluates its official control points/knots at the v2 sample
timestamp, and transforms future positions into the same current body frame.
It does not publish `v_max`, goals, trajectories, MAVROS or PX4 commands.

```bash
roslaunch learning_speed_rl observation_c_trajectory_fusion.launch \
  namespace:=uav1 \
  cloud_topic:=stage3/cloud_registered_filtered \
  odom_topic:=Odometry \
  trajectory_topic:=planning/bspline \
  trajectory_sampling_mode:=distance
```

Current configurable experiment defaults are 20 samples, 0.25 m spacing and
5.0 m maximum spatial horizon. `distance` uses an adaptive spatial polyline to
approximate future B-spline arc length. `time` remains available for ablation;
in that mode the same spacing parameter is interpreted as seconds while the
maximum spatial horizon is retained.

The atomic `learning_speed/observation_c` message contains v2 surrogate/masks,
future body-frame positions and offsets, actual body-frame velocity, body-frame
position tracking-error vector/norm, the last EGO-applied `v_max` at or before
the observation stamp, trajectory id/start/frame/sampling metadata, and a
strict validity result. Any missing/malformed input, frame/stamp mismatch,
ended trajectory, unavailable velocity, or invalid speed state publishes an
invalid packet and `learning_speed/observation_c/valid=false`.

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

For the single-UAV fixed-speed baseline, use the dedicated immutable `fixed`
source.  It is mutually exclusive with the mock input and with any future RL
backend, and it still passes through `SpeedSafetyFilter` before EGO:

```bash
# Preview is non-controlling.  --control is required for a real run.
scripts/run_sh/fixed_speed_baseline.sh --v-max 0.12 --run-id preview_012
scripts/run_sh/fixed_speed_baseline.sh --control --v-max 0.12 --run-id v012_r01
```

The audited levels are 0.08, 0.12, 0.16 and 0.20 m/s.  Every run keeps EGO's
static ceiling at 0.20 m/s and changes only the fixed source.  Per-run JSON,
CSV, bag and logs are written below
`runtime_artifacts/fixed_speed_baseline/runs/<run_id>/`; the wrapper rebuilds
`baseline_runs.csv`, `baseline_summary.csv`, `baseline_summary.json` and the
training-data field-availability audit at the baseline root.

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
- `learning_speed/action_stamped`: additive atomic audit mirror of the raw and
  filtered values with their policy-cycle ROS timestamp; it has no subscriber
  in EGO or the control path.
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

## Frozen SAC transition contract and manual calibration

`training/data_contract.py` freezes `learning_speed_sac_transition_v1.0` as:

```text
state_t -> requested_v_max -> filtered_v_max -> applied_v_max
        -> state_t+1 -> reward -> terminated/truncated
```

The v1 policy input contains exactly `lidar_surrogate[3200]`,
`future_positions_body[20][3]`, `actual_velocity_body[3]`,
`tracking_error_body[3]` and `previous_applied_v_max`. Mission/planner state,
clearance/clutter metrics, lidar masks/semantic and diagnostics are provenance
or calibration-only fields and are never returned by `PolicyStateV1.policy_input()`.

A transition candidate is accepted only when `state_t` is a valid atomic
Observation C received before the requested action and its trajectory
id/start/frame equals the newest official `planning/bspline` received before
that action. `state_t+1` is the first newer valid Observation C received after
the EGO applied acknowledgement. The adapter publishes requested and filtered
values atomically on the additive `learning_speed/action_stamped` audit topic;
the EGO applied acknowledgement remains headerless and uses its local ROS
callback receipt time. Calibration candidates keep `reward=null`,
`reward_defined=false` and `training_ready=false`.

Attach the read-only manual calibration recorder to an already running stack:

```bash
scripts/run_sh/learning_speed_calibration.sh --namespace uav1 --run-id manual_001
```

The stack must have been started with Learning Speed explicitly enabled; the
project-wide default remains disabled. If Observation C is already running,
add `--observation-c-running`. Output is written only below
`runtime_artifacts/learning_speed/calibration/<run_id>/`:

- `calibration_samples.csv`: Observation C validity, body speed/tracking,
  action chain, obstacle/clutter diagnostics, planner/mission and safety state;
- `observation_diagnostics.jsonl`: masks/semantic, diagnostics and the existing
  raw/inflated clearance representations kept outside the policy input;
- `transition_candidates.jsonl`: causally checked contract records with no
  reward and therefore not training-ready;
- `planner_failure_episodes.csv`: failure intervals and recovered/unrecovered;
- `run_summary.json`: mission result, safety terminal and validity totals.

The recorder finalizes one second after the latched mission-done signal, or on
normal ROS shutdown/`Ctrl+C` for a deliberately truncated manual run.

For the controlled UAV1 two-environment fixed-speed calibration matrix, the
batch wrapper uses the same task configuration within each environment and
changes only the immutable fixed source value:

```bash
scripts/run_sh/learning_speed_manual_batch.sh
```

It runs `0.30, 0.50, 0.75, 1.00, 1.25, 1.50 m/s` once in environment A
(`outdoor_village.world`, UAV1 spawn `(-14, 0)`) and once in environment B
(`worksite.world`, UAV1 spawn `(0, 0)`).  A second attempt is allowed only when
the first attempt ended without a mission/safety terminal or planner/safety
failure evidence.  Experimental failures are retained without retry.  The
result table and data-quality-only analysis are written to
`runtime_artifacts/learning_speed/calibration/manual_calibration_runs.csv` and
`manual_calibration_report.md`; neither script defines a reward or starts SAC.

The obstacle density is the fraction of 3200 angular bins labelled known
obstacle, and the clutter count is the corresponding occupied-bin count. They
are diagnostic proxies, not object counts, physical volume, or a replacement
for the project's existing clearance semantics.
