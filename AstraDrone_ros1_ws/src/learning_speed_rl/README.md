# learning_speed_rl

This ROS1 package owns the Learning Speed observation, policy and speed-adapter
boundary. It does **not** own waypoints, path planning, collision avoidance,
PX4 control, or any tower/swarm mission state.

The deployed v1 path is:

```text
EGO inflated occupancy + odometry + PositionCommand + local goal
  -> fixed observation contract
  -> MockSpeedPolicy
  -> finite validation + reviewed min/max clamp
  -> learning_speed/action_stamped (request identity + requested/filtered)
  -> EGO dynamic velocity-limit interface
  -> learning_speed/applied_v_max_stamped (same request identity)
```

For every legal in-range action, `requested_v_max == filtered_v_max`; the
adapter does not apply slew, low-pass, hysteresis, or maximum-step shaping.
EGO compares consecutive applied constraints at the 10 Hz outer-loop rate. It
forces one extra replan only when `delta_v < -0.3 m/s` or
`delta_v > +0.5 m/s`; otherwise EGO continues using its native replanning
rules without a Learning-Speed-induced replan.

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
rostopic pub -1 /uav1/learning_speed/mock_v_max learning_speed_rl/SpeedRequestStamped \
  "{header: {stamp: now}, version: learning_speed_request_v1.0, episode_id: manual_mock, step_index: 0, request_id: 1, requested_v_max: 0.08}"
```

The EGO launch must separately opt into its dynamic interface. The audited
three-UAV launch does this when its `learning_speed_enabled` argument is true.
The adapter maximum is always overridden from that launch's static `max_vel`,
so the policy can reduce and restore the reviewed ceiling but cannot exceed it.

For the single-UAV fixed-speed baseline, use the dedicated immutable `fixed`
source.  It is mutually exclusive with the mock input and with any future RL
backend, and it still passes through the finite/range-only `SpeedSafetyFilter`
before EGO:

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
- `learning_speed/mock_v_max` (`SpeedRequestStamped`): versioned mock/Episode
  request with `episode_id`, `step_index` and monotonic `request_id`.
- `learning_speed/applied_v_max` (`std_msgs/Float64`): compatibility state
  value consumed by Observation C and existing scalar diagnostics; never used
  for Episode action pairing.

Publications:

- `learning_speed/raw_v_max`: raw policy request.
- `learning_speed/v_max`: scalar compatibility mirror of the filtered value.
- `learning_speed/action_stamped`: the sole EGO dynamic-limit input, carrying
  the request identity and atomic requested/filtered values.
- `learning_speed/applied_v_max_stamped`: published by EGO after application;
  it echoes the identity and is the sole formal action-pairing acknowledgement.
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

`training/` contains environment/artifact boundaries, the versioned transition
contract, the reviewed framework-neutral Stage 1 reward, and an optional
training-only PyTorch SAC learner;
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

`training/data_contract.py` freezes `learning_speed_sac_transition_v1.3` as:

```text
state_t -> requested_v_max -> filtered_v_max -> applied_v_max
        -> state_t+1 -> reward -> terminated/truncated
```

The v1 policy input contains exactly `lidar_surrogate[3200]`,
`future_positions_body[20][3]`, `actual_velocity_body[3]`,
`tracking_error_body[3]` and `previous_applied_v_max`. Mission/planner state,
clearance/clutter metrics, lidar masks/semantic and diagnostics are provenance
or calibration-only fields and are never returned by `PolicyStateV1.policy_input()`.

Each v1.3 transition also carries a separate
`learning_speed_progress_reward_context_v1.0`. It pairs the latest headerless
`tower_mission/progress` and mission-state receipts available at `state_t` and
`state_t+1`, binds both snapshots to the corresponding Observation C receipt
times, and records run/episode provenance plus `P_t`, `P_t_plus_1` and
`Delta_P`. This context remains diagnostics/evaluation evidence; it is not a
policy input and is not used by the Stage 1 reward.

A transition candidate requires a valid atomic `state_t` received before the
requested action and a strictly newer valid `state_t+1` received after the EGO
applied acknowledgement. A native EGO replan may publish a new B-spline between
`state_t` and the action without invalidating the transition. The trajectory
identity embedded in each Observation C and the latest official trajectory at
action time are both retained as independent provenance; equality is not a
validity condition. The adapter publishes requested and filtered values
atomically on `learning_speed/action_stamped`; EGO echoes the same identity on
`learning_speed/applied_v_max_stamped`. Episode v0.1 and the online recorder
pair action/application by identity equality; timestamps remain causal and
future-leak checks rather than identity guesses. Calibration candidates keep `reward=null`,
`reward_defined=false` and `training_ready=false` only in the frozen legacy
artifacts. The current recorder loads `config/stage1_reward.yaml`, evaluates
the causal `state_t` signals through `LearningSpeedReward.evaluate()`, and binds a
finite, versioned reward to every valid, same-episode, non-truncated candidate.

## Paper-guided Reward v3

`training/reward.py` implements `astradrone_paper_guided_reward_v3.0` as a
paper-guided adaptation of Eq. (6)--(11) in *Learning Speed
Adaptation for Flight in Clutter*. It is not an exact reproduction of the
paper's unpublished feature coefficients or complete lambda values. Select it
with `config/stage1_reward.yaml`. The maintained default is
`reward.mode: stage_1`; `stage_2` is implemented but not authorized for
training.

Stage 1 uses only:

```text
r_stage1 = r_speed_stage1 + r_smoothing + r_error + r_danger
```

The continuous N/D/Unknown complexity feature keeps the reviewed
`6.0/2.5 m`, `0.040/0.080`, and `1.75/1.25/0.75/1.25 m/s` anchors. Reward v3
replaces the v2 `max(q_nearest,q_density)` owner with the calibrated continuous
fusion `1-(1-q_nearest)^0.46*(1-q_density)^0.54`. Its three Eq. (10) branches
use quadratic Bernstein weights, so the speed slope changes continuously from
positive through zero to negative. Low/Medium/High remain diagnostics rather
than reward switches. Smoothing uses current and previous applied `v_max`.
Speed and danger use the causal actual-speed norm, not the action constraint.
Tracking error reuses the existing Observation C body-frame error norm and is
clipped before squaring. Danger is nonzero only for the explicit frozen
dangerous terminal. Progress remains outside Reward. The existing 1.0 m
continuous tracking safety gate is unchanged.

The active offline candidate is `lambda_error=2.0`, `e_max=0.40 m`; it is not a
published paper value and does not authorize formal training. Reward v3 has
offline replay/landscape evidence only and still requires bounded runtime
qualification. Stage 2 changes only `r_speed` to
`lambda_speed3 * actual_speed`. The paper's CNN-freezing step is not copied
because this package does not use the same CNN architecture.

The transition contract can bind a valid Stage 1 evaluation with all component
terms and marks only that resulting record training-ready. Invalid Observation,
episode-boundary and truncated inputs cannot produce a valid reward transition.
The recorder remains read-only with respect to flight/control and now performs
that binding online; it publishes no reward topic and starts no SAC or policy.
Frozen historical calibration artifacts remain unchanged and reward-null.

Design and offline replay evidence are consolidated in the repository-root
`AstraDroneOpen_项目技术演进与LearningSpeed阶段汇总.md`.

## AstraDroneEnv Episode v0.1 and 10 Hz causal scheduler

`training/astra_drone_env.py` owns one continuous UAV1 Episode and publishes
policy requests on a fixed 0.1 s ROS/simulation-time grid. It does not wait for
the previous transition to close before publishing the next request. Each
in-flight step retains its own `episode_id`, zero-based `step_index`, state,
request marker, atomic action, applied acknowledgement, trajectory provenance
and post-hold Observation:

```text
strictly causal valid state_t
 -> 10 Hz SpeedRequestStamped(episode_id, step_index, request_id)
 -> SpeedActionStamped(same identity)
 -> EGO SpeedAppliedStamped(same identity)
 -> first unused valid Observation C after applied receipt + 0.1 s
 -> existing LearningSpeedReward.evaluate + SacTransitionV1
```

The pending queue consumes every identity and next Observation at most once.
Action/application matching is direct `(episode_id, request_id)` equality with
`step_index` and values cross-checked; `state_t` must predate the action, while `state_t+1` must have a
strictly newer source stamp and receipt no earlier than the hold boundary.
Native mapping/planning/replanning stays asynchronous, and state/action/next
trajectory identities remain independent provenance rather than an equality
gate.

Episode mode requires SpeedAdapter `timing/mock_request_driven=true`. The
adapter emits one initial reviewed constraint before active-motion readiness,
then each Episode mock request immediately executes the existing
MockSpeedPolicy -> range-only SpeedSafetyFilter -> `SpeedActionStamped` -> EGO
chain. Its independent mock timer is disabled, so there is one formal 10 Hz
outer-loop owner. Fixed mode and ordinary mock mode retain their default timer
behavior. Scalar `v_max/applied_v_max` topics remain state mirrors, not a
second formal pairing path.

`AstraDroneEpisodeConfig` fixes the policy period to 0.1 s and supports a
positive maximum step count, maximum ROS duration, or both. Episode start
requires a valid Observation C, mission active, bridge `TRACK_EGO`, planner
`EXEC_TRAJ` and configured minimum actual speed. Existing mission success,
mission failure, collision proxy, EGO emergency and continuous tracking gate
are the only termination sources. A configured limit reached without one of
those signals sets Episode `terminated=false`, `truncated=true` after the last
fully closed transition. The frozen v1.3 transition remains non-truncated and
reward-defined; the Episode boundary is recorded separately so Stage 1 reward
semantics are not changed.

`reset()` still raises `NotImplementedError`; reset remains owned by the
training-only Hector coordinator. `AstraDroneEnv` itself does not own a SAC
learner, Replay Buffer, checkpoint, teleport, normalization or multi-UAV RL. The old
blocking `step()` API, sequential wait helper, 0.2 s fallback and unused
pre-step observation timeout were removed. Historical 0.5/1.0 s and blocking
0.1 s runtime artifacts remain unchanged as evidence.

The action-identity revalidation closed 100/100 scheduled requests at exactly
10 Hz with 100/100 action and stamped applied acknowledgements, request IDs
1..100, zero timeout/causal mismatch/deadline drop, finite reward for every
transition and a real `max_episode_steps` truncated boundary. The later full
mission ended in a preserved planner-failure landing after the Episode window;
it is not rewritten as Episode termination. The source report and its hash are
mapped in `AstraDroneOpen_项目技术演进与LearningSpeed阶段汇总.md`.

## Training-only SAC integration

`training/sac.py` and `training/sac_replay.py` provide the optional SAC
training path. They are not imported by `speed_adapter_node.py` and do not add
a neural inference mode to flight/full-stack launches. The worksite entry is:

```bash
cd /home/yanzu/AstraDroneOpen
scripts/run_sh/learning_speed_sac_training.sh
```

Every formal training run uses a unique directory below
`runtime_artifacts/rl_training/`; the runner refuses to reuse a prior training
directory. The operator script performs preflight, creates the run directory,
launches the unchanged formal SAC stack, saves the full console, and follows
key runner/coordinator Episode, transition, reset, checkpoint and failure logs
in the same terminal. Independent evaluation reads a checkpoint from that training run
and writes to a separate `runtime_artifacts/rl_evaluation/<EVAL_ID>/` directory.
The existing `runtime_artifacts/sac_python_packages/` directory remains the
Python dependency location and is not a training run directory.

It uses the existing request-driven adapter, `AstraDroneEnv` causal scheduler,
Stage 1 reward, and Hector Episode/reset coordinator. The coordinator's
`action_owner` defaults to `fixed`; the SAC launch explicitly selects `sac`,
so fixed and SAC providers cannot own one Episode simultaneously. During
reset, a separate `__sac_warmup_generation_N__` fixed bootstrap restores only
the cleared applied-speed scalar and never enters a formal Episode or replay.

The Actor and twin critics are 3267→256→256 MLPs. Actor output is a
tanh-squashed one-dimensional normalized action; live SpeedSafetyFilter params
define its affine `v_max` mapping. Learner-side normalization is disabled in
the formal config. The current exploration-stability profile uses Actor LR
`1e-5`, 100 critic-only startup updates and log-std `[-3,-1]`; the launch
defaults to the sole maintained profile, `config/sac_training_v1.yaml`. The
first formal run is 10,000
valid transitions from an empty Replay Buffer, with checkpoints only at 5,000
and 10,000 and no evaluation inside training. Training disables the fixed
Episode count and continues through early terminal/reset boundaries until the
valid-transition count is exactly 10,000; the per-Episode 500-step ceiling is
not a normal training stop. Worksite training resets use the
reproducible, audited +/-1 m XY nominal-Hover square; evaluation uses fixed
nominal Hover and neither creates training Replay nor updates networks.
PyTorch is an optional training
dependency pinned in `requirements-sac.txt`; fixed/mock and full-stack defaults
remain usable without it. The consolidated runtime history and current
training boundary are in the repository-root
`AstraDroneOpen_项目技术演进与LearningSpeed阶段汇总.md`.

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
- `transition_candidates.jsonl`: causally checked records with a finite Stage 1
  reward and versioned components for valid, same-episode, non-truncated
  transitions; progress context remains diagnostic-only;
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
