# Hector EGO training backend

Training-only execution and state backend for qualifying this isolated chain:

```text
Gazebo /ground_truth/state -> explicit truth odometry adapter -> /uav1/Odometry -> EGO
EGO -> traj_server -> PositionCommand -> Hector Pose/Twist controllers -> Gazebo
```

The default backend launch does not launch PX4, MAVROS, EgoMavrosBridge,
FAST-LIO, Observation C, SAC, an episode scheduler, or a reset coordinator.
Qualification profiles are explicit and write only to an explicit
`runtime_artifacts/` directory.

The separate `hector_training_observation_c.launch` profile adds the existing
Gazebo Livox plugin in its native PointCloud2 mode plus Learning Speed
Observation v2/C. It uses `world -> base_link` truth state, an explicit
`base_link -> mid360_link` mount, causal-at-or-before pose lookup, and
qualification-only temporal-generation clear services. It still launches no
PX4, MAVROS, FAST-LIO, bridge, SAC, or Episode coordinator. The default
backend launch remains observation-free.

The Hector source overlay must be built and sourced before this package. The
runtime launch defaults to `enable_control:=false`; controlled qualification
requires both `enable_control:=true` and `run_qualification:=true`.

The launch accepts only `state_backend:=gazebo_truth_training`. The adapter
preserves Gazebo simulation stamps and p3d pose/twist values after validating
the identity-aligned `world -> base_link` frames. The p3d plugin is configured
without `localTwist`, so linear and angular twist are expressed in Gazebo
`world`, matching EGO's existing planning-odometry velocity assumption. A
latched metadata topic explicitly marks the source as Gazebo truth; it never
claims FAST-LIO provenance. Full-stack FAST-LIO/PX4/MAVROS launches are not
included or modified.

## Training Episode + teleport reset profile

`hector_training_episode_reset.launch` is the formal training-only coordinator
for the first fixed task:

```text
Hover -> ENTRY_GATE -> terminal latch -> synchronized teleport reset -> Hover
```

It uses the existing fixed/mock safety-filter path at a fixed `v_max`; it does
not contain SAC inference. Every active Episode is bound by an exact
`episode_id`, Observation `reset_generation`, stamped action/applied identity,
and a fresh post-barrier EGO `trajectory_id`. The adapter's
`coordinator_managed_goals` mode keeps PositionCommand gated while the fresh
ENTRY B-spline and Observation C are validated, then opens only through the
`activate_trajectory` service. This mode defaults off everywhere else.

GUI qualification leaves Gazebo open after the coordinator finishes:

```bash
roslaunch hector_ego_training_backend hector_training_episode_reset.launch \
  gui:=true episode_count:=5 coordinator_required:=false \
  output_dir:=/absolute/runtime_artifacts/path/data
```

Automated qualification:

```bash
roslaunch hector_ego_training_backend hector_training_episode_reset.launch \
  gui:=false episode_count:=20 coordinator_required:=true \
  require_automated_acceptance:=true \
  output_dir:=/absolute/runtime_artifacts/path/data
```

The coordinator writes `qualification_events.jsonl`, `episode_results.json`,
`reset_results.json`, and `qualification_summary.json`. A reset clears
Observation C first and Observation v2 second, stops both Hector controllers,
pauses Gazebo, teleports with all twists zero, restarts and explicitly engages
the controllers, then requires post-barrier truth, Mid360, five-frame v2,
stamped fixed-action acknowledgement, a fresh EGO B-spline, and valid current-
generation Observation C before the next Episode can start.

## Worksite training profile

`hector_worksite_training_episode_reset.launch` keeps the same Hector model,
controllers and default PID, but selects AstraDroneOpen's current
`simulation/astra_gazebo_worlds/worksite.world` and reviewed UAV1 ingress:

```text
Hover/Home checkpoint = (0.0, 0.0, 3.0)
ENTRY_GATE sector 7, angle 292.5 deg, radius 15 m
ENTRY_GATE            = (-4.3148485145, 5.8522070123, 3.0)
```

Unlike the synthetic four-point qualification map, this profile registers raw
Mid360 points into `world` with source-stamped Gazebo truth odometry and feeds
the resulting `/uav1/training/cloud_registered` directly to EGO. No-return
zero points are rejected with the same 0.2--40 m range contract used by the
existing Observation-v2 input. The 3200-bin surrogate and Observation C are
unchanged and continue to consume the original raw Mid360 stream.

The worksite reset barrier additionally clears the stateless registration
adapter's pose/pending-cloud history. Observation C, registered EGO cloud and
Observation v2 generations must all advance exactly once before teleport and
post-reset warm-up can complete.
