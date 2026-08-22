# Hector EGO training backend

Training-only execution and state backend for qualifying this isolated chain:

```text
Gazebo /ground_truth/state -> explicit truth odometry adapter -> /uav1/Odometry -> EGO
EGO -> traj_server -> PositionCommand -> Hector Pose/Twist controllers -> Gazebo
```

It does not launch PX4, MAVROS, EgoMavrosBridge, FAST-LIO, Observation C, SAC,
an episode scheduler, or a production reset implementation. The qualification
runner is a one-shot test instrument and writes only to an explicit
`runtime_artifacts/` directory.

The separate `hector_training_observation_c.launch` profile adds the existing
Gazebo Livox plugin in its native PointCloud2 mode plus Learning Speed
Observation v2/C. It uses `world -> base_link` truth state, an explicit
`base_link -> mid360_link` mount, causal-at-or-before pose lookup, and
qualification-only temporal-generation clear services. It still launches no
PX4, MAVROS, FAST-LIO, bridge, SAC, Episode scheduler, or production reset
coordinator. The default backend launch remains observation-free.

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
