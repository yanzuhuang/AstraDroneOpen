#ifndef ASTRA_TOWER_MISSION_MISSION_COMPLETION_H_
#define ASTRA_TOWER_MISSION_MISSION_COMPLETION_H_

namespace astra_tower_mission {

// Orbit permission controls when coordinated vehicles may leave staging.  It
// must not disable bookkeeping when that coordination gate is not configured.
inline bool shouldReleaseOrbitBookkeeping(bool permission_required,
                                          bool permission_granted,
                                          bool first_waypoint_reached,
                                          bool already_released) {
  if (already_released) return false;
  return permission_required ? permission_granted : first_waypoint_reached;
}

struct MissionCompletionSignals {
  bool mission_success{false};
  bool mission_failure{false};
  bool mission_done{false};
  bool orbit_complete{false};
};

// DONE describes a completed return/landing sequence.  It is a mission
// success only when the inspection completed and no task failure initiated
// that return.  ERROR is always terminal failure.
inline MissionCompletionSignals missionCompletionSignals(
    bool done_state, bool error_state, bool failure_latched,
    bool orbit_complete) {
  MissionCompletionSignals signals;
  signals.mission_done = done_state || error_state;
  signals.orbit_complete = orbit_complete;
  signals.mission_success =
      done_state && !failure_latched && orbit_complete;
  signals.mission_failure =
      failure_latched || error_state ||
      (done_state && !signals.mission_success);
  return signals;
}

}  // namespace astra_tower_mission

#endif  // ASTRA_TOWER_MISSION_MISSION_COMPLETION_H_
