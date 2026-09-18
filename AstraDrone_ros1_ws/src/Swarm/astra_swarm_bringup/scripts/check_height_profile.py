#!/usr/bin/env python3
"""Expand launch parameters and check height contracts without starting ROS nodes."""
import json
import math
import sys
from pathlib import Path


def validate(values):
    errors = []
    rows = []

    def require(ok, description):
        if not ok:
            errors.append(description)

    for uid in (1, 2, 3):
        m = '/uav{}/tower_mission/'.format(uid)
        b = '/uav{}/ego_mavros_bridge/'.format(uid)
        e = '/uav{}/drone_{}_ego_planner_node/'.format(uid, uid - 1)
        top = values[m + 'mission/inspection_top_height']
        offsets = values[m + 'mission/layer_offsets']
        layers = [top + offset for offset in offsets]
        staging = values[m + 'staging/height']
        takeoff = values[b + 'takeoff_height']
        ascent = values[m + 'staging/ascent_intermediate_heights']
        lo = values[m + 'mission/minimum_height']
        hi = values[m + 'mission/maximum_height']
        ceiling = values[m + 'mission/virtual_ceil_height']
        ground = values[e + 'grid_map/ground_height']
        ego_ceiling = values[e + 'grid_map/virtual_ceil_height']
        bridge_lo = values[b + 'min_relative_height']
        bridge_hi = values[b + 'max_relative_height']
        numbers = [top, staging, takeoff, lo, hi, ceiling, ground,
                   ego_ceiling, bridge_lo, bridge_hi] + layers + ascent
        require(all(math.isfinite(x) for x in numbers), 'UAV{} finite heights'.format(uid))
        require(offsets and offsets[0] == 0 and all(a > z for a, z in zip(layers, layers[1:])),
                'UAV{} descending layers beginning at top'.format(uid))
        require(lo <= staging < top, 'UAV{} staging below top'.format(uid))
        sequence = [staging] + ascent + [top]
        require(all(a < z for a, z in zip(sequence, sequence[1:])),
                'UAV{} strictly ascending intermediate heights'.format(uid))
        targets = layers + [staging, takeoff, values[b + 'return_height']]
        require(all(lo <= z <= min(hi, ceiling) for z in targets),
                'UAV{} mission envelope'.format(uid))
        require(all(bridge_lo <= z <= bridge_hi and ground < z < ego_ceiling for z in targets),
                'UAV{} bridge/EGO envelope'.format(uid))
        require(values[m + 'mission/transit_height'] == takeoff,
                'UAV{} transit/takeoff pairing'.format(uid))
        if values[m + 'low_altitude/enabled']:
            require(top == takeoff == values[m + 'low_altitude/entry_height'],
                    'UAV{} same-height entry/transit pairing'.format(uid))
        require(values[m + 'candidate/height_offsets_m'] == [0.0],
                'UAV{} nominal-height candidates'.format(uid))
        require(values[m + 'return/descent_intermediate_heights'] and
                all(math.isfinite(x) for x in values[m + 'return/descent_intermediate_heights']),
                'UAV{} finite nonempty descent list'.format(uid))
        require(lo <= values[m + 'recovery/recovery_height'] <=
                values[m + 'mission/recovery_height_max'] <= min(ceiling, bridge_hi),
                'UAV{} recovery envelope'.format(uid))
        require(lo <= values[m + 'return_egress/transit_height'] <= min(ceiling, bridge_hi),
                'UAV{} return egress envelope'.format(uid))
        # 0.5 m/s is the unchanged bridge position slew-rate default.
        timeout = values.get(b + 'takeoff_timeout', 60.0)
        require(timeout > takeoff / values.get(b + 'max_position_rate', 0.5),
                'UAV{} takeoff time budget'.format(uid))
        require(values[e + 'grid_map/map_size_z'] + ground > ego_ceiling,
                'UAV{} map Z covers ceiling'.format(uid))
        rows.append(dict(uav=uid, top=top, takeoff=takeoff, staging=staging,
                         entry=top, ascent=ascent, layers=layers,
                         mission_range=[lo, hi], ego_range=[ground, ego_ceiling],
                         bridge_range=[bridge_lo, bridge_hi], takeoff_timeout=timeout))
    manager = '/astra_swarm_manager/'
    require(values[manager + 'mission_heights'] == [r['top'] for r in rows], 'manager mission heights')
    require(values[manager + 'takeoff_heights'] == [r['takeoff'] for r in rows], 'manager takeoff heights')
    for uid in (1, 2, 3):
        require(values[manager + 'layer_offsets'] == values['/uav{}/tower_mission/mission/layer_offsets'.format(uid)],
                'manager layer offsets UAV{}'.format(uid))
    require(values[manager + 'formation_role_order'] == [3, 2, 1], 'role order')
    for owner in ('astra_swarm_safety', 'astra_swarm_rviz_diagnostics'):
        key = '/' + owner + '/mission_heights'
        if key in values:
            require(values[key] == [r['top'] for r in rows], owner + ' mission heights')
    if errors:
        raise ValueError('; '.join(errors))
    return rows


def expand(launch, arguments):
    import roslaunch
    path = Path(launch)
    if not path.is_file():
        path = Path(__file__).resolve().parents[1] / 'launch' / launch
    # Explicitly prevent simulation inclusion; loading config starts no nodes.
    args = [a for a in arguments if not a.startswith('start_sim:=')]
    config = roslaunch.config.load_config_default(
        [(str(path), args + ['start_sim:=false'])], None, assign_machines=False)
    return {k: p.value for k, p in config.params.items()}


def main():
    try:
        rows = validate(expand(sys.argv[1], sys.argv[2:]))
    except (ValueError, KeyError, IndexError) as error:
        print('Height contract FAIL: {}'.format(error), file=sys.stderr)
        return 2
    print('Height contract PASS (parameter expansion only; no flight evidence)')
    print(json.dumps(rows, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
