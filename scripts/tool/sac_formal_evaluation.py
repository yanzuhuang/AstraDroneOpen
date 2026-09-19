#!/usr/bin/env python3
"""Paired, evaluation-only episodes with a fresh ROS/Gazebo process per case."""
import argparse
import copy
import csv
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import random
import signal
import socket
import subprocess
import time
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[2]
FIELDS = ['environment', 'map_seed', 'method', 'episode', 'terminal', 'success',
          'collision', 'planner_failure', 'tracking_failure', 'mission_time',
          'mean_actual_speed', 'tracking_p95', 'mean_requested_vmax', 'mean_applied_vmax']
METHODS = {'fixed_0.60': 0.60, 'fixed_1.00': 1.00, 'fixed_1.40': 1.40, 'sac': 0.0}
CHECKPOINT = ROOT / 'runtime_artifacts/rl_training/ego_speed_async_100ep_endurance_20260901_114200_r01/sac_checkpoint_episode_0100.pt'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def prepare(output, episodes=100):
    output.mkdir(parents=True, exist_ok=False)
    forest = ROOT / 'simulation/forest'
    spec = importlib.util.spec_from_file_location('forest_qa', forest / 'verify_forest_randomization_v1.py')
    qa = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(qa)
    pool = json.loads((forest / 'forest_pool_v1.json').read_text())
    template = ET.parse(forest / 'learning_speed_forest_v1_seed8.world')
    source_models = {m.get('name'): m for m in template.getroot().find('world').findall('model')}
    evaluation_config = yaml.safe_load((ROOT / 'AstraDrone_ros1_ws/src/learning_speed_rl/config/sac_training_v1.yaml').read_text())
    evaluation_config['episode']['max_steps'] = 1800
    evaluation_config['episode']['max_duration_sec'] = 180.0
    (output / 'evaluation_config.yaml').write_text(yaml.safe_dump(evaluation_config))
    cases = []
    for episode in range(1, episodes + 1):
        seed = 10000 + episode - 1
        reset_seed = 20000 + episode - 1
        folder = output / 'maps' / str(seed)
        folder.mkdir(parents=True)
        layout = qa.generated_layout(seed, pool)
        tree = ET.parse(forest / 'learning_speed_forest_v1_base.world')
        world = tree.getroot().find('world')
        for item in layout:
            model = copy.deepcopy(source_models[item['name']])
            model.find('pose').text = '{} {} {} 0 0 0'.format(item['x'], item['y'], item['z'])
            for cylinder in model.findall('.//geometry/cylinder'):
                cylinder.find('radius').text = str(item['radius'])
                cylinder.find('length').text = str(item['height'])
            world.append(model)
        generated = folder / 'learning_speed_forest_v1_seed8.world'
        tree.write(generated, encoding='utf-8', xml_declaration=True)
        parsed = qa.parse_world(generated, pool)
        assert qa.layout_hash(parsed) == qa.layout_hash(layout)
        for n in list(range(8)) + [9]:
            (folder / 'learning_speed_forest_v1_seed{}.world'.format(n)).symlink_to(
                forest / 'learning_speed_forest_v1_seed{}.world'.format(n))
        (folder / 'learning_speed_forest_v1_base.world').symlink_to(forest / 'learning_speed_forest_v1_base.world')
        config = copy.deepcopy(pool)
        config['seed_pool'][8]['raw_seed'] = seed
        config['seed_pool'][8]['replacement_reason'] = 'frozen evaluation seed; no outcome-based filtering'
        write_json(folder / 'forest_pool_v1.json', config)
        rng = random.Random(reset_seed)
        cases.append({'episode': episode, 'forest_seed': seed, 'reset_seed': reset_seed,
                      'worksite_hover': [rng.uniform(-1, 1), rng.uniform(-1, 1), 3.0],
                      'forest_hover': [5.0, 0.0, 3.0], 'map_dir': str(folder),
                      'layout_sha256': qa.layout_hash(layout), 'world_sha256': digest(generated)})
    manifest = {'version': 1, 'checkpoint': str(CHECKPOINT), 'checkpoint_sha256': digest(CHECKPOINT),
                'checkpoint_episode': 100, 'episode_time_limit_sec': 180, 'max_policy_steps': 1800, 'methods': METHODS, 'cases': cases,
                'training_raw_seeds': [0, 10, 11, 18, 23, 32, 36, 37],
                'reset_protocol': 'one fresh ROS master, Gazebo, Hector, EGO and Observation stack per Episode',
                'worksite_task': 'Hover to ENTRY_GATE; not a complete tower orbit',
                'forest_task': '(5,0,3) to (57.5,0,3)',
                'map_admission': 'predefined seeds, no outcome-based rejection'}
    write_json(output / 'manifest.json', manifest)
    with (output / 'episodes.csv').open('x') as stream:
        csv.DictWriter(stream, fieldnames=FIELDS).writeheader()
    return manifest


def require_free_ports():
    for port in (11341, 11375):
        with socket.socket() as sock:
            sock.bind(('127.0.0.1', port))


def run_case(output, manifest, case, environment, method):
    require_free_ports()
    folder = output / 'runs' / environment / method / '{:03d}'.format(case['episode'])
    folder.mkdir(parents=True, exist_ok=False)
    env = os.environ.copy()
    env.update(ROS_MASTER_URI='http://127.0.0.1:11341', GAZEBO_MASTER_URI='http://127.0.0.1:11375',
               ROS_HOME=str(folder / 'ros_home'), ROS_LOG_DIR=str(folder / 'ros_logs'))
    launch = 'hector_{}_sac_training.launch'.format(environment)
    args = ['roslaunch', 'hector_ego_training_backend', launch, 'runner_mode:=evaluation',
            'evaluation_episodes:=1', 'evaluation_checkpoint_episode:=100',
            'evaluation_checkpoint_path:=' + manifest['checkpoint'],
            'evaluation_fixed_vmax:=' + str(METHODS[method]), 'output_dir:=' + str(folder),
            'sac_config:=' + str(output / 'evaluation_config.yaml'), 'max_episode_time:=180.0',
            'run_id:=eval_{}_{}_{:03d}'.format(environment, method, case['episode']), 'gui:=false']
    if environment == 'worksite':
        args += ['hover_x:=' + str(case['worksite_hover'][0]), 'hover_y:=' + str(case['worksite_hover'][1])]
    else:
        args += ['forest_dir:=' + case['map_dir'], 'forest_config:=' + case['map_dir'] + '/forest_pool_v1.json',
                 'evaluation_forest_seed:=8']
    write_json(folder / 'case.json', dict(case, environment=environment, method=method, command=args))
    start = time.time()
    timed_out = False
    with (folder / 'console.log').open('x') as log:
        process = subprocess.Popen(args, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            process.wait(timeout=900)
        except subprocess.TimeoutExpired:
            timed_out = True
        finally:
            # Signal only the process group owned by this individual evaluation.
            for sig, wait in [(signal.SIGINT, 20), (signal.SIGTERM, 10), (signal.SIGKILL, 2)]:
                try:
                    os.killpg(process.pid, sig)
                except ProcessLookupError:
                    break
                try:
                    process.wait(timeout=wait)
                except subprocess.TimeoutExpired:
                    continue
                time.sleep(1)
            process.wait()
    write_json(folder / 'process.json', {'pid': process.pid, 'returncode': process.returncode,
               'wall_time': time.time() - start, 'timed_out': timed_out})
    require_free_ports()
    if timed_out:
        raise RuntimeError('infrastructure timeout; preserved at ' + str(folder))
    summary_path = folder / 'sac_episode_summaries.json'
    runtime_path = folder / 'sac_runtime_summary.json'
    if not summary_path.exists() or not runtime_path.exists():
        raise RuntimeError('missing closed Episode evidence: ' + str(folder))
    episodes = json.loads(summary_path.read_text())
    runtime = json.loads(runtime_path.read_text())
    if len(episodes) != 1:
        raise RuntimeError('expected exactly one closed Episode: ' + str(folder))
    e = episodes[0]
    reason = str(e.get('coordinator_terminal_reason', '')) + ';' + str(e['episode']['terminal_reason'])
    collision = 'collision' in reason
    planner = 'planner_failure' in reason
    tracking = 'tracking' in reason
    truncated = bool(e['episode']['truncated'])
    success = (e.get('coordinator_terminal_outcome') == 'SUCCESS' and not truncated
               and not (collision or planner or tracking) and not e['closure']['fail_closed_reason'])
    row = dict(environment=environment, map_seed=case['forest_seed'] if environment == 'forest' else case['reset_seed'],
               method=method, episode=case['episode'], terminal=reason, success=int(success), collision=int(collision),
               planner_failure=int(planner), tracking_failure=int(tracking),
               mission_time=e['episode']['episode_elapsed'], mean_actual_speed=e['actual_speed_mps']['mean'],
               tracking_p95=e['tracking_error_m']['p95'], mean_requested_vmax=e['requested_v_max']['mean'],
               mean_applied_vmax=e['applied_v_max']['mean'])
    with (output / 'episodes.csv').open('a') as stream:
        csv.DictWriter(stream, fieldnames=FIELDS).writerow(row)
    write_json(folder / 'classification.json', dict(row, truncated=truncated,
               failure=not success and not truncated, measurement_window='closed policy transitions'))
    if (e['closure']['fail_closed_reason'] or not runtime.get('training_counters_unchanged')
            or not runtime.get('network_parameters_unchanged')
            or runtime.get('replay_buffer_created') or runtime.get('learner_created')):
        raise RuntimeError('evaluation integrity failure: ' + str(folder))
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--prepare', action='store_true')
    parser.add_argument('--episodes', type=int, default=100)
    parser.add_argument('--run', action='store_true')
    args = parser.parse_args()
    output = args.output.resolve()
    if ROOT / 'runtime_artifacts' not in output.parents:
        parser.error('output must be inside repository runtime_artifacts')
    if not 1 <= args.episodes <= 100:
        parser.error('episodes must be 1..100')
    if args.prepare == args.run:
        parser.error('choose exactly one of --prepare or --run')
    if args.prepare:
        prepare(output, args.episodes)
        return
    manifest = json.loads((output / 'manifest.json').read_text())
    if digest(manifest['checkpoint']) != manifest['checkpoint_sha256']:
        raise RuntimeError('checkpoint changed')
    with (output / '.evaluation_started').open('x') as marker:
        marker.write(str(time.time()))
    try:
        for environment in ('worksite', 'forest'):
            for case in manifest['cases']:
                for method in METHODS:
                    print(environment, case['episode'], method, flush=True)
                    run_case(output, manifest, case, environment, method)
        write_json(output / 'status.json', {'status': 'completed', 'episodes': 8 * len(manifest['cases'])})
    except BaseException as error:
        write_json(output / 'status.json', {'status': 'blocked', 'error': str(error)})
        raise


if __name__ == '__main__':
    main()
