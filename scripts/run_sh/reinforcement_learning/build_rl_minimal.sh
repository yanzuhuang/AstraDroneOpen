#!/usr/bin/env bash
# Build only audited packages in an isolated overlay; leave other branch caches alone.
set -eo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/../../.." && pwd)"
overlay="${ASTRA_RL_OVERLAY:-/tmp/astra_rl_minimal_overlay}"
hector_overlay="${ASTRA_HECTOR_OVERLAY:-/tmp/astra_hector_training_overlay}"
"$script_dir/prepare_hector_training_overlay.sh" --overlay "$hector_overlay"
mkdir -p "$overlay/src"
python3 - "$repo_root" "$overlay" <<'PY'
import json, pathlib, sys
root, overlay = map(pathlib.Path, sys.argv[1:])
packages = json.loads((root / 'docs/rl_dependency_audit.json').read_text())['retained_packages']
for name, relative in packages.items():
    source, target = root / relative, overlay / 'src' / name
    if target.is_symlink() and target.resolve() == source.resolve():
        continue
    if target.exists() or target.is_symlink():
        raise SystemExit('Refusing to replace overlay entry: ' + str(target))
    target.symlink_to(source, target_is_directory=True)
PY
source /opt/ros/noetic/setup.bash
source "$hector_overlay/devel/setup.bash" --extend
cd "$overlay"
catkin_make -j2 -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
printf 'RL overlay ready: %s/devel/setup.bash\n' "$overlay"
