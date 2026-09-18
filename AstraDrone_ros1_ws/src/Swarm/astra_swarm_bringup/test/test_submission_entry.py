#!/usr/bin/env python3
"""Exercise real shell parsing and launch expansion, never start ROS nodes."""
import json
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[5]
ENTRY = ROOT / 'scripts/run_sh/three_uav/three_uav_inspection.sh'
OLD = ENTRY.with_name('three_uav_multi_height_inspection.sh')


class SubmissionEntryTest(unittest.TestCase):
    def check_mode(self, args, expected, descent=False, entry=ENTRY):
        result = subprocess.run(['bash', '-x', str(entry)] + args + ['--check-height-profile'],
                                cwd=str(ROOT), text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        rows = json.loads(result.stdout[result.stdout.index('[\n'):])
        self.assertEqual([r['top'] for r in rows], expected)
        self.assertEqual([r['layers'] for r in rows],
                         [[h, h-4] if descent else [h] for h in expected])
        return result

    def test_public_modes(self):
        for h in (3, 12, 30):
            self.check_mode(['--same-layer', '--altitude', str(h)], [h]*3)
        self.check_mode(['--multi-layer'], [26, 20, 14])
        self.check_mode(['--multi-layer', '--layer-descent'], [26, 20, 14], True)

    def test_legacy_wrapper(self):
        self.check_mode([], [26, 20, 14], entry=OLD)
        self.check_mode(['--single-layer'], [26, 20, 14], entry=OLD)
        self.check_mode(['--multi-layer'], [26, 20, 14], True, OLD)

    def test_flags_and_record_profiles(self):
        for mode in ('none', 'light', 'full'):
            with tempfile.TemporaryDirectory(prefix='three_uav_inspection_test_',
                                             dir=str(ROOT / 'runtime_artifacts')) as parent:
                args = ['--same-layer', '--altitude', '3', '--control', '--gui', '--rviz', '--record', mode]
                if mode != 'none':
                    args += ['--results-dir', str(Path(parent) / 'run')]
                result = self.check_mode(args, [3]*3)
                for token in ('enable_control:=true', 'gui:=true', 'start_rviz:=true',
                              'evidence_candidate_record_mode:=' + mode,
                              'evidence_record_enabled:=' + ('false' if mode == 'none' else 'true')):
                    self.assertIn(token, result.stderr)

    def test_invalid_arguments_fail_before_launch(self):
        cases = [['--same-layer', '--multi-layer'], ['--layer-descent'],
                 ['--multi-layer', '--altitude', '3'], ['--altitude'],
                 ['--altitude', 'nan'], ['--altitude', 'inf'], ['--altitude', '0'],
                 ['--altitude', '31'], ['--record', 'bad'],
                 ['--multi-layer', '--learning-speed'], ['--profile', 'same_3m', '--same-layer']]
        for args in cases:
            result = subprocess.run([str(ENTRY)] + args + ['--check-height-profile'],
                                    text=True, capture_output=True)
            self.assertEqual(result.returncode, 2, (args, result.stderr))

    def test_existing_evidence_is_not_overwritten(self):
        with tempfile.TemporaryDirectory(prefix='three_uav_inspection_test_',
                                         dir=str(ROOT / 'runtime_artifacts')) as directory:
            marker = Path(directory) / 'run_metadata.txt'
            marker.write_text('retained evidence')
            result = subprocess.run([str(ENTRY), '--record', 'light', '--results-dir', directory,
                                     '--check-height-profile'], text=True, capture_output=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(marker.read_text(), 'retained evidence')


if __name__ == '__main__':
    unittest.main()
