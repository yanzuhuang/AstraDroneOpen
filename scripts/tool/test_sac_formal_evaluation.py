"""Offline integrity checks; these do not qualify a live flight or map reset."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FormalEvaluationIntegrityTest(unittest.TestCase):
    def test_incomplete_batch_cannot_produce_formal_figures(self):
        plot = load('evaluation_plot', ROOT / 'scripts/tool/plot_sac_formal_evaluation.py')
        with tempfile.TemporaryDirectory(dir='/tmp') as folder:
            path = Path(folder)
            (path / 'episodes.csv').write_text('environment,method\nworksite,sac\n')
            (path / 'manifest.json').write_text(json.dumps({'cases': [{}] * 3}))
            with self.assertRaisesRegex(ValueError, '24'):
                plot.summarize(path)
            self.assertEqual(list(path.glob('*.png')), [])

    def test_frozen_seeds_and_initial_conditions(self):
        manifest_path = ROOT / 'runtime_artifacts/sac_formal_evaluation_20260919_100maps_r02/manifest.json'
        if not manifest_path.exists():
            self.skipTest('local frozen evaluation artifacts are not distributed with Git')
        manifest = json.loads(manifest_path.read_text())
        cases = manifest['cases']
        self.assertEqual(len(cases), 100)
        seeds = {c['forest_seed'] for c in cases}
        self.assertEqual(len(seeds), 100)
        self.assertFalse(seeds & set(manifest['training_raw_seeds']))
        self.assertEqual(len({c['layout_sha256'] for c in cases}), 100)
        for case in cases:
            x, y, z = case['worksite_hover']
            self.assertTrue(-1 <= x <= 1 and -1 <= y <= 1 and z == 3)

    def test_network_digest_detects_actor_and_critic_mutation(self):
        import torch
        from learning_speed_rl.training.sac import SacAgent, SacConfig
        module = load('evaluation_runner', ROOT / 'AstraDrone_ros1_ws/src/learning_speed_rl/scripts/sac_training_runner.py')
        runner = object.__new__(module.SacTrainingRunner)
        runner.agent = SacAgent(SacConfig())
        initial = runner._network_hash()
        with torch.no_grad():
            next(runner.agent.actor.parameters()).flatten()[0] += 1
        actor_changed = runner._network_hash()
        self.assertNotEqual(initial, actor_changed)
        with torch.no_grad():
            next(runner.agent.critic1.parameters()).flatten()[0] += 1
        self.assertNotEqual(actor_changed, runner._network_hash())


if __name__ == '__main__':
    unittest.main()
