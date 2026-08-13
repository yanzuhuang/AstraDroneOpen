"""Training output must never leak into the ROS source package."""

from pathlib import Path
import unittest

from learning_speed_rl.training import validate_artifact_root


class TrainingArtifactRootTest(unittest.TestCase):
    def test_accepts_only_the_repository_runtime_tree(self):
        repository_root = Path(__file__).resolve().parents[4]
        allowed = repository_root / "runtime_artifacts" / "learning_speed"
        self.assertEqual(validate_artifact_root(str(allowed)), allowed.resolve())
        nested = allowed / "checkpoints" / "trial_001"
        self.assertEqual(validate_artifact_root(str(nested)), nested.resolve())

    def test_rejects_similar_names_and_external_trees(self):
        repository_root = Path(__file__).resolve().parents[4]
        with self.assertRaises(ValueError):
            validate_artifact_root(
                str(repository_root / "runtime_artifacts" / "learning_speed_old")
            )
        with self.assertRaises(ValueError):
            validate_artifact_root("/tmp/runtime_artifacts/learning_speed")


if __name__ == "__main__":
    unittest.main()
