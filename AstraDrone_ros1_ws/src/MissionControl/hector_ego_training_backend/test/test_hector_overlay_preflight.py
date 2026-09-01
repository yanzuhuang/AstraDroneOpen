#!/usr/bin/env python3

import os
from pathlib import Path
import subprocess
import tempfile
import unittest


class HectorOverlayPreflightTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.script = (
            Path(__file__).resolve().parents[5]
            / "scripts"
            / "run_sh"
            / "prepare_hector_training_overlay.sh"
        )

    def _make_canonical_fixture(self, root):
        for directory in (
            "hector_gazebo",
            "hector_localization",
            "hector_models",
            "hector_quadrotor/hector_quadrotor_gazebo/urdf",
            "hector_quadrotor/hector_quadrotor_description/urdf",
        ):
            (root / directory).mkdir(parents=True, exist_ok=True)
        (root / "hector_quadrotor/hector_quadrotor_gazebo/package.xml").write_text(
            "<package/>\n", encoding="utf-8"
        )
        (root / "hector_quadrotor/hector_quadrotor_gazebo/urdf/quadrotor_plugins.gazebo.xacro.in").write_text(
            "<robot/>\n", encoding="utf-8"
        )
        (root / "hector_quadrotor/hector_quadrotor_description/urdf/quadrotor.gazebo.xacro").write_text(
            "<robot/>\n", encoding="utf-8"
        )

    def _make_overlay_fixture(self, root, generated=True):
        for directory in (
            "src/hector_gazebo",
            "src/hector_localization",
            "src/hector_models",
            "src/hector_quadrotor/hector_quadrotor_gazebo/urdf",
            "devel",
            "build",
        ):
            (root / directory).mkdir(parents=True, exist_ok=True)
        if generated:
            (root / "src/hector_quadrotor/hector_quadrotor_gazebo/urdf/quadrotor_plugins.gazebo.xacro").write_text(
                "<robot/>\n", encoding="utf-8"
            )
        (root / "devel/setup.bash").write_text("true\n", encoding="utf-8")
        (root / "devel/.catkin").write_text(
            f"{root}/src", encoding="utf-8"
        )
        (root / "build/Makefile").write_text("all:\n", encoding="utf-8")

    def _check(self, canonical, overlay):
        return subprocess.run(
            [
                str(self.script),
                "--check-only",
                "--source",
                str(canonical),
                "--overlay",
                str(overlay),
            ],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            env={**os.environ, "LC_ALL": "C"},
        )

    def test_complete_staged_overlay_passes(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            canonical = root / "canonical"
            overlay = root / "overlay"
            self._make_canonical_fixture(canonical)
            self._make_overlay_fixture(overlay)
            result = self._check(canonical, overlay)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn("preflight PASS", result.stdout)

    def test_missing_generated_xacro_fails_even_when_devel_setup_exists(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            canonical = root / "canonical"
            overlay = root / "overlay"
            self._make_canonical_fixture(canonical)
            self._make_overlay_fixture(overlay, generated=False)
            result = self._check(canonical, overlay)
            self.assertEqual(result.returncode, 2)
            self.assertIn("generated Hector xacro is missing", result.stderr)

    def test_missing_canonical_template_fails_closed(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            canonical = root / "canonical"
            overlay = root / "overlay"
            self._make_canonical_fixture(canonical)
            self._make_overlay_fixture(overlay)
            (canonical / "hector_quadrotor/hector_quadrotor_gazebo/urdf/quadrotor_plugins.gazebo.xacro.in").unlink()
            result = self._check(canonical, overlay)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("canonical Hector source is missing", result.stderr)

    def test_relocated_catkin_prefix_fails_closed(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            canonical = root / "canonical"
            overlay = root / "overlay"
            self._make_canonical_fixture(canonical)
            self._make_overlay_fixture(overlay)
            (overlay / "devel/.catkin").write_text(
                "/tmp/old-stage/src", encoding="utf-8"
            )
            result = self._check(canonical, overlay)
            self.assertEqual(result.returncode, 2)
            self.assertIn("stale source prefix", result.stderr)


if __name__ == "__main__":
    unittest.main()
