#!/usr/bin/env python3
# Copyright 2026 Hesai Technology. All rights reserved.
# SPDX-License-Identifier: GPL-2.0

"""Offline regression tests; no ROS nodes, bags, drivers, or builds are started."""

import copy
import contextlib
import io
from pathlib import Path
import subprocess
import tempfile
import unittest

import yaml

from prepare_runtime_config import prepare_config
from check_config import ConfigChecker


class RuntimeConfigTests(unittest.TestCase):
    def test_static_checker_rejects_malformed_types(self):
        config = {
            "preprocess": {
                "lidar_type": False,
                "scan_line": "128",
                "timestamp_unit": True,
                "blind": "0.5",
            },
            "point_filter_num": 0,
            "common": {"imu_gyr_unit": "auto"},
            "mapping": {
                "imu_init_delay": False,
                "imu_init_duration": 0.5,
                "imu_init_max_gyr_norm": 0.03,
                "det_range": "50",
                "extrinsic_T": [0, 0, 0],
                "extrinsic_R": [1, 0, 0, 0, 1, 0, 0, 0, 1],
            },
            "pcd_save": {"pcd_save_en": "false"},
            "map_file_path": 7,
        }
        checker = ConfigChecker("synthetic", "jt128", 1)
        with contextlib.redirect_stdout(io.StringIO()):
            checker.check(config)
        self.assertGreaterEqual(checker.failures, 6)

    def test_runner_syntax_and_invalid_cli(self):
        runner = Path(__file__).with_name("run_fastlio.sh")
        syntax = subprocess.run(["bash", "-n", str(runner)], capture_output=True, text=True)
        self.assertEqual(syntax.returncode, 0, syntax.stderr)
        for tail in (["--imu-gyr-unit", "degrees"], ["--imu-gyr-unit"]):
            result = subprocess.run(["bash", str(runner), "jt128", "live"] + tail,
                                    capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("--imu-gyr-unit", result.stderr)

    def test_unit_precedence_both_ros_versions(self):
        for ros in (1, 2):
            for yaml_unit in ("auto", "deg", "rad"):
                for override in (None, "auto", "deg", "rad"):
                    with self.subTest(ros=ros, yaml=yaml_unit, override=override):
                        params = {"common": {"imu_gyr_unit": yaml_unit},
                                  "pcd_save": {"pcd_save_en": False},
                                  "preprocess": {"scan_line": 128}}
                        config = params if ros == 1 else {"/**": {"ros__parameters": params}}
                        original = copy.deepcopy(config)
                        result, unit = prepare_config(config, ros, override)
                        self.assertEqual(unit, override or yaml_unit)
                        self.assertEqual(config, original)
                        out = result if ros == 1 else result["/**"]["ros__parameters"]
                        self.assertFalse(out["pcd_save"]["pcd_save_en"])
                        self.assertEqual(out["preprocess"], params["preprocess"])

    def test_missing_defaults_and_map_settings(self):
        config, unit = prepare_config({}, 1, map_path="/tmp/地图 with spaces.pcd")
        self.assertEqual(unit, "auto")
        self.assertTrue(config["pcd_save"]["pcd_save_en"])
        self.assertEqual(config["map_file_path"], "/tmp/地图 with spaces.pcd")
        config, _ = prepare_config({"pcd_save": {"leaf_size": 0.2, "interval": 10}}, 1, "rad", "map.pcd")
        self.assertEqual(config["pcd_save"]["leaf_size"], 0.2)
        self.assertEqual(config["pcd_save"]["interval"], 10)

    def test_invalid_config_fails(self):
        for config, ros in ((None, 1), ({}, 2), ({"common": None}, 1),
                            ({"common": {"imu_gyr_unit": "degrees"}}, 1)):
            with self.subTest(config=config, ros=ros), self.assertRaises(ValueError):
                prepare_config(config, ros)

    def test_runner_config_function_without_ros(self):
        runner = Path(__file__).with_name("run_fastlio.sh").read_text()
        # Exercise the real shell config function in isolation, without sourcing
        # ROS, playing a bag, or invoking any mapping lifecycle functions.
        function = runner.split("prepare_map_config() {", 1)[1].split("\nsave_map_now()", 1)[0]
        self.assertNotIn("detect_bag_imu_unit", runner)
        with tempfile.TemporaryDirectory(prefix="fastlio-unit-test-") as folder:
            root = Path(folder)
            (root / "config").mkdir()
            cfg = root / "config" / "jt128.yaml"
            cfg.write_text("common:\n  imu_gyr_unit: deg\npcd_save:\n  pcd_save_en: false\n")
            for override in ("", "rad", "deg", "auto"):
                script = '''set -euo pipefail
REPO_ROOT="$1"; SCRIPT_DIR="$2"; IMU_UNIT_OVERRIDE="$3"
MODEL=jt128; ROS_MAJOR=1; MAP_PATH="$4"; FASTLIO_WS="$1"
die() { echo "$*" >&2; exit 1; }
TEMP_CONFIG=""
trap 'if [[ -n "$TEMP_CONFIG" ]]; then rm -f -- "$TEMP_CONFIG"; fi' EXIT
prepare_map_config() {''' + function + '''
prepare_map_config
python3 - "$TEMP_CONFIG" "$IMU_UNIT_OVERRIDE" "$MAP_PATH" <<'PY'
import sys,yaml
with open(sys.argv[1]) as f: p=yaml.safe_load(f)
assert p['common']['imu_gyr_unit'] == (sys.argv[2] or 'deg')
assert p['pcd_save']['pcd_save_en'] is bool(sys.argv[3])
if sys.argv[3]: assert p['map_file_path'] == sys.argv[3]
PY
'''
                for map_path in ("", str(root / "地图 with spaces" / "地图.pcd")):
                    result = subprocess.run(["bash", "-c", script, "test", str(root),
                                             str(Path(__file__).parent), override, map_path],
                                            capture_output=True, text=True)
                    self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
