import json
import sys
import unittest
from contextlib import redirect_stderr, redirect_stdout
from io import StringIO
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

import run_fusion
from fusion.campaign import CampaignConfig, CampaignResult, RunSpec
from fusion.orchestrator import FusionResult


class EngineSelectionTests(unittest.TestCase):
    def invoke_cli(self, *args):
        self.stdout = StringIO()
        self.stderr = StringIO()
        with patch.object(sys, "argv", ["run_fusion.py", *args]), redirect_stdout(self.stdout), redirect_stderr(self.stderr):
            return run_fusion.main()

    def test_new_campaign_saves_v2_and_an_engine_less_plan_loads_as_legacy(self):
        with TemporaryDirectory() as tmp:
            path = Path(tmp) / "campaign.json"
            new = CampaignConfig(runs=[RunSpec()])
            new.save(path)
            self.assertEqual(json.loads(path.read_text(encoding="utf-8"))["engine"], "v2")
            self.assertEqual(CampaignConfig.load(path).engine, "v2")

            path.write_text(json.dumps({"runs": [{}], "radar_count": 14}), encoding="utf-8")
            old = CampaignConfig.load(path)
            self.assertEqual(old.engine, "legacy")
            self.assertEqual((old.radars_south, old.radars_north), (7, 7))

    def test_single_run_defaults_to_v2_and_legacy_selection_is_preserved(self):
        variants = (
            ([], "v2", 8000),
            (["--rate-hz", "20"], "v2", 8000),
            (["--engine", "v2"], "v2", 8000),
            (["--engine", "v2", "--rate-hz", "40"], "v2", 8000),
            (["--rate-hz", "25"], "legacy", 8000),
            (["--engine", "legacy", "--rate-hz", "30"], "legacy", 8000),
            (["--tm-port", "8042"], "legacy", 8042),
        )
        for args, engine, tm_port in variants:
            with (self.subTest(args=args),
                  patch.object(run_fusion, "run", return_value=FusionResult(ok=True)) as legacy,
                  patch("fusion.campaign_v2.run_single_v2", return_value=CampaignResult(ok=True)) as v2):
                self.assertEqual(self.invoke_cli(*args), 0)
                selected, other = (v2, legacy) if engine == "v2" else (legacy, v2)
                selected.assert_called_once()
                other.assert_not_called()
                self.assertEqual(selected.call_args.args[0].traffic_manager_port, tm_port)

    def test_campaign_uses_its_saved_engine_and_accepts_an_explicit_override(self):
        variants = (({}, [], "legacy"), ({"engine": "legacy"}, [], "legacy"),
                    ({"engine": "v2"}, [], "v2"), ({}, ["--engine", "v2"], "v2"),
                    ({"engine": "v2"}, ["--engine", "legacy"], "legacy"))
        with TemporaryDirectory() as tmp:
            path = Path(tmp) / "campaign.json"
            for fields, args, engine in variants:
                path.write_text(json.dumps({"runs": [{}], **fields}), encoding="utf-8")
                with (self.subTest(fields=fields, args=args),
                      patch.object(run_fusion, "run_campaign", return_value=CampaignResult(ok=True)) as run):
                    self.assertEqual(self.invoke_cli("--campaign", str(path), *args), 0)
                    self.assertEqual(run.call_args.args[0].engine, engine)

    def test_explicit_v2_rejects_the_legacy_traffic_manager_port(self):
        with self.assertRaises(SystemExit) as exit:
            self.invoke_cli("--engine", "v2", "--tm-port", "8042")
        self.assertEqual(exit.exception.code, 2)
        self.assertIn("Use --engine legacy", self.stderr.getvalue())

    def test_v2_rejects_fractional_millisecond_steps_before_opening_carla(self):
        with TemporaryDirectory() as tmp, patch("fusion.campaign_v2.open_carla_session") as open_carla:
            path = Path(tmp) / "campaign.json"
            plan = CampaignConfig(runs=[RunSpec()], rate_hz=30)
            self.assertIn("tick period in whole milliseconds", plan.validate()[0])
            plan.save(path)
            for args in (("--engine", "v2", "--rate-hz", "30"), ("--campaign", str(path))):
                with self.subTest(args=args):
                    self.assertEqual(self.invoke_cli(*args), 1)
                    self.assertIn("tick period in whole milliseconds", self.stdout.getvalue())
            open_carla.assert_not_called()
