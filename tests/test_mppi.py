import json
import math
import tempfile
import unittest
from pathlib import Path

ANT = Path(__file__).resolve().parents[1] / "examples" / "ant.xml"


class MPPITest(unittest.TestCase):
    def test_short_episode_on_ant(self):
        from mpc.mppi import MPPIConfig, evaluate_design
        cfg = MPPIConfig(threads=2, episode_control_steps=5, warmup_samples=64)
        r = evaluate_design(ANT.read_text(), "flat", 1, 0, cfg)
        self.assertTrue(math.isfinite(r["fitness"]))
        self.assertEqual(r["episodes"][0]["env_steps"], 20)      # 5 control steps x 4 env steps
        self.assertFalse(r["episodes"][0]["unstable"])

    def test_same_seed_same_score(self):
        from mpc.mppi import MPPIConfig, evaluate_design
        cfg = MPPIConfig(threads=1, episode_control_steps=3, warmup_samples=64)
        a = evaluate_design(ANT.read_text(), "flat", 1, 7, cfg)["fitness"]
        b = evaluate_design(ANT.read_text(), "flat", 1, 7, cfg)["fitness"]
        self.assertEqual(a, b)

    def test_speed_guard_ends_episode(self):
        from mpc.mppi import MPPIConfig, evaluate_design
        cfg = MPPIConfig(threads=2, episode_control_steps=5, warmup_samples=64, max_speed=1e-6)
        ep = evaluate_design(ANT.read_text(), "flat", 1, 0, cfg)["episodes"][0]
        self.assertTrue(ep["unstable"])
        self.assertLess(ep["env_steps"], 20)

    def test_solver_iterations(self):
        from mpc.mppi import MPPIConfig, load_model
        self.assertEqual(MPPIConfig().min_solver_iterations, 20)
        self.assertGreaterEqual(load_model(ANT.read_text(), min_iterations=20).opt.iterations, 20)
        self.assertEqual(load_model(ANT.read_text(), mjx_solver=True, min_iterations=20).opt.iterations, 1)

    def test_terrains_compile(self):
        from mpc.mppi import load_model
        for terrain in ("flat", "ridged", "beams", "frozen_lake"):
            self.assertGreater(load_model(ANT.read_text(), terrain).nu, 0)

    def test_failed_design_gets_minus_inf_and_failure_file(self):
        from mpc.evaluate import evaluate_multiple_designs
        with tempfile.TemporaryDirectory() as tmp:
            bad = Path(tmp) / "bad.xml"
            bad.write_text("<mujoco><worldbody/></mujoco>")       # no actuators -> ValueError
            [(_, fitness)] = evaluate_multiple_designs([str(bad)], output_dir=tmp, threads=1)
            self.assertEqual(fitness, float("-inf"))
            err = json.loads((Path(tmp) / "failure_bad.json").read_text())["error"]
            self.assertIn("MPC evaluation failed", err)


if __name__ == "__main__":
    unittest.main()
