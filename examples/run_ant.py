"""Run the MPPI controller on the example ant and print its score.

    python examples/run_ant.py                 # full episode: 250 control steps = 1000 env steps = 50 s
    python examples/run_ant.py --steps 50      # quick check
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mpc.mppi import MPPIConfig, evaluate_design  # noqa: E402


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--terrain", default="flat", choices=["flat", "ridged", "beams", "frozen_lake"])
    p.add_argument("--steps", type=int, default=250, help="control steps per episode")
    p.add_argument("--episodes", type=int, default=1)
    p.add_argument("--threads", type=int, default=8)
    p.add_argument("--seed", type=int, default=0)
    a = p.parse_args()

    xml = (Path(__file__).parent / "ant.xml").read_text()
    cfg = MPPIConfig(episode_control_steps=a.steps, threads=a.threads)
    r = evaluate_design(xml, a.terrain, a.episodes, a.seed, cfg, verbose=True)
    for i, ep in enumerate(r["episodes"]):
        print(f"episode {i}: score {ep['score']:.1f}, distance {ep['distance']:.2f} m, "
              f"{ep['env_steps']} env steps, terminated={ep['terminated']}, {ep['seconds']:.1f} s")
    print(f"fitness (mean score): {r['fitness']:.1f}")


if __name__ == "__main__":
    main()
