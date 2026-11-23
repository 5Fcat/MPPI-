# CPU MPPI evaluator for RoboMorph designs

A sampling-based MPC controller (MPPI) that scores robot designs without training a policy.
It is used in my master's research as a CPU-only replacement for the SAC training step in
[RoboMorph](https://github.com/kevinxqiu/robomorph): each candidate design is controlled by MPPI
for one episode, and the forward reward collected in that episode is the design's fitness.

- **Algorithm**: a Python + MuJoCo re-implementation of the MPPI controller in
  [RoboGrammar](https://github.com/allanzhao/RoboGrammar) (`core/src/optim.cpp`:
  `MPPIOptimizer` + `DefaultInputSampler`). RoboGrammar is C++/Bullet and cannot run
  RoboMorph's MuJoCo models, so the logic was rewritten, not copied.
- **Parameters**: the MPPI settings the RoboMorph authors used in RoboMorph V3 (§III-C),
  shared by the first author by email, with the changes listed below.
- **Scores** are on the same scale as RoboMorph's SAC fitness (sum of forward reward), so the two
  evaluators can be compared design by design.

## Install

Python 3.10–3.12.

```bash
pip install -r requirements.txt      # mujoco==3.2.6 (the version RoboMorph pins), numpy<2
```

## Quick start

```bash
python examples/run_ant.py --steps 50          # short check on the example ant
python examples/run_ant.py                      # full episode (250 control steps = 50 s simulated)
python -m unittest -v tests.test_mppi           # tests (a few seconds)
```

Score a batch of MuJoCo XMLs (`design_*.xml` in a directory, or one file):

```bash
python -m mpc.eval_batch --xml_dir path/to/designs --out runs/batch --episodes 3 --threads 8
python -m mpc.eval_batch --xml examples/ant.xml --out runs/ant --terrain ridged
```

If the directory also holds a RoboMorph SAC `results.csv`, `summary.txt` reports the Spearman rank
correlation and top-k overlap between SAC and MPC scores.

From code:

```python
from mpc.mppi import MPPIConfig, evaluate_design

result = evaluate_design(xml_string, terrain="flat", episodes=2, seed=0, cfg=MPPIConfig(threads=8))
result["fitness"]        # mean episode score
```

`mpc.evaluate.evaluate_multiple_designs` has the same interface as RoboMorph's
`train_multiple_models` (returns `[(xml_path, fitness), ...]`, `-inf` plus a `failure_<name>.json`
for designs that fail), so it can be dropped into the RoboMorph evolution loop.

## How it works

At every control step:

1. Sample N action sequences over the planning horizon. Half start from the previous plan
   (warm start), half repeat recently executed actions periodically (RoboGrammar's history sampler),
   each with Gaussian noise smoothed by a 5-tap FIR low-pass filter.
2. Roll all samples out in MuJoCo in parallel (`mujoco.rollout`, one `MjData` per thread) and
   compute each one's discounted forward reward.
3. Average the sequences with weights `exp(kappa * (R - max R))`.
4. Execute the first action, shift the plan by one step, and repeat.

No value function is used, as in RoboGrammar's design search.

## Parameters

| | RoboGrammar | RoboMorph V3 (authors' settings) | This code (`MPPIConfig` defaults) |
|---|---|---|---|
| Samples per control step | 64 (512 on the first) | 64 | 64 (512 on the first) |
| Noise std (history / warm-start half) | 0.05 / 0.25 | 0.05 / 0.25 | 0.1 / 0.5 before filtering (≈ 0.05 / 0.25 after) |
| Horizon | 16 control steps | 16 control steps | 16 control steps (3.2 s) |
| Env steps per control step | 16 | 4 | 4 (0.2 s, 5 Hz) |
| Temperature `kappa` | 1.0 | 1.0 | 1.0 |
| Discount (per control step) | 0.99 | — | 0.99 |
| Episode | 128 control steps | ~250 control steps | 250 control steps (1000 env steps, 50 s) |
| Value function | none | none | none |

One env step is 5 MuJoCo steps of 0.01 s, as in RoboMorph's `FlatEnv`.

### Changes from those settings

1. **Noise ×2.** The FIR coefficients are RoboGrammar's, designed for its 15 Hz control rate. At this
   code's 5 Hz the filter cuts harder and scales the noise std by about 0.54, so the raw std is
   doubled to bring the effective std back to the authors' 0.05 / 0.25.
2. **Fall penalty (planning only).** RoboMorph's reward has no alive bonus, so a 3.2 s look-ahead
   learns to lunge forward and fall. Samples that terminate get −100 in planning
   (`fall_penalty`); the reported score is not changed.
3. **Solver settings and a glitch guard.** RoboMorph's MJX override (Newton, 1 iteration) diverges on
   CPU MuJoCo under random torques, and MuJoCo's auto-reset then looks like a huge forward jump.
   The XML's own solver is used, with at least 20 iterations (`min_solver_iterations`). A MuJoCo reset,
   a non-finite state or torso speed above 10 m/s (`max_speed`) counts as a physics glitch: in planning,
   that sample gets zero reward from then on; in the executed episode, the episode ends and is flagged
   `unstable`. `mjx_solver=True` restores the MJX setting for comparison.

### Environment, matched to RoboMorph

- Reward per env step = −(torso x velocity) (robots walk in −x); no control cost, no alive bonus
- Episode ends when torso height < 0.05 m or > 5 m
- Reset: `qpos += U(−0.1, 0.1)`, `qvel = 0.1·N(0, 1)`
- Actions in [−1, 1], mapped linearly to the actuator `ctrlrange`
- Terrains: `flat`, `ridged`, `beams`, `frozen_lake` with RoboMorph's geometry.
  `mpc/terrain_ext.py` adds parameterised terrains (`ridged_s<scale>`, `beams_s<scale>`, `ice_<mu>`,
  `slope_<deg>`, `rocks_<h>`, `rough_<a>`).

Note: in MuJoCo the contact friction is the larger of the two geoms' values, so RoboMorph's
`frozen_lake` (floor friction 0.05, robot geoms 1.0) behaves like flat ground unless the robot's
friction is lowered too or the floor has a higher `priority`.

## Files

```
mpc/mppi.py           MPPI controller and design evaluation
mpc/terrain_ext.py    extra terrains
mpc/evaluate.py       drop-in evaluator for the RoboMorph evolution loop
mpc/eval_batch.py     batch scoring + comparison with SAC scores
examples/ant.xml      example robot
examples/run_ant.py   example script
tests/test_mppi.py    tests
```

The code also has a branch for XMLs of the lab's Hyblock modular robots (detected by `<custom>`
numerics in the XML). That branch needs a `hyblock` module that is not part of this repository;
RoboMorph XMLs never use it.

## References

- MPPI: G. Williams, P. Drews, B. Goldfain, J. M. Rehg, E. A. Theodorou. *Aggressive driving with
  model predictive path integral control.* ICRA 2016, pp. 1433–1440.
- MPPI as used in RoboGrammar: K. Lowrey, A. Rajeswaran, S. Kakade, E. Todorov, I. Mordatch.
  *Plan online, learn offline: Efficient learning and exploration via model-based control.*
  arXiv:1811.01848, 2018.
- RoboGrammar: A. Zhao, J. Xu, M. Konaković-Luković, J. Hughes, A. Spielberg, D. Rus, W. Matusik.
  *RoboGrammar: Graph Grammar for Terrain-Optimized Robot Design.* ACM TOG 39(6), 2020.
- RoboMorph: K. Qiu et al. *RoboMorph: Evolving Robot Morphology using Large Language Models.*
  ICRA 2026, arXiv:2407.08626.

## License

MIT (see `LICENSE`).
