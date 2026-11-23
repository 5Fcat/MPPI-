# MPPI

CPU MPPI controller used in my research to evaluate RoboMorph designs (MuJoCo 3.2.6).

- Algorithm: re-implementation of the MPPI controller in [RoboGrammar](https://github.com/allanzhao/RoboGrammar) (Zhao et al. 2020), which cites Lowrey et al. 2018; MPPI itself is from Williams et al. (ICRA 2016).
- Parameters: RoboMorph V3 settings from the RoboMorph authors.

The controller is in `mpc/mppi.py`; `mpc/terrain_ext.py` is only the terrain helper it imports.
