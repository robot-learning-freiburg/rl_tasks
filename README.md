# A Collection of Robot Tasks - Real and Imaginary!

In this package we collect our custom tasks setup as Gym environments.

## Installation / Dependencies

This package has a hard dependency on ROS. Not all environments do, but in general the package does, so it's beneficial to just install it to a ROS workspace. We recommend [ROSVENV](https://github.com/ARoefer/rosvenv) to help with workspace management and Python encapsulation.
Aside from ROS, the package has a hard dependency on the [`prime_bullet`](https://github.com/ARoefer/prime_bullet) for PyBullet. Follow its installation instructions, then proceed here.

To install the remaining Python dependencies of this package use `pip install -r requirements.txt` at the root of the package.

If you want to use the real environments as well, you will have to install [`rl_franka`](https://rlgit.informatik.uni-freiburg.de/fmm/rl_franka) and [`another_aruco`](https://rlgit.informatik.uni-freiburg.de/aroefer/another_aruco) as well.

## Package Structures and Environments

The package structure is as follows:

```
.
├── config  -> Holds hydra configuration files
├── launch  -> Holds ROS launch files needed for real scenarios
├── objects -> Holds object meshes and URDFs
├── robots  -> Holds robot URDFs
├── scripts -> Holds additional scripts
├── src     
    └── rl_tasks -> Holds python environments
└── tests   -> Holds optional tests for environments.
```

Generally, each environment should be placed in a Python file inside `src/rl_tasks` and registered with a unique name in the `ENV_TYPES` dictionary in `src/rl_tasks/__init__.py`.

Environments take only two parameters at their instantiation:

 1. An OmegaConf config
 2. A boolean to show optional GUI or other visualizations (such as markers to be displayed in RVIZ)

Currently, the package contains the following environments:

- `door_env`: A robot arm needs to open a miniature door.
- `hatch_env`: A robot arm needs to open a sliding hatch.
- `peg_env`: A robot arm needs to insert a peg into a hole.
- `real_door_env`: A real panda arm needs to open a miniature door.
- `real_drawer_env`: A real panda arm needs to open a drawer.

Environments can be tested using:

```bash
roscd rl_tasks
python scripts/test_env.py door_env
```
