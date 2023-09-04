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

## Recording Data

Currently, the package provides the `record_demos.py` script for recording demos as `npz` files using teleoperation. Currently only via gamepad. Use `rosrun joy joy_node` to start the joystick listener.
The help of the script offers explanation of its capabilities:

```bash
$ python scripts/record_demos.py --help
usage: record_demos.py [-h] [--hydra-root HYDRA_ROOT]
                       [--overrides [OVERRIDES [OVERRIDES ...]]] [--hide-gui]
                       [--observations [OBSERVATIONS [OBSERVATIONS ...]]] [--save-failures]
                       hy out_dir

Using hydra without losing control of your code.

positional arguments:
  hy                    Hydra config to use. Relative to the root of hydra configs.
  out_dir               Directory to save demos to.

optional arguments:
  -h, --help            show this help message and exit
  --hydra-root HYDRA_ROOT
                        Root of hydra configuration files.
  --overrides [OVERRIDES [OVERRIDES ...]]
                        Overrides for hydra config
  --hide-gui            Hide interactive GUI or additional visualizations
  --observations [OBSERVATIONS [OBSERVATIONS ...]]
                        Observations to capture. All if not specified.
  --save-failures       Save both successful and failed episodes.
```

Generally, the script saves demos to a target directory, whenever the environment indicates success. The user can also manually reset the episode with or without saving the collected data. The structure of the recorded files is as follows:

```
demo_000.npz
 - ACTION -> List of all action dicts
 - REWARD -> List of all rewards
 - DONE   -> List of all
 - obs1   -> list of obs1
 ...
 - obsN   -> list of obsN
```

To be able to use teleop on your environment, its hydra config needs to contain a `teleop` config with a `gamepad` subconfig.
The gamepad config specifies actions via linear maps of controls to the action space. An example for a 3d-motion action:

```yaml
teleop:
  gamepad:
    # Cols.:  LX  LY  RX  RY  DX  DY  LT  RT   A  LB  RB  L3  R3 BIAS
    motion: [[ 0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,   0],
             [ 0,  0,  0, -1,  0,  0,  0,  0,  0,  0,  0,  0,  0,   0],
             [ 0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,   0]]
```

The columns are associated with differen gamepad axes or buttons, while the rows are the action space. In addition, there is the `BIAS` column to apply some constant action. Since the script uses `X`, `Y`, `B` for its operation, theses are removed from the assignable inputs.
