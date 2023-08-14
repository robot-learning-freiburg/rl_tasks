import hydra
import numpy as np
import rospy

from pathlib  import Path

from argparse import ArgumentParser

from rl_tasks import ENV_TYPES

if __name__ == '__main__':
    parser = ArgumentParser(description='Using hydra without losing control of your code.')
    parser.add_argument('hy', type=str, help='Hydra config to use. Relative to root of "config" dir')
    parser.add_argument('--hydra-root', default='../config/envs', type=str, help='Root of hydra configuration files.')
    parser.add_argument('--overrides', default=[], type=str, nargs='*', help='Overrides for hydra config')
    parser.add_argument('--hide-gui', action='store_true', help='Hide interactive GUI or additional visualizations')
    args = parser.parse_args()

    p_file = Path(__file__).absolute()

    p_config = (p_file.parent / Path(f'{args.hydra_root}/{args.hy}.yaml')).absolute()
    if not p_config.exists():
        print(f'Could not find config file {p_config} in path {p_config.parent}. Options are:\n')
        p_dir = Path(args.hydra_root).absolute()
        for p in p_dir.glob('*.yaml'):
            print(f'\t{p}')
        exit(-1)

    # Point hydra to the root of your config dir. Here it's hard-coded, but you can also
    # use "MY_MODULE.__path__" to localize it relative to your python package
    hydra.initialize(config_path=args.hydra_root)
    cfg = hydra.compose(args.hy, overrides=args.overrides)
    env = ENV_TYPES[cfg.type](cfg, show_gui=not args.hide_gui)

    try:
        rospy.init_node('test_env')
    except rospy.exceptions.ROSException:
        print('Not starting ROS node because one already exists.')

    env.reset()
    
    neutral_action = env.neutral_action

    while not rospy.is_shutdown():
        rospy.sleep(1 / 30)
        env.step(neutral_action)
        # print(env.observation())
        terminated, success = env.is_terminated()
        # print(f'Is terminated: {terminated}\nIs success: {success}')
        if terminated:
            env.reset()
