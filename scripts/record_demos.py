import hydra
import numpy as np
import rospy

from pathlib  import Path

from argparse import ArgumentParser

from rl_tasks import ENV_TYPES, \
                     Gamepad

if __name__ == '__main__':
    parser = ArgumentParser(description='Record demos from the environments.')
    parser.add_argument('hy', type=str, help='Hydra config to use. Relative to the root of hydra configs.')
    parser.add_argument('out_dir', type=str, help='Directory to save demos to.')
    parser.add_argument('--hydra-root', default='../config/envs', type=str, help='Root of hydra configuration files.')
    parser.add_argument('--overrides', default=[], type=str, nargs='*', help='Overrides for hydra config')
    parser.add_argument('--hide-gui', action='store_true', help='Hide interactive GUI or additional visualizations')
    parser.add_argument('--observations', default=[], nargs='*', help='Observations to capture. All if not specified.')
    parser.add_argument('--save-failures', action='store_true', help='Save both successful and failed episodes.')
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
    
    if 'teleop' not in cfg:
        print('Need "teleop" config in configuration')
        exit(-1)
    
    if 'gamepad' not in cfg.teleop:
        print('Need "gamepad" teleop config')
        exit(-1)

    # Create an environment
    env = ENV_TYPES[cfg.type](cfg, show_gui=not args.hide_gui)

    #                           LX LY RX RY DX DY LT RT  A  B  X  Y LB RB CL CR  U L3 R3  BIAS
    gamepad_premap = np.array([[ 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # LX
                               [ 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # LY
                               [ 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # RX
                               [ 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # RY
                               [ 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # DX
                               [ 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # DY
                               [ 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # LT
                               [ 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # RT
                               [ 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  # A
                               [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],  # LB
                               [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],  # RB
                               [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],  # L3
                               [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],  # R3
                               [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]]) # BIAS 

    # Validate input map
    action_map = {}
    for a, m in cfg.teleop.gamepad.items():
        if a not in env.action_space.keys():
            print('Action "{}" is not defined for the environment. Options are:\n  {}'.format(a, '\n  '.join(list(env.action_space.keys()))))
            exit(-1)
        action_map[a]  = np.asarray(m)
        expected_shape = env.action_space[a].shape + (gamepad_premap.shape[0],)
        if action_map[a].shape != expected_shape:
            print(f'Action map {a} needs to be {expected_shape} but is {action_map[a].shape[1]}')
            exit(-1)
        # Compose the final map
        action_map[a] = action_map[a].dot(gamepad_premap)


    # Check observation argument
    if len(args.observations) == 0:
        observations = list(env.observation_space.keys())
    elif len(set(args.observations).difference(env.observation_space.keys())) > 0:
        difference = set(args.observations).difference(env.observation_space.keys())
        print('Specified observations do not exist in the environment:\n  {}\n'.format(sorted(difference)))
        print('These observations are exposed by the environment:\n  {}'.format('\n  '.join(list(env.observation_space.keys()))))
        exit(-1)
    else:
        observations = args.observations

    # Connect to ROS
    try:
        rospy.init_node('test_env')
    except rospy.exceptions.ROSException:
        print('Not starting ROS node because one already exists.')

    out_dir = Path(args.out_dir)
    if not out_dir.exists():
        out_dir.mkdir()
    elif not out_dir.is_dir():
        print(f'Path "{out_dir}" already exists but is not a directory.')
        exit(-1)

    gamepad = Gamepad()

    # Recording loop management    
    demo_count = 0
    done = True
    success    = False
    traj_obs   = None

    neutral_action = env.neutral_action

    print('STARTING RECORDING!'
          '\n  Press X to reset the env, discarding the current episode.'
          '\n  Press B to save the current demonstration even without the environment indicating termination.'
          '\n  Press Y to quit. Episode will not be saved.')

    while not rospy.is_shutdown():
        if done:
            if success:
                save_path = out_dir / f'demo_{demo_count:03d}.npz'
                print(f'Saving demo to "{save_path}"')
                np.savez(save_path, **traj_obs)
                demo_count += 1

            traj_obs = {o: [] for o in observations}
            traj_obs.update({'ACTION': [], 'REWARD': [], 'DONE': []})

            obs = env.reset()
            for k, o in obs.items():
                if k in traj_obs:
                    traj_obs[k].append(o)
            
            # Clear button presses
            gamepad.reset()
            print(f'ENV RESET. STARTING EPISODE: {demo_count}\nWAITING FOR INPUT')
        
        while not gamepad.is_ready:
            pass

        rospy.sleep(1 / 30)

        action = neutral_action.copy()
        for a, m in action_map.items():
            action[a] = m.dot(np.vstack((gamepad.stacked_axes, [[1]]))).flatten()

        traj_obs['ACTION'].append(action)

        obs, reward, done, info = env.step(action)

        traj_obs['REWARD'].append(reward)
        traj_obs['DONE'].append(done)

        for k, o in obs.items():
            if k in traj_obs:
                traj_obs[k].append(o)
        
        if done:
            success = info['success'] or args.save_failures

        # Episode is done when env says, or user says
        done    = done or gamepad.is_pressed('X') or gamepad.is_pressed('B')
        success = (success or gamepad.is_pressed('B')) and not gamepad.is_pressed('X')

        if gamepad.is_pressed('Y'):
            print('Ending application.')
            break
