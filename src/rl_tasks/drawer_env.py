import numpy as np

from functools   import lru_cache

from prime_bullet import Simulator,         \
                         MultiBody,         \
                         Link,              \
                         PerspectiveCamera, \
                         Transform,         \
                         Point3,            \
                         Vector3,           \
                         Quaternion,        \
                         Frame,             \
                         AABB,              \
                         MeshBody,          \
                         CartesianController, \
                         CartesianRelativePointController, \
                         CartesianRelativeVirtualPointController, \
                         CartesianRelativeVPointCOrientationController, \
                         CartesianRelativePointCOrientationController


from gymnasium.spaces import Box  as BoxSpace, \
                             Dict as DictSpace
from gymnasium        import Env

from .utils     import BoxSampler, \
                       NoiseSampler


class DrawerEnv(Env):
    def __init__(self, cfg, show_gui=False):
        self.sim = Simulator(cfg.action_frequency, use_egl=False, real_time=False) #not show_gui)
        self.sim.init('gui' if show_gui else 'direct')

        self.dt        = 1 / cfg.action_frequency
        self.workspace = AABB(Point3(0.3, -0.85, 0), 
                              Point3(0.85, 0.85, 0.9))

        self.robot = self.sim.load_urdf(cfg.robot.path, useFixedBase=True)
        self.eef   = self.robot.get_link(cfg.robot.eef)
        self.gripper_joints = [self.robot.joints[f] for f in cfg.robot.fingers]

        self._target_position = cfg.open_threshold

        self.table = self.sim.create_box(Vector3(0.6, 1, 0.05), 
                                         Transform.from_xyz(0.5, 0, -0.025), 
                                         color=(1, 1, 1, 1), 
                                         mass=0)
        # self.peg   = self.robot.links['peg'] #
        self.shelf  = self.sim.load_urdf(cfg.shelf.path, useFixedBase=True)
        self.reference_link = self.shelf.links[cfg.shelf.reference_link]
        self.board_sampler  = BoxSampler(cfg.shelf.sampler.min, 
                                         cfg.shelf.sampler.max)

        if self.sim.visualizer is not None:
            self.sim.visualizer.set_camera_position(self.shelf.pose.position, 0.6, -25, 65)


        robot_init_state = cfg.robot.initial_pose


        self.robot.set_joint_positions(cfg.robot.initial_pose.q, override_initial=True)

        initial_rot = self.eef.pose.quaternion
        self._init_pose = Transform(Point3(*robot_init_state.position), initial_rot)
        self.robot.set_joint_positions(self.eef.ik(self._init_pose, 1000), override_initial=True)
        self.robot.set_joint_positions({j.name: robot_init_state.gripper_width / len(self.gripper_joints) for j in self.gripper_joints}, override_initial=True)

        self.eef_ft_sensor = self.robot.get_ft_sensor(cfg.robot.ft_joint)

        self.render_camera  = PerspectiveCamera(self.sim, self.render_size[:2], 
                                                50, 0.1, 10.0, 
                                                Transform(Point3(0.4, 0, 0.1), 
                                                          Quaternion.from_euler(0, np.deg2rad(30), np.deg2rad(120))).dot(Transform.from_xyz(-1.2, 0, 0.1)),
                                                self.shelf)
        self.gripper_camera  = PerspectiveCamera(self.sim, (84, 84), 
                                                50, 0.1, 10.0, 
                                                Transform.from_xyz_rpy(0, -0.05, 0, 0, 0, np.deg2rad(90)),
                                                self.robot.links['gripper_cam']) if cfg.gripper_camera else None

        self.noise_samplers = {k: NoiseSampler(s.shape, 
                                               cfg.noise[k].variance, 
                                               cfg.noise[k].constant) for k, s in self.observation_space.sample().items() if k in cfg.noise}

        # print(f'Original: {temp_eef_pose}\nResolved EEF state: {self.eef.pose}\nDesired: {self._init_pose}\nPeg pos: {peg_position}')

        if cfg.robot.controller == 'relative':
            self.controller     = CartesianRelativePointCOrientationController(self.robot, self.eef)
        elif cfg.robot.controller == 'virtual':
            self.controller     = CartesianRelativeVPointCOrientationController(self.robot, self.eef, 0.02)


        self._elapsed_steps = 0

    @property
    def config_space(self):
        """Returns the names of all sampled parameters in this scene.

        Returns:
            list: List of parameter names.
        """
        return sum([[f'{k}_noise_{x}' for x in 'xyz'] for k in self.noise_samplers.keys()], []) + \
                    [f'drawer_pose_{x}' for x in 'x,y,z,qx,qy,qz,qw'.split(',')] + \
                    [f'ee_pose_{x}' for x in 'x,y,z,qx,qy,qz,qw'.split(',')]

    def config_dict(self):
        """Returns the complete state of the environment as a dictionary.
           The dictionary can be used to restore the environment to a previous state.

        Returns:
            dict: Current scene state.
        """
        out = {} 
        for k, n in self.noise_samplers.items(): 
            out.update(dict(zip([f'{k}_noise_{x}' for x in 'xyz'], n.sample())))
        out.update(dict(zip([f'drawer_pose_{x}' for x in 'x,y,z,qx,qy,qz,qw'.split(',')], self.shelf.pose.array())))
        out.update(dict(zip([f'ee_pose_{x}' for x in 'x,y,z,qx,qy,qz,qw'.split(',')], self.eef.pose.array())))
        return out

    @property
    def visualizer(self):
        return self.sim.visualizer

    @property
    @lru_cache(1)
    def observation_space(self):
        d = DictSpace({'position':      BoxSpace(low=self.workspace.min.numpy(), 
                                                 high=self.workspace.max.numpy()),
                       'gripper_width': BoxSpace(low=0.03, high=0.11, shape=(1,)),
                       'force':         BoxSpace(np.ones(3) * -5, np.ones(3) * 5),
                       'torque':        BoxSpace(np.ones(3) * -5, np.ones(3) * 5),
                       'drawerpos':     BoxSpace(low=0.00, high=1.5708, shape=(1,)),
                       'handlepos':     BoxSpace(low=0.00, high=0.7854, shape=(1,)),
                       })
        if self.gripper_camera is not None:
            d['rgb_gripper'] = BoxSpace(low=-1, high=1, shape=(3, 84, 84))
        return d

    @property
    @lru_cache(1)
    def action_space(self):
        """End effector position and gripper width relative displacement"""
        return DictSpace({'motion':  BoxSpace(-np.ones(3), np.ones(3)),
                          'gripper': BoxSpace(0, 1, shape=(1,))})

    @property
    def neutral_action(self):
        """Generates an action which does not change the scene.

        Returns:
            dict: Neutral action.
        """
        return {'motion': np.zeros(3), 'gripper': 0.0}


    @property
    def render_size(self):
        return (640, 480, 3)

    def render(self, mode='rgb_array'):
        return self.render_camera.rgb()
    
    def get_camera_obs(self):
        rgb = np.moveaxis(self.gripper_camera.rgb(), 2, 0)
        return rgb
    
    def reset(self, initial_conditions=None):
        """Resets the environment. Optionally the state can be set using a dictionary
           returned by config_dict().

        Args:
            initial_conditions (dict, optional): Scene configuration to override sampling.

        Returns:
            dict: First observation after reset.
        """
        if initial_conditions is not None:
            raise NotImplementedError

        if self.visualizer is not None:
            dbg_pos, dbg_dist, dbg_pitch, dbg_yaw = self.visualizer.get_camera_position()

            dbg_rel_pos = dbg_pos - self.shelf.pose.position

        for v in self.noise_samplers.values():
            v.reset()

        self.sim.reset()

        shelf_position  = Point3(*self.board_sampler.sample())
        self.shelf.pose = Transform(shelf_position, Quaternion.from_euler(0, 0, np.pi))

        if self.visualizer is not None:
            self.visualizer.set_camera_position(self.shelf.pose.position + dbg_rel_pos, dbg_dist, dbg_pitch, dbg_yaw)

        x_goal = self.eef.pose
        # Only used to restore PID state after reset
        reset_controller = CartesianController(self.robot, self.eef)

        # Let the robot drop a bit
        for _ in range(5):
            reset_controller.act(x_goal)
            self._set_gripper_absolute_goal(0.5)
            self.sim.update()

        # Wait for PID to restore the initial position
        while (np.abs(reset_controller.delta) >= [1e-2, 0.1]).max():
            # print(f'EE: {self.eef.pose}\nDesired: {x_goal}\nDelta: {np.abs(reset_controller.delta).max()}')
            reset_controller.act(x_goal)
            self._set_gripper_absolute_goal(0.5)
            self.sim.update()

        self.controller.reset()

        self._elapsed_steps = 0

        return self.observation()

    def step(self, action):
        if type(action) != dict:
            raise Exception(f'Action needs to be a dict with the fields "motion" and "gripper"')

        action_motion = action['motion'] / max(np.abs(action['motion']).max(), 1)
        self.controller.act(action_motion * self.dt)

        if 'gripper' in action:
            self._set_gripper_absolute_goal(np.clip(action['gripper'], 0, 1))
        # print(switch)

        self.sim.update()

        obs = self.observation()
        done, success = self.is_terminated()
        reward = 100 * int(success)
        self._elapsed_steps += 1
        return obs, reward, done, {'success' : success}

    @property
    def reference_frame(self):
        return self.reference_link.pose

    def observation(self):
        out = {'position'      : self.eef.pose.position.numpy(),
               'gripper_width' : sum(self.robot.joint_state[j.name].position for j in self.gripper_joints),
               'force'         : self.eef_ft_sensor.get().linear.numpy(),
               'torque'        : self.eef_ft_sensor.get().angular.numpy(),
               'drawerpos'     : self.shelf.joint_state['drawer_top_joint'].position,
               }
        if self.gripper_camera is not None:
            out['rgb_gripper'] = self.get_camera_obs()

        for k in out:
            if k in self.noise_samplers:
                out[k] += self.noise_samplers[k].sample()
        
        return out

    def close(self):
        self.sim.kill()

    def is_terminated(self):
        """Checks if the episode is terminated

        Returns:
            tuple: (Done, Success)
        """        
        # Robot ran away
        if not self.workspace.inside(self.eef.pose.position):
            print('Robot left workspace')
            return True, False

        door_pos = self.shelf.joint_state['drawer_top_joint'].position

        # print(peg_pos_in_target)

        # Horizontal goal, vertical goal
        if door_pos >= self._target_position:
            print('Drawer is open')
            return True, True

        return False, False

    def _set_gripper_absolute_goal(self, target):
        self.robot.apply_joint_pos_cmds({j.name: self.robot.joints[j.name].q_max * target for j in self.gripper_joints}, [800]*2)