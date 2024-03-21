import numpy as np
import rospy
import tf2_ros

from functools       import lru_cache
try:
    from rl_franka.panda import Panda, \
                                RobotUnresponsiveException
    import kv_lite as kv
    import kv_control as kvc
    import cv_bridge

    from kv_lite import gm
    
    from roebots import ROSVisualizer
    
    from geometry_msgs.msg import WrenchStamped as WrenchStampedMsg
    from std_msgs.msg      import Float64       as Float64Msg
    from sensor_msgs.msg   import Joy           as JoyMsg, \
                                  Image         as ImageMsg
except ModuleNotFoundError: # Just don't load this if we don't have the panda lib present
    Panda = None
    JoyMsg = type(None)
    ImageMsg = type(None)

import prime_bullet as pb
from prime_bullet import Point3,            \
                         Vector3,           \
                         Quaternion,        \
                         Transform,         \
                         AABB

from multiprocessing import RLock

from gymnasium.spaces import Box  as BoxSpace, \
                             Dict as DictSpace
from gymnasium        import Env

from .utils     import BoxSampler, \
                       NoiseSampler


VCONTROL_CLAMP = 0.06

class TotalRobotFailure(Exception):
    pass


class RealDoorEnv(Env):
    def __init__(self, cfg, show_gui=False):
        try:
            rospy.init_node('real_door_env')
        except rospy.exceptions.ROSException:
            print(f'{type(self)}: Not starting ROS node because one already exists.')

        # Only used for IK
        self._sim = pb.Simulator()
        self._sim.init('direct')
        
        with open('/tmp/robot_description.urdf', 'w') as f:
            f.write(rospy.get_param('robot_description'))

        self._ik_model = self._sim.load_urdf('/tmp/robot_description.urdf', useFixedBase=True)


        self.workspace = AABB(Point3(0.4, -0.30,  0.2), 
                              Point3(0.8,  0.18,  0.6))

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.lin_stiffness = cfg.robot.stiffness.linear
        self.ang_stiffness = cfg.robot.stiffness.angular

        self._robot   = Panda(cfg.robot.state_prefix, 
                              cfg.robot.controllers_prefix)

        self._ref_frame            = cfg.reference_frame
        self._door_frame           = cfg.door_frame
        self._door_measurement_frame = cfg.door_measurement_frame
        self._target_rotation      = cfg.open_angle
        self._handle_safe_height   = cfg.handle_safe_height
        self._handle_safe_delta    = cfg.handle_safe_delta
        self._f_ext_limit  = np.asarray(cfg.robot.f_ext_limit)
        self._is_fmm       = cfg.robot.is_fmm

        self._robot_T_ref         = None
        self._ref_T_robot         = None
        self._current_drawer_pose = None
        self._ref_R_door          = None
        self._should_reset        = False

        self._robot_frame = cfg.robot.reference_frame
        self._ee_frame    = cfg.robot.endeffector_frame

        self._arm_reset_pose = np.asarray(cfg.robot.initial_pose.q)

        self._ik_model.set_joint_positions(self._arm_reset_pose)
        self._ik_ee_link = self._ik_model.links[self._ee_frame]
        self._ik_ee_rot  = self._ik_ee_link.pose.quaternion
        self._ee_rot     = None

        self._vis = ROSVisualizer('~/vis', world_frame='panda_link0') if show_gui else None

        self.starting_position_sampler = BoxSampler(cfg.robot.initial_pose.position.min,
                                                    cfg.robot.initial_pose.position.max)

        self.noise_samplers = {}

        self.ref_P_v_goal = None

        self.dt = 1 /30

        self.pub_lin_stiffness = rospy.Publisher('/controllers/cartesian_impedance_controller/update_stiffness/linear/f', 
                                                 Float64Msg, tcp_nodelay=True, queue_size=1)
        self.pub_ang_stiffness = rospy.Publisher('/controllers/cartesian_impedance_controller/update_stiffness/angular/f', 
                                                 Float64Msg, tcp_nodelay=True, queue_size=1)

        self._sub_joy = rospy.Subscriber('/joy', JoyMsg, callback=self._cb_joy, queue_size=1)

        if cfg.static_camera:
            self._cv_bridge = cv_bridge.CvBridge()
            self._image_crop = np.array([cfg.image_crop[1],
                                         cfg.image_crop[0],
                                         cfg.image_crop[2]], dtype=np.int32)
            self._sub_rgb   = rospy.Subscriber('/camera/color/image_raw', ImageMsg, self._cb_camera_frame, queue_size=1)
        else:
            self._sub_rgb = None

        self._elapsed_steps = 0
        self._n_reset = 0
        self._joint_reset_every = cfg.robot.joint_reset_frequency
        self._goal_lookup_timer = rospy.Timer(rospy.Duration(0.1), self._goal_look_up)

    def _cb_camera_frame(self, msg : ImageMsg):
        self._last_image = self._cv_bridge.imgmsg_to_cv2(msg)[self._image_crop[0]:self._image_crop[0]+self._image_crop[2],
                                                              self._image_crop[1]:self._image_crop[1]+self._image_crop[2], :3]

    def _goal_look_up(self, *args):
        try:
            if self._door_measurement_frame is not None:
                tf_stamped = self.tfBuffer.lookup_transform(self._door_measurement_frame, self._door_frame, rospy.Time(0))
            else:
                tf_stamped = self.tfBuffer.lookup_transform(self._ref_frame, self._door_frame, rospy.Time(0))

            self._ref_R_door = Quaternion(tf_stamped.transform.rotation.x, 
                                          tf_stamped.transform.rotation.y, 
                                          tf_stamped.transform.rotation.z,
                                          tf_stamped.transform.rotation.w)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

    @property
    def config_space(self):
        return sum([[f'{k}_noise_{x}' for x in 'xyz'] for k in self.noise_samplers.keys()], []) + \
                    [f'ee_pose_{x}' for x in 'x,y,z,qx,qy,qz,qw'.split(',')]

    def config_dict(self):
        out = {}
        for k, n in self.noise_samplers.items(): 
            out.update(dict(zip([f'{k}_noise_{x}' for x in 'xyz'], n.sample())))
        robot_P_ee = self._robot.state.O_T_EE[:3, 3].flatten()
        robot_R_ee = Quaternion.from_matrix(self._robot.state.O_T_EE)
        out.update(dict(zip([f'ee_pose_{x}' for x in 'x,y,z'.split(',')], robot_P_ee)))
        out.update(dict(zip([f'ee_pose_{x}' for x in 'qx,qy,qz,qw'.split(',')], robot_R_ee)))
        return out

    @property
    def visualizer(self):
        return self._vis

    @property
    @lru_cache(1)
    def observation_space(self):
        out = DictSpace({'position':      BoxSpace(low=self.workspace.min.numpy(), 
                                                    high=self.workspace.max.numpy()),
                        #   'gripper_width': BoxSpace(low=0.03, high=0.11, shape=(1,)),
                          'force':         BoxSpace(np.ones(3) * -5, np.ones(3) * 5),
                          'torque':        BoxSpace(np.ones(3) * -5, np.ones(3) * 5)
                         })
        if self._sub_rgb is not None:
            out['rgb_static'] = BoxSpace(low=-1, high=1, shape=(3, self._image_crop[2], self._image_crop[2]))
        return out

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
        return {'motion': np.zeros(3), 'gripper': 0.5}

    @property
    def render_size(self):
        return (640, 480, 3)

    def render(self, mode='rgb_array'):
        return None

    def _is_tower_collision(self, q=None):
        if not self._is_fmm:
            return False

        q = self._robot.state.q if q is None else q
        # Experimentally found -0.6 * q0^2 - 0.45 as approximation for tower collisions
        q1_threshold = -1 * q[0]**6 - 0.45
        return q[1] < q1_threshold

    def home(self):
        rospy.sleep(0.3)
        for tries in range(50):
            try:
                self._robot.authorize_reset()
                self._robot.move_joint_position(self._arm_reset_pose, 0.15)
                break
            except RobotUnresponsiveException:
                print(f'Robot is not moving. Trying again {tries}')
        else:
            raise TotalRobotFailure('Robot does not move.')

    def _run_evac_behavior(self):
        """Decides which strategy to take when moving the gripper back from the handle."""
        while self._ref_R_door is None:
            print('Waiting for door to be registered')
            rospy.sleep(0.05)
        
        ref_T_ee_goal = self._ref_T_robot.dot(Transform.from_matrix(self._robot.state.O_T_EE))
        if ref_T_ee_goal.position.z <= 0.26 or ref_T_ee_goal.position.z >= 0.42:
            mode = 'move_back'
        else:
            mode = 'move_to_center'
        
        while True:
            if mode == 'move_to_center':
                ref_T_ee_goal = self._ref_T_robot.dot(Transform.from_matrix(self._robot.state.O_T_EE))

                if np.abs(self._handle_safe_height - ref_T_ee_goal.position.z) > self._handle_safe_delta:
                    robot_T_ee_goal = self._robot.state.O_T_EE * 1
                    robot_T_ee_goal[2, 3] += 0.03 * np.sign(self._handle_safe_height - ref_T_ee_goal.position.z)
                    self._robot.async_move_ee_cart_absolute(robot_T_ee_goal)
                    # print('bot')
                else:
                    angle = self._calculate_door_angle()
                    if angle <= 0.08:
                        break
                    print(f'Waiting for door angle to be less than 0.08. Currently: {angle}')
            elif mode == 'move_back':
                ref_T_ee_goal = self._ref_T_robot.dot(Transform.from_matrix(self._robot.state.O_T_EE))

                if ref_T_ee_goal.position.x > 0.54:
                    robot_T_ee_goal = self._robot.state.O_T_EE * 1
                    robot_T_ee_goal[0, 3] -= 0.03
                    self._robot.async_move_ee_cart_absolute(robot_T_ee_goal)
                else:
                    angle = self._calculate_door_angle()
                    if angle <= 0.08:
                        break
                    print(f'Waiting for door angle to be less than 0.08. Currently: {angle}')
            
            rospy.sleep(0.3)

    def reset(self, initial_conditions=None):
        if initial_conditions is not None:
            raise NotImplementedError

        # Establish reference frame if it's not existing
        # Reset reference frame every episode to accomodate object drift
        self._robot_T_ref = None
        while self._robot_T_ref is None:
            try:
                tf_stamped = self.tfBuffer.lookup_transform(self._robot_frame, self._ref_frame, rospy.Time(0))
                


                quat = Quaternion(tf_stamped.transform.rotation.x,
                                  tf_stamped.transform.rotation.y,
                                  tf_stamped.transform.rotation.z,
                                  tf_stamped.transform.rotation.w)

                self._robot_T_ref = Transform(Point3(tf_stamped.transform.translation.x, 
                                                     tf_stamped.transform.translation.y, 
                                                     tf_stamped.transform.translation.z),
                                              quat)
                self._ref_T_robot = self._robot_T_ref.inv()
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print(f'Trying to lookup reference frame {self._ref_frame} in {self._robot_frame}')

        print('Moving robot to save pose to abandon handle...')
        self._robot.cm.recover_error_state()
        self._run_evac_behavior()

        print('Moved robot to safe pose to leave handle.')
        while not self._robot.is_operational:
            print('Waiting for robot to become operational again...')
            rospy.sleep(0.3)

        # Reset joint space position every couple of episodes
        if self._n_reset % self._joint_reset_every == 0:
            print('Moving robot to home...')
            self.home()

        print('Calculating new starting pose...')
        while True:
            starting_position = self._robot_T_ref.dot(Point3(*self.starting_position_sampler.sample()))
            starting_pose     = Transform(starting_position, self._ik_ee_rot)

            # Exclude gripper from IK result
            q_start = self._ik_ee_link.ik(starting_pose)[:7]

            # if ik_success:
            if self._is_tower_collision(q_start):
                print('IK SOLUTION WOULD COLLIDE WITH TOWER. GOING AGAIN.')
                continue

            break

        print(f'Moving robot to new starting pose: {starting_pose} ({q_start})')
        rospy.sleep(0.3)
        for tries in range(30):
            try:
                self._robot.authorize_reset()
                self._robot.move_joint_position(q_start, vel_scale=0.15)
                break
            except RobotUnresponsiveException:
                print(f'Robot is not moving. Trying again {tries}')
        else:
            raise TotalRobotFailure('Robot does not move.')


        self._ee_rot = Quaternion.from_matrix(self._robot.state.O_T_EE)
        self._robot.set_gripper_position(0.08)

        self.ref_P_v_goal = Point3(*self._robot.state.O_T_EE[:3, 3].flatten())

        self._robot.cm.activate_controller(self._robot.CART_IMPEDANCE_CONTROLLER)
        rospy.sleep(0.3)
        msg_lin_stiffness = Float64Msg(data=self.lin_stiffness)
        msg_ang_stiffness = Float64Msg(data=self.ang_stiffness)
        self.pub_lin_stiffness.publish(msg_lin_stiffness)
        self.pub_ang_stiffness.publish(msg_ang_stiffness)
        rospy.sleep(0.3)

        if self._sub_rgb is not None:
            while self._last_image is None:
                print('Waiting for new RGB observation')
                rospy.sleep(0.05)

        print('Reset done!')
        self._elapsed_steps = 0
        
        self._n_reset +=1

        self._should_reset = False

        return self.observation()

    def step(self, action):
        if type(action) != dict:
            raise Exception(f'Action needs to be a dict with the fields "motion" and "gripper"')

        action_motion = Vector3(*(action['motion'] / max(np.abs(action['motion']).max(), 1)))

        ref_T_ee  = self._ref_T_robot.dot(Transform.from_matrix(self._robot.state.O_T_EE))
        ref_P_ee_pos = ref_T_ee.position
        
        self.ref_P_v_goal += action_motion * self.dt

        ee_vp_delta = self.ref_P_v_goal - ref_P_ee_pos

        if ee_vp_delta.norm() > VCONTROL_CLAMP:
            self.ref_P_v_goal = ref_P_ee_pos + (ee_vp_delta * Vector3(1, 0.9, 1)).normalized() * VCONTROL_CLAMP

        ref_T_ee_goal = Transform(self.ref_P_v_goal, ref_T_ee.quaternion)

        robot_T_ee_goal = self._robot_T_ref.dot(ref_T_ee_goal)
        robot_T_ee_goal.quaternion = self._ee_rot

        if self._vis is not None:
            self._vis.begin_draw_cycle('action')
            self._vis.draw_vector('action', self._robot.state.O_T_EE[:3, 3], 
                                            self._robot_T_ref.dot(action_motion))
            self._vis.draw_poses('action', np.eye(4), 0.1, 0.003, [robot_T_ee_goal.matrix()])
            self._vis.render('action')
        
        self._robot.async_move_ee_cart_absolute(robot_T_ee_goal.matrix())

        rospy.sleep(self.dt)

        obs = self.observation()
        done, success = self.is_terminated()
        reward = 100 * int(success)
        self._elapsed_steps += 1
        return obs, reward, done, {'success' : success}

    def observation(self):
        while self._ref_T_robot is None or self._robot.state.O_T_EE is None:
            print('Waiting for reference frame and endeffector frame')
            rospy.sleep(0.1)

        out = {'position'      : (self._ref_T_robot.dot(Point3(*self._robot.state.O_T_EE[:3, 3].flatten()))).numpy(),
            #    'gripper_width' : sum(self.robot.joint_state[j.name].position for j in self.gripper_joints),
               'force'         : (self._ref_T_robot.dot(Vector3(*self._robot.state.ext_force))).numpy(),
               'torque'        : (self._ref_T_robot.dot(Vector3(*self._robot.state.ext_torque))).numpy()}
        
        if self._sub_rgb is not None:
            while self._last_image is None:
                rospy.sleep(0.05)
            out['rgb_static'] = np.moveaxis(self._last_image, 2, 0)
        return out

    def close(self):
        pass

    def _cb_joy(self, msg : JoyMsg):
        if msg.buttons[1] > 0:
            self._should_reset = True

    def is_terminated(self):
        """Checks if the episode is terminated

        Returns:
            tuple: (Done, Success)
        """
        # Human reset
        if self._should_reset:
            print('Human has signaled end of episode.')
            return True, False

        # Robot has faulted
        if not self._robot.is_operational:
            print('Termination due to inoperative robot')
            return True, False

        # Robot ran away
        if not self.workspace.inside(self._ref_T_robot.dot(Transform.from_matrix(self._robot.state.O_T_EE).position)):
            print('EE is not in safety area')
            return True, False

        if self._is_tower_collision():
            print('Robot is about to collide with the tower')
            return True, False

        if self._robot.state.ext_force is not None:
            force_violation = max(np.abs(self._robot.state.ext_force) > self._f_ext_limit)
            if force_violation:
                print(f'External force violation: {np.abs(self._robot.state.ext_force)} > {self._f_ext_limit}')
                return True, False

        # Horizontal goal, vertical goal
        if self._ref_R_door is not None:
            angle = self._calculate_door_angle() 
            if angle >= self._target_rotation:
                print(f'Terminated due to door being open (angle: {angle})')
                return True, True

        return False, False

    def _calculate_door_angle(self):
        while self._ref_R_door is None:
            pass

        return np.arccos((self._ref_R_door.dot(-Vector3.unit_x()) * Vector3(1, 1, 0)).normalized().y) 

    def _set_gripper_absolute_goal(self, target):
        self.robot.apply_joint_pos_cmds({j.name: self.robot.joints[j.name].q_max * target for j in self.gripper_joints}, [800]*2)


# Let's just not force the installation of the entire FMM environment
if Panda is None:
    class RealDrawerEnv(object):
        def __init__(self, *args, **kwargs):
            raise NotImplementedError(f'The {type(self)} environment can only be instantiated in an environment with the rl_franka library installed.')
