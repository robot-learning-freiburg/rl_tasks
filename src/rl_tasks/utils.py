import numpy as np

class BoxSampler(object):
    """Samples a value uniformly from an n-dimensional space.

    Args:
        b_min (list, np.ndarray): List of minimum values for each dimension.
        b_max (list, np.ndarray): List of maximum values for each dimension.
    """
    def __init__(self, b_min, b_max):
        if len(b_min) != len(b_max):
            raise Exception(f'Box bounds need to be the same size. Min: {len(b_min)} Max: {len(b_max)}')

        self.b_min = np.asarray(b_min)
        self.b_max = np.asarray(b_max)
    
    def sample(self):
        """Samples a value uniformly from the box."""
        return (np.random.random(len(self.b_min)) * (np.asarray(self.b_max) - np.asarray(self.b_min))) + np.asarray(self.b_min)

    @property
    def center(self):
        """Returns the center-point of the box."""
        return 0.5 * (self.b_min + self.b_max)

class NoiseSampler(object):
    def __init__(self, dim, var, constant) -> None:
        """Samples a value from an n-dimensional zero-mean normal distribution.

        Args:
            dim (tuple): Dimensionality of the sample.
            var (tuple): Variance of the normal distribution.
            constant (bool): Calls to sample() will return the same value until reset() is called.
        """
        self.constant = constant
        self.dim = dim
        self.var = var
        self._noise = None
        self.reset()
    
    def sample(self):
        """Returns the current sample."""
        if self.constant and self._noise is not None:
            return self._noise
        return np.random.normal(0, self.var, self.dim)
        
    def reset(self):
        """Resets the sampler to a new sample."""
        self._noise = None
        self._noise = self.sample()

    def set_sample(self, sample):
        """Overrides the current sample."""
        if type(sample) != np.ndarray:
            raise Exception(f'Expected noise sample to be a numpy array but got {type(sample)}')
        
        if sample.shape != self.dim:
            raise Exception(f'Expected noise sample to be of shape {self.dim}, but got {sample.dim}')
        
        self._noise = sample

try:
    import rospy

    from sensor_msgs.msg import Joy as JoyMsg

    class Gamepad():
        def __init__(self, joystick_topic='/joy') -> None:
            self._button_names = 'a b x y lb rb cl cr unknown l3 r3'.split(' ')
            self._button_indices = {b: x for x, b in enumerate(self._button_names)}
            self.reset()

            self._sub_js = rospy.Subscriber(joystick_topic, JoyMsg, self._cb_joy, queue_size=1)

        def reset(self):
            self._left_stick   = np.zeros(2)
            self._right_stick  = np.zeros(2)
            self._d_pad        = np.zeros(2)
            self._left_trigger  = 0.0
            self._right_trigger = 0.0
            self._button_states = None

        def _cb_joy(self, msg : JoyMsg):
            self._left_stick    = np.asarray(msg.axes[:2])
            self._left_trigger  = msg.axes[2]
            self._right_stick   = np.asarray(msg.axes[3:5])
            self._right_trigger = msg.axes[5]
            self._d_pad         = np.asarray(msg.axes[-2:])
            self._button_states = msg.buttons

        @property
        def left_stick(self):
            return self._left_stick

        @property
        def right_stick(self):
            return self._right_stick

        @property
        def dpad(self):
            return self._dpad

        @property
        def stacked_axes(self):
            """Returns the state of all "axes" values as column vector.
               Meant to apply a linear map to it.

               Order is: [L-stick, R-stick, D-pad, LT, RT, A, B, X, Y, LB, RB, CL, CR, U, L3, R3].T 
            """
            if self._button_states is None:
                raise RuntimeError(f'Gamepad has received no messages. Please use .is_ready to wait for that to change.')

            return np.hstack((self._left_stick, self._right_stick, self._d_pad, 
                              (self._left_trigger, self._right_trigger), self._button_states)).reshape((19, 1))

        @property
        def is_ready(self):
            return self._button_states is not None

        def is_pressed(self, key):
            if self._button_states is None:
                raise RuntimeError(f'Gamepad has received no messages. Please use .is_ready to wait for that to change.')

            key = key.lower()
            if key in self._button_indices:
                return self._button_states[self._button_indices[key]]
            raise KeyError(f'Unknown key "{key}". Options are: {list(self._button_states.keys())}')


except ModuleNotFoundError:
    class Gamepad():
        def __init__(self, *args) -> None:
            raise NotImplementedError('Need rospy to instantiate gamepad')