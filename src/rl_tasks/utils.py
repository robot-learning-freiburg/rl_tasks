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
