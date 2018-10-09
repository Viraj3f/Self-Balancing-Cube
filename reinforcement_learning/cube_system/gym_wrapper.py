import numpy as np
import gym
from gym import spaces

from cube_system.simulator import CubeSystem


class CubeEnv(gym.Env):
    def __init__(self):
        self.max_speed = np.deg2rad(500)  # max angular velocity is 500 deg/s

        self.max_current = 10  # amps

        self.action_space = spaces.Box(
            low=-1 * self.max_current,
            high=self.max_current,
            shape=(1,),
            dtype=np.float32)

        high = np.array([1., 1., self.max_speed])
        self.observation_space = spaces.Box(
                low=-high,
                high=high,
                dtype=np.float32)

        self.cs = CubeSystem(terminating_angle=np.deg2rad(15))
        self.cs_generator = None

    def render(self):
        # We're not rendering so don't do anything.
        pass

    def step(self, I):
        action = np.clip(I, -self.max_current, self.max_current)
        self.cs.update_BLDC_current(action)

        time, theta_b, phi_b, I_val, done = next(self.cs_generator)

        state = self._get_state(theta_b, phi_b)
        reward = self._get_reward(theta_b, phi_b, action)
        
        return state, reward, done, {}

    def reset(self):
        self.cs.reset()
        self.cs.current_theta_b = np.deg2rad(2)
        self.cs_generator = self.cs.simulate(
            max_time=5,
            h=0.001,
            sample_time=0.01)

        time, theta_b, phi_b, I_val, done = next(self.cs_generator)

        return self._get_state(theta_b, phi_b)

    def _get_state(self, theta_b, phi_b):
        return np.array([np.cos(theta_b), np.sin(theta_b), phi_b])

    def _get_reward(self, theta_b, phi_b, action):
        return -(theta_b**2 + 0.1*phi_b**2 + 0.001*action**2)
