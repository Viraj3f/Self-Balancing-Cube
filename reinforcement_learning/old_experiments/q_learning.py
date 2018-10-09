"""
Tests to see the performance of Q learning
"""

import random
import numpy as np
from cube_system import CubeSystem
import matplotlib.pyplot as plt


# Number of possible current values
num_actions = 100
I_vals = np.linspace(-10, 10, num_actions)

# theta_b, thetab'
num_buckets = (30, 10)

# boundings for theta
state_bounds = np.zeros((len(num_buckets), 2))

# theta_b is bounded between -45 and 45 degrees
state_bounds[0][0] = np.deg2rad(-10)
state_bounds[0][1] = np.deg2rad(+10)

# theta_b' is bounded between -50 and 50 deg/second
state_bounds[1][0] = np.deg2rad(-500)
state_bounds[1][1] = np.deg2rad(+500)

# Create the Q table
Q = np.zeros(num_buckets + (num_actions,))

# Minimum parameters
min_learning_rate = 0.2
min_explore_rate = 0


def get_state(observation):
    # Return the indexes of the the state
    state = []
    for i in range(len(observation)):
        if observation[i] <= state_bounds[i][0]:
            bucket_index = 0
        elif observation[i] >= state_bounds[i][1]:
            bucket_index = num_buckets[i] - 1
        else:
            # Mapping the state bounds to the bucket array
            bound_width = state_bounds[i][1] - state_bounds[i][0]
            offset = (num_buckets[i] - 1) * state_bounds[i][0] / bound_width
            scaling = (num_buckets[i]-1)/bound_width
            bucket_index = int(round(scaling * observation[i] - offset))
        state.append(bucket_index)

    return tuple(state)


def get_reward(observation, I_val):
    return -(observation[0]**2 + 0.1 * observation[1] + 0.001 * I_val**2)


def select_action(state, explore_rate):
    if random.random() < explore_rate:
        action = np.random.randint(0, num_actions - 1)
    else:
        action = np.argmax(Q[state])

    return action


def get_explore_rate(t):
    return max(min_explore_rate, min(1, 1.0 - np.log10((t+1)/25)))


def get_learning_rate(t):
    return max(min_learning_rate, min(0.5, 1.0 - np.log10((t+1)/25)))


def simulate():
    num_episodes = 1000
    learning_rate = get_learning_rate(0)
    explore_rate = get_explore_rate(0)
    discount_factor = 0.3  # since the world is unchanging

    cs = CubeSystem(terminating_angle=np.deg2rad(10))

    episodes = []
    timesteps = []
    for episode in range(num_episodes):
        # Reset the environment
        cs.reset()
        cs.current_theta_b = np.deg2rad(-1)

        # Generator that will yield the next value in the system
        # every time next() is called.
        cs_generator = cs.simulate(max_time=10, h=0.001, sample_time=0.01)

        # The initial state
        time, theta_b, phi_b, _, _ = next(cs_generator)
        state_0 = get_state((theta_b, phi_b))
        total_reward = 0

        timestep = 0
        should_terminate = False
        thetas = []
        times = []
        currents = []
        velocties = []
        while not should_terminate:
            # Select an action
            action = select_action(state_0, explore_rate)

            # Execute the action
            cs.I_val = I_vals[action]
            time, theta_b, phi_b, _, should_terminate = next(cs_generator)
            obv = (theta_b, phi_b)
            reward = get_reward(obv, I_vals[action])
            total_reward += reward

            # Observe the result
            state = get_state(obv)

            # Update the Q based on the result
            best_q = np.amax(Q[state])
            Q[state_0 + (action,)] += learning_rate * (reward + discount_factor*(best_q) - Q[state_0 + (action,)])

            # Setting up for the next iteration
            state_0 = state

            currents.append(cs.I_val)
            thetas.append(np.rad2deg(theta_b))
            times.append(time)
            velocties.append(np.rad2deg(phi_b))

        episodes.append(episode)
        timesteps.append(timestep)
        print("Episode: {}, timestep {}".format(episode, reward))
        if episode % 10 == 0:
            plt.subplot(2, 2, 1)
            plt.plot(times, thetas)
            plt.subplot(2, 2, 2)
            plt.plot(times, currents)
            plt.subplot(2, 2, 3)
            plt.plot(times, velocties)
            plt.subplot(2, 2, 4)
            plt.plot(episodes, timesteps)
            plt.show()

        # Update parameters
        explore_rate = get_explore_rate(episode)
        learning_rate = get_learning_rate(episode)


if __name__ == "__main__":
    simulate()
