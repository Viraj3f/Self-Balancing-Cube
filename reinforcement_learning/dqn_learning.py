import random
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
from keras.models import Sequential
from keras.layers import Dense
from keras.optimizers import Adam
from cube_system import CubeSystem

EPISODES = 5000


class DQNAgent:
    """
    This model was taken from:
    https://github.com/keon/deep-q-learning/blob/master/dqn.py
    """
    def __init__(self, state_size, action_size):
        self.state_size = state_size
        self.action_size = action_size
        self.memory = deque(maxlen=2000)
        self.gamma = 0.95    # discount rate
        self.epsilon = 1.0  # exploration rate
        self.epsilon_min = 0.01
        self.epsilon_decay = 0.99
        self.learning_rate = 0.01
        self.model = self._build_model()

    def _build_model(self):
        # Neural Net for Deep-Q learning Model
        model = Sequential()
        model.add(Dense(24, input_dim=self.state_size, activation='relu'))
        model.add(Dense(24, activation='relu'))
        model.add(Dense(self.action_size, activation='linear'))
        model.compile(loss='mse',
                      optimizer=Adam(lr=self.learning_rate))
        return model

    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    def act(self, state):
        if np.random.rand() <= self.epsilon:
            return random.randrange(self.action_size)
        act_values = self.model.predict(state)
        return np.argmax(act_values[0])  # returns action

    def replay(self, batch_size):
        minibatch = random.sample(self.memory, batch_size)
        for state, action, reward, next_state, done in minibatch:
            target = reward
            if not done:
                target = (reward + self.gamma *
                          np.amax(self.model.predict(next_state)[0]))
            target_f = self.model.predict(state)
            target_f[0][action] = target
            self.model.fit(state, target_f, epochs=1, verbose=0)
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

    def load(self, name):
        self.model.load_weights(name)

    def save(self, name):
        self.model.save_weights(name)


def get_reward(observation):
    return 1


if __name__ == "__main__":
    plt.ion()
    reward_plot = plt.subplot(1, 3, 1)
    data_plot = plt.subplot(1, 3, 2)
    current_plot = plt.subplot(1, 3, 3)

    saved_model_path = "./saved_models/dqn.h5"

    # Create system and agent parameters
    cs = CubeSystem(terminating_angle=np.deg2rad(10))
    state_size = 3
    action_size = 10000
    I_vals = np.linspace(-10, 10, action_size)
    batch_size = 32
    agent = DQNAgent(state_size, action_size)

    episode_data = np.arange(EPISODES)
    reward_data = np.zeros(EPISODES)
    for e in range(EPISODES):
        # Reset the system
        cs.reset()
        cs.current_theta_b = np.deg2rad(2)
        cs.I_val = +10
        cs_generator = cs.simulate(max_time=5, h=0.001, sample_time=0.01)

        # Convert the observation to its state
        time, theta_b, phi_b, I_val, done = next(cs_generator)
        state = np.array([[theta_b, phi_b, I_val]])

        # Data for plotting
        time_data = []
        theta_b_data = []
        current_data = []
        total_reward = 0
        for step in range(500):
            # Add data to plotting arrays
            time_data.append(time)
            theta_b_data.append(np.rad2deg(theta_b))
            current_data.append(I_val)

            # Determine the next action and update the system
            action = agent.act(state)
            cs.update_BLDC_current(I_vals[action])

            # Get the next state and reward
            time, theta_b, phi_b, I_val, done = next(cs_generator)
            next_state = np.array([[theta_b, phi_b, I_val]])
            reward = get_reward(next_state)
            if done:
                reward = -10

            # Update the agent
            agent.remember(state, action, reward, next_state, done)
            state = next_state
            total_reward += reward

            if done:
                print("Episode: {}, score: {}".format(e, total_reward))
                break

            if len(agent.memory) > batch_size:
                agent.replay(batch_size)

        if e % 10 == 0:
            agent.save(saved_model_path)

        # Plotting angle and current
        data_plot.clear()
        data_plot.plot(time_data, theta_b_data)
        current_plot.clear()
        current_plot.plot(time_data, current_data)

        # Plot reward
        episode_data[e] = e
        reward_data[e] = total_reward
        reward_plot.plot(episode_data[:e + 1], reward_data[:e + 1], 'r-')

        # Update
        plt.draw()
        plt.pause(0.0001)
