import gym
import random
import torch
import numpy as np
from collections import deque
import matplotlib.pyplot as plt
from ddpg_agent import Agent

from cube_system import CubeEnv


def ddpg(n_episodes=1000, max_t=500, print_every=100):
    scores_deque = deque(maxlen=print_every)
    scores = []

    # Create the env and the agent
    terminating_angle = 15
    env = CubeEnv(np.deg2rad(terminating_angle))
    agent = Agent(state_size=3, action_size=1, random_seed=2)

    # Create axes for plotting
    theta_plot = plt.subplot(1, 2, 1)
    theta_plot.set_xlim(0, max_t)
    theta_plot.set_ylim(-terminating_angle, terminating_angle)

    current_plot = plt.subplot(1, 2, 2)
    current_plot.set_xlim(0, max_t)
    current_plot.set_ylim(-env.max_current, env.max_current)

    for i_episode in range(1, n_episodes+1):
        state = env.reset()
        agent.reset()
        score = 0

        theta_b = []
        current = []
        while True:
            action = agent.act(state) * 10
            next_state, reward, done, _ = env.step(action)
            agent.step(state, action, reward, next_state, done)
            state = next_state
            score += reward
            theta_b.append(np.rad2deg(env.cs.current_theta_b))
            current.append(env.cs.I_val)
            if done:
                break

        # Only disiplay the top 3 most recent attempts
        if len(theta_plot.lines) >= 3:
            del theta_plot.lines[0]

        if len(current_plot.lines) >= 3:
            del current_plot.lines[0]

        theta_plot.plot(theta_b)
        current_plot.plot(current)
        plt.draw()
        plt.pause(0.0001)
        scores_deque.append(score)
        scores.append(score)
        print('\rEpisode {}\tAverage Score: {:.2f}'
              .format(i_episode, np.mean(scores_deque)), end="")

        # Save model
        torch.save(agent.actor_local.state_dict(), 'checkpoint_actor.pth')
        torch.save(agent.critic_local.state_dict(), 'checkpoint_critic.pth')

    return scores


plt.ion()
scores = ddpg()

# fig = plt.figure()
# ax = fig.add_subplot(111)
# plt.plot(np.arange(1, len(scores)+1), scores)
# plt.ylabel('Score')
# plt.xlabel('Episode #')
# plt.show()
