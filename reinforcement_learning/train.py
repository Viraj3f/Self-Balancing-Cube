import torch
import numpy as np
from collections import deque
from ddpg_agent import Agent
import matplotlib.pyplot as plt

from cube_system import CubeEnv
from live_plotter import LivePlotter


def ddpg(n_episodes=1000, max_t=500, print_every=100):
    scores_deque = deque(maxlen=print_every)
    scores = []

    # Create the env and the agent
    terminating_angle = 15
    env = CubeEnv(np.deg2rad(terminating_angle))
    agent = Agent(state_size=3, action_size=1, random_seed=2)

    plotter = LivePlotter(env, max_t, terminating_angle, n_episodes)

    for i_episode in range(1, n_episodes+1):
        state = env.reset()
        agent.reset()
        score = 0
        done = False
        plotter.reset()
        while not done:
            # Select the next action and update system
            action = agent.act(state) * 10
            next_state, reward, done, _ = env.step(action)
            agent.step(state, action, reward, next_state, done)

            # Update plots and metrics
            state = next_state
            score += reward
            plotter.add_data_from_env(env)

        scores_deque.append(score)
        scores.append(score)
        plotter.add_score(score)
        print('\rEpisode {}\tScore: {}'
              .format(i_episode, score), end="")

        # Display the plotrs
        plotter.display()

        # Save model
        torch.save(agent.actor_local.state_dict(), 'checkpoint_actor.pth')
        torch.save(agent.critic_local.state_dict(), 'checkpoint_critic.pth')

    return scores


if __name__ == "__main__":
    plt.ion()
    plt.style.use('ggplot')
    scores = ddpg()
