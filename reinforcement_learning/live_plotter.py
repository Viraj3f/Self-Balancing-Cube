import matplotlib.pyplot as plt
import numpy as np


class LivePlotter:
    def __init__(self, env, max_t, terminating_angle, n_episodes):
        # Create axes for plotting
        fig = plt.figure(figsize=(10, 7))

        self.theta_plot = fig.add_subplot(2, 3, 1)
        self.theta_plot.set_xlim(0, max_t)
        self.theta_plot.set_ylim(-terminating_angle, terminating_angle)
        self.theta_plot.set_xlabel("Time step (t = 0.01 s)")
        self.theta_plot.set_ylabel("Angle [degrees]")
        self.theta_plot.set_title("Angle")

        self.theta_plot_zoom = fig.add_subplot(2, 3, 2)
        self.theta_plot_zoom.set_xlim(0, max_t)
        self.theta_plot_zoom.set_ylim(-5, 5)
        self.theta_plot_zoom.set_xlabel("Time step (t = 0.01 s)")
        self.theta_plot_zoom.set_ylabel("Angle [degrees]")
        self.theta_plot_zoom.set_title("Angle (Zoomed)")

        self.current_plot = fig.add_subplot(2, 3, 3)
        self.current_plot.set_xlim(0, max_t)
        self.current_plot.set_ylim(-env.max_current, env.max_current)
        self.current_plot.set_xlabel("Time step (t = 0.01 s)")
        self.current_plot.set_ylabel("I [A]")
        self.current_plot.set_title("Current")

        self.phi_plot = fig.add_subplot(2, 3, 4)
        self.phi_plot.set_xlim(0, max_t)
        self.phi_plot.set_ylim(
                -np.rad2deg(env.max_speed), np.rad2deg(env.max_speed))
        self.phi_plot.set_xlabel("Time step (t = 0.01 s)")
        self.phi_plot.set_ylabel("Angular velocity [deg/s]")
        self.phi_plot.set_title("Angular velocity")

        self.score_plot = fig.add_subplot(2, 3, 5)
        self.score_plot.set_xlim(0, n_episodes)
        self.score_plot.set_xlabel("Episode")
        self.score_plot.set_ylabel("Score")
        self.score_plot.set_title("Score")

        plt.tight_layout()

        # Create lists for storing plot data
        self.scores = []
        self.reset()

    def reset(self):
        self.theta_b = []
        self.current = []
        self.phi_b = []

    def add_data_from_env(self, env):
        self.theta_b.append(np.rad2deg(env.cs.current_theta_b))
        self.current.append(env.cs.I_val)
        self.phi_b.append(np.rad2deg(env.cs.current_phi_b))

    def add_score(self, score):
        self.scores.append(score)

    def display(self):
        # Display the plots
        if len(self.theta_plot.lines) >= 3:
            del self.theta_plot.lines[0]
            del self.theta_plot_zoom.lines[0]
            del self.current_plot.lines[0]
            del self.phi_plot.lines[0]

        if len(self.score_plot.lines) >= 1:
            del self.score_plot.lines[0]

        self.theta_plot.plot(self.theta_b)
        self.theta_plot_zoom.plot(self.theta_b)
        self.current_plot.plot(self.current)
        self.phi_plot.plot(self.phi_b)
        self.score_plot.plot(self.scores, 'r')
        plt.draw()
        plt.pause(0.0001)
