import numpy as np


def runge_kutta_iteration(f, y, h):
    """
    Simulates a step using fourth order runge_kutta approximations
    """
    k1 = h * f(y)
    k2 = h * f(y + k1 / 2)
    k3 = h * f(y + k2 / 2)
    k4 = h * f(y + k3)
    return 1 / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


class CubeSystem:
    """
    Simulates the coupled ODEs:

    theta_dot_b = phi_b
    theta_dot_w = phi_w
    phi_dot_b = a0 * phi_b + a1 * phi_w + a2 * sin(theta_b) + a3 * T
    phi_dot_w = b0 * phi_b + b1 * phi_w + b2 * sin(theta_b) + b3 * T
    """
    def __init__(self, terminating_angle=np.pi/2):
        # System parameters
        l = 0.085  # noqa: E741
        lb = 0.075
        mb = 0.419
        mw = 0.204
        Ib = 3.34e-3
        Iw = 0.57e-3
        Cb = 1.02e-3
        Cw = 0.05e-3
        g = 9.81

        self.a0 = -Cb / (Ib + mw * l**2)
        self.a1 = Cw / (Ib + mw * l**2)
        self.a2 = (mb * lb + mw * l) * g / (Ib + mw * l**2)
        self.a3 = -1 / (Ib + mw * l**2)

        self.b0 = -self.a0
        self.b1 = (Ib + Iw + mw * l**2) / (Iw * (Ib + mw * l**2)) * -Cw
        self.b2 = self.a2
        self.b3 = (Ib + Iw + mw * l**2) / (Iw * (Ib + mw * l**2))

        # Initial values
        self.current_theta_b = 0
        self.current_theta_w = 0
        self.current_phi_b = 0
        self.current_phi_w = 0
        self.I_val = 0

        self.terminating_angle = terminating_angle

    # System equations
    def f1(self, y):
        """
        Used for solving theta_dot = phi
        """
        return y

    def f3(self, y):
        """
        Elements of y are phi_b, phi_w, theta_b and T
        """
        phi_b = y[0]
        phi_w = y[1]
        theta_b = y[2]
        T = y[3]
        return self.a0 * phi_b + self.a1 * phi_w + \
            self.a2 * np.sin(theta_b) + self.a3 * T

    def f4(self, y):
        """
        Elements of y are phi_b, phi_w, theta_b and T
        """
        phi_b = y[0]
        phi_w = y[1]
        theta_b = y[2]
        T = y[3]
        return self.b0 * phi_b + self.b1 * phi_w + \
            self.b2 * np.sin(theta_b) + self.b3 * T

    def reset(self):
        """
        Resets all the system states
        """
        self.current_theta_b = 0
        self.current_theta_w = 0
        self.current_phi_b = 0
        self.current_phi_w = 0
        self.I_val = 0

    def simulate(self, max_time, h):
        # Simulation parameters
        N = int(max_time/h)
        phi_b = np.zeros(N)
        phi_w = np.zeros(N)
        theta_b = np.zeros(N)
        theta_w = np.zeros(N)
        time = np.zeros(N)

        # Initial values
        theta_b[0] = self.current_theta_b
        theta_w[0] = self.current_theta_w
        phi_b[0] = self.current_phi_b
        phi_w[0] = self.current_phi_w

        I_values = []

        passed_terminating_angle = False
        for i in range(0, N - 1):
            if abs(theta_b[i]) > self.terminating_angle:
                passed_terminating_angle = True

            T = self.I_val
            self._update_system(i, h, theta_b, theta_w, phi_b, phi_w, time, T)
            I_values.append(self.I_val)
            yield time[i], theta_b[i], phi_b[i], passed_terminating_angle

        return time[-1], theta_b[-1], phi_b[-1], True

    def _update_system(self, i, h, theta_b, theta_w, phi_b, phi_w, time, T):
        y = np.array([phi_b[i], phi_w[i], theta_b[i], T])

        # Iterate theta_b
        theta_b[i + 1] = \
            np.clip(
                theta_b[i] + runge_kutta_iteration(self.f1, phi_b[i], h),
                -np.pi / 2,
                np.pi / 2)

        # Iterate theta_w
        theta_w[i + 1] = theta_w[i] + \
            runge_kutta_iteration(self.f1, phi_w[i], h)

        # Iterate phi_b
        phi_b[i + 1] = phi_b[i] + \
            runge_kutta_iteration(self.f3, y, h)

        # Iterate phi_w
        phi_w[i + 1] = phi_w[i] + \
            runge_kutta_iteration(self.f4, y, h)

        # Iterate time
        time[i + 1] = time[i] + h

        # Update state variables
        self.current_theta_b = theta_b[i + 1]
        self.current_theta_w = theta_w[i + 1]
        self.current_phi_b = phi_b[i + 1]
        self.current_phi_w = phi_w[i + 1]
