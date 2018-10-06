import numpy as np
import matplotlib.pyplot as plt
from cube_system import CubeSystem
from pid import PID


cs = CubeSystem()
cs.current_theta_b = np.deg2rad(+2)
controller = PID(-80, -200, -8, 0, 0.01)

time_data = []
theta_b_data = []
current_data = []
for time, theta, _, _, _ in cs.simulate(max_time=5, h=0.001, sample_time=0.01):
    cs.update_BLDC_current(np.clip(controller.compute(theta, time), -10, 10))
    time_data.append(time)
    theta_b_data.append(np.rad2deg(theta))
    current_data.append(cs.I_val)


plt.subplot(1, 2, 1)
plt.plot(time_data, theta_b_data)
plt.subplot(1, 2, 2)
plt.plot(time_data, current_data)
plt.show()
