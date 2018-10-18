import numpy as np
import matplotlib.pyplot as plt
import sys
sys.path.append('..')
from cube_system import CubeSystem
from pid import PID


cs = CubeSystem()
cs.current_theta_b = np.deg2rad(-6)
controller = PID(-80, -200, -8, 0, 0.02)

time_data = []
theta_b_data = []
current_data = []
for time, theta, _, _, _ in cs.simulate(max_time=2, h=0.001, sample_time=0.02):
    cs.update_BLDC_current(np.clip(controller.compute(theta, time), -10, 10))
    time_data.append(time)
    theta_b_data.append(np.rad2deg(theta))
    current_data.append(cs.I_val)


plt.style.use('ggplot')
plt.subplot(1, 2, 1)
plt.plot(time_data, theta_b_data)
plt.xlabel("Time [s]")
plt.ylabel("theta [degrees]")

plt.subplot(1, 2, 2)
plt.plot(time_data, current_data)
plt.xlabel("Time [s]")
plt.ylabel("BLDC Current (A)")
plt.show()
