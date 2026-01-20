import math
import numpy as np
import matplotlib.pyplot as plt
from RK4Tracker import Combined_RK4



# plt.tight_layout()
# plt.show()

if __name__ == "__main__":
    tracker = Combined_RK4(initial_angles=[0, 0, 0], alpha=0.98, damping=1.0)
    t = 0.0
    dt = 0.01
    duration = 5.0
    while t < duration:
        gyro_data = [10.0, 0.0, 0.0]  # Simulated gyro data (degrees per second)
        accel_data = [0.0, 0.0, 9.81]  # Simulated accelerometer data (m/s^2)
        accel, gyro = tracker.update(gyro_data, accel_data, dt)

        accel = np.around(accel, decimals=3)
        gyro = np.around(gyro, decimals=3)

        print(f"time: {t:.2f}s")
        print(f"Position: {accel[0]}")
        # print(f"Velocity: {accel[1]}")
        print(f"  Angles: {gyro}")
        t += dt
    pass


''' Exapmle data
  '''