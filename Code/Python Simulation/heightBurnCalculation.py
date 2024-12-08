import matplotlib.pyplot as plt
import numpy as np

mass = 0.966
g = 9.81

f15 = [(0, 0), (0.15, 7.5), (0.25, 12.2), (0.3, 17), (0.35, 20.5), (0.39, 22.5), (0.42, 25.5), (0.48, 23), (0.5, 21), (0.7, 17.5), (1.05, 16), (1.42, 14.9), (2.5, 13), (3.4, 13), (3.45, 0)]
f15_t = [item[0] for item in f15]
f15_f = [item[1] for item in f15]

e16 = [(0, 0), (0.15, 1.75), (0.24, 6), (0.4, 18.5), (0.475, 24.5), (0.52, 27), (0.64, 22), (0.9, 19), (1.25, 17.5), (1.75, 17.4), (2.02, 18), (2.1, 0)]
e16_t = [item[0] for item in e16]
e16_f = [item[1] for item in e16]

# Time step and total simulation time
dt = 0.01  # seconds
total_time = 15  # seconds to allow full flight

# Altitude to ignite the E16 motor on descent
ignition_altitude = 26.5  # meters

# Initialize arrays for time, height, and velocity
time_steps = np.arange(0, total_time, dt)
heights = []
velocities = []
thrust_values = []  # Track thrust over time for plotting
height = 0
velocity = 0
e16_ignited = False
e16_start_time = 0

# Simulate rocket motion
for t in time_steps:    
    # Determine the thrust based on the current motor
    if not e16_ignited:
        # Use F15 thrust curve before E16 ignition
        thrust = np.interp(t, f15_t, f15_f) if t <= f15_t[-1] else 0
    else:
        # Use E16 thrust curve after ignition time
        e16_time = t - e16_start_time
        thrust = np.interp(e16_time, e16_t, e16_f) if e16_time <= e16_t[-1] else 0

    # Check if we should ignite the E16 motor during descent
    if not e16_ignited and height <= ignition_altitude and height != 0 and velocity < 0:
        print(t)
        e16_ignited = True
        e16_start_time = t  # Record the time when E16 motor is ignited
        mass -= 0.095

    # Net force (thrust - weight)
    net_force = thrust - mass * g
    acceleration = net_force / mass

    # Update velocity and height
    velocity += acceleration * dt
    height += velocity * dt

    # Stop simulation if rocket hits the ground
    if height <= 0 and velocity < 0:
        height = 0  # Ensure height stays at zero if it hits the ground

    # Append current values to lists
    heights.append(height)
    velocities.append(velocity)
    thrust_values.append(thrust)

# Plot height vs. time
plt.plot(time_steps[:len(heights)], heights, label="Height")
plt.plot(time_steps[:len(velocities)], velocities, label="Velocity")
plt.xlabel("Time (s)")
plt.ylabel("Height (m)")
plt.title("Rocket Height Over Time with E16 Ignition on Descent")
plt.legend()
plt.grid(True)
plt.show()
