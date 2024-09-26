import numpy as np
import math

# Helper function to convert IMU acceleration to global frame using heading (yaw)
def rotate_acceleration(ax, ay, theta):
    """
    Rotate the local IMU frame acceleration (ax, ay) into the global frame using the yaw (theta).
    """
    # Rotation matrix for yaw (heading)
    R = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])
    acc = np.array([ax, ay])
    
    # Return acceleration in the global frame
    return np.dot(R, acc)

# Simulation parameters
delta_time = 0.1  # Time step in seconds
total_time = 20.0  # Total simulation time in seconds
num_steps = int(total_time / delta_time)  # Number of simulation steps

# Initialize variables for position, velocity, and heading
position = np.array([0.0, 0.0])  # Initial position (x, y)
velocity = np.array([0.0, 0.0])  # Initial velocity (vx, vy)
heading = 0.0  # Initial heading (yaw) in radians

# Simulation data: constant acceleration in the IMU's local frame
local_acc = np.array([0.1, 0.0])  # Constant forward acceleration in the local frame

# To store positions for logging
positions_log = []

# Simulation loop
for step in range(num_steps):
    # Simulate a gradual heading change (e.g., turning in a circle)
    heading += 0.02  # Increment heading by a small amount each time step (simulating yaw change)

    # Rotate acceleration from local frame to global frame
    global_acc = rotate_acceleration(local_acc[0], local_acc[1], heading)

    # Integrate velocity (v = v0 + a * dt)
    velocity += global_acc * delta_time

    # Integrate position (x = x0 + v * dt)
    position += velocity * delta_time

    # Log the current position
    positions_log.append((position[0], position[1]))

# Save results to a file
output_file = "imu_position_estimation.txt"
with open(output_file, 'w') as f:
    for pos in positions_log:
        f.write(f"{pos[0]}, {pos[1]}\n")

print(f"Position data saved to {output_file}")
