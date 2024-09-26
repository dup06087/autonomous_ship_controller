import numpy as np
import matplotlib.pyplot as plt

# Define the path to the file with position data
input_file = "imu_position_estimation.txt"

# Load the data from the file
positions = np.loadtxt(input_file, delimiter=',')

# Extract x and y coordinates
x_positions = positions[:, 0]
y_positions = positions[:, 1]

# Plot the x, y trajectory
plt.figure(figsize=(8, 6))
plt.plot(x_positions, y_positions, marker='o', color='b', label="IMU Position Trajectory")
plt.title("IMU-Based Position Estimation")
plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.grid(True)
plt.legend()

# Save the plot as a PNG file
output_image_file = "imu_position_plot.png"
plt.savefig(output_image_file)

print(f"Plot saved as {output_image_file}")
