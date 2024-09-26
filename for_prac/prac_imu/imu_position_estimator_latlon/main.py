import numpy as np
import math
import matplotlib.pyplot as plt

# Earth's radius in meters
R = 6378137.0

# Function to convert meters to latitude/longitude changes
def meters_to_latlon(delta_north, delta_east, current_lat):
    """
    Convert position changes from meters (North/East) to latitude/longitude degrees.
    delta_north -> Change in North (Latitude)
    delta_east -> Change in East (Longitude)
    """
    delta_lat = delta_north / R  # Latitude update (North direction)
    delta_lon = delta_east / (R * np.cos(np.radians(current_lat)))  # Longitude update (East direction)
    return np.degrees(delta_lat), np.degrees(delta_lon)

# Simulation parameters
delta_time = 0.1  # Time step in seconds
total_time = 20.0  # Total simulation time in seconds
num_steps = int(total_time / delta_time)  # Number of simulation steps

# Initialize variables for latitude, longitude, velocity, and heading
latitude = 37.0  # Initial latitude
longitude = 127.0  # Initial longitude
velocity = np.array([0.0, 0.0])  # Initial velocity (vx, vy) in meters
heading = 0.0  # Initial heading (yaw) in degrees

# Simulation data: constant acceleration in the IMU's local frame (forward, left)
local_acc = np.array([0.0, 0.1])  # Constant forward acceleration in the local frame

# To store positions for logging and plotting
positions_log = []
latitudes = []
longitudes = []

# Simulation loop
for step in range(num_steps):
    # Simulate a gradual heading change (e.g., turning in a circle)
    heading -= 1  # Increment heading by a small amount each time step (simulating yaw change)

    # Convert heading to radians
    heading_rad = math.radians(heading)

    # Integrate velocity (v = v0 + a * dt)
    velocity += local_acc * delta_time

    # Calculate delta_north and delta_east using the provided formulas
    delta_x = velocity[0] * delta_time  # Change in x (local forward, meters)
    delta_y = velocity[1] * delta_time  # Change in y (local left, meters)

    delta_north = delta_x * math.cos(heading_rad) - -delta_y * math.sin(heading_rad)
    delta_east = delta_x * math.sin(heading_rad) + -delta_y * math.cos(heading_rad)

    # Convert delta_north and delta_east to latitude and longitude changes
    delta_lat, delta_lon = meters_to_latlon(delta_north, delta_east, latitude)

    # Update latitude and longitude
    latitude += delta_lat
    longitude += delta_lon

    # Log the current latitude, longitude, and heading for saving and plotting
    positions_log.append((latitude, longitude, heading))
    latitudes.append(latitude)
    longitudes.append(longitude)

# Save results to a file
output_file = "imu_position_latlon.txt"
with open(output_file, 'w') as f:
    for pos in positions_log:
        f.write(f"{pos[0]}, {pos[1]}, {math.degrees(pos[2])}\n")

print(f"Latitude, Longitude, and Heading data saved to {output_file}")

# Plot the trajectory using latitude and longitude
plt.figure(figsize=(8, 6))
plt.plot(longitudes, latitudes, marker='o', color='b', label="IMU Position Trajectory")
plt.title("IMU-Based Position Estimation (Lat/Lon in ROS ENU Frame)")
plt.xlabel("Longitude")
plt.ylabel("Latitude")
plt.grid(True)
plt.legend()

# Save the plot as an image file
output_image_file = "imu_position_plot_latlon.png"
plt.savefig(output_image_file)
print(f"Plot saved as {output_image_file}")

# Optionally, you can display the plot
# plt.show()  # Uncomment this if you want to show the plot interactively
