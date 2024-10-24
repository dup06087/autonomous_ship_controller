import json
from math import radians, cos, sin, atan2, sqrt, degrees

# Constants
EARTH_RADIUS = 6371000  # Earth radius in meters

# Function to compute the displacement from latitude and longitude using Haversine formula
def compute_gps_displacement(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1

    a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))

    distance = EARTH_RADIUS * c  # Distance in meters
    return distance

# Function to compute bearing between two GPS points
def compute_bearing(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])

    dlon = lon2 - lon1
    x = sin(dlon) * cos(lat2)
    y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)
    bearing = atan2(x, y)
    
    return degrees(bearing) % 360  # Normalize to 0-360 degrees

# Function to convert displacement to local x, y coordinates using initial heading
def convert_to_local_coordinates(distance, bearing, initial_heading):
    # Convert bearing and heading to radians
    bearing_rad = radians(bearing)
    initial_heading_rad = radians(initial_heading)

    # Calculate x and y displacements in the local coordinate system
    delta_x = distance * cos(bearing_rad - initial_heading_rad)
    delta_y = distance * sin(bearing_rad - initial_heading_rad)

    return delta_x, delta_y

# Function to read GNSS data from the file and match by time
def load_gnss_data(gnss_file, time1, time2):
    with open(gnss_file, 'r') as f:
        data = [json.loads(line.strip()) for line in f]

    # Find the GNSS entries that match the times
    gps_point1 = next(item for item in data if item['time'] == time1)
    gps_point2 = next(item for item in data if item['time'] == time2)

    return gps_point1, gps_point2

# Function to extract time from the point cloud filename
def extract_time_from_filename(filename):
    time_str = filename.split('_')[-1].replace('.pcd', '')  # Extract time string
    return float(time_str)  # Convert to float

# Main function to calculate the GPS displacement based on point cloud filenames
def main(pcd_file1, pcd_file2, gnss_file):
    # Extract times from the point cloud filenames
    time1 = extract_time_from_filename(pcd_file1)
    time2 = extract_time_from_filename(pcd_file2)

    print(f"Extracted times: {time1}, {time2}")

    # Load the corresponding GNSS data for the two times
    gps_point1, gps_point2 = load_gnss_data(gnss_file, time1, time2)

    # Extract latitude, longitude, and heading from both points
    lat1, lon1, heading1 = gps_point1['latitude'], gps_point1['longitude'], gps_point1['heading']
    lat2, lon2, heading2 = gps_point2['latitude'], gps_point2['longitude'], gps_point2.get('heading', heading1)

    # Compute GPS displacement (distance)
    gps_distance = compute_gps_displacement(lat1, lon1, lat2, lon2)

    # Calculate the bearing between the two points
    bearing = compute_bearing(lat1, lon1, lat2, lon2)

    # Calculate heading change
    heading_change = heading2 - heading1

    # Convert GPS displacement into local coordinates based on the initial heading
    delta_x, delta_y = convert_to_local_coordinates(gps_distance, bearing, heading1)

    # Output the results
    print(f"Point 1 (lat, lon, heading): ({lat1}, {lon1}, {heading1})")
    print(f"Point 2 (lat, lon, heading): ({lat2}, {lon2}, {heading2})")
    print(f"GPS Displacement: {gps_distance:.2f} meters")
    print(f"Bearing from Point 1 to Point 2: {bearing:.2f} degrees")
    print(f"Heading Change: {heading_change:.2f} degrees")
    print(f"Local Displacement: x = {delta_x:.2f} meters, y = {delta_y:.2f} meters (relative to initial heading)")

if __name__ == "__main__":
    pcd_file1 = "./extracted_pointclouds/pointcloud_1724857705.0.pcd"
    pcd_file2 = "./extracted_pointclouds/pointcloud_1724857705.2.pcd"
    gnss_file = "./matched_gps_data.txt"  # Path to your GNSS data file
    main(pcd_file1, pcd_file2, gnss_file)
