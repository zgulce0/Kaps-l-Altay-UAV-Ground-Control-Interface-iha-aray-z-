import matplotlib.pyplot as plt
import numpy as np
from pymavlink import mavutil
import pandas as pd
from mpl_toolkits.mplot3d import Axes3D

def read_drone_bin_file(bin_file_path):
    mlog = mavutil.mavlink_connection(bin_file_path)
    
    timestamps = []
    latitudes = []
    longitudes = []
    altitudes = []
    
    print("Reading BIN file...")
    
    while True:
        msg = mlog.recv_match()
        if msg is None:
            break
            
        if msg.get_type() == 'GPS':
            if hasattr(msg, 'Lat') and hasattr(msg, 'Lng') and hasattr(msg, 'Alt'):
                lat = msg.Lat / 1e7
                lng = msg.Lng / 1e7
                alt = msg.Alt / 1000
                
                if lat != 0 and lng != 0:
                    timestamps.append(msg.TimeUS / 1e6)
                    latitudes.append(lat)
                    longitudes.append(lng)
                    altitudes.append(alt)
        
        elif msg.get_type() == 'GLOBAL_POSITION_INT':
            lat = msg.lat / 1e7
            lng = msg.lon / 1e7
            alt = msg.alt / 1000
            
            if lat != 0 and lng != 0:
                timestamps.append(msg.time_boot_ms / 1000)
                latitudes.append(lat)
                longitudes.append(lng)
                altitudes.append(alt)
    
    return timestamps, latitudes, longitudes, altitudes

def plot_flight_path(timestamps, latitudes, longitudes, altitudes):
    """
    Create comprehensive flight path visualizations
    """
    if not latitudes:
        print("No GPS data found in the BIN file!")
        return
    
    print(f"Found {len(latitudes)} GPS points")
    
    # Create figure with subplots
    fig = plt.figure(figsize=(15, 12))
    
    # 1. 2D Flight Path (Top View)
    ax1 = plt.subplot(2, 2, 1)
    plt.plot(longitudes, latitudes, 'b-', linewidth=2, alpha=0.7, label='Flight Path')
    plt.plot(longitudes[0], latitudes[0], 'go', markersize=10, label='Start')
    plt.plot(longitudes[-1], latitudes[-1], 'ro', markersize=10, label='End')
    plt.xlabel('Longitude (degrees)')
    plt.ylabel('Latitude (degrees)')
    plt.title('2D Flight Path (Top View)')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.axis('equal')
    
    # 2. 3D Flight Path
    ax2 = fig.add_subplot(2, 2, 2, projection='3d')
    ax2.plot(longitudes, latitudes, altitudes, 'b-', linewidth=2, alpha=0.7)
    ax2.scatter(longitudes[0], latitudes[0], altitudes[0], color='green', s=100, label='Start')
    ax2.scatter(longitudes[-1], latitudes[-1], altitudes[-1], color='red', s=100, label='End')
    ax2.set_xlabel('Longitude (degrees)')
    ax2.set_ylabel('Latitude (degrees)')
    ax2.set_zlabel('Altitude (m)')
    ax2.set_title('3D Flight Path')
    ax2.legend()
    
    # 3. Altitude Profile over Time
    ax3 = plt.subplot(2, 2, 3)
    if timestamps:
        time_minutes = [(t - timestamps[0]) / 60 for t in timestamps]
        plt.plot(time_minutes, altitudes, 'r-', linewidth=2)
        plt.xlabel('Time (minutes)')
    else:
        plt.plot(range(len(altitudes)), altitudes, 'r-', linewidth=2)
        plt.xlabel('Data Point Index')
    plt.ylabel('Altitude (m)')
    plt.title('Altitude Profile')
    plt.grid(True, alpha=0.3)
    
    # 4. Speed Analysis (if we have timestamps)
    ax4 = plt.subplot(2, 2, 4)
    if timestamps and len(timestamps) > 1:
        speeds = []
        for i in range(1, len(latitudes)):
            # Calculate distance between consecutive points (rough approximation)
            dlat = latitudes[i] - latitudes[i-1]
            dlon = longitudes[i] - longitudes[i-1]
            # Convert to meters (rough approximation)
            dist = np.sqrt((dlat * 111000)**2 + (dlon * 111000 * np.cos(np.radians(latitudes[i])))**2)
            dt = timestamps[i] - timestamps[i-1]
            if dt > 0:
                speed = dist / dt  # m/s
                speeds.append(speed)
            else:
                speeds.append(0)
        
        time_minutes = [(timestamps[i] - timestamps[0]) / 60 for i in range(1, len(timestamps))]
        plt.plot(time_minutes, speeds, 'g-', linewidth=2)
        plt.xlabel('Time (minutes)')
        plt.ylabel('Speed (m/s)')
        plt.title('Ground Speed')
        plt.grid(True, alpha=0.3)
    else:
        plt.text(0.5, 0.5, 'No timestamp data\nfor speed calculation', 
                ha='center', va='center', transform=ax4.transAxes)
        plt.title('Speed Analysis')
    
    plt.tight_layout()
    plt.show()
    
    # Print flight statistics
    print("\nFlight Statistics:")
    print(f"Total GPS points: {len(latitudes)}")
    print(f"Max altitude: {max(altitudes):.1f} m")
    print(f"Min altitude: {min(altitudes):.1f} m")
    if timestamps:
        duration = (timestamps[-1] - timestamps[0]) / 60
        print(f"Flight duration: {duration:.1f} minutes")

# Main execution
def main():
    # Replace with your BIN file path
    bin_file_path = "ucus_logu.bin"  # Update this path
    
    try:
        # Read the BIN file
        timestamps, lats, lons, alts = read_drone_bin_file(bin_file_path)
        
        # Plot the flight path
        plot_flight_path(timestamps, lats, lons, alts)
        
    except FileNotFoundError:
        print(f"Error: Could not find file '{bin_file_path}'")
        print("Please update the bin_file_path variable with the correct path to your BIN file")
    except Exception as e:
        print(f"Error reading BIN file: {e}")
        print("Make sure the file is a valid drone log file and pymavlink is installed")

if __name__ == "__main__":
    main()