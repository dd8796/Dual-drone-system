from pymavlink import mavutil

# Connect to the vehicle
connection = mavutil.mavlink_connection('tcp:127.0.0.1:5762')  # Adjust IP and port if needed

# Wait for a heartbeat to establish communication
print("Waiting for heartbeat...")
connection.wait_heartbeat()
print("Heartbeat received! System ready.")

# Function to get battery status
def get_battery_status():
    # Request battery status from the MAVLink vehicle
    msg = connection.recv_match(type='BATTERY_STATUS', blocking=True)
    if not msg:
        print("No BATTERY_STATUS message received")
        return
    # Extract battery information
    voltage = msg.voltages[0] / 1000.0  # Voltage in volts
    current = msg.current_battery / 100.0  # Current in amps
    remaining = msg.battery_remaining  # Remaining battery percentage
    print(f"Battery Voltage: {voltage:.2f} V")
    print(f"Battery Current: {current:.2f} A")
    print(f"Battery Remaining: {remaining}%")

# Example usage
while True:
    try:
        get_battery_status()
    except KeyboardInterrupt:
        print("Exiting...")
        break

