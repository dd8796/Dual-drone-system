from pymavlink import mavutil

# Connect to the flight controller
connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')  # Adjust if needed

# Wait for a heartbeat to establish communication
print("Waiting for heartbeat...")
connection.wait_heartbeat()
print("Heartbeat received! Communication established.")

# Function to publish voltage
def publish_voltage(voltage):
    # Convert voltage to millivolts for MAVLink
    voltage_mV = float(voltage * 1000)

    # Send only the voltage in the BATTERY_STATUS message
    connection.mav.battery_status_send(
        0,                          # id (ID of the battery, 0 for a single battery setup)
        1,                          # battery_function (MAV_BATTERY_FUNCTION_ALL)
        3,                          # type (MAV_BATTERY_TYPE_LIPO)
        0,                          # temperature (no temperature provided)
        [15] * 10,    # voltages (fill remaining cells with -1)
        -1,                         # current_battery (not used here)
        -1,                         # current_consumed (not tracked)
        -1,                         # energy_consumed (not tracked)
        20,                         # battery_remaining (not used here)
    )
    print(f"Voltage published: {voltage:.2f} V")

# Example usage
import time

while True:
    try:
        # Input voltage from the user
        voltage = float(input("Enter the voltage (in volts) to publish: "))
        
        # Publish the voltage
        publish_voltage(voltage)

        # Wait for a short interval before the next input
        time.sleep(1.0)

    except KeyboardInterrupt:
        print("Exiting...")
        break
    except ValueError:
        print("Please enter a valid voltage value.")
