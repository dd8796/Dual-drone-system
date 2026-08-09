# Commands

## Attach an USB to WSL
```bash
# On windows to bind the port to wsl
usbipd list
usbipd bind --busid bus_id # IF BUG => usbipd bind --force --busid bus_id
# => usbipd unbind --busid bus_id
usbipd attach --wsl --busid bus_id
# => usbipd detach --busid bus_id
# On linux to find the associated port
ls -l /dev/serial/by-id/
```
## Launch Commands for SITL and real UAV connection
### Olive
```bash
# SITL (SGDB)
sim_vehicle.py -v ArduPlane -f plane -I1 --no-mavproxy --custom-location=48.6126523,2.3963258,0,0 
# SITL (DUAV)
sim_vehicle.py -v ArduPlane -f plane -I1 --no-mavproxy --custom-location=45.4389468,-0.4283327,32.79,0 
# REAL => a trnasferer sur windows
mavproxy.py --master=/dev/ttyUSB1 --out=tcpin:127.0.0.1:5770 --out=tcpin:127.0.0.1:5772 --out=tcpin:127.0.0.1:5773 --streamrate=-1
```
### Popeye
```bash
# SITL (SGDB)
sim_vehicle.py -v ArduCopter -f hexa -I2 --no-mavproxy --custom-location=48.6127587,2.3963258,0,0 
# SITL (DUAV)
sim_vehicle.py -v ArduCopter -f hexa -I2 --no-mavproxy --custom-location=45.4389468,-0.4283327,0,0 
# REAL => a trnasferer sur windows
mavproxy.py --master=/dev/ttyUSB0 --out=tcpin:127.0.0.1:5780 --out=tcpin:127.0.0.1:5782 --out=tcpin:127.0.0.1:5783 --streamrate=-1
```
## Launching ROS2 application

Make sure to use the good pathing and configuration for Popeye (`src/popeye/popeye/PARAMS_utils.py`) :
<div style="text-align: center;">
  <img src="configuration.png" alt="Software Architecture" width="250" />
</div>

### Olive node
```bash
colcon build --packages-select olive && source install/setup.bash && ros2 launch olive olive_nodes.launch.py 
```
### Popeye node
```bash
colcon build --packages-select popeye && source install/setup.bash && ros2 launch popeye popeye_nodes.launch.py
```
### CGS node
```bash
colcon build --packages-select control_ground_station && source install/setup.bash && ros2 launch control_ground_station cgs_nodes.launch.py
```

## Test commands for services
```bash
# Test Takeoff action
ros2 action send_goal --feedback /POPEYE/takeoff_act interfaces/action/TakeoffAct "{alt: 2}"
# Test Land action
ros2 action send_goal --feedback /POPEYE/land interfaces/action/Land "{}"
# Test Land action
ros2 service call 
```
## Launch and test Mavros
Tuto [Home - MAVROS Tutorial](https://masoudir.github.io/mavros_tutorial/)
[mavros - ROS Wiki](https://wiki.ros.org/mavros)
[Manage Quality of Service Policies in ROS 2](https://www.mathworks.com/help/ros/ug/manage-quality-of-service-policies-in-ros2.html)

ros2 action send_goal --feedback /POPEYE/precision_land interfaces/action/PrecisionLand "{}"

### Launch Popeye
```bash
# SITL & REAL
ros2 launch mavros apm.launch fcu_url:=tcp://localhost:5780 gcs_url:=udp://@
```

```bash
# To stream mavlink all mavlink messages
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 10, on_off: True}"
```

### Useful commands
```bash
# Changing mode /mavros/set_mode
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}"
# Arm /mavros/cmd/arming
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: True}"
# takeoff /mavros/cmd/takeoff
ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 10.0}"
# home_pos /mavros/cmd/set_home 
ros2 service call /mavros/cmd/set_home mavros_msgs/srv/CommandHome \ "{current_gps: false, \ yaw: 0.0, \ latitude: 45.4389463, \ longitude: -0.4283310, \ altitude: 10.0}"
# To stream mavlink all mavlink messages
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 10, on_off: True}"
# CommandINT /mavros/cmd/command_int
ros2 service call /mavros/cmd/command_int mavros_msgs/srv/CommandInt \
"{broadcast: false, \
frame: 3, \
command: 192, \
current: 0, \
autocontinue: 0, \
param1: -1.0, \
param2: 0, \
param3: 0.0, \
param4: 0.0, \
x: 454389463, \
y: -4283321, \
z: 10.0}"

# List
/mavros/landing_target/describe_parameters
/mavros/landing_target/get_parameter_types
/mavros/landing_target/get_parameters
/mavros/landing_target/list_parameters
/mavros/landing_target/set_parameters
/mavros/landing_target/set_parameters_atomically
```
### Useful topics
```bash
/mavros/state
/mavros/setpoint_position/local
/mavros/setpoint_velocity/cmd_vel
```
To remove RQT config in case of a graphic issue :
```bash
cd ~/.config/ros.org/ && rm rqt_gui.ini
```
