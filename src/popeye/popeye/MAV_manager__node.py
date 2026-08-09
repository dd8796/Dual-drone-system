#!/usr/bin/env python3
# Callback groups https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html

# General importation
from time import sleep
import time
from popeye.PARAMS_utils import *
# Import ROS2 utils
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
# Import MAVLink utils
import pymavlink.dialects.v20.common as mavlink
from pymavlink import mavutil
from pymavlink.mavutil import mavlink as mavkit
# Import Intefaces
from interfaces.srv import SetMode, Arm, Rtl, Disarm, Drop
from interfaces.action import Takeoff, Land, Reposition, PrecisionLand, PrecisionRepo
from interfaces.msg import Fire, Attitude, GpsPosition 
# Import MAV utils
import popeye.MAV_manager__utils as mav_utils
# Import geopy utils
import geopy.distance as geodst

############################################################################################################################################################################################################################
##### Node MAVLink Manager ############################################################################################################################################################################################################################
class MAVManagerNode(Node):
    def __init__(self):
        super().__init__('MAV_manager_node', namespace='POPEYE')

        ### Connexion to MAVLink
        try:
            if on_raspi:
                self.mav_master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
            else:
                self.mav_master = mavutil.mavlink_connection('tcp:127.0.0.1:5780', baud=115200)
        except Exception as e:
            self.get_logger().error(f"Erreur MAVLink : {e}")
            raise RuntimeError("Impossible de se connecter à MAVLink.")
        self.get_logger().warning("En attente du heartbeat MAVLink...")
        self.mav_master.wait_heartbeat()
        self.get_logger().info("Connexion MAVLink établie !")

        ### Request data stream from MAVLINK
        ## STATUSTEXT
        self.mav_master.mav.command_long_send(
            self.mav_master.target_system, self.mav_master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            253, 1e6, 0, 0, 0, 0, 0)
        ## GLOBAL_POSITION_INT
        self.mav_master.mav.command_long_send(
            self.mav_master.target_system, self.mav_master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            33, 1e6, 0, 0, 0, 0, 0)
        ## EXTENDED_SYS_STATE
        self.mav_master.mav.command_long_send(
            self.mav_master.target_system, self.mav_master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            245, 1e6, 0, 0, 0, 0, 0)
        ## MISSION_CURRENT
        self.mav_master.mav.command_long_send(
            self.mav_master.target_system, self.mav_master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            42, 1e6, 0, 0, 0, 0, 0)
        ## ATTITUDE
        self.mav_master.mav.command_long_send(
            self.mav_master.target_system, self.mav_master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            30, 1e6, 0, 0, 0, 0, 0)
        
        ### ROS2 Callbacks
        ## Callback groups
        non_critical_group = ReentrantCallbackGroup()
        ## TIMER CALLBACK (it must run in paralell of services and actions but running it concurently to itself is useless)
        self.create_timer(0.05, self.timer_cb__read_mavlink, callback_group=MutuallyExclusiveCallbackGroup())
        ## Publishers
        self.pub__fire_coor = self.create_publisher(Fire,        'fire',         10, callback_group=non_critical_group)
        self.pub__attitude  = self.create_publisher(Attitude,    'uav_attitude', 10, callback_group=non_critical_group)
        self.pub__position  = self.create_publisher(GpsPosition, 'uav_position', 10, callback_group=non_critical_group)
        ## Subscribers
        self.create_subscription(GpsPosition, 'CAM/fire_pos', self.sub_cb__cam_fire_pos, 10, callback_group=non_critical_group)
        self.create_subscription(GpsPosition, 'CAM/park_pos', self.sub_cb__cam_park_pos, 10, callback_group=non_critical_group)
        ## Services
        self.create_service(SetMode, 'set_mode',       self.srv_cb__set_mode,       callback_group=non_critical_group)
        self.create_service(Arm,     'arm',            self.srv_cb__arm,            callback_group=non_critical_group)
        self.create_service(Drop,    'payload_drop',   self.srv_cb__payload_drop,   callback_group=non_critical_group)
        self.create_service(Drop,    'payload_reload', self.srv_cb__payload_reload, callback_group=non_critical_group)
        self.create_service(Rtl,     'rtl',            self.srv_cb__rtl,            callback_group=non_critical_group)
        self.create_service(Disarm,  'disarm',         self.srv_cb__disarm,         callback_group=non_critical_group)
        ## Action servers
        ActionServer(self, Takeoff,       'takeoff',        self.act_cb__takeoff,        callback_group=non_critical_group)
        ActionServer(self, Reposition,    'reposition',     self.act_cb__reposition,     callback_group=non_critical_group)
        ActionServer(self, Land,          'land',           self.act_cb__land,           callback_group=non_critical_group)
        ActionServer(self, PrecisionLand, 'precision_land', self.act_cb__precision_land, callback_group=non_critical_group)
        ActionServer(self, PrecisionRepo, 'precision_repo', self.act_cb__precision_repo, callback_group=non_critical_group)
        ### General Parameters
        ## Popeye state
        self.landed_state = None
        self.lon_cam_park = None
        ## Timers for debug 
        self.elapsed_time = -time.time()
        self.start_time = time.time()
        
        self.get_logger().info("NODE MAV_manager__node STARTED.")
        
    ############################################################################################################################################################################################################################
    ##### TIMER CALLBACK ############################################################################################################################################################################################################################
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to reiceive ALL  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def timer_cb__read_mavlink(self):
        ### Receive Mavlink messages
        msg = self.mav_master.recv_match(type=["EXTENDED_SYS_STATE", "STATUSTEXT", "GLOBAL_POSITION_INT", "ATTITUDE"], blocking=False)
        if msg is None:
            return
        
        ### Save the data returning from the messages
        msg_type = msg.get_type()
        ## For TEXT messages
        if msg_type == "STATUSTEXT":
            if 'FIRE' in msg.text:
                data = msg.text.split()
                self.is_fire = True
                self.fire_pos_lat = float(data[1])
                self.fire_pos_lon = float(data[3])
                msg_pub = Fire()
                msg_pub.is_fire  = self.is_fire
                msg_pub.lat_fire = self.fire_pos_lat
                msg_pub.lon_fire = self.fire_pos_lon
                self.pub__fire_coor.publish(msg_pub)
                # self.get_logger().info(f"FIRE > Fire_lat: {self.fire_pos_lat} Fire_lon: {self.fire_pos_lon} Is_fire: {self.is_fire}")
            else:
                self.get_logger().info('RECEIVED > %s' % msg.text)
        ## For GLOBAL_POSITION_INT messages
        elif msg_type == "GLOBAL_POSITION_INT":
            self.uav_lat = msg.lat/1e7
            self.uav_lon = msg.lon/1e7
            self.uav_pos = (self.uav_lat, self.uav_lon)
            self.uav_alt = msg.relative_alt/1e3
            msg_pub     = GpsPosition()
            msg_pub.lat = self.uav_lat
            msg_pub.lon = self.uav_lon
            msg_pub.alt = self.uav_alt
            self.pub__position.publish(msg_pub)
            # self.get_logger().info(f"RECEIVED > Lat: {self.uav_lat} Lon: {self.uav_lon} Alt: {self.uav_alt}")
        ## For LANDED_STATE messages
        elif msg_type == "EXTENDED_SYS_STATE":
            landed_state_id = msg.landed_state
            if landed_state_id == 0:
                self.landed_state = "MAV_LANDED_STATE_UNDEFINED"
            elif landed_state_id == 1:
                self.landed_state = "MAV_LANDED_STATE_ON_GROUND"
            elif landed_state_id == 2:
                self.landed_state = "MAV_LANDED_STATE_IN_AIR"
            elif landed_state_id == 3:
                self.landed_state = "MAV_LANDED_STATE_TAKEOFF"
            elif landed_state_id == 4:
                self.landed_state = "MAV_LANDED_STATE_LANDING"
            ## Bench the perf
            # self.nb_msg+=1
            # self.elapsed_time += time.time()
            # print(f"LANDEDE_STATE:{self.landed_state} --- time:{self.elapsed_time:.5f} --- Nb msg:{self.nb_msg}/{(time.time()-self.start_time)/1:.0f}")
            # self.elapsed_time = -time.time()
        elif msg_type == "MISSION_CURRENT":
            self.get_logger().info(f"RECEIVED > {msg.to_dict()}")
        ## For ATTITUDE messages
        elif msg_type == "ATTITUDE":
            self.roll  = msg.roll
            self.pitch = msg.pitch
            self.yaw   = msg.yaw
            msg_pub       = Attitude()
            msg_pub.yaw   = self.yaw
            msg_pub.pitch = self.pitch
            msg_pub.roll  = self.roll
            self.pub__attitude.publish(msg_pub)
            # self.get_logger().info(f"RECEIVED > Yaw: {self.yaw} Pitch: {self.pitch} Roll: {self.roll}")
            
    ############################################################################################################################################################################################################################
    ##### SUBSCRIBERS CALLBACKS ############################################################################################################################################################################################################################
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Subscriber for CAM FIRE POS  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def sub_cb__cam_fire_pos(self, msg):
        print("cam_pos")
        self.pos_cam_fire = (msg.lat, msg.lon)
        self.alt_cam_fire = msg.alt
        print()
        # self.get_logger().info(f" >>> CAM_FIRE_POS:{self.pos_cam_fire} <<<")
        # print()
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Subscriber for CAM PARK POS  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def sub_cb__cam_park_pos(self, msg):
        self.lat_cam_park = msg.lat
        self.lon_cam_park = msg.lon
        self.pos_cam_park = (msg.lat, msg.lon)
        self.alt_cam_park = msg.alt
        # print()
        # self.get_logger().info(f" >>> CAM_PARK_POS:{self.pos_cam_park} <<<")
        # print()

    ############################################################################################################################################################################################################################
    ##### ACTIONS CALLBACK ############################################################################################################################################################################################################################
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Action server to PRECISION LAND ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def act_cb__precision_land(self, goal_handle):
        print()
        self.get_logger().info(f"> Call action PRECISION LAND")
        
        ## If no park target is found, return fail
        while self.lon_cam_park is None or self.landed_state is None:
            self.get_logger().warning("      -> Failure: No park target as been published yet.")
            sleep(1)
            # goal_handle.abort()
            # return PrecisionLand.Result(success=False)
        
        ### Precision landing main loop
        alt, is_pland_success, min_alt, tolerance, dist = (0., False, 6., 0.7, 0.)
        for i in range(100):
            ## If the uav is above park target
            alt = self.uav_alt
            if dist < tolerance:
                alt = self.uav_alt-0.5
                if self.uav_alt < min_alt:
                    is_pland_success = True
                    break
            ## Reposition the drone above park target
            if not mav_utils.mav_reposition(self.mav_master, self.lat_cam_park, self.lon_cam_park, alt):
                self.get_logger().warn("      -> Failure: MAV_REPOSITION command not valid")
                goal_handle.abort()
                return PrecisionLand.Result(success=False)
            ## Get the distance to target
            dist = geodst.distance(self.uav_pos, self.pos_cam_park).m
            ## If uav is too low
            if alt < min_alt-1.5:
                is_pland_success = False
                break
            ## Publish feedback
            goal_handle.publish_feedback(PrecisionLand.Feedback(current_alt=float(self.uav_alt), dist_target=float(dist) ,land_state=str(self.landed_state)))
            self.get_logger().info(f"      ... Precision land {i} (current_alt:{self.uav_alt}, dist_target={dist}, state:{self.landed_state})")
            sleep(1)
            
        ## Land (if we are at a low alt)
        self.get_logger().info(f"> Call action LAND")
        if not mav_utils.mav_land(self.mav_master):
            self.get_logger().warn("      -> Failure: LAND command not valid")
            goal_handle.abort()
            return PrecisionLand.Result(success=False)
        while self.landed_state != "MAV_LANDED_STATE_ON_GROUND":
            dist = geodst.distance(self.uav_pos, self.pos_cam_park).m
            goal_handle.publish_feedback(PrecisionLand.Feedback(current_alt=self.uav_alt, dist_target=dist ,land_state=self.landed_state))
            self.get_logger().info(f"      ... Landing (current_alt:{self.uav_alt}, dist_target={dist}, state:{self.landed_state})")
            sleep(1)
        
        ## If precision land has failed
        if not is_pland_success:
            self.get_logger().warn("      -> Failure: Precision land failed but drone as landed")
            goal_handle.abort()
            return PrecisionLand.Result(success=False)
        
        ## Else return success
        self.get_logger().info("      -> Success")
        goal_handle.succeed()
        return PrecisionLand.Result(success=True)
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Action server to PRECISION REPOSITION WITH ALT SELECTION ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def act_cb__precision_repo(self, goal_handle):
        print()
        self.get_logger().info(f"> Call action PRECISION REPO")
        
        ### If no park target (aruco=5) is found, return fail
        while self.lon_cam_park is None or self.landed_state != "MAV_LANDED_STATE_IN_AIR":
            self.get_logger().warning("      -> Failure: No park target as been published yet or the drone is not in air.")
            sleep(1)
            goal_handle.abort()
            return PrecisionRepo.Result(success=False)
        
        ### Precision descent main loop
        next_alt, is_prec_desc_success, goal_alt, tolerance, dist, dangerous_alt= (0., False, 6., 0.7, 0., 1.)
        for i in range(100):
            ## Get the current state
            next_alt = self.uav_alt
            dist = geodst.distance(self.uav_pos, self.pos_cam_park).m
            ## If uav is at dangerously low altitude 
            if self.uav_alt < dangerous_alt:
                is_prec_desc_success = False
                break
            ## If the uav is above park target
            elif dist < tolerance:
                print("ok")
                next_alt -= 0.5
                if self.uav_alt < goal_alt:
                    is_prec_desc_success = True
                    break
            ## Reposition the drone above park target
            elif not mav_utils.mav_reposition(self.mav_master, self.lat_cam_park, self.lon_cam_park, next_alt):
                self.get_logger().warn("      -> Failure: MAV_REPOSITION command not valid")
                goal_handle.abort()
                return PrecisionRepo.Result(success=False)
            ## Publish feedback
            goal_handle.publish_feedback(PrecisionRepo.Feedback(current_alt=float(self.uav_alt), dist_target=float(dist), land_state=str(self.landed_state)))
            self.get_logger().info(f"      ... Precision repo {i} (current_alt:{self.uav_alt}, dist_target={dist}, state:{self.landed_state})")
            sleep(1)
            
        print("aaaaaa")
        ## Land (if we are at a dangerously low alt)
        if self.uav_alt <= dangerous_alt:
            self.get_logger().info(f"> Call action LAND")
            if not mav_utils.mav_land(self.mav_master):
                self.get_logger().warn("      -> Failure: LAND command not valid")
                goal_handle.abort()
                return PrecisionRepo.Result(success=False)
            while self.landed_state != "MAV_LANDED_STATE_ON_GROUND":
                dist = geodst.distance(self.uav_pos, self.pos_cam_park).m
                goal_handle.publish_feedback(PrecisionRepo.Feedback(current_alt=self.uav_alt, dist_target=dist ,land_state=self.landed_state))
                self.get_logger().info(f"      ... Landing (current_alt:{self.uav_alt}, dist_target={dist}, state:{self.landed_state})")
                sleep(1)
        
        ## If precision descent has failed
        if not is_prec_desc_success:
            self.get_logger().warn("      -> Failure: Precision land failed but drone as landed")
            goal_handle.abort()
            return PrecisionRepo.Result(success=False)
        
        ## Else return success
        self.get_logger().info("      -> Success")
        goal_handle.succeed()
        return PrecisionRepo.Result(success=True)

    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Action server to TAKEOFF ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def act_cb__takeoff(self, goal_handle):
        print()
        self.get_logger().info(f"> Call action TAKEOFF (Alt:{goal_handle.request.alt})")
        
        if not mav_utils.mav_takeoff(self.mav_master, goal_handle.request.alt):
            self.get_logger().warn("      -> Failure: command not valid")
            goal_handle.abort()
            return Takeoff.Result(success=False)
        
        while self.landed_state != "MAV_LANDED_STATE_IN_AIR":
            goal_handle.publish_feedback(Takeoff.Feedback(current_alt=self.uav_alt, state=self.landed_state))
            self.get_logger().info(f"      ... Taking off (current_alt:{self.uav_alt}, state:{self.landed_state})")
            sleep(1)
        self.get_logger().info("      -> Success")
        goal_handle.succeed()
        return Takeoff.Result(success=True)
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Action server to REPOSITION ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def act_cb__reposition(self, goal_handle):
        print()
        self.get_logger().info(f"> Call action REPOSITION (lat:{goal_handle.request.lat}, lon:{goal_handle.request.lon}, alt:{goal_handle.request.alt})")
        self.get_logger().warn(f"> Update calculation of distance !")
        
        if not mav_utils.mav_reposition(self.mav_master, goal_handle.request.lat, goal_handle.request.lon, goal_handle.request.alt):
            self.get_logger().warn("      -> Failure: command not valid")
            goal_handle.abort()
            return Reposition.Result(success=False)
        
        time_at_position = 0; sleep_time = 1
        tolerance = 1.0e-06
        while time_at_position < 5:
            dist_to_goal = ((goal_handle.request.lat-self.uav_lat)**2 + (goal_handle.request.lon-self.uav_lon)**2)**0.5
            delta_alt    = goal_handle.request.alt-self.uav_alt
            time_at_position = time_at_position+sleep_time if (dist_to_goal<=tolerance and delta_alt<0.15) else 0
            goal_handle.publish_feedback(Reposition.Feedback(time_at_position=float(time_at_position), dist_to_goal=dist_to_goal, delta_alt=delta_alt))
            self.get_logger().info(f"      ... Repositioning (dist_to_goal:{dist_to_goal}, delta_alt:{delta_alt:.1f}, time_at_position:{time_at_position})")
            sleep(sleep_time)
        self.get_logger().info("      -> Success")
        goal_handle.succeed()
        return Reposition.Result(success=True) # goal_hadndle.results.sucess
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Action server to LAND ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def act_cb__land(self, goal_handle):
        print()
        self.get_logger().info(f"> Call action LAND")
        
        if not mav_utils.mav_land(self.mav_master):
            self.get_logger().warn("      -> Failure: command not valid")
            goal_handle.abort()
            return Land.Result(success=False)
        
        while self.landed_state != "MAV_LANDED_STATE_ON_GROUND":
            goal_handle.publish_feedback(Land.Feedback(current_alt=self.uav_alt, state=self.landed_state))
            self.get_logger().info(f"      ... Landing (current_alt:{self.uav_alt}, state:{self.landed_state})")
            sleep(1)
        self.get_logger().info("      -> Success")
        goal_handle.succeed()
        return Land.Result(success=True)
             
    ############################################################################################################################################################################################################################
    ##### SERVICES CALLBACK ############################################################################################################################################################################################################################
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Service server to SET MODE ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def srv_cb__set_mode(self, request, response):
        print()
        self.get_logger().info(f"> Call service SET_MODE (mode:{request.mode_name})")
        if mav_utils.mav_set_mode(self.mav_master, request.mode_name):
            response.success = True
            self.get_logger().info("      -> Success.")
        else:
            response.success = False
            self.get_logger().warning("      -> Failure")
        return response
    #--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Service server to ARM ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def srv_cb__arm(self, request, response):
        print()
        self.get_logger().info(f"> Call service ARM (force:{request.force})")
        if mav_utils.mav_arm(self.mav_master, request.force):
            response.success = True
            self.get_logger().info("      -> Success.")
        else:
            response.success = False
            self.get_logger().warning("      -> Failure")
        return response
    #--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Service server to PAYLOAD DROP ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def srv_cb__payload_drop(self, request, response):
        print()
        self.get_logger().info(f"> Call service PAYLOAD DROP")
        if mav_utils.mav_payload_drop(self.mav_master):
            response.success = True
            self.get_logger().info("      -> Success.")
        else:
            response.success = False
            self.get_logger().warning("      -> Failure")
        return response
    #--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Service server to PAYLOAD RELOAD ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def srv_cb__payload_reload(self, request, response):
        print()
        self.get_logger().info(f"> Call service PAYLOAD RELOAD")
        if mav_utils.mav_payload_reload(self.mav_master):
            response.success = True
            self.get_logger().info("      -> Success.")
        else:
            response.success = False
            self.get_logger().warning("      -> Failure")
        return response
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Service server to REPOSITION ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def srv_cb__reposition(self, request, response):
        print()
        self.get_logger().info(f"> Call service REPOSITION (lat:{request.lat}, lon:{request.lon}, Alt:{request.alt})")
        if mav_utils.mav_reposition(self.mav_master, request.lat, request.lon, request.alt):
            response.success = True
            self.get_logger().info("      -> Success.")
        else:
            response.success = False
            self.get_logger().warning("      -> Failure")
        return response
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Service server to RTL ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def srv_cb__rtl(self, request, response):
        print()
        self.get_logger().info(f"> Call service RTL.")
        if mav_utils.mav_rtl(self.mav_master):
            response.success = True
            self.get_logger().info("      -> Success.")
        else:
            response.success = False
            self.get_logger().warning("      -> Failure")
        return response
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Service server to DISARM ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def srv_cb__disarm(self, request, response):
        print()
        self.get_logger().info(f"> Call service DISARM (force:{request.force})")
        if mav_utils.mav_disarm(self.mav_master, request.force):
            response.success = True
            self.get_logger().info("      -> Success.")
        else:
            response.success = False
            self.get_logger().warning("      -> Failure")
        return response

############################################################################################################################################################################################################################
##### Node entry point ############################################################################################################################################################################################################################
def main(args=None):
    rclpy.init(args=args)
    
    ### Creating the mutlithread executor
    node = MAVManagerNode()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=10)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
