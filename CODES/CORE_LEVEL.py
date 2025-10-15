#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, roslaunch, os
from enum import Enum
from std_msgs.msg import UInt8

class CoreNodeController():
    def __init__(self):
        self.ros_package_path = os.path.dirname(os.path.realpath(__file__)).replace('core/nodes', '')

        # Subscriber for mission and step topics
        self.sub_mission = rospy.Subscriber('/mission', UInt8, self.cbReceiveMode, queue_size=1)
        rospy.Subscriber('/mission/parking', UInt8, self.cbParkingStep, queue_size=1)
        rospy.Subscriber('/mission/tunnel', UInt8, self.cbTunnelStep, queue_size=1)

        # Publisher to advance missions
        self.pub_mission = rospy.Publisher('/mission', UInt8, queue_size=1)

        # Define mission states
        self.Mission = Enum('Mission', 'Idle TrafficLight Intersection Construction Parking LevelCrossing Tunnel')
        # Define sub-steps
        self.ParkingStep = Enum('ParkingStep', 'default process')
        self.TunnelStep = Enum('TunnelStep', 'default process nav end')

        # State variables
        self.is_triggered = False
        self.current_mode = self.Mission.Idle.value
        self.current_step_parking = self.ParkingStep.default.value
        self.current_step_tunnel = self.TunnelStep.default.value

        # Launcher enum and flags
        self.Launcher = Enum('Launcher', 'control_lane conrtol_tunnel detect_lane detect_traffic_light detect_intersection detect_construction detect_parking detect_level detect_tunnel')
        self.launch_control_lane_launched = False
        self.launch_conrtol_tunnel_launched = False
        self.launch_detect_lane_launched = False
        self.launch_detect_traffic_light_launched = False
        self.launch_detect_intersection_launched = False
        self.launch_detect_construction_launched = False
        self.launch_detect_parking_launched = False
        self.launch_detect_level_launched = False
        self.launch_detect_tunnel_launched = False

        # ROSLaunch UUID
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

        # Main loop at 10 Hz
        loop_rate = rospy.Rate(10)
        rospy.sleep(1)

        # Start in Idle, then immediately advance
        self.pub_mission.publish(self.Mission.Idle.value)

        while not rospy.is_shutdown():
            if self.is_triggered:
                self.fnControlNode()
            loop_rate.sleep()

    def cbReceiveMode(self, mode_msg):
        rospy.loginfo("Received mission: %d", mode_msg.data)
        self.current_mode = mode_msg.data
        self.is_triggered = True

    def cbParkingStep(self, mode_msg):
        self.current_step_parking = mode_msg.data
        self.is_triggered = True

    def cbTunnelStep(self, mode_msg):
        self.current_step_tunnel = mode_msg.data
        self.is_triggered = True

    def fnControlNode(self):
        # Sequence: TrafficLight -> open lane/control -> Intersection -> Construction -> LevelCrossing -> Tunnel

        # Idle: advance to TrafficLight
        # if self.current_mode == self.Mission.Idle.value:
        #     rospy.loginfo("Idle: advancing to TrafficLight mission")
        #     self.pub_mission.publish(self.Mission.TrafficLight.value)

        # # TrafficLight: launch TL detector
        # elif self.current_mode == self.Mission.TrafficLight.value:
        #     rospy.loginfo("Mission TrafficLight start")
        #     self.fnLaunch(self.Launcher.detect_traffic_light.value, True)
        #     rospy.loginfo("Waiting to next mission")

        # # Intersection: first ensure lane detection + control are up, then intersection detection
        # elif self.current_mode == self.Mission.Intersection.value:
        #     rospy.loginfo("Mission Intersection start: ensuring lane/control are running")
        #     # Shutdown TL detector
        #     self.fnLaunch(self.Launcher.detect_traffic_light.value, False)
        #     # Open lane detection and control, never shut them down later
        self.fnLaunch(self.Launcher.detect_lane.value, True)
        self.fnLaunch(self.Launcher.control_lane.value, True)
        #     # Launch intersection detector
        #     self.fnLaunch(self.Launcher.detect_intersection.value, True)
        #     rospy.loginfo("Waiting to next mission")

        # # Construction: add construction detection
        # elif self.current_mode == self.Mission.Construction.value:
        #     rospy.loginfo("Mission Construction start")
        #     self.fnLaunch(self.Launcher.detect_intersection.value, False)
        #     self.fnLaunch(self.Launcher.detect_construction.value, True)
        #     rospy.loginfo("Waiting to next mission")

        # # Parking (if still used)
        # elif self.current_mode == self.Mission.Parking.value:
        #     rospy.loginfo("Mission Parking start")
        #     self.fnLaunch(self.Launcher.detect_construction.value, False)
        #     self.fnLaunch(self.Launcher.detect_parking.value, True)
        #     rospy.loginfo("Waiting to next mission")

        # # LevelCrossing
        # elif self.current_mode == self.Mission.LevelCrossing.value:
        rospy.loginfo("LevelCrossing :)")
        #     self.fnLaunch(self.Launcher.detect_parking.value, False)
        self.fnLaunch(self.Launcher.detect_level.value, True)
        #     rospy.loginfo("Waiting to next mission")

        # # Tunnel: pipeline tunnel detection & control
        # elif self.current_mode == self.Mission.Tunnel.value:
        #     rospy.loginfo("Mission Tunnel start")
        #     self.fnLaunch(self.Launcher.detect_level.value, False)
        #     self.fnLaunch(self.Launcher.detect_tunnel.value, True)
        #     # Note: do NOT shut down detect_lane or control_lane here
        #     if self.current_step_tunnel == self.TunnelStep.nav.value:
        #         rospy.loginfo("Tunnel nav: launching control_tunnel node")
        #         self.fnLaunch(self.Launcher.conrtol_tunnel.value, True)
        #         rospy.loginfo("Waiting to nav end")
        #     elif self.current_step_tunnel == self.TunnelStep.end.value:
        #         rospy.loginfo("Navigation end: shutting down tunnel nodes")
        #         self.fnLaunch(self.Launcher.conrtol_tunnel.value, False)
        #         self.fnLaunch(self.Launcher.detect_tunnel.value, False)
        #         rospy.loginfo("Main control flow completed")

        # Reset trigger flag
        self.is_triggered = False

    def fnLaunch(self, launch_num, is_start):
        # Identical launch/shutdown logic for each node (omitted here for brevity)
        if launch_num == self.Launcher.control_lane.value:
            if is_start == True:
                if self.launch_control_lane_launched == False:
                    self.launch_control_lane = roslaunch.scriptapi.ROSLaunch()
                    self.launch_control_lane = roslaunch.parent.ROSLaunchParent(self.uuid, [self.ros_package_path + "control/launch/control_lane.launch"])
                    self.launch_control_lane_launched = True
                    self.launch_control_lane.start()
                else:
                    pass
            else:
                if self.launch_control_lane_launched == True:
                    self.launch_control_lane_launched = False
                    self.launch_control_lane.shutdown()
                else:
                    pass

        elif launch_num == self.Launcher.conrtol_tunnel.value:
            if is_start == True:
                if self.launch_conrtol_tunnel_launched == False:
                    self.launch_conrtol_tunnel = roslaunch.scriptapi.ROSLaunch()
                    self.launch_conrtol_tunnel = roslaunch.parent.ROSLaunchParent(self.uuid, [self.ros_package_path + "control/launch/control_tunnel.launch"])
                    self.launch_conrtol_tunnel_launched = True
                    self.launch_conrtol_tunnel.start()
                else:
                    pass
            else:
                if self.launch_conrtol_tunnel_launched == True:
                    self.launch_conrtol_tunnel_launched = False
                    self.launch_conrtol_tunnel.shutdown()
                else:
                    pass

        elif launch_num == self.Launcher.detect_lane.value:
            if is_start == True:
                if self.launch_detect_lane_launched == False:
                    self.launch_detect_lane = roslaunch.scriptapi.ROSLaunch()
                    self.launch_detect_lane = roslaunch.parent.ROSLaunchParent(self.uuid, [self.ros_package_path + "detect/launch/detect_lane.launch"])
                    self.launch_detect_lane_launched = True
                    self.launch_detect_lane.start()
                else:
                    pass
            else:
                if self.launch_detect_lane_launched == True:
                    self.launch_detect_lane_launched = False
                    self.launch_detect_lane.shutdown()
                else:
                    pass

        elif launch_num == self.Launcher.detect_traffic_light.value:
            if is_start == True:
                if self.launch_detect_traffic_light_launched == False:
                    self.launch_detect_traffic_light = roslaunch.scriptapi.ROSLaunch()
                    self.launch_detect_traffic_light = roslaunch.parent.ROSLaunchParent(self.uuid, [self.ros_package_path + "detect/launch/detect_traffic_light.launch"])
                    self.launch_detect_traffic_light_launched = True
                    self.launch_detect_traffic_light.start()
                else:
                    pass
            else:
                if self.launch_detect_traffic_light_launched == True:
                    self.launch_detect_traffic_light_launched = False
                    self.launch_detect_traffic_light.shutdown()
                else:
                    pass

        elif launch_num == self.Launcher.detect_intersection.value:
            if is_start == True:
                if self.launch_detect_intersection_launched == False:
                    self.launch_detect_intersection = roslaunch.scriptapi.ROSLaunch()
                    self.launch_detect_intersection = roslaunch.parent.ROSLaunchParent(self.uuid, [self.ros_package_path + "detect/launch/detect_intersection.launch"])
                    self.launch_detect_intersection_launched = True
                    self.launch_detect_intersection.start()
                else:
                    pass
            else:
                if self.launch_detect_intersection_launched == True:
                    self.launch_detect_intersection_launched = False
                    self.launch_detect_intersection.shutdown()
                else:
                    pass

        elif launch_num == self.Launcher.detect_construction.value:
            if is_start == True:
                if self.launch_detect_construction_launched == False:
                    self.launch_detect_construction = roslaunch.scriptapi.ROSLaunch()
                    self.launch_detect_construction = roslaunch.parent.ROSLaunchParent(self.uuid, [self.ros_package_path + "detect/launch/detect_construction.launch"])
                    self.launch_detect_construction_launched = True
                    self.launch_detect_construction.start()
                else:
                    pass
            else:
                if self.launch_detect_construction_launched == True:
                    self.launch_detect_construction_launched = False
                    self.launch_detect_construction.shutdown()
                else:
                    pass

        elif launch_num == self.Launcher.detect_parking.value:
            if is_start == True:
                if self.launch_detect_parking_launched == False:
                    self.launch_detect_parking = roslaunch.scriptapi.ROSLaunch()
                    self.launch_detect_parking = roslaunch.parent.ROSLaunchParent(self.uuid, [self.ros_package_path + "detect/launch/detect_parking.launch"])
                    self.launch_detect_parking_launched = True
                    self.launch_detect_parking.start()
                else:
                    pass
            else:
                if self.launch_detect_parking_launched == True:
                    self.launch_detect_parking_launched = False
                    self.launch_detect_parking.shutdown()
                else:
                    pass

        elif launch_num == self.Launcher.detect_level.value:
            if is_start == True:
                if self.launch_detect_level_launched == False:
                    self.launch_detect_level = roslaunch.scriptapi.ROSLaunch()
                    self.launch_detect_level = roslaunch.parent.ROSLaunchParent(self.uuid, [self.ros_package_path + "detect/launch/detect_level.launch"])
                    self.launch_detect_level_launched = True
                    self.launch_detect_level.start()
                else:
                    pass
            else:
                if self.launch_detect_level_launched == True:
                    self.launch_detect_level_launched = False
                    self.launch_detect_level.shutdown()
                else:
                    pass

        elif launch_num == self.Launcher.detect_tunnel.value:
            if is_start == True:
                if self.launch_detect_tunnel_launched == False:
                    self.launch_detect_tunnel = roslaunch.scriptapi.ROSLaunch()
                    self.launch_detect_tunnel = roslaunch.parent.ROSLaunchParent(self.uuid, [self.ros_package_path + "detect/launch/detect_tunnel.launch"])
                    self.launch_detect_tunnel_launched = True
                    self.launch_detect_tunnel.start()
                else:
                    pass
            else:
                if self.launch_detect_tunnel_launched == True:
                    self.launch_detect_tunnel_launched = False
                    self.launch_detect_tunnel.shutdown()
                else:
                    pass

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('core_node_controller')
    CoreNodeController().main()
