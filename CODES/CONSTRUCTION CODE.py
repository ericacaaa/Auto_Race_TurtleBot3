#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, os, cv2
import numpy as np
from enum import Enum
from std_msgs.msg import UInt8, Float64, Bool
from sensor_msgs.msg import CompressedImage, LaserScan
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import Twist

class DetectSign():
    def __init__(self):
        self.fnPreproc()

        # subscribes compressed image
        self.sub_img = rospy.Subscriber('/image_input', CompressedImage, self.cbFindTrafficSign, queue_size = 1)
        self.sub_scan_obstacle = rospy.Subscriber('/scan', LaserScan, self.cbScanObstacle, queue_size=1)

        # publishes next mission
        self.pub_mission = rospy.Publisher('/mission', UInt8, queue_size=1)
        self.pub_mission_step = rospy.Publisher('/mission/construction', UInt8, queue_size=1)
        self.pub_max_vel = rospy.Publisher('/control/max_vel', Float64, queue_size = 1)
        self.pub_fake_lane = rospy.Publisher('/control/lane', Float64, queue_size=1)
        self.pub_lane_toggle = rospy.Publisher('/detect/lane_toggle', Bool, queue_size=1)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_yellow_lane_toggle = rospy.Publisher('/detect/yellow_toggle', Bool, queue_size=1)
        self.pub_white_lane_toggle = rospy.Publisher('/detect/white_toggle', Bool, queue_size=1)
        # publishes traffic sign image in compressed type 
        self.pub_image_traffic_sign = rospy.Publisher('/detect/sign/compressed', CompressedImage, queue_size = 1)


        self.Mission = Enum('Mission', 'Idle TrafficLight Intersection Construction Parking LevelCrossing Tunnel')
        self.ConstructionStep = Enum('ConstructionStep', 'default process')
        self.current_step = self.ConstructionStep.default.value
        
        self.cvBridge = CvBridge()
        self.white_counter = 0
        self.counter = 1
        self.count = 0

        self.is_saw_construction = False
        self.is_obstacle_detected = False
        self.is_moving_complete = False

    def cbMovingComplete(self, data):
        self.is_moving_complete = True

    def cbScanObstacle(self, scan):
        # angle_scan = 25
        angle_scan = 5
        scan_start = 0 - angle_scan
        scan_end = 0 + angle_scan
        # threshold_distance = 0.25
        threshold_distance = 0.3
        is_obstacle_detected = False

        for i in range(scan_start, scan_end):
            if scan.ranges[i] < threshold_distance and scan.ranges[i] > 0.15:
                is_obstacle_detected = True
        
        self.is_obstacle_detected = is_obstacle_detected

    def fnPreproc(self):
        # Initiate SIFT detector
        self.sift = cv2.SIFT_create()

        dir_path = os.path.dirname(os.path.realpath(__file__))
        dir_path = dir_path.replace('detect/nodes', 'detect/')
        dir_path += 'image/'

        self.img_construction = cv2.imread(dir_path + 'construction.png', 0)
        self.kp_construction, self.des_constuction = self.sift.detectAndCompute(self.img_construction, None)

        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 50)

        self.flann = cv2.FlannBasedMatcher(index_params, search_params)

    def fnCalcMSE(self, arr1, arr2):
            squared_diff = (arr1 - arr2) ** 2
            sum = np.sum(squared_diff)
            num_all = arr1.shape[0] * arr1.shape[1] #cv_image_input and 2 should have same shape
            err = sum / num_all
            return err

    def cbFindTrafficSign(self, image_msg):
        # drop the frame to 1/5 (6fps) because of the processing speed. This is up to your computer's operating power.
        # if self.counter % 2 != 0:
        #     self.counter += 1
        #     return
        # else:
        #     self.counter = 1

        #converting compressed image to opencv image
        np_arr = np.frombuffer(image_msg.data, np.uint8)
        cv_image_input = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # converting raw image to opencv image
        # cv_image_input = CvBridge.imgmsg_to_cv2(image_msg, "bgr8")

        MIN_MATCH_COUNT = 6 #9
        MIN_MSE_DECISION = 60000

        # find the keypoints and descriptors with SIFT
        kp1, des1 = self.sift.detectAndCompute(cv_image_input,None)
        image_out_num = 1

        # construction
        if self.is_saw_construction == False:
            matches = self.flann.knnMatch(des1,self.des_constuction,k=2)
            good = []
            for m,n in matches:
                if m.distance < 0.7*n.distance:
                    good.append(m)
            if len(good)>MIN_MATCH_COUNT:
                src_pts = np.float32([kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
                dst_pts = np.float32([self.kp_construction[m.trainIdx].pt for m in good]).reshape(-1,1,2)

                M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
                matches = mask.ravel().tolist()

                mse = self.fnCalcMSE(src_pts, dst_pts)
                if mse < MIN_MSE_DECISION:
                    rospy.loginfo("detect constrcution")
                    image_out_num = 2
                    # self.is_saw_construction = True

        # # Just for Testing
        self.is_saw_construction = True
        # if self.is_saw_construction == True:
        #     self.pub_yellow_lane_toggle.publish(False)
        #     self.pub_white_lane_toggle.publish(True)


        if image_out_num == 1:
            # publishes traffic sign image in compressed type
            self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_compressed_imgmsg(cv_image_input, "jpg"))
            
        elif image_out_num == 2:
            draw_params = dict(matchColor = (255,0,0), # draw matches in green color
                            singlePointColor = None,
                            matchesMask = matches, # draw only inliers
                            flags = 2)

            fianl = cv2.drawMatches(cv_image_input,kp1,self.img_construction,self.kp_construction,good,None,**draw_params)

            # publishes traffic sign image in compressed type
            self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_compressed_imgmsg(fianl, "jpg"))
        
        # Just for testing
        if self.is_saw_construction == True and self.white_counter == 0:
            rospy.sleep(1)
            self.pub_white_lane_toggle.publish(True)
            self.pub_yellow_lane_toggle.publish(False)
            rospy.loginfo("Following White Lane")
            self.white_counter += 1

        if self.is_saw_construction == True:           
            
        # construction process
            if self.is_obstacle_detected == True and self.count == 0:
                rospy.sleep(0.5)
                self.pub_lane_toggle.publish(False)
                rospy.loginfo('Construction Processing')

                self.turn_fn(80, "left")
                self.move_fn(0.29, "Forward")
                self.turn_fn(90, "right")
                self.move_fn(0.42, "Forward")
                self.turn_fn(92, "right")
                self.move_fn(0.25, "Forward")
                self.turn_fn(95, "left")
                self.move_fn(0.1, "Forward")
                
                # for x in range(0, 15):
                #     self.pub_fake_lane.publish(400)
                #     rospy.sleep(0.1)
                # for x in range(0, 14):
                #     self.pub_fake_lane.publish(600)
                #     rospy.sleep(0.1)
                # for x in range(0, 3):
                #     self.pub_fake_lane.publish(500)
                #     rospy.sleep(0.1)
                # for x in range(0, 17):
                #     self.pub_fake_lane.publish(610)
                #     rospy.sleep(0.1)
                # for x in range(0, 13):
                #     self.pub_fake_lane.publish(400)
                #     rospy.sleep(0.1)
                self.pub_white_lane_toggle.publish(True)
                self.pub_yellow_lane_toggle.publish(True)
                self.pub_lane_toggle.publish(True)
                rospy.loginfo('Construction Finished :)')
                
                # rospy.loginfo('SLOW')
                # self.pub_max_vel.publish(0.15)
                self.count += 1
                # go to next mission
                # rospy.sleep(3)
                self.pub_mission.publish(self.Mission.Parking.value)


    def turn_fn(self, turn_angle, dir):
        omega = 0.5
        rate_hz = 20
        target_angle = math.radians(turn_angle)
        angle_turned = 0.0
        # rospy.loginfo(turn_angle)

        if dir == "right":
            count = -1
        elif dir == "left":
            count = 1

        # rospy.loginfo(dir)
        # rospy.loginfo(count)

        rate = rospy.Rate(rate_hz)
        twist = Twist()
        twist.angular.z = count * abs(omega)

        # timestamp of previous iteration
        last_t = rospy.Time.now().to_sec()

        while angle_turned < target_angle and not rospy.is_shutdown():
            # publish spin
            self.pub_cmd_vel.publish(twist)

            # compute dt
            now = rospy.Time.now().to_sec()
            dt = now - last_t
            last_t = now

            # integrate
            angle_turned += abs(omega) * dt

            rate.sleep()

        # stop
        self.pub_cmd_vel.publish(Twist())

    def move_fn(self, distance, move):
        speed = 0.22
        # if move == 'forward':
        #     movetype = 1
        # if move == 'backward':
        #     movetype = -1
        rate_hz = 20
        target_dist = abs(distance)
        dist_moved = 0.0

        # set up the Twist message: negative x for backwards
        twist = Twist()
        twist.linear.x = abs(speed)

        rate = rospy.Rate(rate_hz)
        last_t = rospy.Time.now().to_sec()

        while dist_moved < target_dist and not rospy.is_shutdown():
            # send the backward command
            self.pub_cmd_vel.publish(twist)

            # compute elapsed time
            now = rospy.Time.now().to_sec()
            dt = now - last_t
            last_t = now

            # integrate distance
            dist_moved += abs(speed) * dt

            rate.sleep()




    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('detect_construction')
    node = DetectSign()
    node.main()



















