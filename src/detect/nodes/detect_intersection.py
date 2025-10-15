#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, os, cv2
import numpy as np
from enum import Enum
from std_msgs.msg import UInt8, Float64, Bool
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
# Moving robot in-place (param)
from turtlebot3_autorace_msgs.msg import MovingParam
import math


class DetectSign():
    def __init__(self):
        self.fnPreproc()

        # subscribes compressed image
        
        self.sub_img = rospy.Subscriber('/image_input', CompressedImage, self.cbFindTrafficSign, queue_size = 1)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.cbOdom, queue_size=1)

        # publishes next mission
        self.pub_mission = rospy.Publisher('/mission', UInt8, queue_size=1)
        self.pub_mission_step = rospy.Publisher('/mission/intertsection', UInt8, queue_size=1)
        self.pub_fake_lane = rospy.Publisher('/control/lane', Float64, queue_size=1)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.pub_bot_x = rospy.Publisher('/detect/lane_bot_x', UInt8, queue_size=1)
        self.pub_top_y = rospy.Publisher('/detect/lane_top_y', UInt8, queue_size=1)
        self.pub_top_x = rospy.Publisher('/detect/lane_top_x', UInt8, queue_size=1)

        self.pub_lane_toggle = rospy.Publisher('/detect/lane_toggle', Bool, queue_size=1)
        self.pub_white_lane_toggle = rospy.Publisher('/detect/white_toggle', Bool, queue_size=1)
        self.pub_yellow_lane_toggle = rospy.Publisher('/detect/yellow_toggle', Bool, queue_size=1)
        
        self.pub_image_traffic_sign = rospy.Publisher('/detect/sign/compressed', CompressedImage, queue_size = 1)


        self.Mission = Enum('Mission', 'Idle TrafficLight Intersection Construction Parking LevelCrossing Tunnel')
        self.IntersectionStep = Enum('IntersectionStep', 'default process done')
        self.current_step = self.IntersectionStep.default.value
        
        self.cvBridge = CvBridge()
        self.counter = 1

        self.current_orientation_w = 0.0
        self.current_orientation = 0.0
        self.is_saw_intersection = False
        self.is_saw_left_count = 0
        self.is_saw_left = False
        self.is_saw_right_count = 0
        self.is_saw_right = False
        self.is_moving_complete = False

        self.count = 0 
        self.counter = 0
        self.right_count = 0
        self.left_count = 0

    def cbOdom(self, odom_msg):
        self.current_orientation_w = odom_msg.pose.pose.orientation.w

    def cbMovingComplete(self, data):
        self.is_moving_complete = True

    def fnPreproc(self):
        # Initiate SIFT detector
        self.sift = cv2.SIFT_create()

        dir_path = os.path.dirname(os.path.realpath(__file__))
        dir_path = dir_path.replace('detect/nodes', 'detect/')
        dir_path += 'image/'

        # intersection
        self.img_intersection = cv2.imread(dir_path + 'intersection.png',0)
        self.img_left  = cv2.imread(dir_path + 'left.png',0)
        self.img_right = cv2.imread(dir_path + 'right.png',0)
        self.kp_intersection, self.des_intersection  = self.sift.detectAndCompute(self.img_intersection, None)
        self.kp_left, self.des_left  = self.sift.detectAndCompute(self.img_left, None)
        self.kp_right, self.des_right = self.sift.detectAndCompute(self.img_right, None)

        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 8) 
        # Create more charaterstics to match the original img with the camera img
        search_params = dict(checks = 100) 
        # Check 100 times 

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

        # converting compressed image to opencv image
        np_arr = np.frombuffer(image_msg.data, np.uint8)
        cv_image_input = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # converting raw image to opencv image
        # cv_image_input = CvBridge.imgmsg_to_cv2(image_msg, "bgr8")


        MIN_MATCH_COUNT = 8 #original 9
        MIN_MSE_DECISION = 65000

        # find the keypoints and descriptors with SIFT
        kp1, des1 = self.sift.detectAndCompute(cv_image_input,None)


        image_out_num = 1
        
        if self.is_saw_intersection == False:

            matches_intersection = self.flann.knnMatch(des1,self.des_intersection,k=2)
            
            good_intersection = []
            for m,n in matches_intersection:
                if m.distance < 0.65*n.distance:
                    good_intersection.append(m)
            if len(good_intersection)>MIN_MATCH_COUNT:
                src_pts = np.float32([kp1[m.queryIdx].pt for m in good_intersection ]).reshape(-1,1,2)
                dst_pts = np.float32([self.kp_intersection[m.trainIdx].pt for m in good_intersection]).reshape(-1,1,2)

                M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,3.0)
                matches_intersection = mask.ravel().tolist()

                mse = self.fnCalcMSE(src_pts, dst_pts)
                if mse < MIN_MSE_DECISION:
                    rospy.loginfo("detect intersection sign")
                    image_out_num = 2
                    self.is_saw_intersection = True
        if self.is_saw_intersection == True:
            # rospy.loginfo("Detecting Left and Right Sign")
            if self.is_saw_right == False and self.is_saw_left == False:
                matches_left = self.flann.knnMatch(des1,self.des_left,k=2)
                good_left = []
                
                for m,n in matches_left:
                    if m.distance < 0.55*n.distance:
                        good_left.append(m)
                if len(good_left)>MIN_MATCH_COUNT:
                    src_pts = np.float32([kp1[m.queryIdx].pt for m in good_left ]).reshape(-1,1,2)
                    dst_pts = np.float32([self.kp_left[m.trainIdx].pt for m in good_left]).reshape(-1,1,2)

                    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,3.0)
                    matches_left = mask.ravel().tolist()

                    mse = self.fnCalcMSE(src_pts, dst_pts)
                    if mse < MIN_MSE_DECISION:
                        image_out_num = 3
                        self.is_saw_left_count += 1
                        if self.is_saw_left_count >= 5:
                            self.is_saw_left = True
                            rospy.loginfo("detect left sign")

            if self.is_saw_right == False and self.is_saw_left == False:
                matches_right = self.flann.knnMatch(des1,self.des_right,k=2)
                good_right = []
                # rospy.loginfo("right no good")
                for m,n in matches_right:
                    if m.distance < 0.55*n.distance:
                        good_right.append(m)
                if len(good_right)>MIN_MATCH_COUNT:
                    src_pts = np.float32([kp1[m.queryIdx].pt for m in good_right ]).reshape(-1,1,2)
                    dst_pts = np.float32([self.kp_right[m.trainIdx].pt for m in good_right]).reshape(-1,1,2)

                    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,3.0)
                    matches_right = mask.ravel().tolist()

                    mse = self.fnCalcMSE(src_pts, dst_pts)
                    if mse < MIN_MSE_DECISION:
                        image_out_num = 4
                        self.is_saw_right_count += 1
                        if (self.is_saw_right_count >= 3):
                            self.is_saw_right = True
                            rospy.loginfo("detect right sign")
 


        if image_out_num == 1:
            # publishes traffic sign image in compressed type
            self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_compressed_imgmsg(cv_image_input, "jpg"))

        elif image_out_num == 2:
            draw_params_intersection = dict(matchColor = (255,0,0), # draw matches in green color
                        singlePointColor = None,
                        matchesMask = matches_intersection, # draw only inliers
                        flags = 2)

            final_intersection = cv2.drawMatches(cv_image_input,kp1,self.img_intersection,self.kp_intersection,good_intersection,None,**draw_params_intersection)

            # publishes traffic sign image in compressed type
            self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_compressed_imgmsg(final_intersection, "jpg"))

        elif image_out_num == 3:
            draw_params_left = dict(matchColor = (255,0,0), # draw matches in green color
                        singlePointColor = None,
                        matchesMask = matches_left, # draw only inliers
                        flags = 2)
           

            final_left = cv2.drawMatches(cv_image_input,kp1,self.img_left,self.kp_left,good_left,None,**draw_params_left)

            # publishes traffic sign image in compressed type
            self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_compressed_imgmsg(final_left, "jpg"))

        elif image_out_num == 4:
            draw_params_right = dict(matchColor = (255,0,0), # draw matches in green color
                            singlePointColor = None,
                            matchesMask = matches_right, # draw only inliers
                            flags = 2)
            

            fianl_right = cv2.drawMatches(cv_image_input,kp1,self.img_right,self.kp_right,good_right,None,**draw_params_right)

            # publishes traffic sign image in compressed type
            self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_compressed_imgmsg(fianl_right, "jpg"))
        
        # rospy.loginfo("is saw")
        # rospy.loginfo(self.is_saw_intersection == True)
        # rospy.loginfo("cur = inter")
        # rospy.loginfo(self.current_step == self.IntersectionStep.default.value)
        
        # Just to test
        # self.is_saw_intersection = True

        if self.is_saw_intersection == True and self.count == 0:
            rospy.sleep(4.2)
            self.pub_lane_toggle.publish(False)

            self.turn_fn(25, "left")
            self.move_fn(0.152, 'forward')
            
            
            # Just to test
            rospy.sleep(1)
            # self.is_saw_right= True
            self.count += 1

        if self.is_saw_intersection == True and self.current_step == self.IntersectionStep.default.value:
            # if self.current_orientation_w <= 0.1 and self.current_orientation_w >= -0.1 :

            self.current_step = self.IntersectionStep.process.value
            # rospy.loginfo("intersection and operation")
            self.pub_lane_toggle.publish(False)

        if (self.is_saw_left == True or self.is_saw_right == True) and self.current_step == self.IntersectionStep.process.value:
            # self.current_step = self.IntersectionStep.done.value

            if self.is_saw_right == True and self.right_count == 0:
                # 90 , 40, 150, 120
                self.pub_top_x.publish(90)
                self.pub_bot_x.publish(140)
                rospy.sleep(1)
                rospy.loginfo('RIGHT')
                self.turn_fn(76, "right")
                rospy.sleep(1)
                self.move_fn(0.15, 'forward')
                
                self.pub_yellow_lane_toggle.publish(False)
                self.pub_lane_toggle.publish(True)
                rospy.sleep(6.2)
                self.pub_lane_toggle.publish(False)
                
                
                # self.move_fn(0.18, 'forward')
                self.move_fn(0.3, 'forward')

                self.turn_fn(90, "right")
                self.move_fn(0.12, 'forward')
                

                self.pub_yellow_lane_toggle.publish(True)
                self.pub_lane_toggle.publish(True)
                
                self.right_count += 1
                
            elif self.is_saw_left == True and self.left_count == 0:
                # 90 , 40, 100, 120
                # self.pub_top_x.publish(90)
                # self.pub_bot_x.publish(100)
                # self.pub_bot_x.publish(155)
                rospy.sleep(1)
                self.turn_fn(78, "left")
                rospy.sleep(1)
                self.move_fn(0.15, 'forward')
                
                self.pub_white_lane_toggle.publish(False)
                self.pub_lane_toggle.publish(True)
                rospy.loginfo("Lane Follow")
                rospy.sleep(10)
                # self.pub_lane_toggle.publish(False)

                # Leaving the intersection
                # self.turn_fn(20, "right")
                # self.move_fn(0.13, 'forward')
                # self.turn_fn(55, "left")
                # self.move_fn(0.2, 'forward')
                
                # self.pub_white_lane_toggle.publish(True)
                # self.pub_lane_toggle.publish(True)
                self.left_count += 1

            # go to next mission
            if self.counter == 0:
                self.counter += 1
                self.pub_white_lane_toggle.publish(False)
            rospy.loginfo("Finished Intersection :)")
            rospy.sleep(4.2)

            self.pub_mission.publish(self.Mission.Construction.value)
            
            
            

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
        if move == 'forward':
            count = 1
        if move == 'backward':
            count = -1
        rate_hz = 20
        target_dist = abs(distance)
        dist_moved = 0.0

        # set up the Twist message: negative x for backwards
        twist = Twist()
        twist.linear.x = count * abs(speed)

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
    rospy.init_node('detect_intersection')
    node = DetectSign()
    node.main()