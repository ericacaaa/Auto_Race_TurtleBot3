import rospy, os, cv2, math
import numpy as np
from enum import Enum
from std_msgs.msg import UInt8, Float64, Bool
from sensor_msgs.msg import CompressedImage, LaserScan
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from turtlebot3_autorace_msgs.msg import MovingParam

class DetectSign():
    def __init__(self):

        self.Mission = Enum('Mission', 'Idle TrafficLight Intersection Construction Parking LevelCrossing Tunnel')
        self.ParkingStep = Enum('ParkingStep', 'default process')
        self.current_step_parking = self.ParkingStep.default.value
       
        self.cvBridge = CvBridge()
        self.counter = 1

        self.is_saw_parking = False
        self.start_obstacle_detection = False
        self.is_moving_complete = False
        self.is_obstacle_detected_R = False
        self.is_obstacle_detected_L = False

        self.fnPreproc()

        # subscribes compressed image
        self.sub_img = rospy.Subscriber('/image_input', CompressedImage, self.cbFindTrafficSign, queue_size = 1)
        self.sub_moving_completed = rospy.Subscriber('/control/moving/complete', UInt8, self.cbMovingComplete, queue_size = 1)
        self.sub_scan_obstacle = rospy.Subscriber('/scan', LaserScan, self.cbScanObstacle, queue_size=1)


        # publishes next mission
        self.pub_mission = rospy.Publisher('/mission', UInt8, queue_size=1)
        self.pub_mission_parking = rospy.Publisher('/mission/parking', UInt8, queue_size=1)
        self.pub_max_vel = rospy.Publisher('/control/max_vel', Float64, queue_size = 1)
        self.pub_fake_lane = rospy.Publisher('/control/lane', Float64, queue_size=1)
        self.pub_moving = rospy.Publisher('/control/moving/state', MovingParam, queue_size= 1)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        self.pub_right_side_block = rospy.Publisher('/detect/block_right_side', Bool, queue_size=1)
        self.pub_lane_toggle = rospy.Publisher('/detect/lane_toggle', Bool, queue_size=1)
        self.pub_white_toggle = rospy.Publisher('/detect/white_toggle', Bool, queue_size=1)
        self.pub_yellow_toggle = rospy.Publisher('/detect/yellow_toggle', Bool, queue_size=1)

        self.counter = 0
        # publishes traffic sign image in compressed type
        self.pub_image_traffic_sign = rospy.Publisher('/detect/sign/compressed', CompressedImage, queue_size = 1)


    def cbMovingComplete(self, data):
        self.is_moving_complete = True

    def cbScanObstacle(self, scan):
        angle_scan = 15

        scan_start_left = 32 - angle_scan
        scan_end_left = 32 + angle_scan

        scan_start_right = 200 - angle_scan
        scan_end_right = 200 + angle_scan

        threshold_distance = 0.8

        if self.start_obstacle_detection == True:
            for i in range(scan_start_left, scan_end_left):
                if scan.ranges[i] < threshold_distance and scan.ranges[i] > 0.01:
                    self.is_obstacle_detected_L = True
                    rospy.loginfo("left detected")
           

            for i in range(scan_start_right, scan_end_right):
                if scan.ranges[i] < threshold_distance and scan.ranges[i] > 0.01:
                    self.is_obstacle_detected_R = True
                    rospy.loginfo("right detected")

            self.start_obstacle_detection = False

    def fnPreproc(self):
        # Initiate SIFT detector
        self.sift = cv2.SIFT_create() #xfeatures2d.SIFT_create()

        dir_path = os.path.dirname(os.path.realpath(__file__))
        dir_path = dir_path.replace('detect/nodes', 'detect/')
        dir_path += 'image/'

        self.img_parking = cv2.imread(dir_path + 'parking.png', 0)
        self.kp_parking, self.des_parking = self.sift.detectAndCompute(self.img_parking, None)

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

        MIN_MATCH_COUNT = 9 #9
        MIN_MSE_DECISION = 50000

        # find the keypoints and descriptors with SIFT
        kp1, des1 = self.sift.detectAndCompute(cv_image_input,None)
        image_out_num = 1

        

        # parking
        if self.is_saw_parking == False:
            matches = self.flann.knnMatch(des1,self.des_parking,k=2)
            good = []
            for m,n in matches:
                if m.distance < 0.7*n.distance:
                    good.append(m)
            if len(good)>MIN_MATCH_COUNT:
                src_pts = np.float32([kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
                dst_pts = np.float32([self.kp_parking[m.trainIdx].pt for m in good]).reshape(-1,1,2)

                M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
                matches = mask.ravel().tolist()

                mse = self.fnCalcMSE(src_pts, dst_pts)
                if mse < MIN_MSE_DECISION:
                    rospy.loginfo("detect parking")
                    image_out_num = 2
                    # self.is_saw_parking = True
                    

        if image_out_num == 1:
            # publishes traffic sign image in compressed type
            self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_compressed_imgmsg(cv_image_input, "jpg"))
           
        elif image_out_num == 2:
            draw_params = dict(matchColor = (255,0,0), # draw matches in green color
                            singlePointColor = None,
                            matchesMask = matches, # draw only inliers
                            flags = 2)

            final_parking = cv2.drawMatches(cv_image_input,kp1,self.img_parking,self.kp_parking,good,None,**draw_params)

            # publishes traffic sign image in compressed type
            self.pub_image_traffic_sign.publish(self.cvBridge.cv2_to_compressed_imgmsg(final_parking, "jpg"))
        
        # Turn on parking 
        self.is_saw_parking = True

        # parking process
        if self.is_saw_parking == True and self.counter == 0:
            # rospy.sleep(15)
            rospy.sleep(0.8)
            rospy.loginfo('Yellow lane follow')
            
            # self.pub_lane_toggle.publish(False)
            # self.turn_fn(95, "left")
            # self.move_fn(0.2, 'forward')
            # self.pub_yellow_toggle.publish(True)
            self.pub_right_side_block.publish(True)
            self.pub_white_toggle.publish(False)
            self.pub_lane_toggle.publish(True)
            rospy.sleep(2)
            self.pub_lane_toggle.publish(False)
            self.move_fn(0.2, 'forward', 0.22)
            self.turn_fn(56, "left")
            self.move_fn(0.08, 'forward', 0.22)
            self.pub_lane_toggle.publish(True)


            # rospy.sleep(12)
            rospy.sleep(2.5)
            self.pub_lane_toggle.publish(False)
            self.move_fn(0.02, 'forward', 0.22)

           

            self.pub_mission_parking.publish(self.ParkingStep.process.value)


            self.pub_lane_toggle.publish(False)
            self.start_obstacle_detection = True
            # rospy.loginfo("Left")
            # rospy.loginfo(self.is_obstacle_detected_L)
            # rospy.loginfo("Right")
            # rospy.loginfo(self.is_obstacle_detected_R)
            rospy.sleep(2.5)

            while True:
                if self.is_obstacle_detected_L == True:
                    break
                elif self.is_obstacle_detected_R == True:
                    break

            rospy.loginfo("[PARKING] PARKING...")

            if self.is_obstacle_detected_L == True:
                rospy.loginfo("[PARKING] RIGHT_CLEAR (Go Right)")
                self.move_fn(0.25, 'forward', 0.22)
                self.turn_fn(95, 'right')
                self.move_fn(0.22, 'forward', 0.22)
                rospy.sleep(2)
                self.move_fn(0.27, 'backward', 0.05)
                self.turn_fn(90, 'right')

            elif self.is_obstacle_detected_R == True:
                rospy.loginfo("[PARKING] LEFT_CLEAR (Go Left)")
                self.move_fn(0.25, 'forward', 0.22)
                self.turn_fn(90, 'left')
                self.move_fn(0.22, 'forward', 0.22)
                rospy.sleep(2)
                self.move_fn(0.27, 'backward', 0.05)
                self.turn_fn(96, 'left')
               
            rospy.sleep(1)

            rospy.loginfo("[PARKING] GO OUT...")
            self.move_fn(0.3, 'forward', 0.22)

            self.pub_lane_toggle.publish(True) 
            rospy.sleep(2.5)  
            self.pub_lane_toggle.publish(False)
            self.move_fn(0.3, 'forward', 0.22)

            self.turn_fn(83, 'left')
            self.move_fn(0.1, 'forward', 0.22)

            rospy.loginfo("[PARKING] FINISH...")
            self.counter += 1
            rospy.loginfo("Parking Finished :)")

            self.pub_right_side_block.publish(False)
            self.pub_white_toggle.publish(True)
            self.pub_yellow_toggle.publish(True)   
            self.pub_lane_toggle.publish(True)

            # go to next mission
            self.pub_mission.publish(self.Mission.LevelCrossing.value)
            
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
    def move_fn(self, distance, move, speed):
        
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

        # stop
        # self.pub_cmd_vel.publish(Twist())

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('detect_parking')
    node = DetectSign()
    node.main()