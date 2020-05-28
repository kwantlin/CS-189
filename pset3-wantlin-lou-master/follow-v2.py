"""
Copyright (c) 2015, Mark Silliman
All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
   disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
   disclaimer in the documentation and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

2016: Modified by Serena Booth
2018-01: Modified by Julia Ebert and Florian Berlinger
"""
# PRIORITIZE SAFETY

# travel to the future
# from __future__ import division

import rospy

# import mathy things
import math
import numpy as np
from math import radians
import random

# import cv2 and image things
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# import sound things
from kobuki_msgs.msg import Sound
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

# impoort move things
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent, CliffEvent, WheelDropEvent

MIN_PERIMETER = 100
LIN_SPEED = 0.15
ROT_SPEED = radians(45)
MIN_THRESHOLD = 0.6
LAST_ROW = 375

# Where and what test
HALLWAY = False
RACE = False

class ViewImages:
    """
    Use the RGB camera to find the largest green object
    """

    def __init__(self):

        # init obstacle avoidance stats
        self.min_val = 0
        self.bump = False
        self.obstacle_x = 0

        # init bounding box
        self.x = 0
        self.y = 0
        self.w = 0
        self.h = 0

        # init error and lost frame counter
        self.ang_error = 0
        self.dist_error = 0
        self.lost = 0

        # init perimeter tracker
        self.perimeter_size = -1

        # init dist to a low value so that robot doesn't go at first
        self.dist = -1

        # init other dist vars
        self.median = -1
        self.mean = -1
        self.min = -1
        self.max = -1

        rospy.init_node('ViewImages', anonymous=False)

        # init time and speed tracker
        self.last_move = rospy.Time.now()
        self.last_speed = 0

        # ctrl + c -> call self.shutdown function
        rospy.on_shutdown(self.shutdown)

        # How often should provide commands? 5 Hz
        self.rate = rospy.Rate(10)

        self.bridge = CvBridge()

        rospy.Subscriber('/camera/rgb/image_raw', Image, self.process_image, queue_size=1, buff_size=2 ** 24)
        rospy.Subscriber('/camera/depth/image', Image, self.process_depth_image, queue_size=1, buff_size=2 ** 24)
        rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self.process_bump_sensing)

        self.cmd_vel = rospy.Publisher('wanderer_velocity_smoother/raw_cmd_vel',Twist, queue_size=1)

        self.soundhandle = SoundClient()
        rospy.sleep(2)

        self.beep = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=10)
        rospy.sleep(2)

        # print msg
        rospy.loginfo("Initialized!")

    def run(self):
        """
        Run the robot until Ctrl+C
        We're not moving, so not much happens here
        :return: None
        """
        move_cmd = Twist()
        move_cmd.linear.x = 0
        move_cmd.angular.z = 0

        while not rospy.is_shutdown():
            # bump logic as previous psets
            if self.bump:
                self.bump = False
                # move backwards
                move_cmd.linear.x = LIN_SPEED * -1
                for i in range(5):
                    self.cmd_vel.publish(move_cmd)
                    self.rate.sleep()
                rospy.sleep(1)

                # turn randomly in a random direction
                move_cmd.linear.x = 0
                move_cmd.angular.z = ROT_SPEED * ((-1)**random.randint(1,2))

                if self.bump == 0:
                    move_cmd.angular.z = ROT_SPEED * (-1)
                elif self.bump == 2:
                    move_cmd.angular.z = ROT_SPEED

                for i in range(random.randint(5,15)):
                    self.cmd_vel.publish(move_cmd)
                    self.rate.sleep()
                rospy.sleep(1)

                move_cmd.angular.z = 0
            # if somethin in the screen is really close
            elif self.min_val < MIN_THRESHOLD:
                # make sure it's not the sock/leg warmer, and is actually an obstacle
                if self.obstacle_x <= self.x or self.obstacle_x >= self.x + self.w or abs(self.min_val - self.dist) > 0.1:
                    move_cmd.linear.x = 0
                    # turn away
                    if self.obstacle_x > 320:
                        move_cmd.angular.z = ROT_SPEED / 2
                    else:
                        move_cmd.angular.z = -ROT_SPEED / 2
                    # self.min_val = 100
                    for i in range(10):
                        self.cmd_vel.publish(move_cmd)
                        self.rate.sleep()
                    self.last_move = rospy.Time.now()
            else:
                rospy.loginfo("Perimeter " + str(self.perimeter_size))
                rospy.loginfo("Distance is " + str(self.dist))

                # normalize angle error to rot speed
                ang_error_norm = -float(self.ang_error) / 100

                # set min and max rot speed
                if ang_error_norm < -ROT_SPEED:
                    ang_error_norm = -ROT_SPEED
                elif ang_error_norm > ROT_SPEED:
                    ang_error_norm = ROT_SPEED

                move_cmd.angular.z = ang_error_norm

                if RACE == False:
                    # normalize dist error to lin speed
                    self.dist_error = self.dist - 0.5
                    dist_error_norm = float(self.dist_error) / 2

                    if dist_error_norm < 0:
                        # if NaN (self.dist gets set to -1)
                        if dist_error_norm > -0.7:
                            self.lost = 0
                        # if too close
                        else:
                            self.lost += 1
                        # if it's been more than 2 seconds
                        if rospy.Time.now() > self.last_move + rospy.Duration(2):
                            dist_error_norm = 0
                            # if been lost for a while rotate and beep
                            if self.lost > 20:
                                move_cmd.angular.z = ROT_SPEED / 4
                                self.beep.publish(4)
                        else:
                            # continue as previous
                            dist_error_norm = self.last_speed
                    else:
                        # set max lin speed
                        if dist_error_norm > LIN_SPEED:
                            dist_error_norm = LIN_SPEED

                        # reset lost stats
                        self.lost = 0
                        self.last_speed = dist_error_norm
                        self.last_move = rospy.Time.now()

                    move_cmd.linear.x = dist_error_norm
                else:
                    move_cmd.linear.x = LIN_SPEED

                self.cmd_vel.publish(move_cmd)

    def threshold(self, img):
        """
        Apply the threshold to the image to get only green parts
        :param img: RGB camera image
        :return: Masked image
        """

        # Define range of green color in HSV
        if HALLWAY == True:
            lower_green = np.array([35,120,0])
            upper_green = np.array([80,255,255])
        else:
            lower_green = np.array([35,0,0])
            upper_green = np.array([80,255,200])

        # Apply a blur
        img = cv2.medianBlur(img, 5)
        # Convert image to easier-to-work-with HSV format
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Threshold to get only green values
        mask = cv2.inRange(img_hsv, lower_green, upper_green)
        masked_img = cv2.bitwise_and(img, img, mask=mask)

        # Color space conversion back from HSV
        masked_img = cv2.cvtColor(masked_img, cv2.COLOR_HSV2BGR)

        return masked_img

    def bound_green_object(self, img):
        """
        Draw a bounding box around the largest green object in the scene
        :param img: RGB camera image
        :return: Image with bounding box
        """

        self.x = self.y = self.w = self.h = 0

        # Apply the threshold to get the green parts
        masked_img = self.threshold(img)

        # Get contours
        img_gray = cv2.cvtColor(masked_img, cv2.COLOR_BGR2GRAY)
        _, contours, _ = cv2.findContours(img_gray, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            # Find the largest contour
            areas = [cv2.contourArea(c) for c in contours]
            max_index = np.argmax(areas)
            max_contour = contours[max_index]

            self.perimeter_size = cv2.arcLength(max_contour, True)

            if self.perimeter_size < MIN_PERIMETER:
                return img

            # Draw rectangle bounding box on image
            self.x, self.y, self.w, self.h = cv2.boundingRect(max_contour)

            # if wider than long, ignore
            if float(self.h) / self.w < 1:
                self.x = self.y = self.w = self.h = 0
                return img

            cv2.rectangle(img, (self.x, self.y), (self.x + self.w, self.y + self.h), color=(0, 255, 0), thickness=2)

            # get center
            M = cv2.moments(max_contour)
            if M['m00'] > 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                # draw a circle on the center
                cv2.circle(img, (cx, cy), 20, (0,0,255), -1)
                # calc ang error
                self.ang_error = cx - 320

        return img

    def process_image(self, data):
        """
        Process an RGB image from the camera whenever received
        :param data: Raw RGB image
        :return: None
        """
        try:

            # Convert the image from ROS format to OpenCV format
            # 'bgr8' means it will encode as 8-bit values in BGR channels
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # Apply a threshold to your image
            cv_image = self.bound_green_object(cv_image)
            # Display the modified image
            cv2.imshow('picture', cv_image)
            cv2.waitKey(3)
        except CvBridgeError, e:
            rospy.loginfo(e)

    def process_depth_image(self, data):
        """
        Process a depth image from the camera whenever received
        :param data: Raw depth image
        :return: None
        """
        try:
            depth_image = self.bridge.imgmsg_to_cv2(data)

            # reset min_val to big value
            self.min_val = 100

            # find min dist, and note the x for that location
            def find_min_val(depth_image):
                for x in range(0, LAST_ROW):
                    counter = 0
                    for y in depth_image[x]:
                        counter += 1
                        if y < self.min_val:
                            self.obstacle_x = counter
                            self.min_val = y
                            if self.min_val <= MIN_THRESHOLD:
                                return

            find_min_val(depth_image)

            self.dist = -1
            self.median = -1

            bounding_rect = depth_image[self.y:(self.y + self.h), self.x:(self.x+self.w)]
            median_depth = np.nanmedian(bounding_rect)

            # set dist to median of bounding box, if box is not 0 in area, and if not a box of NaNs (otherwise self.dist = -1)
            if len(bounding_rect) > 0 and math.isnan(median_depth) == False:
                self.dist = self.median = median_depth

        except CvBridgeError, e:
            rospy.loginfo(e)


    def process_bump_sensing(self, data):
        """
        If bump data is received, process the data
        data.bumper: LEFT (0), CENTER (1), RIGHT (2)
        data.state: RELEASED (0), PRESSED (1)
        :param data: Raw message data from bump sensor
        :return: None
        """
        if data.state == BumperEvent.PRESSED:
            self.bump = True
            self.bumpDir = data.bumper
        rospy.loginfo("Bumper Event :)")
        rospy.loginfo(data.bumper)

    def shutdown(self):
        """
        Pre-shutdown routine. Stops the robot before rospy.shutdown
        :return: None
        """
        cv2.destroyAllWindows()
        rospy.loginfo("Stop")
        # wait for robot to stop before shutdown
        rospy.sleep(5)


if __name__ == '__main__':
    try:
        robot = ViewImages()
        robot.run()
    except Exception, err:
        rospy.loginfo("ViewImages node terminated.")
        print err
