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

import cv2
import rospy
import numpy as np
from math import radians
import random
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent, CliffEvent, WheelDropEvent
from sensor_msgs.msg import PointCloud2, LaserScan, Image
from cv_bridge import CvBridge, CvBridgeError

LIN_SPEED = 0.1
ROT_SPEED = radians(90)


class DepthScan:
    """
    Use the depth camera to find objects within a certain distance range
    """

    def __init__(self):

        #reset count and bump
        self.count = 0
        self.bump = False

        # Initialize Node
        rospy.init_node('DepthScan', anonymous=False)

        # tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")

        # What function to call when you ctrl + c
        rospy.on_shutdown(self.shutdown)

        # Create a publisher which can "talk" to TurtleBot and tell it to move
        self.cmd_vel = rospy.Publisher('wanderer_velocity_smoother/raw_cmd_vel',Twist, queue_size=10)

        # Use a CvBridge to convert ROS image type to CV Image (Mat)
        self.bridge = CvBridge()

        # Subscribe to depth and bumper topics
        rospy.Subscriber('/camera/depth/image', Image, self.process_depth_image, queue_size=1, buff_size=2 ** 24)
        rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self.process_bump_sensing)

        # TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 5 Hz
        self.rate = rospy.Rate(10)

    def run(self):
        """
        Run until Ctrl+C pressed
        :return: None
        """
        move_cmd = Twist()

        go = False
        rotate = False
        while not rospy.is_shutdown():
            # rospy.spin()
            if self.bump:
                # move backwards
                move_cmd.linear.x = LIN_SPEED * -1
                for i in range(5):
                    self.cmd_vel.publish(move_cmd)
                    self.rate.sleep()
                rospy.sleep(1)

                # don't move just turn
                move_cmd.linear.x = 0
                move_cmd.angular.z = ROT_SPEED * ((-1)**random.randint(1,2))
                for i in range(random.randint(0,15)):
                    self.cmd_vel.publish(move_cmd)
                    self.rate.sleep()
                rospy.sleep(1)
                move_cmd.angular.z = 0
                self.bump = False

            else:
                if self.min_val >= 0.7:
                    rotate = False
                    if go == False:
                        go = True
                    else:
                        print("GOING because min is " + str(self.min_val))
                        move_cmd.linear.x = LIN_SPEED
                        move_cmd.angular.z = 0
                        go = False
                else:
                    go = False
                    if rotate == False:
                        rotate = True
                    else:
                        print("NOT going because min is " + str(self.min_val))
                        move_cmd.linear.x = 0
                        move_cmd.angular.z = ROT_SPEED
                        rotate = False

            self.cmd_vel.publish(move_cmd)
            self.rate.sleep()

    def process_depth_image(self, data):
        """
        Process a depth image from the camera whenever received
        :param data: Raw depth image
        :return: None
        """
        try:
            # Use bridge to convert to CV::Mat type.
            # NOTE: SOME DATA WILL BE 'NaN'
            # and numbers correspond to distance to camera in meters
            # This imports as the default data encoding. For the ASUS Xtion cameras,
            # this is '32FC1' (single precision floating point [32F], single channel [C1])
            cv_image = self.bridge.imgmsg_to_cv2(data)

            # Use inRange() to segment. Get only obstacles within 0.1 - 1.0 m of camera
            segmented_depth_mask = cv2.inRange(cv_image, 0.1, 1.0)

            # self.count = 0
            # for x in range(0, 230):
            #     for y in segmented_depth_mask[x]:
            #         if y != 0:
            #             self.count += 1

            self.min_val = 100
            for x in range(0, 350):
                for y in cv_image[x]:
                    if y < self.min_val:
                        self.min_val = y

            # print(self.min_val)

            # values = np.array(cv_image)
            # print(values)

            # for x in np.nditer(segmented_depth_mask):
            #     if x != 0:
            #         self.count += 1
            # print(len(segmented_depth_mask[0]))

            # Alternatively, show image of the segmented depth mask


            # cv_nofloor = [0]*400
            # for x in range(0,400):
            #     cv_nofloor[x] = cv_image[x]
            # norm_img = cv_nofloor
            #
            # cv2.normalize(norm_img, norm_img, 0, 1, cv2.NORM_MINMAX,
            #               mask=np.isfinite(norm_img).astype(np.uint8))
            # cv2.imshow('Normalized image', norm_img)
            # cv2.imshow('Depth Image', segmented_depth_mask[0,400])

            cv2.waitKey(3)
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
        # Close CV Image windows
        cv2.destroyAllWindows()
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(5)


if __name__ == '__main__':
    try:
        robot = DepthScan()
        robot.run()
    except Exception, err:
        rospy.loginfo("DepthScan node terminated.")
        print err
