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

import math
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

MIN_PERIMETER = 50
LAST_ROW = 375

class ViewImages:
    """
    Use the RGB camera to find the largest green object
    """

    def __init__(self):

        self.x = 0
        self.y = 0
        self.w = 0
        self.h = 0

        self.perimeter_size = -1

        # init dist to a low value so that robot doesn't go at first
        self.dist = 0

        self.median = -1
        self.mean = -1
        self.min = -1
        self.max = -1

        rospy.init_node('ViewImages', anonymous=False)

        # ctrl + c -> call self.shutdown function
        rospy.on_shutdown(self.shutdown)

        # How often should provide commands? 5 Hz
        self.rate = rospy.Rate(5)

        self.bridge = CvBridge()

        rospy.Subscriber('/camera/rgb/image_raw', Image, self.process_image, queue_size=1, buff_size=2 ** 24)
        rospy.Subscriber('/camera/depth/image', Image, self.process_depth_image, queue_size=1, buff_size=2 ** 24)

        # print msg
        rospy.loginfo("Initialized!")

    def run(self):
        """
        Run the robot until Ctrl+C
        We're not moving, so not much happens here
        :return: None
        """
        while not rospy.is_shutdown():
            rospy.loginfo("Perimeter " + str(self.perimeter_size))
            if self.dist > 6:
                rospy.loginfo("Too Close")
            else:
                rospy.loginfo("Distance is " + str(self.dist))
            rospy.loginfo("dist median is " + str(self.median))
            rospy.loginfo("dist mean is " + str(self.mean))
            rospy.loginfo("dist min is " + str(self.min))
            rospy.loginfo("dist max is " + str(self.max))
            rospy.loginfo("x is " + str(self.x))
            rospy.loginfo("y is " + str(self.y))
            rospy.sleep(0.5)

    def threshold(self, img):
        """
        Apply the threshold to the image to get only green parts
        :param img: RGB camera image
        :return: Masked image
        """

        # Define range of green color in HSV
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

            # Draw contours (for debugging)
            # cv2.drawContours(img, max_contour, -1, (0,255,0), 3)

            # Draw rectangle bounding box on image
            self.x, self.y, self.w, self.h = cv2.boundingRect(max_contour)

            # this will ignore floor, but doesn't seem to help...

            # if (self.y + self.h) > LAST_ROW:
            #     if self.y < LAST_ROW:
            #         self.h = LAST_ROW - self.y
            #     else:
            #         self.x = self.y = self.w = self.h = 0
            #         return img
            cv2.rectangle(img, (self.x, self.y), (self.x + self.w, self.y + self.h), color=(0, 255, 0), thickness=2)


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
            # Use bridge to convert to CV::Mat type.
            # NOTE: SOME DATA WILL BE 'NaN'
            # and numbers correspond to distance to camera in meters
            # This imports as the default data encoding. For the ASUS Xtion cameras,
            # this is '32FC1' (single precision floating point [32F], single channel [C1])
            depth_image = self.bridge.imgmsg_to_cv2(data)

            # self.dist = 5
            # print("x_img " + str(x))
            # print("y_img " + str(y))
            # print("w_img " + str(w))
            # print("h_img " + str(h))

            # dist_x = math.floor(self.w / 2 + self.x)
            # dist_y = math.floor(self.h / 2 + self.y)
            # # iterate through all values in the top 350 rows and find the min
            # self.dist = depth_image[dist_x][dist_y]

            # norm_img = depth_image
            # cv2.normalize(norm_img, norm_img, 0, 1, cv2.NORM_MINMAX,
            #               mask=np.isfinite(norm_img).astype(np.uint8))
            self.dist = -1
            self.median = -1
            self.mean = -1
            self.min = -1
            self.max = -1
            bounding_rect = depth_image[self.y:(self.y + self.h), self.x:(self.x+self.w)]
            median_depth = np.nanmedian(bounding_rect)
            if len(bounding_rect) > 0 and math.isnan(median_depth) == False:
                self.dist = self.median = median_depth
                self.mean = np.nanmean(bounding_rect)
                self.min = np.nanmin(bounding_rect)
                self.max = np.nanmax(bounding_rect)

            # bounded_values = []
            # if len(bounding_rect) > 0:
            #     for x in np.nditer(bounding_rect):
            #         if math.isnan(x) == False:
            #             bounded_values.append(x)
            #
            # # print(bounding_rect)
            # if len(bounded_values) > 0:
            #     self.dist = np.median(bounded_values)


            # for x in range(0, 375):
            #     for y in depth_image[x]:
            #         if y < self.dist:
            #             self.dist = y



            # dist_x = self.x
            # dist_y = self.y
            # self.dist = -1
            # while self.dist <= 0 or math.isnan(self.dist):
            #     dist_x += 1
            #     if dist_x >= (self.x + self.w):
            #         dist_x = self.x
            #         dist_y += 1
            #         if dist_y >= (self.y + self.h):
            #             self.dist = -1
            #             break
            #     self.dist = depth_image[dist_y][dist_x]

            # Use inRange() to segment. Get only obstacles within 0.1 - 1.0 m of camera
            # segmented_depth_mask = cv2.inRange(cv_image2, 0.1, 1.0)

            # Normalize image to display it
            # norm_img = cv_image
            # cv2.normalize(norm_img, norm_img, 0, 1, cv2.NORM_MINMAX,
            #               mask=np.isfinite(norm_img).astype(np.uint8))
            # cv2.imshow('Raw image', depth_image)
            # cv2.imshow('Normalized image', norm_img)
            #
            # # Alternatively, show image of the segmented depth mask

            # cv2.imshow('Depth Image', depth_image[self.y:self.y + self.h][self.x:self.x + self.w])
            # cv2.waitKey(3)
        except CvBridgeError, e:
            rospy.loginfo(e)


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
