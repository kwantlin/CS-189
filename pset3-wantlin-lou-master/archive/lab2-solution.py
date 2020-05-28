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

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ViewImages:
    """
    Use the RGB camera to find the largest blue object
    """

    def __init__(self):
        rospy.init_node('ViewImages', anonymous=False)

        # ctrl + c -> call self.shutdown function
        rospy.on_shutdown(self.shutdown)

        # print msg
        rospy.loginfo("Hello World!")

        # How often should provide commands? 5 Hz
        self.rate = rospy.Rate(5)

        self.bridge = CvBridge()

        rospy.Subscriber('/camera/rgb/image_raw', Image, self.process_image, queue_size=1, buff_size=2 ** 24)

    def run(self):
        """
        Run the robot until Ctrl+C
        We're not moving, so not much happens here
        :return: None
        """
        while not rospy.is_shutdown():
            rospy.spin()

    def threshold(self, img):
        """
        Apply the threshold to the image to get only blue parts
        :param img: RGB camera image
        :return: Masked image
        """

        # Define range of blue color in HSV
        lower_blue = np.array([100, 50, 50])
        upper_blue = np.array([150, 255, 255])

        # Apply a blur
        img = cv2.medianBlur(img, 5)
        # Convert image to easier-to-work-with HSV format
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Threshold to get only blue values
        mask = cv2.inRange(img_hsv, lower_blue, upper_blue)
        masked_img = cv2.bitwise_and(img, img, mask=mask)

        # Color space conversion back from HSV
        masked_img = cv2.cvtColor(masked_img, cv2.COLOR_HSV2BGR)

        return masked_img

    def bound_blue_object(self, img):
        """
        Draw a bounding box around the largest blue object in the scene
        :param img: RGB camera image
        :return: Image with bounding box
        """

        # Apply the threshold to get the blue parts
        masked_img = self.threshold(img)

        # Get contours
        img_gray = cv2.cvtColor(masked_img, cv2.COLOR_BGR2GRAY)
        _, contours, _ = cv2.findContours(img_gray, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            # Find the largest contour
            areas = [cv2.contourArea(c) for c in contours]
            max_index = np.argmax(areas)
            max_contour = contours[max_index]

            # Draw contours (for debugging)
            # cv2.drawContours(img, max_contour, -1, (0,255,0), 3)

            # Draw rectangle bounding box on image
            x, y, w, h = cv2.boundingRect(max_contour)
            cv2.rectangle(img, (x, y), (x + w, y + h), color=(0, 255, 0), thickness=2)

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
            cv_image = self.bound_blue_object(cv_image)
            # Display the modified image
            cv2.imshow('picture', cv_image)
            cv2.waitKey(3)
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
