#!/usr/bin/env python

"""
GazeboMapper
2018-01: Modified by Julia Ebert and Florian Berlinger
2019-01: Mapper module written by Mark Peternson
2020-04: Modified by Billy Koech and Ernest Ochieng for gazebo
"""

# import ROS and math modules
import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent, CliffEvent, WheelDropEvent
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from math import *
import math
import numpy as np
import random

# EKF related modules usd to solve for the position and orientation
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import (Point, PointStamped, PoseWithCovarianceStamped,
                               Quaternion, Twist)


#Import mapping class as MapDrawer
from map_util import MapDrawer

# open cv for displaying depth image
import cv2
from cv_bridge import CvBridge, CvBridgeError


# Motion onfigurations
LIN_SPEED = 0.4  # m/s
ROT_SPEED = radians(45)  # 45 deg/s (in radians)
FLOOR_HEIGHT = 100  # bottom pixels of img to ignore
NUM_SEGMENTS = 3  # Number of segments to divide the depth image into

class Mapper:
    """
    Use the depth camera to wander safely
    """

    def __init__(self):
        # Initialize node
        rospy.init_node('Mapper', 'time', anonymous=False)

        # Postion and orientation variables of turtlebot
        self.position = None
        self.orientation = None
        self.start_mapping = False

        # Create map array
        # The map takes on one of three values - -1 for unknown, 0 for empty,
        # and 1 for occupied
        self.map = -np.ones((40, 30))

        # TODO: create mapper object from MapDrawer class and pass in
        # the start position of the turtlebot in grid coordinates
        ####################################################################
        # code here
        self.start_pos = (35,15)
        self.mapper = MapDrawer(self.start_pos)
        self.cur_pos = self.start_pos
        self.depth_row = np.empty(640)
        #initialize all entriesto Nan
        self.depth_row[:] = np.NaN
        self.empty_cell = (-1, -1)

        ####################################################################

        # Robot linear movement state ('forward', 'left', 'right', None)
        self.state = None

        # Direction to wander (left, right, None) (set by data from depth camera)
        self.wander_direction = None

        # tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")

        # What function to call when you ctrl + c
        rospy.on_shutdown(self.shutdown)

        # Use a CvBridge to convert ROS image type to CV Image (Mat)
        self.bridge = CvBridge()


        # Create a publisher which can "talk" to TurtleBot and tell it to move
        self.cmd_vel = rospy.Publisher('/cmd_vel',Twist, queue_size=10)

        # Subscribe to depth topic
        rospy.Subscriber('/camera/depth/image_raw', Image, self.process_depth_image, queue_size=1, buff_size=2 ** 24)

        # Subscribe to EKF topic to get pose data
        rospy.Subscriber('/robot_pose_ekf/odom_combined',
                         PoseWithCovarianceStamped, self.process_ekf)

        # TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move?
        # Use 8 HZ. NOTE: Higher frequencies break the map image
        self.rate = rospy.Rate(8)

        # Wait for position data to be ready
        now = rospy.Time.now()
        while rospy.Time.now() - now < rospy.Duration(1) or self.position is None:
            pass

        # Don't start mapping until your position data is valid
        self.start_mapping = True


    def run(self):
        """
        Run (Mapper) until Ctrl+C is pressed
        :return: None
        """

        # as long as you haven't ctrl + c keeping doing...
        while not rospy.is_shutdown():

            # Determine movement based on depth camera
            self.state = self.wander_direction

            # Get movement command (Twist) based on robot state and publish
            self.cmd_vel.publish(self.get_move_cmd())

            # TODO: update display using the self.mapper.UpdateMapDisplay() function
            ####################################################################
            # code here
            self.mapper.UpdateMapDisplay(self.map, self.cur_pos)

            ####################################################################

            # wait for 0.125 seconds (8 HZ) and publish again
            self.rate.sleep()

    # TODO: implement a function that converts real world coordinates to grid coordinates
    # Hint: when you first lauch the simulation the turtlebot starts at (35,15) in the grid
    def WorldPositionToMap(self, position):
        # Function that converts the turtlebot's position in the world to grid coordinates
        # on the OG Map

        ####################################################################
        # code here
        x = int(round(35 + position[0]*5.52))
        y = int(round(15 + position[1]*6.22))
        map_pos = (x,y)

        ####################################################################

        # print(map_pos)
        return map_pos

    def WorldPositionToMap_depth(self, position):
        # Function that converts the turtlebot's position in the world to grid coordinates
        # on the OG Map

        ####################################################################
        # code here
        x = int(round(35 + position[0]*5.6))
        y = int(round(15 + position[1]*6.3))
        map_pos = (x,y)

        ####################################################################

        # print(map_pos)
        return map_pos





    def get_move_cmd(self):
        """
        Create movement command according to robot's state
        :return: Twist of movement to make
        """

        # Start with command with 0 linear and angular velocity (stop command)
        move_cmd = Twist()

        if self.state == 'reverse':
            move_cmd.linear.x = -1 * LIN_SPEED / 2.0
        elif self.state == 'forward':
            move_cmd.linear.x = LIN_SPEED
            # choose a random turning direction
            move_cmd.angular.z = random.randint(-1,1) * ROT_SPEED
        elif self.state == 'left':
            move_cmd.angular.z = ROT_SPEED
        elif self.state == 'right':
            move_cmd.angular.z = -ROT_SPEED

        return move_cmd

    def depth_to_displace(self, x, y, ang, darr):
        new_row = [(-100, -100) for i in range(640)]
        for i in range(270,370):
            if math.isnan(darr[i]) or darr[i]>1.5:
                continue
            else:
                adjusted_ang = ang+((i-320)*(58*(math.pi)/180)/640)
                new_x = x + (math.cos(adjusted_ang)*darr[i])
                new_y = y + (math.sin(adjusted_ang)*darr[i])
                newtuple = (new_x, new_y)
                # print(newtuple)
                new_row[i] = (new_x,new_y)
        # print(new_row[300:340])
        return new_row

    def find_empty(self):
        for i in range(35, 5, -1):
            for j in range(5, 25, 1):
                if self.map[i][j] == -1:
                    return (i,j)
        return (-1, -1)

    def orient(self, curr_pos, goal_pos):
        return math.atan2(
            goal_pos[1] - curr_pos[1],
            goal_pos[0] - curr_pos[0])

    def displacement(self, curr_pos, goal_pos):

        # print(goal_pos)
        # print(curr_pos)
        tuple = (curr_pos[0] - goal_pos[0], curr_pos[1] - goal_pos[1])
        return tuple

    def angle_compare(self, curr_angle, goal_angle):
        """
        Determine the difference between the angles, normalized from -pi to +pi
        :param curr_angle: current angle of the robot, in radians
        :param goal_angle: goal orientation for the robot, in radians
        """
        pi2 = 2 * math.pi
        # Normalize angle difference
        angle_diff = (curr_angle - goal_angle) % pi2
        # Force into range 0-2*pi
        angle_diff = (angle_diff + pi2) % pi2
        # Adjust to range of -pi to pi
        # if (angle_diff > math.pi):
        #     angle_diff -= pi2
        return angle_diff


    def mark_enclosed(self, x, y):
        count = 0
        if y <29 and y>0 and x <39 and x>0 and (self.map[x-1][y] == 1 or x < 0):
            count += 1
        if x <39 and x>0 and y <29 and y>0 and (self.map[x][y+1] == 1 or y > 29):
            count += 1
        if x <39 and x>0 and y <29 and y>0 and (self.map[x][y-1] == 1 or y < 0):
            count += 1
        if y <29 and y>0 and x <39 and x>0 and (self.map[x+1][y] == 1 or x > 39):
            count += 1

        if count >=3 :
            return True
        else:
            return False



    def process_ekf(self, data):
        """
        Updates the current position of the robot and marks any cells under the
        robot as Empty.
        :param data: Raw message data from the EKF
        :return: None
        """
        # Save the position and orientation
        pos = data.pose.pose.position
        self.position = (pos.x, pos.y)
        quat = data.pose.pose.orientation
        list_quat = [quat.x, quat.y, quat.z, quat.w]
        self.orientation = euler_from_quaternion(list_quat)[-1]

        # Extract the relevant covariances (uncertainties).
        # Note that these are uncertainty on the robot VELOCITY, not position
        cov = np.reshape(np.array(data.pose.covariance), (6, 6))
        x_var = cov[0, 0]
        y_var = cov[1, 1]
        rot_var = cov[5, 5]

        # diplay position and orientation of turtlebot
        rospy.loginfo({"position": self.position})
        rospy.loginfo({"orientation": self.orientation})



        # TODO: Update the path traced by the turtlebot on the map with the current position
        ####################################################################
        # code here
        #update depth slice
        world_x = self.position[0]
        world_y = self.position[1]
        updated_depth_row = self.depth_to_displace(world_x, world_y, self.orientation, self.depth_row)

        for i in range(len(updated_depth_row)):
            if updated_depth_row[i] != (-100, -100):
                x1 = (self.position)[0]
                y1 = (self.position)[1]
                tuple = self.WorldPositionToMap(updated_depth_row[i])
                x = tuple[0]
                y = tuple[1]
                if x>=39:
                    x = 39
                elif x<=0:
                    x = 0
                if y>=29:
                    y = 29
                elif y<=0:
                    y=0

                slope = (self.displacement(self.position, updated_depth_row[i]))
                # print("Slope")
                # print(slope)

                for i in range(10):
                    x1 -= slope[0]/10
                    y1 -= slope[1]/10
                    # print("Original Point " + str(x1) + "  " + str(y1))
                    tuple = self.WorldPositionToMap((x1,y1))
                    # print("Tuple")
                    # print(tuple)
                    if tuple[0]<39 and tuple[0]>0 and tuple[1]<29 and tuple[1]>0 and i < 8:
                        self.map[tuple[0]][tuple[1]] = 0

                self.map[x][y] = 1


        x = self.WorldPositionToMap(self.position)[0]
        y = self.WorldPositionToMap(self.position)[1]
        self.cur_pos = (x,y)
        self.map[x][y] = 0
        self.map[x-1][y] = 0
        self.map[x-1][y-1] = 0
        self.map[x][y-1] = 0

        # self.mark_empty((x-2, y-1), (x-2, y), 0)
        for x1 in range(40):
            for y1 in range(30):
                if self.map[x1][y1] == -1:
                    if self.mark_enclosed(x1, y1):
                        self.map[x1][y1] = 1

        ####################################################################


    def process_depth_image(self, data):
        """
        Process a depth image from the camera whenever received
        :param data: Raw depth image
        :return: None
        """

        try:
            # Use bridge to convert to CV::Mat type.
            # NOTE: SOME DATA WILL BE 'NaN'
            # Numbers correspond to distance to camera in meters
            cv_image = self.bridge.imgmsg_to_cv2(data, 'passthrough')

            # Set the direction to wander
            mod_img = self.set_wander_direction(cv_image)

            # make the depth image available globally
            self.depth_image = mod_img
            self.depth_row = cv_image[300]
            # print(self.depth_row)

            # Show the modified image
            # Displaying both depth image and map image may cause opencv to freeze
            #cv2.imshow('Depth Image', mod_img)
            #cv2.waitKey(5)


        except CvBridgeError, e:
            rospy.loginfo(e)



    def set_wander_direction(self, img):
        """
        Determine which direction to wander based on whether there's a closer object in left/right side of depth image
        :param img: Depth image in OpenCV format
        :return: Modified (and mutates self.wander_direction)
        """
        # Use inRange() to segment. Get only nearby obstacles
        # note: These range values haven't been calibrated
        depth_img_mask = cv2.inRange(img, 0, 1)

        depth_array = np.array(depth_img_mask)

        img_height, img_width = img.shape[:2]

        # Split image into left, middle, right segments and remove the floor
        img_segments = [depth_array[:-FLOOR_HEIGHT, i*img_width/NUM_SEGMENTS:(i+1)*img_width/NUM_SEGMENTS]
            for i in range(NUM_SEGMENTS)]

        # Turn away if something on the sides; otherwise move forward
        filled_space = [np.count_nonzero(seg) for seg in img_segments]

        if filled_space[1] == 0:
            # Clear ahead; go forward
            new_wander_direction = 'forward'
        elif np.argmax(filled_space) == 0:
            # Stuff on left; turn right
            new_wander_direction = 'right'
        elif np.argmax(filled_space) == 2:
            # Stuff on right; turn left
            new_wander_direction = 'left'
        else:
            if self.find_empty:
                print("Find Empty")
                print(self.find_empty())
                dest_orientation = self.orient(self.position, self.find_empty())
                angle_diff = self.angle_compare(self.orientation, dest_orientation)
                if angle_diff < math.pi/2 or angle_diff > 3*math.pi/2:
                    new_wander_direction = 'left'
                else:
                    new_wander_direction = 'right'
            else:
                self.shutdown()


        if new_wander_direction == 'forward' or self.wander_direction == 'forward' or not self.wander_direction:
            # Set new wander direction if:
            # - previously or currently going forward
            # - previously wander_dirction was None
            self.wander_direction = new_wander_direction
        # Otherwise keep the same turn direction (aka don't change turning direction
        # once you've started turning)

        # Draw a green rectangle over the direction the robot is moving
        if self.wander_direction == 'right':
            x = img_width / NUM_SEGMENTS * (NUM_SEGMENTS - 1)
        elif self.wander_direction == 'left':
            x = 0
        else:
            x = img_width / NUM_SEGMENTS
        # Convert to color image so you can draw a colored box
        color_img = cv2.cvtColor(depth_img_mask, cv2.COLOR_GRAY2RGB)
        cv2.rectangle(color_img, (x, 0), (x + img_width/NUM_SEGMENTS, img_height-FLOOR_HEIGHT), color=(0,255,0), thickness=2)
        return color_img

    def shutdown(self):
        """
        Pre-shutdown routine. Stops the robot before rospy.shutdown
        :return: None
        """
        # save map
        rospy.loginfo("Saving Map")

        # TODO: Save map to local file system using the self.mapper.SaveMap()
        # the file name should include the .jpg extension, for example: 'solution_map.jpg'
        ####################################################################
        # code here
        self.mapper.SaveMap("solution_4b_office.jpg", self.cur_pos)

        ####################################################################
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(5)


if __name__ == '__main__':
    try:
        mapper = Mapper()
        mapper.run()
    except Exception, err:
        rospy.loginfo("Mapper node terminated.")
        print err
