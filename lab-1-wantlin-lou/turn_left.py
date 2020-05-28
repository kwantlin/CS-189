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
2018-01: Modified by Julia Ebert
"""

import rospy
from geometry_msgs.msg import Twist
from math import radians

# constant for speed 
LIN_SPEED = 0.2
ROT_SPEED = radians(90)


class TurnLeft:
    """
    Turn left for a fixed duration.
    """

    def __init__(self):
        # Initialize
        rospy.init_node('TurnLeft', anonymous=False)

        # What to do you ctrl + c (call shutdown function written below)
        rospy.on_shutdown(self.shutdown)

        # Create publisher for controlling robot
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

        rospy.sleep(1)  # give rospy time to register the publisher
         
        # 10 Hz
        self.rate = rospy.Rate(10)

    def run(self):
        """
        Run for 10 iterations publishing a left turn twist, then sleep 1 s
        :return: None
        """
        # Generate a 'left' twist object.
        turn_left = Twist()
        turn_left.linear.x = 0
        turn_left.angular.z = ROT_SPEED  # 90 deg/s in radians/s

        for i in range(10):
            self.cmd_vel.publish(turn_left)
            self.rate.sleep()
        rospy.sleep(1)

    def shutdown(self):
        """
        Pre-shutdown routine. Stops the robot before rospy.shutdown 
        :return: None
        """
        # stop turtlebot
        rospy.loginfo("Stop!")
        # publish a zeroed out Twist object
        self.cmd_vel.publish(Twist())
        # sleep before final shutdown
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        robot = TurnLeft()
        robot.run()
    except Exception, err:
        rospy.loginfo("TurnLeft node terminated.")
        print err
