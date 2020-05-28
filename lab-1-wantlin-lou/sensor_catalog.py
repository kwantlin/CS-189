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

# for motor control
# info: http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
from geometry_msgs.msg import Twist

# sensory information related to kobuki base
# bumpers (x3), cliff sensor, wheel drop sensors, led, button press, etc.
# Full list available: https://github.com/yujinrobot/kobuki_msgs/tree/hydro/msg 
# and additional info: http://docs.ros.org/indigo/api/kobuki_msgs/html/index-msg.html
from kobuki_msgs.msg import BumperEvent, ButtonEvent, CliffEvent, Led, WheelDropEvent


class ConnectSensorsDoNothing:
    """
    An example dummy class that connects to the various sensors but doesn't do
    anything with the data.
    Use the methods and calls as an example to build your own code.
    """

    def __init__(self):
        # Initialize node
        rospy.init_node('ConnectSensorsDoNothing', anonymous=False)

        # boolean class vars for bump, button, cliff, led, wheel drop event processing
        self.bump = False
        self.button = False
        self.cliff = False
        self.wheel_drop = False

        # ctrl + c -> calls self.shutdown function
        rospy.on_shutdown(self.shutdown)

        # print msg 
        rospy.loginfo("Hello World!")
                
        # Publish motion control; Twist instructions
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

        # Example twist object (default when calling Twist()): 
        move_obj = Twist()
        move_obj.linear.x = 0
        move_obj.linear.y = 0
        move_obj.linear.z = 0
        move_obj.angular.x = 0 
        move_obj.angular.y = 0
        move_obj.angular.z = 0

        # Publish led1
        self.led1 = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=10)

        # Publish led2
        self.led2 = rospy.Publisher('/mobile_base/commands/led2', Led, queue_size=10)

        # Subscribe to receive bump data; process with processBumpSensing
        # when bumper node detects an event, that event is sent to process_bump_sensing for processing.
        rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self.process_bump_sensing)

        # Subscribe to receive button data; process with process_button_sensing
        rospy.Subscriber('mobile_base/events/button', ButtonEvent, self.process_button_sensing)

        # Subscribe to receive cliff data; process with process_cliff_sensing
        rospy.Subscriber('mobile_base/events/cliff', CliffEvent, self.process_cliff_sensing)

        # Subscribe to receive wheel drop data; process with process_wheel_sensing
        rospy.Subscriber('mobile_base/events/wheel_drop', WheelDropEvent, self.process_wheel_sensing)

        # How often should provide commands? 10 HZ
        self.rate = rospy.Rate(10)

    def run(self):
        """
        Run the robot loop until Ctrl+C is pressed
        :return: None
        """
        # do nothing.
        while not rospy.is_shutdown():
            # Twist() sends an zeroed Twist object
            self.cmd_vel.publish(Twist())
            # self.led1.publish(Led.RED)
            # self.led2.publish(Led.GREEN)
            self.rate.sleep()

    def shutdown(self):
        """
        Pre-shutdown routine. Stops the robot before rospy.shutdown 
        :return: None
        """
        # print msg 
        rospy.loginfo("Stop")
        # zero motion by using empty Twist object
        self.cmd_vel.publish(Twist())
        # wait for robot to stop before shutdown
        rospy.sleep(5)

    def process_bump_sensing(self, data):
        """
        If bump data is received, process the data
        data.bumper: LEFT (0), CENTER (1), RIGHT (2)
        data.state: RELEASED (0), PRESSED (1)    
        :param data: Raw bump sensor data 
        :return: None
        """
        if data.state == BumperEvent.PRESSED:
            self.bump = True
            if data.bumper == BumperEvent.LEFT:
                rospy.loginfo("Left bumper")
            elif data.bumper == BumperEvent.CENTER:
                rospy.loginfo("Center bumper")
            elif data.bumper == BumperEvent.RIGHT:
                rospy.loginfo("Right bumper")
        else:
            self.bump = False
        rospy.loginfo("Bumper Event")
        rospy.loginfo(data.bumper)

    def process_button_sensing(self, data):
        """
        If button data is received, process the data
        data.button: Button0 (0), Button1 (1), or Button2 (2)
        data.state: RELEASED (0), PRESSED (1)    
        :param data: Raw button data 
        :return: None
        """
        if data.state == ButtonEvent.PRESSED:
            self.button = True
        else:
            self.button = False
        rospy.loginfo("Button Event")
        rospy.loginfo(data.button)

    def process_cliff_sensing(self, data):
        """
        If cliff data is received, process the data
        data.sensor: LEFT (0), CENTER (1), RIGHT (2)
        data.state: FLOOR (0), CLIFF (1)
        data.bottom: unint16: distance to floor when cliff detected    
        :param data: Raw cliff sensor data 
        :return: None
        """
        if data.state == CliffEvent.CLIFF:
            self.cliff = True
        else:
            self.cliff = False
        rospy.loginfo("Cliff Event")
        rospy.loginfo(data.sensor)

    def process_wheel_sensing(self, data):
        """
        If wheel drop data is received, process the data
        data.wheel: LEFT (0), RIGHT (1)
        data.state: RAISED (0), DROPPED (1)
        :param data: Raw wheel sensor data
        :return: None
        """
        if data.state == WheelDropEvent.DROPPED:
            self.wheel_drop = True
        else:
            self.wheel_drop = False
        rospy.loginfo("Wheel Drop Event")
        rospy.loginfo(data.wheel)

 
if __name__ == '__main__':
    try:
        robot = ConnectSensorsDoNothing()
        robot.run()
    except Exception, err:
        rospy.loginfo("ConnectSensorsDoNothing node terminated.")
        print err
