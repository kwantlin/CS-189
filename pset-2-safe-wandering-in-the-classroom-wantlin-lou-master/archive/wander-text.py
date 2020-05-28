import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent, CliffEvent, WheelDropEvent
from sensor_msgs.msg import PointCloud2, LaserScan, Image

def scan_callback(msg):
    global g_range_ahead
    g_range_ahead = min(msg.ranges)

class DepthScan:
    """
    Use the depth camera to find objects within a certain distance range
    """

    def __init__(self):

        g_range_ahead = 1 # anything to start
        driving_forward = True

        # tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")

        # What function to call when you ctrl + c
        rospy.on_shutdown(self.shutdown)

        # Create a publisher which can "talk" to TurtleBot and tell it to move
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.scan = rospy.Subscriber('scan', LaserScan, scan_callback)

        rospy.init_node('DepthScan')
        state_change_time = rospy.Time.now()

        # TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 5 Hz
        self.rate = rospy.Rate(10)

    def run(self):
        """
        Run until Ctrl+C pressed
        :return: None
        """
        while not rospy.is_shutdown():
            if driving_forward:
                if (g_range_ahead < 0.8 or rospy.Time.now() > state_change_time):
                    driving_forward = False
                    state_change_time = rospy.Time.now() + rospy.Duration(5)
            else: # we're not driving_forward
                if rospy.Time.now() > state_change_time:
                    driving_forward = True # we're done spinning, time to go forward!
                    state_change_time = rospy.Time.now() + rospy.Duration(30)
            twist = Twist()
            if driving_forward:
             twist.linear.x = 1
            else:
             twist.angular.z = 1
            cmd_vel_pub.publish(twist)

            rate.sleep()

    def shutdown(self):
        """
        Pre-shutdown routine. Stops the robot before rospy.shutdown
        :return: None
        """
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
