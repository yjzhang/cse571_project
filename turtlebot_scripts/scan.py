#!/usr/bin/env python

# modified from the goforward.py script.

import pygame

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class BasicController():
    """
    This class combines scans and user inputs.
    """

    def __init__(self, model=None):
        # initialize
        rospy.init_node('BasicController', anonymous=False)

        self.scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        print 'subscribed'

	# tell user how to stop TurtleBot
	rospy.loginfo("To stop TurtleBot CTRL + C")

        # What function to call when you ctrl + c    
        rospy.on_shutdown(self.shutdown)
	# Create a publisher which can "talk" to TurtleBot and tell it to move
        # Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
	#TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 20 HZ
        self.r = rospy.Rate(20);
        # initialize control model of data -> action
        self.model = model
        self.previous_data = None

    def scan_callback(self, data):
        """
        The data is of the type LaserScan.
        """
        self.previous_data = data

    def get_command(self):
        """
        Gets the movement command given the scan data (state).
        """
        data = self.previous_data
        print data.ranges
        print data.intensities
        # Twist is a datatype for velocity
        move_cmd = Twist()
	# let's go forward at 0.2 m/s
        move_cmd.linear.x = 0.2
	# let's turn at 0 radians/s
	move_cmd.angular.z = 0
        return move_cmd

    def run(self):
        """
        Runs the main loop
        """
        while not rospy.is_shutdown():
            cmd = self.get_command()
            #self.cmd_vel.publish(move_cmd)
            self.r.sleep()

    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
	# a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
	# sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        s = BasicController()
        s.run()
    except:
        rospy.loginfo("BasicController node terminated.")

