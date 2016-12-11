
'''
Copyright (c) 2015, Mark Silliman
All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

# A very basic TurtleBot script that moves TurtleBot forward indefinitely. Press CTRL + C to stop.  To run:
# On TurtleBot:
# roslaunch turtlebot_bringup minimal.launch
# On work station:
# python movement_scan.py

import pygame
import numpy as np

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import vehicle_controller
import dagger_nn

# nn_dagger_kinect or nn_dagger_hokuyo
#SAVE_FILE='../turtlebot_data/nn_dagger_kinect'
SAVE_FILE='nn_dagger'

def subsample(scan):
    """
    Takes every 10th point of the scan.
    """
    new_scan = [scan[i] for i in range(0, len(scan), 10)]
    return new_scan

def remove_inf(scan):
    """
    Converts all inf points to 100.
    """
    def convert(n):
        if np.isinf(n) or np.isnan(n):
            return 0.0
        else:
            return n
    return map(convert, scan)

def reset(controller):
    controller.train()
    controller.model.save()
    controller.model.save_train_data()
    if np.random.rand()<0.99:
        print 'control: policy_learn'
        controller.control = 'policy_learn'
    else:
        print 'control: user'
        controller.control = 'user'

class GoForward():
    def __init__(self, model):
        # initiliaze
        rospy.init_node('GoForward', anonymous=False)

        # tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")

        # What function to call when you ctrl + c    
        rospy.on_shutdown(self.shutdown)

        # Create a publisher which can "talk" to TurtleBot and tell it to move
        # Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.previous_data = None
        self.model = dagger_nn.NNDaggerModel(save_file_prefix=SAVE_FILE)
        self.controller = vehicle_controller.DaggerPursuitController(None)
        self.controller.model = self.model

        #TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
        r = rospy.Rate(10);

        # Twist is a datatype for velocity
        self.controller.control = 'policy'
        move_cmd = Twist()
        pygame.display.set_mode((200,200))
        # steps: the number of frames per round
        steps = 0
        # rounds: the number of times training is done
        rounds = 0
        while not rospy.is_shutdown():
            steps += 1
            move_cmd.linear.x = 0
            move_cmd.angular.z = 0
            if self.previous_data:
                state = remove_inf(subsample(self.previous_data.ranges))
            else:
                state = []
            print state
            control = self.controller.next_action(state)
            print control
            if 'FWD' in control:
                move_cmd.linear.x = 0.5
            if 'LEFT' in control:
                move_cmd.angular.z = 3
            if 'BACK' in control:
                move_cmd.linear.x = -0.5
            if 'RIGHT' in control:
                move_cmd.angular.z = -3
            # as long as you haven't ctrl + c keeping doing...
            # publish the velocity
            self.cmd_vel.publish(move_cmd)
            print move_cmd
            print 'len(state): ', len(state)
            # wait for 0.1 seconds (10 HZ) and publish again
            r.sleep()

    def scan_callback(self, data):
        """
        The data is of the type LaserScan.
        """
        self.previous_data = data

    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
    # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
    # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)

if __name__ == '__main__':
        GoForward(model=None)
        #rospy.loginfo("GoForward node terminated.")

