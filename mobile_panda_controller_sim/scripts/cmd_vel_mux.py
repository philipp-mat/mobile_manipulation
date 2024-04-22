#!/usr/bin/env python
import tf
import actionlib
import rospy

from control_msgs.msg import FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import numpy as np
from scipy.interpolate import interp1d

from mobile_panda_controller.kbhit import KBHit


class cmd_vel_mux( object ):

    def __init__(self):

      # create all publishers and subscribers:
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.raw_sub = rospy.Subscriber("/cmd_vel_raw", Twist, self.callback, queue_size=10)

                         
        self.kb_timer = rospy.Timer(rospy.Duration(0.01), self.keycb)
        self.kb = KBHit()
        rospy.on_shutdown(self.kb.set_normal_term) 
        
        #the flag which tells if to publish "cmd_vel" 
        self.publish_cmd_vel = 1         
                        
        return
        
    def callback(self, data):
        twist = Twist()
        if self.publish_cmd_vel==1:
            twist = data
            self.cmd_pub.publish(twist)
        else:  
            twist.linear.x = 0 # move forward at 0.1 m/s
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
            self.cmd_pub.publish(twist)

            return
        
    def keycb(self, tdat):
        # check if there was a key pressed, and if so, check it's value and
        # toggle flag:
        if self.kb.kbhit():
            c = self.kb.getch()
            if ord(c) == 27:
                print(' ESC to exit')
                rospy.signal_shutdown("Escape pressed!")
            if c == 's':
                rospy.loginfo("You pressed 's'.... stop")
                self.publish_cmd_vel = 0
                twist = Twist()
                twist.linear.x = 0
                twist.linear.y = 0
                twist.linear.z = 0
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = 0
                
                self.cmd_pub.publish(twist)
                rospy.set_param('return_to_base', False)
            if c == 'p':
                rospy.loginfo("You pressed 'p'.... starting publishing")
                rospy.set_param('return_to_base', False)
                self.publish_cmd_vel = 1
            if c == 'r':
                rospy.loginfo("You pressed 'r'.... returning to base")
                rospy.set_param('return_to_base', True)
                self.publish_cmd_vel = 1
            self.kb.flush()
        return  
        
        
def main():
    mux = cmd_vel_mux()
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('cmd_vel_mux')
    main()
