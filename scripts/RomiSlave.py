#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
#from a_star = Astar()

from std_msgs.msg import String

COMMAND_RATE = 20

class RosRomiNode:

    def __init__(self):
        #self.a_star = AStar()
        rospy.Subscriber("romi/cmd_vel", Twist, self.motorCallback)

    def motorCallback(self, msg):
        left = 0.0
        right = 0.0
        if msg.linear.x != 0:
            left = msg.linear.x * 400
        if msg.linear.y != 0:
            right = msg.linear.y * 400
        print("Left: ", int(left), " Right: ", int(right))
        #a_star.motors(int(left),int(right))



if __name__ == '__main__':
    rospy.init_node("pololu_node")
    COMMAND_RATE = rospy.get_param('~command_rate', COMMAND_RATE)
    r = rospy.Rate(COMMAND_RATE)
    node = RosRomiNode()
    while not rospy.is_shutdown():
        #node.publish_encoder_states()
        r.sleep()