#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from a_star import AStar
from MockBot import MockBot
from math import sin, cos, pi

from std_msgs.msg import String, Int16, Float32

COMMAND_RATE = 20

class RosRomiNode:

    def __init__(self):
        #Create a_star instance to talk to arduino based a_star Romi board
        self.a_star = AStar()
        #self.a_star = MockBot() 
        self.encoder_resolution = 12    #according to pololu encoder count
        self.gear_reduction = 120   #according to romi website
        self.wheel_diameter = 0.07  #70mm diameter wheels according to website
        self.wheel_track = 0.141 #149mm outside to outside according to drawing, 8mm thick tires
        self.motorMax = 400

        #internal data
        self.leftEncoder = None   #last encoder state variables for odometry
        self.rightEncoder = None
        self.x = 0
        self.y = 0
        self.th = 0

        self.leftMotor = 0
        self.rightMotor = 0

        #time variables for odometry pulled from http://github.com/hbrobotics/ros_arduino_bridge
        self.rate = float(50)   #hard coding this for now, will be param'd later
        self.tickes_per_meter = self.encoder_resolution * self.gear_reduction / (self.wheel_diameter * pi)

        now = rospy.Time.now()
        self.then = now
        self.t_delta = rospy.Duration(1.0 / self.rate)
        self.t_next = now + self.t_delta

        rospy.Subscriber("lmotor_cmd", Float32, self.lmotorCallback)
        rospy.Subscriber("rmotor_cmd", Float32, self.rmotorCallback)

        self.lwheelPub = rospy.Publisher('lwheel', Int16, queue_size=5)
        self.rwheelPub = rospy.Publisher('rwheel', Int16, queue_size=5)

    def lmotorCallback(self, msg):
        leftMotor = msg.data * self.motorMax
        if leftMotor > self.motorMax:
            leftMotor = self.motorMax
        elif leftMotor < -self.motorMax:
            leftMotor = -self.motorMax
        self.leftMotor = int(leftMotor)
        self.updateMotors()

    def rmotorCallback(self, msg):
        rightMotor = msg.data * self.motorMax
        if rightMotor > self.motorMax:
            rightMotor = self.motorMax
        elif rightMotor < -self.motorMax:
            rightMotor = -self.motorMax
        self.rightMotor = int(rightMotor)
        self.updateMotors()

    def updateMotors(self):
        print("Left: ", self.leftMotor, "Right: ", self.rightMotor)
        self.a_star.motors(self.leftMotor,self.rightMotor)

    def pollEncoders(self):
        now = rospy.Time.now()
        if now > self.t_next:
            try:
                leftEnc, rightEnc = self.a_star.read_encoders()
                self.lwheelPub.publish(leftEnc)
                self.rwheelPub.publish(rightEnc)
            except:
                return

        self.t_next = now + self.t_delta



if __name__ == '__main__':
    rospy.init_node("pololu_node")
    COMMAND_RATE = rospy.get_param('~command_rate', COMMAND_RATE)
    r = rospy.Rate(COMMAND_RATE)
    node = RosRomiNode()
    while not rospy.is_shutdown():
        #node.publish_encoder_states()
        node.pollEncoders()
        r.sleep()
