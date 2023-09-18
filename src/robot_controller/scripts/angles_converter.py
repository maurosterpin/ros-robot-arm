#!/usr/bin/env python3
import rospy
import math
from robot_controller.srv import AnglesConverter, AnglesConverterResponse

def convert_radians_to_degrees(req):
    # Function that is called every time the service radians_to_degrees is called
    # It receives the Request message as input with the angles in radians
    # and returns the Result message as output with the angles in degrees
    res = AnglesConverterResponse()
    res.base = int(((req.base+(math.pi/2))*180)/math.pi)
    res.shoulder = 180-int(((req.shoulder+(math.pi/2))*180)/math.pi)
    res.elbow = int(((req.elbow+(math.pi/2))*180)/math.pi)
    res.gripper = int(((-req.gripper)*180)/(math.pi/2))
    return res

def convert_degrees_to_radians(req):
    # Function that is called every time the service radians_to_degrees is called
    # It receives the Request message as input with the angles in degrees
    # and returns the Result message as output with the angles in radians
    res = AnglesConverterResponse()
    res.base = ((math.pi*req.base) - ((math.pi/2)*180))/180
    res.shoulder = (((180-req.shoulder)*math.pi)-((math.pi/2)*180))/180
    res.elbow = ((math.pi*req.elbow) - ((math.pi/2)*180))/180
    res.gripper = -((math.pi/2)*req.gripper)/180
    return res


if __name__ == "__main__":
    # Inizialize a ROS node called angles_converter
    rospy.init_node('angles_converter')

    # Inizialize two services for the angle conversions 
    radians_to_degrees = rospy.Service('radians_to_degrees', AnglesConverter, convert_radians_to_degrees)
    degrees_to_radians = rospy.Service('degrees_to_radians', AnglesConverter, convert_degrees_to_radians)

    # keeps the node up and running
    rospy.spin()