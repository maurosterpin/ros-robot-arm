#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import imutils
import numpy as np
from std_msgs.msg import Float32

bridge = CvBridge()


def callback(img_msg):
    # This function is called each time a new message is published on the topic /chatter
    # The message that has been published is then passed as input to this function
    # rospy.loginfo("I heard the image")
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        height, width, channels = cv_image.shape
 
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        lower_blue = np.array([90, 60, 0])   
        upper_blue = np.array([121, 255, 255]) 

        mask = cv2.inRange(hsv,lower_blue, upper_blue)  

        m = cv2.moments(mask, False)

        try:
            calc_y, calc_x = m['m10']/m['m00'], m['m01']/m['m00']
            calc_x = 455.6 - calc_x
            calc_y = 475.31 - calc_y
            c_x = calc_x
            c_y = 0.000841556 * calc_y
            calc_x = (0.004555809 * (calc_x)) - 0.904830
            calc_y = (0.005649718 * (calc_y + 16)) + 0.094830
            # if calc_y < 0:
            #     calc_y = calc_y + c_y
            # else:
            #     calc_y = calc_y - c_y
            # cxy[0], cxy[1] = calc_x, calc_y
            # print("x: ", calc_x)
            # print("y: ", calc_y)
            rospy.set_param('x', calc_x)
            rospy.set_param('y', calc_y)
            # xy = [calc_x, calc_y]
            # pubx.publish(calc_x)
            # puby.publish(calc_y)
        except ZeroDivisionError:
            calc_x, calc_y = height/2, width/2
            # xy = [calc_x, calc_y]
            # pubx.publish(calc_x)
            # puby.publish(calc_y)

    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
   
    
if __name__ == '__main__':
    # Inizialize a ROS node called image_listener
    rospy.init_node('image_listener', anonymous=True)

    # Inizialize a ROS node called image_publisher
    # rospy.init_node('image_publisher', anonymous=True)

    # register a subscriber on the topic /chatter that will listen for String messages
    # when a new message is received, the callback function is triggered and starts its execution
    rospy.Subscriber("/image_raw", Image, callback)

    # pubx = rospy.Publisher("image_x", Float32, queue_size=10)
    # puby = rospy.Publisher("image_y", Float32, queue_size=10)

    r = rospy.Rate(1)

    # keeps the node up and running
    rospy.spin()
