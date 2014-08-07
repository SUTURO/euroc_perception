#!/usr/bin/env python
import numpy as np
import cv2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def callback(data):
	print "Got Data"
	bridge = CvBridge()
	a = bridge.imgmsg_to_cv2(data,"rgb8")
	#a = np.fromstring(str(data.serialize_numpy), dtype=np.uint8 )
	print "Converted Data"
	print a
	#a.reshape(a.size/3,3)
	print "Reshaped"
	cv2.imwrite("/home/smurpheus/euroc_ws/src/euroc_perception/euroc_perception_color_recognizer/src/test.png", a)
	rospy.signal_shutdown("reason")

def listener():
	print "Started"
	rospy.init_node('listen', anonymous=True)
	print "Started2"
	rospy.Subscriber("/euroc_interface_node/cameras/tcp_rgb_cam/", Image, callback)
	print "Started3"
	rospy.spin()

if __name__ == "__main__":
	print "Started"
	listener()