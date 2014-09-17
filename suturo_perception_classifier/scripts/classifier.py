#!/usr/bin/python
# -*- encoding: utf-8 -*-

from __future__ import division
from suturo_perception_classifier.classifier import Classifier as CF
import numpy as np
import rospy
import cv2
import math
from threading import Lock
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from suturo_perception_msgs.msg import EurocObject
from suturo_perception_msgs.srv import Classifier
from std_msgs.msg import _ColorRGBA
from moveit_msgs.msg import CollisionObject

def dump(request):
 return request


def main():
    c = CF()
    rospy.init_node("Classifier", anonymous=True)
    # Create the service
    rospy.loginfo("Starting Classifier")
    rospy.Service("/suturo/Classifier", Classifier, c.classify_object)
    # spin the wheel
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

