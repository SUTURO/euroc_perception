#!/usr/bin/python
# -*- encoding: utf-8 -*-

from __future__ import division

import rospy
from suturo_perception_msgs.srv import Classifier

from suturo_perception_classifier.classifier import Classifier as CF


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

