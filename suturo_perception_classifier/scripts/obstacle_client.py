#!/usr/bin/python
# -*- encoding: utf-8 -*-
from __builtin__ import object

__author__ = 'smurpheus'

import rospy

from suturo_perception_msgs.srv import Classifier
from suturo_perception_msgs.srv import GetCameraPerception



if __name__ == "__main__":
    s = 'height,centroid,color'
    rospy.wait_for_service('suturo/perception/GetGripper')
    try:
        perceived_objects = rospy.ServiceProxy('suturo/perception/GetGripper', GetCameraPerception=
        object =  perceived_objects(s).objects
        print object
        rospy.wait_for_service('suturo/perception/Classifier')
        try:
            classified = rospy.ServiceProxy('suturo/perception/Classifier', Classifier)
            for each in object:
                classified_object =  classified(each).classifiedObject
                print classified_object
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

