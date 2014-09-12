__author__ = 'smurpheus'

import rospy
from suturo_msgs.msg import Task

class Classifier(object):

    def __init__(self):
        rospy.loginfo("Starting ColorDetector")
        rospy.Service("/suturo/yaml_pars0r", Task, self.set_yaml_infos)
        # spin the wheel
        rospy.spin()

    def set_yaml_infos(self, data):
        pass

    def classify_object(self, object):
        return object