__author__ = 'smurpheus'

import rospy
from suturo_msgs.msg import Task

class Classifier(object):

    def __init__(self):
        rospy.loginfo("Starting ColorDetector")
        # rospy.Service("/suturo/yaml_pars0r", Task, self.set_yaml_infos)
        # spin the wheel
        # rospy.spin()

    def set_yaml_infos(self, data):
        objects = data['objects']
        for object in objects:
            color = object.color
            shape = object.shape
            rospy.logdebug('Object is being "learned: %s"' %object)

    def get_surrounding_cuboid(self, object):
        if object.shape.type == 'cylinder':
            z = object.shape.length
            x = 2 * object.shape.radius
            y = x
            return [x, y, z]
        if object.shape.type == 'box':
            return object.shape.size
        return [0, 0 ,0]

    def randomify_objects(self, objects, size_treshold, color_treshold):
        for object in objects:
            pass

    def classify_object(self, object):
        return object