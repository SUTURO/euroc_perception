__author__ = 'smurpheus'

import rospy
from suturo_msgs.msg import Task


class Classifier(object):
    def __init__(self):
        # rospy.loginfo("Starting ColorDetector")
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("/suturo/yaml_pars0r", Task, self.set_yaml_infos)
        # rospy.Service("/suturo/yaml_pars0r", Task, self.set_yaml_infos)
        # spin the wheel
        rospy.spin()

    def set_yaml_infos(self, data):
        print("Trololololo   " + str(data.objects))

        objects = data.objects
        for object in objects:
            name = object.name
            color = object.color
            shape = object.shapes
            print('Object is being "learned: %s"' % self.get_surrounding_cuboid(object))

    def get_surrounding_cuboid(self, object):
        # for each in object.shapes:
        if len(object.shapes) == 1:
            shape = object.shapes[0]
            print(shape.dimensions)
            if shape.shape_type == 3:
                z = shape.dimensions[0]
                x = 2 * shape.dimensions[1]
                y = x
                return [x, y, z]
            if shape.shape_type == 1:
                return [shape.dimensions[0], shape.dimensions[1], shape.dimensions[2]]
            return [0, 0, 0]  #
        if len(object.shapes) > 1:
            z, y, x = 0, 0, 0
            for shape in object.shapes:
                if shape.shape_type == 3:
                    z += shape.dimensions[0]
                    if x < 2 * shape.dimensions[1]: x = 2 * shape.dimensions[1]
                    if y < 2 * shape.dimensions[1]: y = 2 * shape.dimensions[1]
                if shape.shape_type == 1:
                    z += shape.dimensions[2]
                    if x < 2 * shape.dimensions[0]: x = shape.dimensions[0]
                    if y < 2 * shape.dimensions[1]: y = shape.dimensions[1]
            return [x, y, z]


    def randomify_objects(self, objects, size_treshold, color_treshold):
        for object in objects:
            pass

    def classify_object(self, object):
        return object