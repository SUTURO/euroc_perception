__author__ = 'smurpheus'

import random
import copy

from sklearn import tree
import rospy
from suturo_msgs.msg import Task


class Classifier(object):
    original_objects = None
    cubeized_objects = {}
    size_tresh_perc = 0.01
    color_tresh = 10
    number_of_data = 10

    def __init__(self):
        # rospy.loginfo("Starting ColorDetector")
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("/suturo/yaml_pars0r", Task, self.set_yaml_infos)
        self.clf = tree.DecisionTreeClassifier()
        # rospy.Service("/suturo/yaml_pars0r", Task, self.set_yaml_infos)
        # spin the wheel
        rospy.spin()

    def set_yaml_infos(self, data):
        objects = data.objects
        self.original_objects = objects
        all_data = {}
        for object in objects:
            print object
            name = object.name
            color = object.color
            primitive = object.primitives
            self.cubeized_objects[name] = {'color': color, 'dimensions': self.get_surrounding_cuboid(object)}
            randomized_objects = self.randomize_objects(self.cubeized_objects, self.number_of_data,
                                                        self.size_tresh_perc,
                                                        self.color_tresh)
            all_data[name] = randomized_objects
        data, labels = self.convert_to_dataset(all_data)
        self.clf.fit(data, labels)

    def get_surrounding_cuboid(self, object):
        # for each in object.primitives:
        if len(object.primitives) == 1:
            primitive = object.primitives[0]
            print(primitive.dimensions)
            if primitive.type == 3:
                z = primitive.dimensions[0]
                x = 2 * primitive.dimensions[1]
                y = x
                return [x, y, z]
            if primitive.type == 1:
                return [primitive.dimensions[0], primitive.dimensions[1], primitive.dimensions[2]]
            return [0, 0, 0]  #
        if len(object.primitives) > 1:
            z, y, x = 0, 0, 0
            for primitive in object.primitives:
                if primitive.type == 3:
                    z += primitive.dimensions[0]
                    if x < 2 * primitive.dimensions[1]: x = 2 * primitive.dimensions[1]
                    if y < 2 * primitive.dimensions[1]: y = 2 * primitive.dimensions[1]
                if primitive.type == 1:
                    z += primitive.dimensions[2]
                    if x < 2 * primitive.dimensions[0]: x = primitive.dimensions[0]
                    if y < 2 * primitive.dimensions[1]: y = primitive.dimensions[1]
            return [x, y, z]


    def randomize_objects(self, objects, number, size_treshold, color_treshold):

        for object in objects:
            randomized = []
            for i in range(0, number):
                r = int(objects[object]['color'][:2], 16)
                g = int(objects[object]['color'][2:4], 16)
                b = int(objects[object]['color'][-2:], 16)
                r_rand = random.randrange(color_treshold)
                g_rand = random.randrange(color_treshold)
                b_rand = random.randrange(color_treshold)
                if r + r_rand > 255:
                    r -= r_rand
                else:
                    r += r_rand
                if g + g_rand > 255:
                    g -= g_rand
                else:
                    g += g_rand
                if b + b_rand > 255:
                    b -= b_rand
                else:
                    b += b_rand
                if len(hex(r)[2:]) < 2:
                    r_hex = '0' + hex(r)[2:]
                else:
                    r_hex = hex(r)[2:]
                if len(hex(g)[2:]) < 2:
                    g_hex = '0' + hex(g)[2:]
                else:
                    g_hex = hex(g)[2:]
                if len(hex(b)[2:]) < 2:
                    b_hex = '0' + hex(b)[2:]
                else:
                    b_hex = hex(b)[2:]

                x = objects[object]['dimensions'][0]
                y = objects[object]['dimensions'][1]
                z = objects[object]['dimensions'][2]
                # decimize
                xd = x * (10 ** len(str(x))) * size_treshold
                yd = y * (10 ** len(str(y))) * size_treshold
                zd = z * (10 ** len(str(z))) * size_treshold
                x_rand = random.randrange(int(xd)) / (10. ** len(str(x)))
                y_rand = random.randrange(int(yd)) / (10. ** len(str(y)))
                z_rand = random.randrange(int(zd)) / (10. ** len(str(z)))
                plus_or_min = random.randrange(2)
                if plus_or_min > 0:
                    new_x = x + x_rand
                else:
                    new_x = x - x_rand
                if plus_or_min > 0:
                    new_y = y + y_rand
                else:
                    new_y = y - y_rand
                if plus_or_min > 0:
                    new_z = z + z_rand
                else:
                    new_z = z - z_rand
                rand_obj = copy.copy(objects[object])
                rand_obj['color'] = r_hex + g_hex + b_hex
                rand_obj['dimensions'][0] = new_x
                rand_obj['dimensions'][1] = new_y
                rand_obj['dimensions'][2] = new_z
                randomized.append(rand_obj)
        return randomized

    def convert_euroc_object(self, eu_object):
        color = eu_object['color']
        r = int(color[:2], 16)
        g = int(color[2:4], 16)
        b = int(color[-2:], 16)
        x = eu_object['dimensions'][0]
        y = eu_object['dimensions'][1]
        z = eu_object['dimensions'][2]
        return [r, g, b, x, y, z]

    def convert_to_dataset(self, raw_data):
        data = []
        labels = []
        for key in raw_data:
            for object in raw_data[key]:
                data.append(self.convert_euroc_object(object))
            labels += [key] * len(raw_data[key])
        return data, labels

    def classify_object(self, object):
        class_dict = {'red_cube': 1, 'green_cylinder': 2, 'blue_handle': 0}
        r = object.color.r
        g = object.color.g
        b = object.color.b
        edges = object.primitives[0].dimensions[0]
        edges.sort()
        classifyable_object = [r, g, b] + edges
        class_name = self.clf.predict(classifyable_object)
        print dir("%s %s"%(object, class_name))
        object.c_type = class_dict[class_name[0]]
        return object