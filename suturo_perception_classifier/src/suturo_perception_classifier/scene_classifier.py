__author__ = 'ofenrohr'

import random
import copy
import numpy as np

import rospy
import roslib
import cv2
from os import system
from suturo_msgs.msg import Task
from suturo_perception_msgs.srv import ClassifierResponse
from sklearn import tree
from visualization_msgs.msg import Marker
from std_msgs.msg import Header


euroc_suturo_shape_mapping = {'3': '2', '1': '1'}


class SceneClassifier(object):
    original_objects = None
    cubeized_objects = {}
    size_tresh_perc = 0.01
    color_tresh = 10
    number_of_data = 10

    def __init__(self, task, logging):
        self.logging = logging
        if self.logging >= 1: print(">>>> Classifier will be initialized for task %s" %task)
        rospy.Subscriber("/suturo/yaml_pars0r", Task, self.set_yaml_infos)
        self.marker_publisher = rospy.Publisher("/suturo/classifier_marker", Marker)
        self.clf = tree.DecisionTreeClassifier()
        self.marker_id = 0
        self.max_height = 0


    def set_yaml_infos(self, data):
        if self.logging >= 3: print("data: %s"%data)
        if self.logging >= 2: print("data.task_name: %s"%data.task_name)
        objects = data.objects
        task_name = data.task_name
        self.original_objects = objects
        all_data = {}
        if self.logging >= 1: print(">>>> Receiving Objects from YAML")
        if self.logging >= 2: print("Objects Received from YAML: \r\n %s"%objects)
        for object in objects:
            name = object.name
            color = object.color
            primitive = object.primitives
            cub_res = self.get_surrounding_cuboid(object)
            self.cubeized_objects[name] = {'color': color, 'dimensions': cub_res}
            randomized_objects = self.randomize_objects(self.cubeized_objects, self.number_of_data,
                                                        self.size_tresh_perc,
                                                        self.color_tresh)
            all_data[name] = randomized_objects
            self.max_height = max(self.max_height, max(cub_res))
        data, labels = self.convert_to_dataset(all_data)
        if "task 3" in task_name:
          rnd_data, rnd_labels = self.create_random_obstacles(number=1000)
        else:
          rnd_data, rnd_labels = [],[]
        if self.logging >= 3: print("All Data + Labels \r\n %s  \r\n %s"
                                    %( (labels + rnd_labels), (data+rnd_data)))
        self.clf.fit(data+rnd_data, labels+rnd_labels)
        if self.logging >=3:
            tree.export_graphviz(self.clf, out_file='tree.dot', feature_names=['h', 's', 'v', 'site'])
            print("max height: %s\n"%self.max_height)
        # system('dot -Tpng tree.dot -o tree.png')
        # system('feh tree.png &')

    def get_surrounding_cuboid(self, object):
        if len(object.primitives) == 1:
            primitive = object.primitives[0]
            print(primitive.dimensions)
            dimensions = list(primitive.dimensions)
            dimensions.sort()
            if primitive.type == 3:
                z = dimensions[0]
                x = 2 * dimensions[1]
                y = x
                return [x, y, z]
            if primitive.type == 1:
                return [dimensions[0], dimensions[1], dimensions[2]]
            return [0, 0, 0]  #
        if len(object.primitives) > 1:
            z, y, x = 0, 0, 0
            for primitive in object.primitives:
                dimensions = list(primitive.dimensions)
                dimensions.sort()
                if primitive.type == 3:
                    z += dimensions[0]
                    if x < 2 * dimensions[1]: x = 2 * dimensions[1]
                    if y < 2 * dimensions[1]: y = 2 * dimensions[1]
                if primitive.type == 1:
                    z += dimensions[2]
                    if x < 2 * dimensions[0]: x = dimensions[0]
                    if y < 2 * dimensions[1]: y = dimensions[1]
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
        hsv_color = cv2.cvtColor(np.uint8([[[b, g, r]]]), cv2.COLOR_BGR2HSV)
        x = eu_object['dimensions'][0]
        y = eu_object['dimensions'][1]
        z = eu_object['dimensions'][2]
        return [[hsv_color[0][0][0], hsv_color[0][0][1], hsv_color[0][0][2], x],
                [hsv_color[0][0][0], hsv_color[0][0][1], hsv_color[0][0][2], y],
                [hsv_color[0][0][0], hsv_color[0][0][1], hsv_color[0][0][2], z]]

    def convert_to_dataset(self, raw_data):
        data = []
        labels = []
        for key in raw_data:
            for object in raw_data[key]:
                data += self.convert_euroc_object(object)
            labels += [key] * len(raw_data[key]) * 3
        return data, labels

    def publish_marker_for_object(self, object, txt):
        marker = Marker()
        marker.header.frame_id = "sdepth_pcl"
        marker.header.stamp = rospy.Time(0)
        marker.ns = "scene_classifier_marker"
        self.marker_id = self.marker_id + 1
        marker.id = self.marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = object.c_centroid.x
        marker.pose.position.y = object.c_centroid.y
        marker.pose.position.z = object.c_centroid.z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.text = txt
        marker.lifetime = rospy.Duration.from_sec(10)
        self.marker_publisher.publish(marker)

    def classify_object(self, object):
        class_dict = {'red_cube': 1, 'green_cylinder': 2, 'blue_handle': 0, 'obstacle': 0}
        # load object
        unclassified_object = object.unclassifiedObject
        if self.logging >= 2:
            print("Got Object to Classify: \r\n %s" %unclassified_object)
        height = unclassified_object.c_height
        volume = unclassified_object.c_volume
        h = unclassified_object.c_avg_col_h
        s = unclassified_object.c_avg_col_s
        v = unclassified_object.c_avg_col_v
        #Convert c++ hsv room to python one (take care only to 180 to fit in 8 bit)
        h = int(h / 360. * 180)
        s = int(s * 255)
        v = int(v * 255)
        hsv_color = np.uint8([[[h, s, v]]])
        # bgr_color = cv2.cvtColor(hsv_color, cv2.COLOR_HSV2BGR)
        # r = bgr_color[0][0][2]
        # g = bgr_color[0][0][1]
        # b = bgr_color[0][0][0]
        #sort edges
        if unclassified_object.c_cuboid_success:
            edges = list(unclassified_object.object.primitives[0].dimensions)
            edges.sort()
        #build classifyable object and classify it
        classifyable_unclassified_object = [h, s, v, height]# + edges
        if self.logging >= 1:
            print("Object to Classify: \r\n %s"%classifyable_unclassified_object)
        class_name = self.clf.predict(classifyable_unclassified_object)
        # print class_name
        # class_name = self.lolloosed(r,g,b)
        if (volume > 0.0001) or (height > (self.max_height + self.max_height*0.05)):
            class_name[0] = 'obstacle'
        unclassified_object.c_shape = class_dict[class_name[0]]
        unclassified_object.object.id = class_name[0]
        resp = ClassifierResponse()
        resp.classifiedObject = unclassified_object
        if self.logging >= 1:
            print("Classified Object as: %s"%class_name)
        self.publish_marker_for_object(unclassified_object, class_name[0] + "\nheight: " + str(height))
        return resp

    def create_random_obstacle(self, max_height=2):
        height = random.randrange(0, int(max_height) * 1000) / 1000
        h = random.randrange(0, 180)
        s = random.randrange(230, 255)
        v = random.randrange(230, 255)
        volume = random.randrange(0, 1000) / 1000 + 0.0001
        return [h, s, v, height]
    
    def create_random_target_zone(self):
        height = random.randrange(0, 2000) / 1000000 + 0.001; # 1 - 3 mm
        h = random.randrange(0, 180)
        s = random.randrange(230, 255)
        v = random.randrange(230, 255)
        volume = random.randrange(0, 1000) / 10000000
        return [h, s, v, height]
        

    def create_random_obstacles(self, max_height=2, number=100):
        obstacles = []
        for i in range(0,number):
            obstacles.append(self.create_random_obstacle(max_height=max_height))
        labels = ['obstacle'] * (number * 2)
        for i in range(0,number):
            obstacles.append(self.create_random_target_zone())
        return obstacles, labels

    def lolloosed(self, r, g, b):
        colors = [r, g, b]
        ind = colors.index(max(colors))
        if ind == 0:
            return ['red_cube']
        if ind == 1:
            return ['green_cylinder']
        if ind == 2:
            return ['blue_handle']
        return ['unkown']