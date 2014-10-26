__author__ = 'smurpheus'

import random
import copy
import numpy as np
import rospy
import cv2

from visualization_msgs.msg import Marker
from Parser import Parser
from suturo_msgs.msg import Task
from suturo_perception_msgs.srv import ClassifierResponse
from suturo_perception_msgs.msg import EurocObject
from sklearn import tree
from sklearn.naive_bayes import GaussianNB


euroc_suturo_shape_mapping = {'3': '2', '1': '1', '2': '3'}


class Classifier(object):
    selectable_attributes = ['color_class', 'height', 'volume', 'cuboid', 'color_hsv', 'color_rgb']
    color_classes = ['0000ff', '00ff00', '00ffff', 'ff0000', 'ff00ff', 'ffff00', 'unknown']
    color_class_number_mapping = {'0000ff': 0, '00ff00': 1, '00ffff': 2, 'ff0000': 3, 'ff00ff': 4, 'ffff00': 5,
                                  'unknown': 6}
    selected_attributes = ['color_class', 'height']
    original_objects = None
    cubeized_objects = {}
    size_tresh_perc = 0.1
    color_tresh = 10
    number_of_data = 1000
    classifier_list = {}

    def __init__(self, task, logging):
        self.logging = logging
        if self.logging >= 1: print(">>>> Classifier will be initialized for task %s with log_lvl %s" % (task, logging))
        rospy.Subscriber("/suturo/yaml_pars0r", Task, self.set_yaml_infos)
        self.marker_publisher = rospy.Publisher("/suturo/obstacle_classifier_marker", Marker)
        self.marker_id = 0
        self.clf = tree.DecisionTreeClassifier()

    def set_yaml_infos(self, data):
        objects = data.objects
        self.original_objects = objects
        if self.logging >= 1:
            print(">>>> Receiving Objects from YAML")
        if self.logging >= 2:
            print("Objects Received from YAML: \r\n %s" % objects)
        pars = Parser()
        parsed = pars.convert_yaml_objects(objects, self.selected_attributes)

        randomized_objects = self.randomize_objects(parsed, self.number_of_data,
                                                    self.size_tresh_perc,
                                                    self.color_tresh)
        data, labels = pars.convert_to_dataset(randomized_objects)
        # print self.create_random_obstacles(number=1000, attributes=self.selected_attributes)
        rnd_data, rnd_labels = self.create_random_obstacles(number=1000,
                                                            attributes=self.selected_attributes)
        # @TODO relative to task
        for each in self.original_objects:
            self.classifier_list[each.name] = self.create_classifier_for_object(each, data + rnd_data,
                                                                                labels + rnd_labels)
        if self.logging >= 3:
            print("All Data + Labels \r\n %s  \r\n %s" % ((labels + rnd_labels), (data + rnd_data)))
        self.clf.fit(data + rnd_data, labels + rnd_labels)
        if self.logging >= 3:
            tree.export_graphviz(self.clf, out_file='tree.dot', feature_names=['h', 's', 'v', 'site'])
            # system('dot -Tpng tree.dot -o tree.png')
            # system('feh tree.png &')

    def publish_marker_for_object(self, object, txt):
        marker = Marker()
        # marker.header.frame_id = "sdepth_pcl"
        marker.header.frame_id = object.object.header.frame_id
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

    def create_classifier_for_object(self, cobject, data, labels):
        new_labels = []
        for each in labels:
            if each != cobject.name:
                new_labels.append('unknown')
            else:
                new_labels.append(each)
        # cl = tree.DecisionTreeClassifier()
        cl = GaussianNB()
        cl.fit(data, new_labels)
        return cl

    def randomize_objects(self, objects, number, size_treshold, color_treshold):
        randomized = []
        for cobject in objects:
            for i in range(0, number):
                r_obj = copy.copy(cobject)
                if 'cuboid' in cobject:
                    x = cobject['cuboid'][0]
                    y = cobject['cuboid'][1]
                    z = cobject['cuboid'][2]
                    # decimize
                    xd = random.uniform(0.0, size_treshold)
                    yd = random.uniform(0.0, size_treshold)
                    zd = random.uniform(0.0, size_treshold)
                    x_rand = x * xd
                    y_rand = y * yd
                    z_rand = z * zd
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

                if 'height' in cobject:
                    old_height = cobject['height']
                    height__tresh = old_height * size_treshold
                    r_obj['height'] = random.uniform(old_height - height__tresh, old_height + height__tresh)
                if 'volume' in cobject:
                    old_vol = cobject['volume']
                    vol_tresh = old_vol * size_treshold
                    r_obj['volume'] = random.uniform(old_vol * vol_tresh, old_vol + vol_tresh)
                if 'cuboid' in cobject:
                    r_obj['cuboid'] = [new_x, new_y, new_z]

                if 'color_hsv'  in cobject or 'color_rgb' in cobject:
                    r = cobject['color_rgb'][0]
                    g = cobject['color_rgb'][1]
                    b = cobject['color_rgb'][2]
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
                if 'color_hsv' in cobject:
                    hsv_color = cv2.cvtColor(np.uint8([[[b, g, r]]]), cv2.COLOR_BGR2HSV)
                    r_obj['color_hsv'] = [hsv_color[0][0][0], hsv_color[0][0][1], hsv_color[0][0][2]]
                if 'color_rgb' in cobject:
                    r_obj['color_rgb'] = [r, g, b]
                randomized.append(r_obj)
        return randomized

    def classify_object(self, cobject):
        # load object
        unclassified_object = cobject.unclassifiedObject
        if self.logging >= 2:
            print("Got Object to Classify: \r\n %s" % unclassified_object)
        height = unclassified_object.c_height
        h = unclassified_object.c_avg_col_h
        s = unclassified_object.c_avg_col_s
        v = unclassified_object.c_avg_col_v
        # Convert c++ hsv room to python one (take care only to 180 to fit in 8 bit)
        h = int(h / 360. * 180)
        s = int(s * 255)
        v = int(v * 255)
        hsv_color = np.uint8([[[h, s, v]]])
        bgr_color = cv2.cvtColor(hsv_color, cv2.COLOR_HSV2BGR)
        r = bgr_color[0][0][2]
        g = bgr_color[0][0][1]
        b = bgr_color[0][0][0]
        color_class = self.color_class_number_mapping[unclassified_object.c_color_class]
        volume = unclassified_object.c_volume
        # build classifyable object and classify it
        classifyable_unclassified_object = []
        if 'color_class' in self.selected_attributes:
            classifyable_unclassified_object.append(color_class)
        if 'height' in self.selected_attributes:
            classifyable_unclassified_object.append(height)
        if 'volume' in self.selected_attributes:
            classifyable_unclassified_object.append(volume)
        if 'cuboid' in self.selected_attributes:
            classifyable_unclassified_object += unclassified_object.object.primitives[0].dimensions
        if 'color_hsv' in self.selected_attributes:
            classifyable_unclassified_object += [h, s, v]
        if 'color_rgb' in self.selected_attributes:
            classifyable_unclassified_object += [r, g, b]

        if self.logging >= 1:
            print("Object to Classify: \r\n %s" % classifyable_unclassified_object)
        class_name = 'unknown'
        c_type = EurocObject.OBSTACLE
        for each in self.classifier_list:
            classn = self.classifier_list[each].predict(classifyable_unclassified_object)[0]
            if classn != 'unknown':
                class_name = classn
                c_type = EurocObject.OBJECT

        # print class_name
        # unclassified_object.c_shape = self.get_shape_of_class(class_name[0])
        unclassified_object.object.id = class_name
        unclassified_object.c_type = c_type
        resp = ClassifierResponse()
        resp.classifiedObject = unclassified_object
        if self.logging >= 1:
            print("Classified Object as: %s" % class_name)
        self.publish_marker_for_object(unclassified_object, unclassified_object.object.id + "\nheight: " + str(height))
        return resp

    def create_random_obstacle(self, max_height=2, attributes=[]):
        rnd_obj = []
        if 'color_class' in attributes:
            rnd_obj.append(
                self.color_class_number_mapping[self.color_classes[random.randrange(len(self.color_classes))]])
        if 'height' in attributes:
            rnd_obj.append(random.randrange(0, int(max_height) * 1000) / 1000.)
        if 'volume' in attributes:
            rnd_obj.append(random.uniform(0.01, 0.5))  # TODO BAAAD hard coded.... change it!
        if 'cuboid' in attributes:
            pass  # todo dunno how to rand this
        if 'color_hsv' in attributes:
            h = random.randrange(0, 180)
            s = random.randrange(230, 255)
            v = random.randrange(230, 255)
            rnd_obj.append([h, s, v])
        if 'color_rgb' in attributes:
            r = random.randrange(0, 255)
            g = random.randrange(0, 255)
            b = random.randrange(0, 255)  # todo colors needs to be depended....
            rnd_obj.append([r, g, b])
        return rnd_obj

    def create_random_obstacles(self, max_height=2, number=100, attributes=[]):
        obstacles = []
        for i in range(0, number):
            obstacles.append(self.create_random_obstacle(max_height=max_height, attributes=attributes))
        labels = ['obstacle'] * number
        return obstacles, labels
