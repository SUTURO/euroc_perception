__author__ = 'smurpheus'

import numpy as np
import cv2
import math


class Parser(object):
    color_classes = ['0000ff', '00ff00', '00ffff', 'ff0000', 'ff00ff', 'ffff00']
    selectable_attributes = ['color_class', 'height', 'volume', 'cuboid', 'color_hsv', 'color_rgb']
    color_class_number_mapping = {'0000ff':0, '00ff00':1, '00ffff':2, 'ff0000':3, 'ff00ff':4, 'ffff00':5}
    object_dict = {}


    def convert_yaml_object(self, yaml_object, attributes=[]):
        result = {}
        result['name'] = yaml_object.name
        if 'color_class' in attributes:
            result['color_class'] = self.color_class_number_mapping[yaml_object.color]

        if 'volume' in attributes:
            result['volume'] = self.get_volume(yaml_object)

        cuboid = self.get_surrounding_cuboid(yaml_object)
        if 'height' in attributes:
            print "height %s of %s" % (cuboid, yaml_object.name)
            result['height'] = cuboid[2]

        if 'cuboid' in attributes:
            result['cuboid'] = cuboid

        color = yaml_object.color
        r = int(color[:2], 16)
        g = int(color[2:4], 16)
        b = int(color[-2:], 16)

        if 'color_rgb' in attributes:
            result['color_rgb'] = [r, g, b]

        if 'color_hsv' in attributes:
            hsv_color = cv2.cvtColor(np.uint8([[[b, g, r]]]), cv2.COLOR_BGR2HSV)
            result['color_hsv'] = [hsv_color[0][0][0], hsv_color[0][0][1], hsv_color[0][0][2]]

        return result


    def convert_yaml_objects(self, objects, attributes=[]):
        result = []
        for each in objects:
            result.append(self.convert_yaml_object(each, attributes=attributes))
        self.object_dict = result
        return result

    def convert_to_dataset(self, raw_data):
        data \
            = []
        labels = []
        for object in raw_data:
            so = []
            for attribute in self.selectable_attributes:
                if object.has_key(attribute):
                    if isinstance(object[attribute], list):
                        for each in object[attribute]:
                            so.append(each)
                    else:
                        so.append(object[attribute])
            data.append(so)
            labels.append(object['name'])
        return data, labels


    def get_shape_of_class(self, objectclass):
        for object in self.original_objects:
            if object.name == objectclass:
                break
        if len(object.primitives) > 1:
            return 0
        else:
            return object.primitives[0].type


    def get_volume(self, object):
        volume = 0
        for primitive in object.primitives:
            if primitive.type == 3:
                z = primitive.dimensions[0]
                bottom = math.pi * primitive.dimensions[1] ** 2
                volume += z * bottom
            # Type 1 is a cube
            if primitive.type == 1:
                volume += primitive.dimensions[0] * primitive.dimensions[1] * primitive.dimensions[2]
        return volume

    def get_surrounding_cuboid(self, object):

        # Test if the given object is a primitive
        if len(object.primitives) == 1:
            primitive = object.primitives[0]
            dimensions = list(primitive.dimensions)
            #Type 3 is a cylinder
            if primitive.type == 3:
                z = dimensions[0]
                x = 2 * dimensions[1]
                y = x
                return [x, y, z]
            #Type 1 is a cube
            if primitive.type == 1:
                return [dimensions[0], dimensions[1], dimensions[2]]
            return [0, 0, 0]  #

        #Test wether the object is a composed onee
        if len(object.primitives) > 1:
            z, y, x = 0, 0, 0
            for primitive in object.primitives:
                dimensions = list(primitive.dimensions)
                if primitive.type == 3:
                    z += dimensions[0]
                    if x < 2 * dimensions[1]: x = 2 * dimensions[1]
                    if y < 2 * dimensions[1]: y = 2 * dimensions[1]
                if primitive.type == 1:
                    z += dimensions[2]
                    if x < 2 * dimensions[0]: x = dimensions[0]
                    if y < 2 * dimensions[1]: y = dimensions[1]
            return [x, z, y]