__author__ = 'smurpheus'

import random
import copy
import numpy as np

import rospy
import cv2
from os import system
from suturo_msgs.msg import Task
from suturo_perception_msgs.srv import ClassifierResponse
from sklearn import tree


# TODO: Avoid double classification

class ObstacleClassifier(object):
    original_objects = None
    cubeized_objects = {}
    # The maximum tolerance (-/+) in meters for the dimensions of the objects
    height_tolerance = 0.005

    def __init__(self, task, logging):
        self.logging = logging
        if self.logging >= 1: print(">>>> Classifier will be initialized for task %s" %task)
        rospy.Subscriber("/suturo/yaml_pars0r", Task, self.set_yaml_infos)
        # self.clf = tree.DecisionTreeClassifier()


    def set_yaml_infos(self, data):
        objects = data.objects
        self.original_objects = objects
        all_data = {}
        if self.logging >= 1: print(">>>> Receiving Objects from YAML")
        if self.logging >= 2: print("Objects Received from YAML: \r\n %s"%objects)
        for object in objects:
            name = object.name
            color = object.color
            primitive = object.primitives
            self.cubeized_objects[name] = {'color': color, 'dimensions': self.get_surrounding_cuboid(object)}

    def get_surrounding_cuboid(self, object):
        if len(object.primitives) == 1:
            primitive = object.primitives[0]
            print(primitive.dimensions)
            dimensions = list(primitive.dimensions)
            # dimensions.sort()
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
                # dimensions.sort()
                if primitive.type == 3:
                    z += dimensions[0]
                    if x < 2 * dimensions[1]: x = 2 * dimensions[1]
                    if y < 2 * dimensions[1]: y = 2 * dimensions[1]
                if primitive.type == 1:
                    z += dimensions[2]
                    if x < 2 * dimensions[0]: x = dimensions[0]
                    if y < 2 * dimensions[1]: y = dimensions[1]
            return [x, y, z]

    def check_dimension_tolerance(self,obj,height):
      """Check if atleast one of obj's dimensions matches the given height of the object cluster from the perception call"""
      print("Checking dimensions for the object called {0}".format(obj.name))
      obj_cube_hash = self.cubeized_objects[obj.name]
      print obj_cube_hash
      [x,y,z] = obj_cube_hash["dimensions"]
      return ( ((x-self.height_tolerance) <= height <= x+self.height_tolerance) or ((y-self.height_tolerance) <= height <= y+self.height_tolerance) or ((z-self.height_tolerance) <= height <= z+self.height_tolerance) )

    def classify_object(self, object):
        # class_dict = {'red_cube': 1, 'green_cylinder': 2, 'blue_handle': 0, 'obstacle': 0}
        # load object
        unclassified_object = object.unclassifiedObject
        if self.logging >= 2 : print("Got Object to Classify: \r\n %s" %unclassified_object)
        height = unclassified_object.c_height
        # h = unclassified_object.c_avg_col_h
        # s = unclassified_object.c_avg_col_s
        # v = unclassified_object.c_avg_col_v
        color_class = unclassified_object.c_color_class

        # Classifier steps:
        # 1) Consider the color
        # 2) Consider the height of the object. This should be close to one of the dimensions of one object
        # 3) Consider the volume (this may be optional if we already get good results)

        # 1)
        object_candidates = self.original_objects
        for obj in object_candidates:
          print obj.color 
          print "vs."
          print color_class
          print obj.color == color_class

        object_candidates = [obj for obj in object_candidates if obj.color == color_class]
        # if self.logging >= 1 : print("Remaining object candidates after color filtering {0}:".format(len(object_candidates) ) )
        if self.logging >= 1 : print("Remaining object candidates after color filtering:" )
        if self.logging >= 1 :
          for c in object_candidates:
            print c.name

        # 2)
        object_candidates = [obj for obj in object_candidates if self.check_dimension_tolerance(obj, height) ]
        if self.logging >= 1 : print("Remaining object candidates after height filtering:")
        if self.logging >= 1 :
          for c in object_candidates:
            print c.name

        # 3)
        # Not implemented yet

        # Evaluate the remaining object_candidates
        if len(object_candidates) > 1:
          # We couldn't filter the data to get only one object. Set the class name to ambiguous 
          unclassified_object.object.id = "ambiguous"
          unclassified_object.c_type = 255 # Classified as UNKNOWN(255)
        elif len(object_candidates) == 1:
          unclassified_object.object.id = object_candidates[0].name
          unclassified_object.c_type = 1 # Classified as object, otherwise 0
        else:
          unclassified_object.object.id = "unknown"
          unclassified_object.c_type = 255 # Classified as UNKNOWN(255)

        # unclassified_object.c_shape = class_dict[class_name[0]] # TODO is this neccessary?
        # unclassified_object.object.id = class_name
        resp = ClassifierResponse()
        resp.classifiedObject = unclassified_object
        if self.logging >= 1: print("Classified Object as: %s"%unclassified_object.object.id)
        return resp

