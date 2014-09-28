#!/usr/bin/python
# -*- encoding: utf-8 -*-

from __future__ import division
import numpy as np
import rospy
import cv2
import math
from threading import Lock
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from suturo_perception_msgs.srv import RecognizeOoI, RecognizeOoIResponse
from suturo_perception_msgs.msg import ObjectOfInterest
from geometry_msgs.msg import PoseStamped, Pose, Point
from std_msgs.msg import Header, ColorRGBA


def rgb_to_hsv(color):
    c = color
    if isinstance(c, ColorRGBA):
        c = cv2.cv.CreateMat(1, 1, cv2.CV_8UC3)
        c[0,0] = (color.r, color.g, color.b)
    res = cv2.cv.CreateMat(1, 1, cv2.CV_8UC3)
    cv2.cv.CvtColor(c, res, cv2.COLOR_RGB2HSV)
    return res[0,0]


def depth_project(x, y, depth_image):
    rows, cols, _ = depth_image.shape
    rospy.logdebug("Projecting (%d / %d) with Image size %dx%d" % (x, y, cols, rows))
    fov_h = 1.047 #TODO: get this from the yaml description
    fov_v = 2.0 * math.atan(math.tan(fov_h / 2.0) * (rows / cols))
    h = math.tan(fov_h / 2.0)
    v = math.tan(fov_v / 2.0)
    depth = depth_image[y,x]
    res_x = depth
    res_y = depth * (h - 2.0 * h * x / cols)
    res_z = depth * (v - 2.0 * v * y / rows)
    return Point(x=res_x, y=res_y, z=res_z)


class ColorDetector(object):
    __area_threshold = 1000

    def __init__(self):
        # Subscribe to the topics
        self.__rgb_subscriber = rospy.Subscriber("/euroc_interface_node/cameras/tcp_rgb_cam", Image,
                                                 self.__callback_bgr, queue_size=1)
        self.__depth_subscriber = rospy.Subscriber("/euroc_interface_node/cameras/tcp_depth_cam", Image,
                                                   self.__callback_depth, queue_size=1)
        self.__bridge = CvBridge()
        self.__lock = Lock()
        self.__hsv_image = None
        self.__depth_image = None
        self.__seq = 0

    def __callback_bgr(self, ros_data):
        # Convert the msg to CV2-Image
        bgr_image = self.__bridge.imgmsg_to_cv2(ros_data, "bgr8")
        # Convert to HSV
        with self.__lock:
            self.__hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

    def __callback_depth(self, ros_data):
        with self.__lock:
            self.__depth_image = self.__bridge.imgmsg_to_cv2(ros_data)

    def get_contour(self, image):
        # Find contours in the image
        contour_image = image.copy()
        contours, _ = cv2.findContours(contour_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        # Fin the contour with max area inside
        max_area = 0
        best_cnt = None
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > max_area and area >= self.__area_threshold:
                max_area = area
                best_cnt = cnt
        return best_cnt

    def get_centroid(self, image):
        moments = cv2.moments(image)
        m01 = moments["m01"]
        m10 = moments["m10"]
        area = moments["m00"]
        if area >= self.__area_threshold:
            pos_x = int(m10 / area)
            pos_y = int(m01 / area)
            return (pos_x, pos_y)
        return (None, None)

    def detect(self, request):
        with self.__lock:
            objects = []
            if self.__hsv_image is not None and self.__depth_image is not None:
                for color in request.colors:
                    rospy.logdebug("Processing color %s" % color)
                    # define low and high color borders
                    hsv_color = rgb_to_hsv(color)
                    low = np.uint8((hsv_color[0], 250, 0))
                    high = np.uint8((hsv_color[0], 255, 255))

                    # Threshold the image
                    img_thresh = cv2.inRange(self.__hsv_image, low, high)
                    #Get best contours
                    contour = self.get_contour(img_thresh)

                    # Find the centroid of the contour
                    if contour is not None:
                        x, y = self.get_centroid(contour)
                        if x >= 0 and y >= 0:
                            rospy.logdebug("Object at (%d / %d)" % (x, y))
                            # Found an object check the depth image
                            position = depth_project(x, y, self.__depth_image)
                            # Append the found result
                            rospy.logdebug("Found Object at %s" % position)
                            objectOfInterest = ObjectOfInterest(
                                color=color,
                                pose=PoseStamped(
                                    header=Header(seq=self.__seq, stamp=rospy.get_rostime(), frame_id="tdepth"),
                                    pose=Pose(position=position)
                                )
                            )
                            objects.append(objectOfInterest)
                            self.__seq += 1
            return RecognizeOoIResponse(objects)

    def __del__(self):
        self.__rgb_subscriber.unregister()
        self.__depth_subscriber.unregister()


def main():
    rospy.init_node("ColorDetector", anonymous=True)
    colorDetector = ColorDetector()
    # Create the service
    rospy.loginfo("Starting ColorDetector")
    rospy.Service("/suturo/RecognizeOoI", RecognizeOoI, colorDetector.detect)
    # spin the wheel
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

