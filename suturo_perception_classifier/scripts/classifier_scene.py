#!/usr/bin/python
# -*- encoding: utf-8 -*-

import rospy
from suturo_perception_msgs.srv import Classifier
from suturo_perception_classifier.scene_classifier import SceneClassifier as CF
import argparse
def dump():
    pass

def main(task, logging):
    c = CF(task, logging)
    rospy.init_node("SceneClassifier", anonymous=True)
    # Create the service
    rospy.loginfo("Starting Scene Classifier")
    rospy.Service("/suturo/perception/SceneClassifier", Classifier, c.classify_object)
    # spin the wheel
    print("Started Scene Classifier Service")
    rospy.spin()


if __name__ == "__main__":
    try:
        parser = argparse.ArgumentParser(prog='Scene Classifier Service', usage='%(prog)s [options]')
        parser.add_argument('--task', default='task1_v1', help='The task for which %(prog) should classify. default will be task1_v1')
        parser.add_argument('--logging', default=1, type=int, help='Logging Level from 0 to 3. 3 Will Print everything 0 will do nothing. Default is 1')
        args, unknown = parser.parse_known_args()
        main(args.task, args.logging)
    except rospy.ROSInterruptException:
        pass

