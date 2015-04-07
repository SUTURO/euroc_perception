#!/usr/bin/python
# -*- encoding: utf-8 -*-

import rospy
from suturo_perception_msgs.srv import Classifier
from suturo_perception_classifier.classifier_src import Classifier as CF
import argparse
def dump():
    pass

def main(task, logging):
    c = CF(task, logging)
    # print("TROLOLOOL<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
    rospy.init_node("Classifier", anonymous=True)
    # Create the service
    rospy.loginfo("Starting Classifier")
    rospy.Service("/suturo/perception/Classifier", Classifier, c.classify_object)
    # spin the wheel
    print("Started Classifier Service")
    rospy.spin()


if __name__ == "__main__":
    try:
        parser = argparse.ArgumentParser(prog='Classifier Service', usage='%(prog)s [options]')
        parser.add_argument('--task', default='task1_v1', help='The task for which %(prog) should classify. default will be task1_v1')
        parser.add_argument('--logging', default=1, type=int, help='Logging Level from 0 to 3. 3 Will Print everything 0 will do nothing. Default is 1')
        args, unknown = parser.parse_known_args()
        main(args.task, args.logging)
    except rospy.ROSInterruptException:
        pass

