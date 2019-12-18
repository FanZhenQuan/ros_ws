#!/usr/bin/env python

import rospy

from toponodes_publisher import INTEREST_POINTS
from topolocaliser import AFFERENCE_TOPIC
from toponavigator import *
from topoplanner import *
from learning_toponav.msg import *


class DecisionMaker(object):
    def __init__(self):
        pass


if __name__ == '__main__':
    rospy.init_node('decision_maker')