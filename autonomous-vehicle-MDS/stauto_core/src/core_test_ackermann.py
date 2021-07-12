#!/usr/bin/env python
# -*- coding: utf-8 -*-

# This is a stauto core_controller.py
# Copyright (c) 2020, choiyungsik

import rospy, roslaunch
import numpy as np
import subprocess
import os
import sys
import rospkg
import math
from enum import Enum
from std_msgs.msg import UInt8
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point

if __name__ == "__main__":
    
    rospy.init_node('ackerman_test')
    pub_ackermann = rospy.Publisher('/ackermann_cmd_state',AckermannDriveStamped,queue_size= 1)
    ackerman = AckermannDriveStamped()
    ackerman.drive.speed = 15
    ackerman.drive.steering_angle = 15
    
    while not rospy.is_shutdown():
        pub_ackermann.publish(ackerman)
        #rospy.spin()
