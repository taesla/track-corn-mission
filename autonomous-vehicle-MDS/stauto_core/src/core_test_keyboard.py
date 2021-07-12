#!/usr/bin/env python
# -*- coding: utf-8 -*-

#This is a stauto core_test_keyboard.py
#Copyright (c) 2020, choiyungsik, jeonjonghyun

import sys, select, termios, tty, math
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from std_msgs.msg import ByteMultiArray, Int32MultiArray
from std_msgs.msg import MultiArrayDimension
from enum import Enum

header_msgs = """
Make a interruption by typing!!
--------------------------------
c : cruise
o : obstacle detected
p : parking_sign detected
t : traffic_light detected
f : safety_zone detected
w : crosswalk detected
r : red_light 
g : green_light
a : straightleft
l : left
s : stop

CTRL_C to quit
"""

def GetKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def fnPublish(key):
    global pub_avoidance,pub_stop,pub_traffic,pub_parking,pub_safety,TrafficArray,pub_cruise,pub_crosswalk

    parking = Bool()
    obstacle = Bool()
    safety = Bool()
    cruise = Bool()
    crosswalk = Bool()

    if key == 'p':
        parking.data = True
        pub_parking.publish(parking)
    elif key == 'c':
        cruise.data = True
        pub_cruise.publish(cruise)
    elif key == 'o':
        obstacle.data = True
        pub_avoidance.publish(obstacle)
    elif key == 'f':
        safety.data = True
        pub_safety.publish(safety)
    elif key == 'w':
        crosswalk.data = True
        pub_crosswalk.publish(crosswalk)
    elif key == 'r':
        TrafficArray.data[0] = 1
        pub_traffic.publish(TrafficArray)
    elif key == 'g':
        TrafficArray.data[1] = 1
        pub_traffic.publish(TrafficArray)
    elif key == 'l':
        TrafficArray.data[2] = 1
        pub_traffic.publish(TrafficArray)
    elif key == 'a':
        TrafficArray.data[3] = 1
        pub_traffic.publish(TrafficArray)
    elif key == 's':
        pub_stop.publish(1)
    elif key == 'd':
        pub_stop.publish(0) 
        #print("stop!!")

    #pub_traffic.publish(TrafficArray)
    for j in range(4):
        TrafficArray.data[j]= 0
    
    parking.data = False
    stop = 0
    obstacle.data = False
    safety.data = False
    cruise.data = False
    crosswalk.data = False


if __name__ == "__main__":
    global Machine_State, TrafficArray,pub_avoidance,pub_stop,pub_traffic,pub_parking,pub_safety,pub_crosswalk
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('stauto_core_test_keyboard')

    Machine_State = Enum('Machine_state', 'cruise avoid_cruise stop traffic parking safety_zone crosswalk')
    state_event = Machine_State.cruise.value
    array = [0,0,0,0]
    TrafficArray = Int32MultiArray()
    TrafficArray.layout.dim.append(MultiArrayDimension())
    TrafficArray.layout.dim[0].label = "traffic_mode"
    TrafficArray.layout.dim[0].size = 4
    TrafficArray.layout.dim[0].stride = 4
    TrafficArray.layout.data_offset = 0
    TrafficArray.data=[0]*4

    pub_avoidance = rospy.Publisher('/detect/obstacle',Bool,queue_size=1)
    pub_stop = rospy.Publisher('stop_line',Int32,queue_size=1)
    pub_parking = rospy.Publisher('/detect/parking_sign',Bool,queue_size=1)
    pub_safety = rospy.Publisher('/detect/safety_sign',Bool,queue_size=1)
    pub_crosswalk = rospy.Publisher('/detect/crosswalk_sign',Bool,queue_size=1)
    pub_traffic = rospy.Publisher('/detect/traffic_sign',Int32MultiArray,queue_size=1)
    pub_cruise = rospy.Publisher('/detect/cruise',Bool,queue_size=1)

    loop_rate = rospy.Rate(10) # 10hz

    try:
        #print(header_msgs)
        while(1):
            key = GetKey()
            fnPublish(key)
            
            if key == '\x03': #CTRL + C
                break

    except Exception as e:
        print(e)
