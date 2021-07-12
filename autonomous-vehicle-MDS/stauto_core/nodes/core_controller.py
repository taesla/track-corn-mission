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
from std_msgs.msg import UInt8, Int32
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import ByteMultiArray, Int32MultiArray, MultiArrayDimension
from geometry_msgs.msg import Point , PoseStamped

class CoreController():
    def __init__(self):
        #subscriber
        self.sub_traffic = rospy.Subscriber('/detect/traffic_sign', Int32MultiArray, self.cbTraffic, queue_size=1)
        self.sub_avoidance = rospy.Subscriber('/adaptive_clustering/is_bool', Bool, self.cbAvoidance, queue_size=1)
        self.sub_parking = rospy.Subscriber('/detect/parking_sign', Bool, self.cbParking, queue_size=1)
        self.sub_safetyzone = rospy.Subscriber('/detect/safety_sign', Bool, self.cbSafetyZone, queue_size=1)
        self.sub_crosswalk = rospy.Subscriber('/detect/crosswalk_sign', Bool, self.cdCrosswalk, queue_size=1)
        self.sub_stop = rospy.Subscriber('stop_line',Int32, self.cbStop,queue_size=1)                       #dynamic obstacle mission
        self.sub_cruise = rospy.Subscriber('/detect/cruise',Bool, self.cbcruise, queue_size=1)
        self.sub_backup = rospy.Subscriber('current_step', PoseStamped, self.cbBackup, queue_size=1)
        self.sub_parking_end = rospy.Subscriber('/parking_end',Bool,self.cbParkingEnd, queue_size=1)

        #publisher
        self.pub_state = rospy.Publisher('/state_machine',Int32MultiArray, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(0.5), self.timer_callback)

        self.Machine_State = Enum('Machine_State', 'cruise avoid_cruise stop traffic parking safety_zone crosswalk backup',start=0)
        self.TrafficSign = Enum('TrafficSign','red green left straightleft',start=0)
        #self.StopSign = Enum('StopSign','obstacle_stop traffic_stop parking_stop crosswalk_stop')

        self.StateGraph = Int32MultiArray()
        self.StateGraph.layout.dim.append(MultiArrayDimension())
        self.StateGraph.layout.dim[0].label = "state_graph"
        self.StateGraph.layout.dim[0].size = 7
        self.StateGraph.layout.dim[0].stride = 7
        self.StateGraph.layout.data_offset = 0
        self.StateGraph.data=[0]*7
        for i in range(7):
            if i == 1:
                self.StateGraph.data[i] = 1
            else:
                self.StateGraph.data[i] = 0

        self.cur_state = self.Machine_State.cruise.value
        self.backup_state = self.Machine_State.cruise.value
        self.cur_traffic = self.TrafficSign.red.value
        self.stop_flag = False
        self.stop_line = 0
        self.stop_line_count=0
        self.stop_line_timer=False
        self.step_num=0
        self.prev_step=0
        self.avoid_count = 0
        self.avoid_flag = False
        self.avoid_timer = False
        self.traffic_timer = False
        self.traffic_count = 0
        self.parking_end = False
        self.crosswalk_end = False
        self.crosswalk_timer = False
        self.crosswalk_count = 0

        self.dynamic_obstacle = False
        self.obstacle_timer = False
        self.obstacle_count=0

        loop_rate = rospy.Rate(10)

    def cbTraffic(self,event_msg):
        self.fnDecideMode(self.Machine_State.traffic.value,event_msg)
        if event_msg.data[self.TrafficSign.red.value] == 1:
            self.cur_traffic = self.TrafficSign.red.value
        elif event_msg.data[self.TrafficSign.green.value] == 1:
            self.cur_traffic = self.TrafficSign.green.value
        elif event_msg.data[self.TrafficSign.left.value] == 1:
            self.cur_traffic = self.TrafficSign.left.value
        elif event_msg.data[self.TrafficSign.straightleft.value] == 1:
            self.cur_traffic = self.TrafficSign.straightleft.value

    def cbStop(self,event_msg):
        #self.stop_line = event_msg.data
        if (event_msg.data == 1):
            self.stop_line_timer = True
            self.stop_line = 1
        if (self.stop_line_timer == True):
            self.stop_line = 1
        if (event_msg.data == 0) and (self.stop_line_timer == False):
            self.stop_line = 0

    # def cbAvoidance(self,event_msg):
    #     if event_msg.data == True:
    #         self.avoid_flag = 1
    #         self.fnDecideMode(self.Machine_State.avoid_cruise.value,self.avoid_count)
    #         #self.avoid_time = rospy.get_rostime()
    #     if event_msg.data == False:
    #         if self.avoid_flag == 1:
    #             self.avoid_count += 1
    #             if self.avoid_count >= 3:
    #                 self.avoid_count = 0
    #                 self.avoid_flag = 0
    #                 self.fnDecideMode(self.Machine_State.cruise.value,0)
    #         elif self.avoid_flag == 0:
    #             self.fnDecideMode(self.Machine_State.cruise.value,0)
    #         #if(rospy.get_rostime() - self.avoid_time >= 1.5):
    #         #    self.fnDecideMode(self.Machine_State.cruise.value,0)

    def cbAvoidance(self,event_msg):
        self.avoid_flag = event_msg.data
        if (event_msg.data == True):
            self.avoid_timer = True
            self.avoid_count = 0
        #     self.cur_state = self.Machine_State.avoid_cruise.value
        # if (self.avoid_timer == True):
        #     self.cur_state = self.Machine_State.avoid_cruise.value
        # elif (self.avoid_timer == False) :
        #     self.cur_state = self.Machine_State.cruise.value

    def cbParking(self,event_msg):
        if event_msg.data == True:
            self.fnDecideMode(self.Machine_State.parking.value,0)

    def cbParkingEnd(self,event_msg):
        self.parking_end = event_msg.data

    def cbcruise(self,event_msg):
        if event_msg.data == True:
            self.fnDecideMode(self.Machine_State.cruise.value,0)

    def cbSafetyZone(self,event_msg):
        if event_msg.data == True:
            self.fnDecideMode(self.Machine_State.safety_zone.value,0)

    def cdCrosswalk(self,event_msg):
        if event_msg.data == True:
            self.fnDecideMode(self.Machine_State.crosswalk.value,0)

    def cbBackup(self,event_msg):
        self.step_num = event_msg.pose.position.z
        print('step num : ' ,self.step_num)
        
        #if (1 <= self.step_num and self.step_num <=16):
        #    self.backup_state = self.Machine_State.avoid_cruise.value
        #    self.dynamic_obstacle = True                                #dynamic obstacle
        #    if(self.step_num == 16):
        #        self.dynamic_obstacle = False

        #elif (22 <= self.step_num and self.step_num <=30):
        #    self.backup_state = self.Machine_State.avoid_cruise.value

        if (15 <= self.step_num and self.step_num <=34):
            if(self.parking_end == False):
                self.backup_state = self.Machine_State.parking.value
            elif(self.parking_end == True):
                self.backup_state = self.Machine_State.cruise.value
        elif(55 <= self.step_num and self.step_num <=72):
           self.backup_state = self.Machine_State.avoid_cruise.value

        #elif(89 <= self.step_num and self.step_num<=92):
        #    self.backup_state = self.Machine_State.crosswalk.value
        #    if (self.step_num == 92):
        #        self.crosswalk_end=False
        elif(88 <= self.step_num and self.step_num <=107):
            self.backup_state = self.Machine_State.safety_zone.value
            #self.stop_flag=False
        elif(108 <= self.step_num and self.step_num <= 114):
            self.backup_state = self.Machine_State.traffic.value
            if (self.prev_step == 110 and self.step_num == 111):
                self.stop_line_timer = True
                self.stop_line = 1

        elif(120<= self.step_num and self.step_num <=124):
            self.backup_state = self.Machine_State.safety_zone.value
        elif(125 <= self.step_num and self.step_num <= 128):
            self.backup_state = self.Machine_State.traffic.value
            if (self.prev_step == 126 and self.step_num == 127):
                self.stop_line_timer = True
                self.stop_line = 1

        elif(129<= self.step_num and self.step_num <=144):
            self.backup_state = self.Machine_State.safety_zone.value

        elif(145<=self.step_num and self.step_num<=151): #155
            self.backup_state = self.Machine_State.avoid_cruise.value
            self.dynamic_obstacle = True                                #dynamic obstacle
            if(self.step_num == 151): #155
                self.dynamic_obstacle = False
        
        elif(193 <= self.step_num and self.step_num <= 202):
          self.backup_state = self.Machine_State.avoid_cruise.value

        elif(208 <= self.step_num and self.step_num <= 213):
            self.backup_state = self.Machine_State.traffic.value
            if (self.prev_step == 211 and self.step_num == 212):
                self.stop_line_timer = True
                self.stop_line = 1

        #elif(228 <= self.step_num and self.step_num <=231):
        #    self.backup_state = self.Machine_State.safety_zone.value
        #    self.crosswalk_count=0
        #    self.stop_flag=False

        # elif(229<=self.step_num and self.step_num<=231):
        #     self.backup_state=self.Machine_State.safety_zone.value
        # elif(self.step_num == 232 or self.step_num == 233):
        #    self.backup_state = self.Machine_State.crosswalk.value
        #    if (self.step_num == 233):
        #        self.crosswalk_end=False

        elif(243 <= self.step_num and self.step_num <= 253):
            self.backup_state = self.Machine_State.traffic.value
            if (self.prev_step == 250 and self.step_num == 251):
                self.stop_line_timer = True
                self.stop_line = 1
        elif(280 <= self.step_num and self.step_num <= 286):
            self.backup_state = self.Machine_State.traffic.value
            if (self.prev_step == 283 and self.step_num == 284):
                self.stop_line_timer = True
                self.stop_line = 1
        elif(400 <= self.step_num and self.step_num <= 408):
            self.backup_state = self.Machine_State.traffic.value
            if (self.prev_step == 406 and self.step_num == 407):
                self.stop_line_timer = True
                self.stop_line = 1
        elif(419 <= self.step_num and self.step_num <= 426):
            self.backup_state = self.Machine_State.traffic.value
            if (self.prev_step == 423 and self.step_num == 424):
                self.stop_line_timer = True
                self.stop_line = 1

        else:
            self.backup_state = self.Machine_State.cruise.value

        self.prev_step=self.step_num
        self.fnDecideMode(self.Machine_State.backup.value,self.backup_state)

    def timer_callback(self,data):
        if(self.traffic_timer == True):
            self.traffic_count = self.traffic_count + 1
            if (self.traffic_count > 200):
                print("traffic mode release!!!")
                #self.cur_traffic = 1
                self.traffic_count = 0
                self.traffic_timer = False
        if(self.crosswalk_timer == True):
            self.crosswalk_count = self.crosswalk_count + 1
            if(self.crosswalk_count > 8):
                print("crosswalk mode release!!!")
                #self.fnDecideMode(self.Machine_State.cruise.value,0)
                self.crosswalk_count = 0
                self.crosswalk_end = True
                self.crosswalk_timer = False
        if(self.stop_line_timer == True):
            self.stop_line_count = self.stop_line_count + 1
            if(self.stop_line_count > 1):
                self.stop_line_timer = False
                self.stop_line_count=0
        if(self.avoid_timer == True):
            self.avoid_count = self.avoid_count + 1
            if(self.avoid_count > 10):
                self.avoid_timer = False
                self.avoid_count=0
        if(self.dynamic_obstacle == True) and (self.avoid_flag == True):
            self.obstacle_count = self.obstacle_count + 1


    def fnDecideMode(self,mode,sub_event):
        for i in range(7):
            self.StateGraph.data[i] = 0

        if mode == self.Machine_State.cruise.value:
            self.cur_state = self.Machine_State.cruise.value

        elif mode == self.Machine_State.traffic.value:
            self.cur_state = self.Machine_State.traffic.value

        elif mode == self.Machine_State.avoid_cruise.value: #self.Machine_State.backup.value and sub_event==self.Machine_State.avoid_cruise.value:
            self.cur_state = self.Machine_State.avoid_cruise.value

        elif mode == self.Machine_State.parking.value:
            self.cur_state = self.Machine_State.parking.value

        elif mode == self.Machine_State.safety_zone.value:
            self.cur_state = self.Machine_State.safety_zone.value

        if(self.cur_state != self.backup_state):
            self.cur_state = self.backup_state

        if mode == self.Machine_State.backup.value and sub_event==self.Machine_State.parking.value:
            self.cur_state = self.Machine_State.parking.value

        if (self.stop_line == 1) and (self.cur_state == self.Machine_State.parking.value):
            self.cur_state = self.Machine_State.stop.value

        elif self.cur_state == self.Machine_State.crosswalk.value: #mode == self.Machine_State.backup.value and sub_event==self.Machine_State.crosswalk.value:
            print(self.stop_line, self.crosswalk_timer, self.crosswalk_count,self.crosswalk_end)

            self.cur_state = self.Machine_State.crosswalk.value

            if (self.stop_line == 1) and (self.crosswalk_timer == False) and (self.crosswalk_end == False):
                self.crosswalk_timer = True

            if self.crosswalk_timer==True:
                self.cur_state = self.Machine_State.stop.value

            elif (self.crosswalk_timer == False) and (self.crosswalk_end == True):
                self.cur_state = self.Machine_State.crosswalk.value

        #elif (self.stop_line == 1) and (self.cur_state == self.Machine_State.safety_zone.value):
        #   self.cur_state = self.Machine_State.stop.value

        if mode == self.Machine_State.backup.value and sub_event==self.Machine_State.avoid_cruise.value:
            print(self.avoid_timer,self.avoid_count, self.obstacle_count)
            if(self.dynamic_obstacle == True) and (self.avoid_flag == True):
                if (self.obstacle_count <= 30):
                    self.cur_state = self.Machine_State.stop.value
                else:
                    self.cur_state = self.Machine_State.safety_zone.value
            elif (self.dynamic_obstacle == True) and (self.avoid_flag == False):
                self.cur_state = self.Machine_State.safety_zone.value
            elif (self.dynamic_obstacle == False) and (self.avoid_timer == True):
                self.cur_state = self.Machine_State.avoid_cruise.value
            elif (self.dynamic_obstacle == False) and (self.avoid_timer == False):
                self.cur_state = self.Machine_State.cruise.value


        if self.cur_state == self.Machine_State.traffic.value or sub_event == self.Machine_State.traffic.value:
            print(self.stop_flag, self.cur_traffic, self.stop_line,self.traffic_count)

            if (self.cur_traffic == 0) and (self.stop_line == 1):
                self.stop_flag=True
                self.cur_state = self.Machine_State.stop.value
                self.traffic_timer=True
                self.traffic_count = 0
            elif (243 <= self.step_num and self.step_num <= 254) and (self.cur_traffic == 1) and (self.stop_line == 1):
                self.stop_flag=True
                self.cur_state = self.Machine_State.stop.value
                self.traffic_timer=True
                self.traffic_count = 0
            elif (self.traffic_timer == True) and (self.stop_flag == True):
                self.cur_state = self.Machine_State.stop.value

            if (self.cur_traffic!=0) or (self.traffic_count > 199):
                print("Traffic Release ", self.traffic_count)

                if(243 <= self.step_num and self.step_num <= 254):
                    if(self.cur_traffic ==2 or self.cur_traffic==3):
                        self.stop_flag=False
                        self.cur_state = self.Machine_State.cruise.value
                    elif(self.cur_traffic ==1) and (self.stop_line == 1):
                        self.cur_state = self.Machine_State.stop.value
                
                elif(125 <= self.step_num and self.step_num <= 128):
                    self.stop_flag=False
                    self.cur_state = self.Machine_State.safety_zone.value


                else:
                    self.stop_flag=False
                    self.cur_state = self.Machine_State.cruise.value

        self.StateGraph.data[self.cur_state] = 1
        self.fnPublishMode()

    def fnPublishMode(self):
        if self.StateGraph.data[self.Machine_State.cruise.value] == 1:
            print('Cruise_mode')
        elif self.StateGraph.data[self.Machine_State.avoid_cruise.value] == 1:
            print('Avoidance_Cruise_mode')
        elif self.StateGraph.data[self.Machine_State.stop.value] == 1:
            print('Stop_mode')
        elif self.StateGraph.data[self.Machine_State.traffic.value] == 1:
            print('Traffic_mode')
        elif self.StateGraph.data[self.Machine_State.parking.value] == 1:
            print('Parking_mode')
        elif self.StateGraph.data[self.Machine_State.safety_zone.value] == 1:
            print('Safety_Zone_mode')
        elif self.StateGraph.data[self.Machine_State.crosswalk.value] == 1:
            print('Crosswalk_mode')
        self.pub_state.publish(self.StateGraph)

    def main(self):
        rospy.spin()

if __name__ == "__main__":

    rospy.init_node('Core_Controller')
    node = CoreController()
    node.main()
