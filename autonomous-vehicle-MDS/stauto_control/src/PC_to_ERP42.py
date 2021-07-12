#! /usr/bin/env python

import rospy
from std_msgs.msg import Int32 
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from math import *
import numpy as np

import serial
import math
import time
import serial

S = chr(0x53)
T = chr(0x54)
X = chr(0x58)
AorM = chr(0x01)
ESTOP = chr(0x00)
GEAR = chr(0x00)
SPEED0 = chr(0x00)
SPEED1 = chr(0x00)
STEER0 = chr(0X02)
STEER1 = chr(0x02)
BRAKE = chr(0x01)
ALIVE = 0
ETX0 = chr(0x0d)
ETX1 = chr(0x0a)
Packet=[]
read=[]
count=0
count_alive=0
##################################################
def GetAorM():
    AorM = chr(0x01)
    return  AorM

def GetESTOP():
    ESTOP = chr(0x00)
    return  ESTOP

def GetGEAR(gear):
    GEAR = chr(gear)
    return  GEAR

def GetSPEED(speed):
    global count
    print(speed)
    SPEED0 = chr(0x00)
    SPEED = int(speed*36) # float to integer
    #print(111111111,SPEED)

    SPEED1 = chr(abs(SPEED)) # m/s to km/h*10

    return SPEED0, SPEED1

def GetSTEER(steer):
    
    steer=steer*71*(180/pi) # rad/s to degree/s*71

    if(steer>=2000):
        steer=1999
    elif(steer<=-2000):
        steer=-1999
    steer_max=0b0000011111010000 # +2000
    steer_0 = 0b0000000000000000
    steer_min=0b1111100000110000 # -2000

    # print(steer)
    if (steer>=0):
        angle=int(steer)
        STEER=steer_0+angle
    else:
        angle=int(-steer)
        angle=2000-angle
        STEER=steer_min+angle

    STEER0=STEER & 0b1111111100000000
    STEER0=STEER0 >> 8
    STEER1=STEER & 0b0000000011111111

    return chr(STEER0), chr(STEER1)

def GetBRAKE(brake):
    BRAKE = chr(brake)
    return  BRAKE


def Send_to_ERP42(gear, speed, steer, brake):
    global S, T, X, AorM, ESTOP, GEAR, SPEED0, SPEED1, STEER0, STEER1, BRAKE, ALIVE, ETX0, ETX1, count_alive
    #print(speed)
    count_alive = count_alive+1

    if count_alive==0xff:
        count_alive=0x00

    AorM = GetAorM()
    ESTOP = GetESTOP()
    GEAR = GetGEAR(gear)
    SPEED0, SPEED1 = GetSPEED(speed)
    STEER0, STEER1 = GetSTEER(steer)
    BRAKE = GetBRAKE(brake)

    ALIVE = chr(count_alive)
    vals = [S, T, X, AorM, ESTOP, GEAR, SPEED0, SPEED1, STEER0, STEER1, BRAKE, ALIVE, ETX0, ETX1]

    for i in range(len(vals)):
        ser.write(vals[i]) # send!
        
   

cur_ENC_backup=0


################################################################################

gear = 0
speed = 0
steer = 0
brake = 0

#def cmd_callback(data):
#    global gear, speed, steer, brake

#    gear = data.gear
#    speed = data.speed
#    steer = data.steer
#    brake = data.brake
#    print(steer)

def acker_callback(msg):
    global speed, steer, brake, gear

    speed = msg.drive.speed
    steer = -(msg.drive.steering_angle)

    brake = int(msg.drive.jerk)
    gear = int(msg.drive.acceleration)
    print(steer*180/np.pi)

def vel_callback(msg):
    global linear, angular

    linear = msg.Twist.linear.x
    angular = msg.Twist.angular.z


if __name__ == '__main__':
    rospy.init_node('serial_node')

    rospy.Subscriber("/ackermann_cmd", AckermannDriveStamped, acker_callback) #Lane_ack_vel , /ackermann_cmd


    rate = rospy.Rate(20)

    port = str(rospy.get_param("~robot_port","/dev/ttyUSB3"))

    ser = serial.serial_for_url(port, baudrate=115200, timeout=1)


    while (ser.isOpen() and (not rospy.is_shutdown())):
	#Send to Controller
        Send_to_ERP42(gear, speed, steer, brake)
        rate.sleep()
	
