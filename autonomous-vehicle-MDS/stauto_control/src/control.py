#! /usr/bin/env python

import rospy
import tf
import numpy as np
import rospkg
import math
import time
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, UInt32, Bool
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import NavSatFix
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from gps_common import *
import copy
from math import *


def imu_callback(data):
    global imu_theta
    orientation_list = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
    roll, pitch, yaw = euler_from_quaternion (orientation_list)

    imu_theta=yaw*(180/np.pi)
    #print(yaw)

def cur_gps_position_callback(data):
    global cur_gps_position

    cur_gps_position[0] = data.latitude
    cur_gps_position[1] = data.longitude
    #print(cur_gps_position)


state_type={ -1 : "None",
             0 : "Cruise",
             1 : "avoid_cruise",
             2 : "stop",
             3 : "traffic",
             4 : "parking",
             5 : "saftyzone",
             6 : "crosswalk",
             7 : "Manual input mode",
             8 : "speedbump"}

def odometry_callback(data):
    global imu_theta, cur_gps_position

    cur_gps_position[0] = data.pose.pose.position.x
    cur_gps_position[1] = data.pose.pose.position.y

    orientation_list = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
    roll, pitch, yaw = euler_from_quaternion (orientation_list)

    imu_theta=yaw*(180/np.pi)

def speed_callback(data):
    global speed

    speed = data.data/36

def parking_path_callback(data):
    global parking_path

    for i in range(len(data.poses)):
        parking_path[i][0] = data.poses[i].pose.position.x
        parking_path[i][1] = data.poses[i].pose.position.y

def local_path_callback(data):
    global local_path

    for i in range(len(data.poses)):
        local_path[i][0] = data.poses[i].pose.position.x
        local_path[i][1] = data.poses[i].pose.position.y
    #print(local_path[0][0])
    #print(sqrt((local_path[0][0]-local_path[3][0])**(2) + (local_path[0][1]-local_path[3][1])**(2)))

def state_callback(state):
    global state_machine

    state_machine = state.data

def teb_callback(data):
    global teb
    teb=data

def lanenet_callback(data):
    global lanenet
    lanenet=data


def gps_accuracy_callback(data):
    global gps_accuracy
    gps_accuracy=data.data

def lane_accuracy_callback(data):
    global lane_accuracy
    lane_accuracy=data.data

def parking_finish_callback(data):
    global parking_finish_sign

    parking_finish_sign=data.data

if __name__ == '__main__':

    rospy.init_node('control')
    listener = tf.TransformListener()

    #Subscriber
    rospy.Subscriber("/final_path",Path,local_path_callback)
    rospy.Subscriber("/parking_path",Path,parking_path_callback)
    #rospy.Subscriber("global_path",Path,local_path_callback)
    #rospy.Subscriber("/gps/current_robot_position",NavSatFix,cur_gps_position_callback)
    rospy.Subscriber("/imu/data",Imu,imu_callback)
    rospy.Subscriber("/odom", Odometry,odometry_callback)
    rospy.Subscriber("/ERP42_speed",Float32,speed_callback)
    rospy.Subscriber("/state_machine",Int32MultiArray,state_callback)
    rospy.Subscriber("/ackermann_cmd_TEB",AckermannDriveStamped, teb_callback)
    rospy.Subscriber("/ackermann_cmd_LANENET",AckermannDriveStamped, lanenet_callback)

    rospy.Subscriber("/gps/accuracy", UInt32, gps_accuracy_callback)
    rospy.Subscriber("lane_accuracy", UInt32, lane_accuracy_callback)
    rospy.Subscriber("/parking_finish", Bool, parking_finish_callback)

    #Publisher
    ackermann_pub = rospy.Publisher('/ackermann_cmd', AckermannDriveStamped, queue_size=10)
    parking_end_pub = rospy.Publisher('/parking_end', Bool, queue_size=10)

    ackermann=AckermannDriveStamped()

    imu_theta=0.
    local_path =np.zeros((100,2))
    parking_path =np.zeros((100,2))
    local_path_length=0
    cur_gps_position = [0,0]
    gps_theta=0.
    speed=0
    path_alpha=0

    state_machine = np.zeros(8)
    prev_state_machine = np.zeros(8)

    gps_accuracy=0
    lane_accuracy=0
    stop_profile_flag=False
    stop_profile_time=0
    camera_theta=0.
    parking_finish_sign=False
    going_gps_n=[0,0]
    going_gps_n1=[0,0]
    going_gps_n2=[0,0]
    going_gps_n3=[0,0]
    going_gps=[0,0]

    max_speed=4.0 #3.5
    min_speed=2.5    #2.5
    rospy.sleep(1.5)

    parking_finish_time=time.time()
    init_time=rospy.Time.now()
    teb=AckermannDriveStamped()
    lanenet=AckermannDriveStamped()

    while not rospy.is_shutdown():

        going_gps[0]=cur_gps_position[0]
        going_gps[1]=cur_gps_position[1]
        #print(going_gps)
        #going_gps_n2[0]=(utm_next_gps[0] - 460000)/100
        #going_gps_n2[1]=(utm_next_gps[1] - 383000)/100

        #test
        '''
        local_path_length=((local_path>0).sum()) / 2
        going_gps_n[0]=local_path[0][0]
        going_gps_n[1]=local_path[0][1]
        going_gps_n1[0]=local_path[int(local_path_length*1/3)][0]
        going_gps_n1[1]=local_path[int(local_path_length*1/3)][1]
        going_gps_n2[0]=local_path[int(local_path_length*2/3)][0]
        going_gps_n2[1]=local_path[int(local_path_length*2/3)][1]
        going_gps_n3[0]=local_path[int(local_path_length-1)][0]
        going_gps_n3[1]=local_path[int(local_path_length-1)][1]
        '''
        if(state_machine[4]==1):
            going_gps_n[0]=parking_path[0][0]
            going_gps_n[1]=parking_path[0][1]
            going_gps_n1[0]=parking_path[1][0]
            going_gps_n1[1]=parking_path[1][1]
            going_gps_n2[0]=parking_path[2][0]
            going_gps_n2[1]=parking_path[2][1]
            going_gps_n3[0]=parking_path[3][0]
            going_gps_n3[1]=parking_path[3][1]
        else:
            going_gps_n[0]=local_path[0][0]
            going_gps_n[1]=local_path[0][1]
            going_gps_n1[0]=local_path[1][0]
            going_gps_n1[1]=local_path[1][1]
            going_gps_n2[0]=local_path[2][0]
            going_gps_n2[1]=local_path[2][1]
            going_gps_n3[0]=local_path[3][0]
            going_gps_n3[1]=local_path[3][1]
            #print(local_path)
            #print(going_gps)
        if (state_machine[1]==1):
            going_gps_theta = atan2(going_gps_n3[1]-going_gps[1], going_gps_n3[0]-going_gps[0])*180/np.pi
        else:
            going_gps_theta = atan2(going_gps_n2[1]-going_gps[1], going_gps_n2[0]-going_gps[0])*180/np.pi
        #print(going_gps_theta)
        going_gps_theta_speed = atan2(going_gps_n3[1]-going_gps_n1[1], going_gps_n3[0]-going_gps_n1[0])*180/np.pi
        #print(going_gps_theta,imu_theta)

        #print(going_gps_theta, imu_theta)
        if((int(imu_theta)*int(going_gps_theta))>=0.):
            alpha=imu_theta-going_gps_theta
        else:
            if(((-90>=imu_theta>=-180) or (180>=imu_theta>=90)) and ((-90>=going_gps_theta>=-180) or (180>=going_gps_theta>=90))):
                if((imu_theta>=0) and (going_gps_theta<0)):
                    alpha=-((180-imu_theta)+(180+going_gps_theta))
                elif((imu_theta<0) and (going_gps_theta>=0)):
                    alpha=(180+imu_theta)+(180-going_gps_theta)
            else:
                alpha=imu_theta-going_gps_theta

        alpha=alpha*(np.pi/180)  # alpha => degree



        if((int(imu_theta)*int(going_gps_theta_speed))>=0.):
            alpha_speed=imu_theta-going_gps_theta_speed
        else:
            if(((-90>=imu_theta>=-180) or (180>=imu_theta>=90)) and ((-90>=going_gps_theta_speed>=-180) or (180>=going_gps_theta_speed>=90))):
                if((imu_theta>=0) and (going_gps_theta_speed<0)):
                    alpha_speed=-((180-imu_theta)+(180+going_gps_theta_speed))
                elif((imu_theta<0) and (going_gps_theta_speed>=0)):
                    alpha_speed=(180+imu_theta)+(180-going_gps_theta_speed)
            else:
                alpha_speed=imu_theta-going_gps_theta

        alpha_speed=alpha_speed*(np.pi/180)  # alpha => degree

        going_gps_theta2 = atan2(going_gps_n1[1]-going_gps_n[1], going_gps_n1[0]-going_gps_n[0])*180/np.pi
        going_gps_theta3 = atan2(going_gps_n2[1]-going_gps_n1[1], going_gps_n2[0]-going_gps_n1[0])*180/np.pi
        if((int(going_gps_theta3)*int(going_gps_theta2))>=0.):
            path_alpha=going_gps_theta3-going_gps_theta2
        else:
            if(((-90>=going_gps_theta3>=-180) or (180>=going_gps_theta3>=90)) and ((-90>=going_gps_theta2>=-180) or (180>=going_gps_theta2>=90))):
                if((going_gps_theta3>=0) and (going_gps_theta2<0)):
                    path_alpha=-((180-going_gps_theta3)+(180+going_gps_theta2))
                elif((going_gps_theta3<0) and (going_gps_theta2>=0)):
                    path_alpha=(180+going_gps_theta3)+(180-going_gps_theta2)
            else:
                path_alpha=going_gps_theta3-going_gps_theta2

        

        if (alpha_speed>=28):
            alpha_speed=28
        elif(alpha_speed<=-28):
            alpha_speed=-28

        #print(alpha)
        #print(local_path[0][0],cur_gps_position[0],local_path[0][1],cur_gps_position[1])
        L=1.3
        Ld=sqrt((local_path[0][0]-cur_gps_position[0])**(2) + (local_path[0][1]-cur_gps_position[1])**(2))
        '''
        speed_ld = speed*0.65
        alpha_ld = abs(alpha*180/np.pi)*0.15
        '''



        
        speed_ld = speed*0.30
        alpha_ld = 0
        
        


        '''
        if(abs(path_alpha)>3):
            if(alpha_ld>2.5):
                alpha_ld=3.5
            elif(alpha_ld<=0.3):
                alpha_ld=0
        else:
            alpha_ld=0
        '''

        '''
        if(alpha_ld>2.0):
            alpha_ld=3.0
        elif(alpha_ld<=1.2):
            alpha_ld=0
        '''



        if (state_machine[1]==1): #avoid cruise
            ld = speed_ld+3.0-alpha_ld #4.3
        else:
            ld = speed_ld+3.0-alpha_ld #4.3
        #print(path_alpha)
        print(round(speed_ld,2), round(alpha_ld,2), round(ld,2), round(abs(path_alpha),2))
        #print(round(speed_ld,6),round((alpha_ld*180/np.pi),4), round(ld,4))
        gps_theta=atan(2*L*sin(alpha)/ld)*(180/np.pi)

        final_angle = gps_theta*1.0 + camera_theta*0

        if (final_angle>=28):
            final_angle=28
        elif(final_angle<=-28):
            final_angle=-28

        if(abs(final_angle)>=10):
            Speed_linear = (max_speed-min_speed)/28*(28-abs(alpha_speed))+min_speed
        else:
            Speed_linear=max_speed

        #print(going_gps_theta, imu_theta,gps_theta)

        #if(state_machine[0]==1 and gps_accuracy>=2):
        if(state_machine[0]==1):
            ackermann.drive.speed = Speed_linear
            ackermann.drive.steering_angle = -final_angle*np.pi/180
            ackermann.drive.jerk = 0
            ackermann.drive.acceleration = 0
            print("Global")
        
        #elif(state_machine[0]==1 and lane_accuracy>=5):
        #    ackermann.drive.speed = Speed_linear #lanenet.drive.speed
        #    ackermann.drive.steering_angle = lanenet.drive.steering_angle
        #    ackermann.drive.jerk = 0
        #    ackermann.drive.acceleration = 0
        #    print("Vision")

        #elif(state_machine[0]==1 and gps_accuracy<2 and lane_accuracy<5):
        #    ackermann.drive.speed = Speed_linear
        #    ackermann.drive.steering_angle = -final_angle*np.pi/180
        #    ackermann.drive.jerk = 0
        #    ackermann.drive.acceleration = 0
        #    print("Global")

        elif(state_machine[1]==1):
            #print(teb.drive.steering_angle)
            
            ackermann.drive.speed = 1.5 #1.8
            ackermann.drive.steering_angle = -final_angle*np.pi/180
            ackermann.drive.jerk = 0
            ackermann.drive.acceleration = 0
            
            # if(teb.drive.speed < 1):
            #     ackermann.drive.speed = teb.drive.speed + 1
            # else:
            #     ackermann.drive.speed = teb.drive.speed
            
            # ackermann.drive.steering_angle = teb.drive.steering_angle
            # ackermann.drive.jerk = teb.drive.jerk
            
            print("TEB!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        elif(state_machine[2]==1):
            ackermann.drive.speed = 0.0
            ackermann.drive.steering_angle = 0.0
            ackermann.drive.jerk = 100
            ackermann.drive.acceleration = 0

        elif(state_machine[3]==1 or state_machine[5]==1 or state_machine[6]==1):
            ackermann.drive.speed = Speed_linear*2/3 
            ackermann.drive.steering_angle = -final_angle*np.pi/180
            ackermann.drive.jerk = 0
            ackermann.drive.acceleration = 0

        elif(state_machine[4]==1):
            #print(1)
            if parking_finish_sign==False:
                ackermann.drive.speed = Speed_linear*1/2
                ackermann.drive.steering_angle = -final_angle*np.pi/180
                ackermann.drive.jerk = 0
                ackermann.drive.acceleration = 0
                parking_finish_time=time.time()
            else:
                if (time.time()-parking_finish_time<4):
                    ackermann.drive.speed = 0.0
                    ackermann.drive.steering_angle = 0.0
                    ackermann.drive.jerk = 100
                    ackermann.drive.acceleration = 0
                    print(1)
                elif (time.time()-parking_finish_time<8.5):
                    ackermann.drive.speed = 3.0
                    ackermann.drive.steering_angle = 0.0
                    ackermann.drive.jerk = 0
                    ackermann.drive.acceleration = 2
                    print(2)
                elif (time.time()-parking_finish_time<10.2):
                    ackermann.drive.speed = 3.0
                    ackermann.drive.steering_angle = -28.0*np.pi/180
                    ackermann.drive.jerk = 0
                    ackermann.drive.acceleration = 2
                    print(3)
                elif (time.time()-parking_finish_time<10.5):
                    ackermann.drive.speed = 0.0
                    ackermann.drive.steering_angle = 0.0
                    ackermann.drive.jerk = 100
                    ackermann.drive.acceleration = 0
                    print(4)
                else:
                    parking_end_pub.publish(True)
        elif(state_machine[6]==1):
            ackermann.drive.speed = Speed_linear*1/2
            ackermann.drive.steering_angle = -final_angle*np.pi/180
            ackermann.drive.jerk = 0
            ackermann.drive.acceleration = 0
        #print(prev_state_machine, state_machine, stop_profile_flag)
        #print(state_machine)
        if((prev_state_machine[0]==1 and state_machine[1]==1) or (prev_state_machine[0]==1 and state_machine[3]==1) or (prev_state_machine[0]==1 and state_machine[4]==1) or (prev_state_machine[0]==1 and state_machine[5]==1)):
            stop_profile_flag=True
            stop_profile_time=time.time()

        if stop_profile_flag==True:
            if(time.time()-stop_profile_time < 0.7):
                ackermann.drive.jerk = 60
                ackermann.drive.speed = 2.5
            else:
                stop_profile_flag=False
                
        prev_state_machine=state_machine
        #print(ackermann.drive)
        ackermann_pub.publish(ackermann)

    else:
        pass
