#!/usr/bin/env python

from std_msgs.msg import Int16MultiArray, Bool
from sensor_msgs.msg import NavSatFix
import rospkg, rospy, time
import numpy as np
from pyproj import Proj
from pyproj import transform
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from math import *

WGS84 = { 'proj':'latlong', 'datum':'WGS84', 'ellps':'WGS84', }
GRS80 = { 'proj':'tmerc', 'lat_0':'38', 'lon_0':'127', 'k':1, 'x_0':0,
    'y_0':0, 'ellps':'GRS80', 'units':'m' }

def grs80_to_wgs84(x, y):
   return transform( Proj(**GRS80), Proj(**WGS84), x, y )

def wgs84_to_grs80(x, y):
   return transform( Proj(**WGS84), Proj(**GRS80), y, x )

def gps_callback(data):
    global lat, lon

    #print(odom_x)
    lat = data.latitude
    lon = data.longitude

def empty_callback(data):
    global empty_spot_array

    empty_spot_array=data.data

def path_converter(gps, step_gps, last_step): # convert wgs84 to grs80
    #print(step_gps)
    pathmsg.header.seq = step_gps
    pathmsg.header.stamp = rospy.Time.now()
    pathmsg.header.frame_id = "map"

    pose = PoseStamped()
    pose.header.seq = step_gps
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "map"

    x,y=wgs84_to_grs80(gps[0],gps[1])
    pose.pose.position.x = x
    #print(x,y)
    pose.pose.position.y = y
    pose.pose.position.z = 0

    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 0

    #rospy.sleep(0.5)
    pathmsg.poses.append(pose)

def find_gps_step(last_step, cur_gps): # find current gps step
    min_length=100
    cur_step=0

    for step_gps in range(last_step-4):
        gps_n = parking_data[step_gps].split(',')
        gps_n_1 = parking_data[step_gps+1].split(',')
        gps_n_2 = parking_data[step_gps+2].split(',')


        # gps_n = [float(gps_n[0]) - float(gps_origin[0]), float(gps_n[1]) - float(gps_origin[1])]
        # gps_n_1 = [float(gps_n_1[0]) - float(gps_origin[0]), float(gps_n_1[1])- float(gps_origin[1])]
        gps_n = [float(gps_n[0]), float(gps_n[1])]
        gps_n_1 = [float(gps_n_1[0]), float(gps_n_1[1])]
        gps_n_2 = [float(gps_n_2[0]), float(gps_n_2[1])]


        utm_gps_n = wgs84_to_grs80(gps_n[0],gps_n[1])
        utm_gps_n_1 = wgs84_to_grs80(gps_n_1[0],gps_n_1[1])
        utm_gps_n_2 = wgs84_to_grs80(gps_n_2[0],gps_n_2[1])

        utm_gps_cur = wgs84_to_grs80(lat,lon)

        length1 = sqrt((utm_gps_cur[0]-utm_gps_n[0])**(2)+(utm_gps_cur[1]-utm_gps_n[1])**(2))
        length2 = sqrt((utm_gps_cur[0]-utm_gps_n_1[0])**(2)+(utm_gps_cur[1]-utm_gps_n_1[1])**(2))

        length = length1+length2
        '''
        line_data_x=[utm_gps_n[0],utm_gps_n_1[0],utm_gps_n_2[0]]
        line_data_y=[utm_gps_n[1],utm_gps_n_1[1],utm_gps_n_2[1]]
        fp1 = np.polyfit(line_data_x,line_data_y,1)
        y= fp1[0]*step_gps+fp1[1]
        length=abs(fp1[0]*utm_gps_cur[0] - utm_gps_cur[1] + fp1[1])/sqrt(fp1[0]**(2)+(-1)**(2)) #find length
        '''

        if(length<min_length):
            min_length=length
            cur_step=step_gps+1
        '''
        elif (length<min_length and cur_gps-2 <= step_gps <= cur_gps+2):
            min_length=length
            cur_step=step_gps+1
        '''
    return cur_step

def pub_path(path, step): #publish going parking path based current gps_step
    parking_pathmsg=Path()

    parking_pathmsg.header.seq = step
    parking_pathmsg.header.stamp = rospy.Time.now()
    parking_pathmsg.header.frame_id = "map"

    #for i in range(len(path.poses)-step-1):
    for i in range(20):
        pose = PoseStamped()
        #print("len",len(path.poses))
        #print("step",step)
        pose.header.seq = step
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        #print(step)
        #print(len(path.poses)-(step+i+1))
        if(len(path.poses)-(step+i+1)>0):
            pose.pose.position.x = path.poses[step+i].pose.position.x
            pose.pose.position.y = path.poses[step+i].pose.position.y
            pose.pose.position.z = 0

            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 0
            #print(path.poses[step+i].pose.position.x)
            #rospy.sleep(0.5)
            parking_pathmsg.poses.append(pose)
        else:
            pass
    #print(len(parking_pathmsg.poses))
    parking_path.publish(parking_pathmsg)


if __name__ == '__main__':

    rospy.init_node('parking_move_node')

    rospy.Subscriber('empty_space',Int16MultiArray,empty_callback)
    rospy.Subscriber("/gps/fix",NavSatFix,gps_callback)
    parking_path = rospy.Publisher("/parking_path",Path, queue_size=10)
    parking_finish = rospy.Publisher("/parking_finish",Bool, queue_size=10)
    empty_spot_array = np.zeros(6)
    pathmsg=Path()

    rospack = rospkg.RosPack()
    rospack.list()
    arg_name = rospack.get_path('stauto_sensor') + "/src/parking_data/"
    
    lat = 0
    lon = 0
    step = 0
    step_gps = 0
    last_step = 0
    parking_data=""
    parking_sign=False
    parking_end=False
    create_path_msg=False
    parking_zone_number=10

    rospy.sleep(1)
    init_time=rospy.Time.now()
    prev_time=time.time()
    '''
    f1 = open(arg_name + "parking1.txt","r")
    f2 = open(arg_name + "parking2.txt","r")
    f3 = open(arg_name + "parking3.txt","r")
    f4 = open(arg_name + "parking4.txt","r")
    f5 = open(arg_name + "parking5.txt","r")
    f6 = open(arg_name + "parking6.txt","r")

    parking1_data = f1.readlines()
    parking2_data = f2.readlines()
    parking3_data = f3.readlines()
    parking4_data = f4.readlines()
    parking5_data = f5.readlines()
    parking6_data = f6.readlines()
    '''
    while not rospy.is_shutdown():
        try:
            #print(parking_end)
            if parking_end==False:
                print(parking_zone_number)
                if parking_sign==False: #find emptied parking space & read parking path
                    #print(parking_sign)
                    for i in range(6):
                        #print(empty_spot_array)
                        #empty_spot_array[3]=1
                        #if(time.time()-prev_time>=20):
                        #    empty_spot_array[2]=1
                        #else:
                        #    empty_spot_array[5]=1
                        if empty_spot_array[i]==1:
                            #print(i)
                            if parking_zone_number != i:
                                parking_zone_number=i
                                parking_sign=True
                                create_path_msg=False
                                f = open(arg_name + "parking"+str(i+1)+".txt","r")
                                parking_data = f.readlines()
                                last_step=len(parking_data)
                                pathmsg=Path()
                                #print(last_step)
                                #print(1.1)
                            
                            else:
                                parking_sign=True
                                #print(1.2)
                        
                        if parking_sign==True:
                            break
                    #print(1)
                if (parking_sign==True) and (create_path_msg==False): #create parking path msg
                    if (step<last_step):

                        gps_n = parking_data[step].split(',')
                        #print(float(gps_n[0]), float(gps_n[1]))
                        gps_n = [float(gps_n[0]), float(gps_n[1])]

                        path_converter(gps_n, step, last_step)
                        step=step+1
                    else:
                        create_path_msg=True
                        step=0
                    #print(2)
                if (parking_sign==True) and (create_path_msg==True): #find current gps step based parking path & publish going parking path to control.py
                    step_gps=find_gps_step(last_step, step_gps)
                    print(step_gps, last_step)
                    if step_gps<last_step-6:
                        pub_path(pathmsg, step_gps)
                        parking_finish.publish(False)
                    else:
                        parking_finish.publish(True)
                        parking_end=True
                    parking_sign=False
                        #print(3)
            else:
                pass
        except rospy.ROSInterruptException:
            pass