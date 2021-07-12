#! /usr/bin/env python

import rospy
#import tf
import numpy as np
import time
import rospkg
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32
#import tf2_ros
from pyproj import Proj
from pyproj import transform
from tf.transformations import euler_from_quaternion, quaternion_from_euler
#from geometry_msgs.msg import TransformStamped
#from tf.transformations import euler_from_quaternion, quaternion_from_euler

from gps_common import *
import copy
from math import *

WGS84 = { 'proj':'latlong', 'datum':'WGS84', 'ellps':'WGS84', }
GRS80 = { 'proj':'tmerc', 'lat_0':'38', 'lon_0':'127', 'k':1, 'x_0':0,
    'y_0':0, 'ellps':'GRS80', 'units':'m' }
#'lat_0':'38.000036', 'lon_0':'127.00038'
def grs80_to_wgs84(x, y):
   return transform( Proj(**GRS80), Proj(**WGS84), x, y )

def wgs84_to_grs80(x, y):
   return transform( Proj(**WGS84), Proj(**GRS80), y, x )
'''
def pub_tf_transform(lat,lon):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()  # pose of turntable_frame w.r.t. turntable_base
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = 'world'
    t.child_frame_id  = 'GPS_link'
    t.transform.translation.x = lat
    t.transform.translation.y = lon
    t.transform.translation.z = 0
    t.transform.rotation.x = 0
    t.transform.rotation.y = 0
    t.transform.rotation.z = 0
    t.transform.rotation.w = 1
    br.sendTransform(t)
'''
def gps_callback(data):
    global lat, lon, utm_lat_lon

    #print(odom_x)
    lat = data.latitude
    lon = data.longitude
    #print(lat, lon)
    utm_lat_lon = wgs84_to_grs80(lat, lon)

def imu_callback(data):
    global imu_yaw
    imu_quaternion = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]

    roll, pitch, imu_yaw = euler_from_quaternion (imu_quaternion)
    imu_yaw=imu_yaw*180/np.pi
    #print(imu_yaw)

def speed_callback(data):
    global speed

    speed = data.data/36

def find_gps_step(last_step, cur_gps):
    min_length=100
    cur_step=0

    for step_gps in range(last_step-4):
        gps_n = gps_data[step_gps].split(',')
        gps_n_1 = gps_data[step_gps+1].split(',')
        gps_n_2 = gps_data[step_gps+2].split(',')


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

        if(length<min_length and cur_gps==0):
            min_length=length
            cur_step=step_gps+1

        elif (length<min_length and cur_gps-2 <= step_gps <= cur_gps+2):
            min_length=length
            cur_step=step_gps+1

    return cur_step

def path_converter(gps, step_gps, last_step):
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
    #print(x)
    pose.pose.position.y = y
    pose.pose.position.z = 0

    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 0

    #rospy.sleep(0.5)
    pathmsg.poses.append(pose)



    if(step_gps == last_step-1):
        path_pub.publish(pathmsg)
        print("global path finish!!!!!!!!!!!")

def current_step_pub(step_gps):

    current_pose = PoseStamped()

    current_pose.header.seq = 0
    current_pose.header.stamp = rospy.Time.now()
    current_pose.header.frame_id = "base_link"

    current_pose.pose.position.x = pathmsg.poses[step_gps].pose.position.x
    current_pose.pose.position.y = pathmsg.poses[step_gps].pose.position.y
    current_pose.pose.position.z = step_gps

    current_pose.pose.orientation.x = 0
    current_pose.pose.orientation.y = 0
    current_pose.pose.orientation.z = 0
    current_pose.pose.orientation.w = 0


    step_pub.publish(current_pose)
    #print(current_pose)
'''
def pub_utm_cur_gps(lat,lon):
    utm_gpsmsg.header.stamp = rospy.Time.now()
    utm_gpsmsg.header.frame_id = "gps_link"
    utm_gpsmsg.latitude=lat
    utm_gpsmsg.longitude=lon
    utm_gpsmsg. position_covariance_type=0
    utm_cur_gps_pub.publish(utm_gpsmsg)
    #print(gpsmsg)
'''
if __name__ == '__main__':

    rospy.init_node('Global_path_planner')
    #listener = tf.TransformListener()

    path_pub = rospy.Publisher('/gps_path', Path, queue_size=10)
    step_pub = rospy.Publisher('current_step', PoseStamped, queue_size=10)
    #utm_cur_gps_pub = rospy.Publisher("/gps/current_robot_position",NavSatFix,queue_size=10)
    rospy.Subscriber("/gps/fix",NavSatFix,gps_callback)
    rospy.Subscriber("/imu/data",Imu,imu_callback)
    rospy.Subscriber("ERP42_speed",Float32,speed_callback)
    start_yaw = rospy.Publisher('yaw_error', Float32, queue_size=10)

    rospack = rospkg.RosPack()
    rospack.list()
    arg_name = rospack.get_path('stauto_sensor') + "/src/gps_data/"

    f = open(arg_name + "gps_data_seoultech.txt","r")
    # f = open(arg_name + "ccc.txt","r")

    gps_data = f.readlines()
    last_step=len(gps_data)
    lat = 0
    lon = 0
    utm_lat_lon=[0,0]
    imu_yaw=0
    speed=0
    theta_error=0
    step_gps = 0
    path_pub_sign=True
    adjust_yaw_sign=True
    prev_adjust_yaw_time=0
    theta_error_count=0
    d_theta=0
    pathmsg=Path()
    utm_gpsmsg=NavSatFix()

    rospy.sleep(1)

    init_time=rospy.Time.now()

    while not rospy.is_shutdown():

        if (path_pub_sign==True):
            if (step_gps<last_step):
                #print(step_gps)

                gps_n = gps_data[step_gps].split(',')
                print(float(gps_n[0]), float(gps_n[1]))
                gps_n = [float(gps_n[0]), float(gps_n[1])]

                path_converter(gps_n, step_gps, last_step)
                step_gps=step_gps+1
            else:
                path_pub_sign=False
                step_gps=0

        else:
            step_gps=find_gps_step(last_step, step_gps)
            current_step_pub(step_gps)

            #########imu error pub##########
            gps_theta1 = atan2(pathmsg.poses[step_gps+1].pose.position.y-pathmsg.poses[step_gps].pose.position.y, pathmsg.poses[step_gps+1].pose.position.x-pathmsg.poses[step_gps].pose.position.x)*180/np.pi
            gps_theta2 = atan2(pathmsg.poses[step_gps+2].pose.position.y-pathmsg.poses[step_gps].pose.position.y, pathmsg.poses[step_gps+2].pose.position.x-pathmsg.poses[step_gps].pose.position.x)*180/np.pi
            #print(adjust_yaw_sign)
            if (adjust_yaw_sign==True):
                if(abs(gps_theta1-gps_theta2)<=4 or 0<step_gps<=5):
                    start_theta=gps_theta2
                    #start_theta = atan2(pathmsg.poses[step_gps+2].pose.position.y-pathmsg.poses[step_gps].pose.position.y, pathmsg.poses[step_gps+2].pose.position.x-pathmsg.poses[step_gps].pose.position.x)*180/np.pi
                    print(start_theta)
                    start_yaw.publish(-start_theta)
                    adjust_yaw_sign=False
                    prev_adjust_yaw_time=time.time()
                else:
                    adjust_yaw_sign=False
                    prev_adjust_yaw_time=time.time()

            else:
                d_theta=(imu_yaw-gps_theta2)-theta_error
                if((time.time()-prev_adjust_yaw_time>=10) and speed>=1.8 and abs(d_theta)<=1.5):
                    theta_error_count+=1

                    if(theta_error_count>=20):
                        adjust_yaw_sign=True
                        theta_error_count=0
                else:
                    theta_error=imu_yaw-gps_theta2
                    theta_error_count=0


            #print(pathmsg.poses[step_gps].pose.position.x)

            #pub_utm_cur_gps(utm_lat_lon[0], utm_lat_lon[1])
            #pub_tf_transform(lat,lon)

    else:
        pass