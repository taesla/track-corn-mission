#!/usr/bin/env python
import serial
import numpy as np
import rospy
from math import *
import tf
from pyproj import Proj
from pyproj import transform
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from tf.transformations import euler_from_quaternion, quaternion_from_euler

WGS84 = { 'proj':'latlong', 'datum':'WGS84', 'ellps':'WGS84', }
GRS80 = { 'proj':'tmerc', 'lat_0':'38', 'lon_0':'127', 'k':1, 'x_0':0,
    'y_0':0, 'ellps':'GRS80', 'units':'m' }
#'lat_0':'38.000036', 'lon_0':'127.00038'
def grs80_to_wgs84(x, y):
   return transform( Proj(**GRS80), Proj(**WGS84), x, y )

def wgs84_to_grs80(x, y):
   return transform( Proj(**WGS84), Proj(**GRS80), y, x )

def pub_odometry(gps, quaternion, vx, vy, vth):
    #print(vth, speed)
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "odom"


    # set the position
    odom.pose.pose = Pose(Point(gps[0], gps[1], 0.), Quaternion(*quaternion))

    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    # publish the message
    odom_pub.publish(odom)


    odom_broadcaster.sendTransform(
        (gps[0], gps[1], 0.),
        quaternion,
        odom.header.stamp,
        "base_link",
        "odom"
    )


def cur_gps_position_callback(data):
    global cur_gps_position

    #cur_gps_position[0] = data.latitude
    #cur_gps_position[1] = data.longitude
    cur_gps_position = wgs84_to_grs80(data.latitude, data.longitude)

def imu_callback(data):
    global vx, vy, vth, imu_quaternion
    imu_quaternion = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
    vth = data.angular_velocity.z
    vx = data.angular_velocity.x
    vy = data.angular_velocity.y

    roll, pitch, yaw = euler_from_quaternion (imu_quaternion)
    print(yaw*180/np.pi)

def speed_callback(data):
    global speed
    speed = data.data/10

if __name__ == '__main__':
    imu_quaternion=[0,0,0,0]
    cur_gps_position=[0,0]
    speed=0
    steer=0
    vx=0
    vy=0
    vth=0

    lf=1.05/2
    lr=1.05/2

    rospy.init_node('Odometry')

    #listener = tf.TransformListener()

    rospy.Subscriber("/gps/fix",NavSatFix,cur_gps_position_callback)
    rospy.Subscriber("/imu/data",Imu,imu_callback)
    rospy.Subscriber("/ERP42_speed",Float32,speed_callback)

    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    odom = Odometry()
    odom_broadcaster = tf.TransformBroadcaster()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():

        #B = atan(lr*tan(steer*(np.pi/180))/(lr+lf))
        #vth = speed*sin(B)/lr
        #vx = speed*cos(vth+B)
        #vy = speed*sin(vth+B)

        pub_odometry(cur_gps_position, imu_quaternion, vx, vy, vth)
        rate.sleep()

    else:
        pass
