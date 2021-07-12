#! /usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Imu
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def global_path_callback(data):
    global path, global_path_callback_sign
    path=data
    #print(11111111111111111111111)

    global_path_callback_sign=True
def current_step_callback(data):
    global step

    step=int(data.pose.position.z)

def pub_path(path, step):
    pathmsg=Path()

    pathmsg.header.seq = step
    pathmsg.header.stamp = rospy.Time.now()
    pathmsg.header.frame_id = "map"

    #for i in range(len(path.poses)-step-1):
    for i in range(20):
        pose = PoseStamped()
        #print("len",len(path.poses))
        #print("step",step)
        pose.header.seq = step
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        print(step)
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
            pathmsg.poses.append(pose)
        else:
            pass

    global_path.publish(pathmsg)

if __name__ == '__main__':
    rospy.init_node('path_planner')

    rospy.Subscriber("/gps_path",Path,global_path_callback)
    rospy.Subscriber("current_step",PoseStamped,current_step_callback)

    global_path = rospy.Publisher("/global_path",Path, queue_size=10)

    rate = rospy.Rate(10)

    step=0
    global_path_callback_sign=False
    path=Path()
    #rospy.sleep(10)
    while not rospy.is_shutdown():
        if (global_path_callback_sign==True):
            rospy.sleep(0.1)
            pub_path(path, step)
            #printprint(1)
        else:
            pass
