#! /usr/bin/env python

import rospy
import numpy as np
import time
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32MultiArray

def state_callback(state):
    global state_machine

    state_machine = state.data

def local_path_callback(data):
    global move_base_local_path

    move_base_local_path=Path()

    local_path =np.zeros((300,2))
    local_path_length=len(data.poses)/2

    for i in range(len(data.poses)):
        local_path[i][0] = data.poses[i].pose.position.x
        local_path[i][1] = data.poses[i].pose.position.y
    #print(len(data.poses), ((local_path!=0).sum()) / 2)
    for j in range(0,4):
        pose = PoseStamped()

        pose.header.seq = 0
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"

        pose.pose.position.x = local_path[int((local_path_length-1)*j/3)][0]
        pose.pose.position.y = local_path[int((local_path_length-1)*j/3)][1]
        pose.pose.position.z = 0

        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 0

        move_base_local_path.poses.append(pose)

def global_path_callback(data):
    global global_path

    global_path=data

if __name__ == '__main__':

    rospy.init_node('path_select')

    rospy.Subscriber("/state_machine",Int32MultiArray,state_callback)
    rospy.Subscriber("/adaptive_clustering/Path_1",Path,local_path_callback)
    rospy.Subscriber("/global_path",Path,global_path_callback)

    final_path = rospy.Publisher("/final_path",Path, queue_size=10)
    state_machine = np.zeros(8)

    move_base_local_path=Path()
    global_path=Path()

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep()
        
        if (state_machine[1]==1):
            final_path.publish(move_base_local_path)
            print("TEB Path")
        else:
            # final_path.publish(move_base_local_path)
            final_path.publish(global_path)
            print("Global Path")
        
        # final_path.publish(global_path)

    else:
        pass
