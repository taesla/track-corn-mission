#!/usr/bin/env python

# Author: christoph.roesmann@tu-dortmund.de

import rospy, math
import tf
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import PolygonStamped, Point32
from adaptive_clustering.msg import Bboxes2d
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from geometry_msgs.msg import Point , PoseStamped
from nav_msgs.msg import Odometry
import shapely.geometry
import shapely.affinity
import numpy as np
import pandas as pd
import scipy.interpolate
import model_predictive_trajectory_generator as planner
import motion_model
bboxes = []
global_path = Path()
euler = []

# motion parameter
L = 1.0  # wheel base
ds = 0.1  # course distanse
v = 10.0 / 3.6  # velocity [m/s]

import os
table_path = os.path.dirname(os.path.abspath(__file__)) + "/lookuptable.csv"


class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

def get_lookup_table():
    data = pd.read_csv(table_path)

    return np.array(data)

def search_nearest_one_from_lookuptable(tx, ty, tyaw, lookup_table):
    mind = float("inf")
    minid = -1

    for (i, table) in enumerate(lookup_table):

        dx = tx - table[0]
        dy = ty - table[1]
        dyaw = tyaw - table[2]
        d = math.sqrt(dx ** 2 + dy ** 2 + dyaw ** 2)
        if d <= mind:
            minid = i
            mind = d

    return lookup_table[minid]

def generate_path(target_states, k0):
    # x, y, yaw, s, km, kf
    lookup_table = get_lookup_table()
    result = []

    for state in target_states:
        bestp = search_nearest_one_from_lookuptable(
            state[0], state[1], state[2], lookup_table)

        target = State(x=state[0], y=state[1], yaw=state[2])
        init_p = np.array(
            [math.sqrt(state[0] ** 2 + state[1] ** 2), bestp[4], bestp[5]]).reshape(3, 1)

        x, y, yaw, p = planner.optimize_trajectory(target, k0, init_p)

        if x is not None:
            print("find good path")
            result.append(
                [x[-1], y[-1], yaw[-1], float(p[0]), float(p[1]), float(p[2])])

    print("finish path generation")
    return result


def calc_lane_states(l_center, l_heading, l_width, v_width, d, nxy):
    """

    calc lane states

    :param l_center: lane lateral position
    :param l_heading:  lane heading
    :param l_width:  lane width
    :param v_width: vehicle width
    :param d: longitudinal position
    :param nxy: sampling number
    :return: state list
    """
    xc = math.cos(l_heading) * d + math.sin(l_heading) * l_center
    yc = math.sin(l_heading) * d + math.cos(l_heading) * l_center

    states = []
    for i in range(nxy):
        delta = -0.5 * (l_width - v_width) + \
            (l_width - v_width) * i / (nxy - 1)
        xf = xc - delta * math.sin(l_heading)
        yf = yc + delta * math.cos(l_heading)
        yawf = l_heading
        states.append([xf, yf, yawf])

    return states

class RotatedRect:
    def __init__(self, cx, cy, w, h, angle):
        self.cx = cx
        self.cy = cy
        self.w = w
        self.h = h
        self.angle = angle*(180/math.pi)

    def get_contour(self):
        w = self.w
        h = self.h
        c = shapely.geometry.box(-w/2.0, -h/2.0, w/2.0, h/2.0)
        rc = shapely.affinity.rotate(c, self.angle)
        return shapely.affinity.translate(rc, self.cx, self.cy)

    def intersection(self, other):
        return self.get_contour().intersection(other.get_contour())

def bboxes_callback(data):
  global bboxes
  bboxes = data.Bboxes2d

def path_callback(data):
    global global_path
    global_path = data

def step_callback(data):
  global step_num
  step_num = data.pose.position.z

def odom_callback(data):
    global odom_pose
    odom_pose = data.pose.pose

def state_lattice_2_path(lattice_path,xc_,yc_,theta_diff,theta_global):
    for i in range(len(xc_)):
        lattice_path.header.stamp = rospy.Time.now()
        lattice_path.header.frame_id = "map"

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"

        pose.pose.position.x = math.cos(theta_diff)*(xc_[i]) - math.sin(theta_diff)*(yc_[i]) + odom_pose.position.x
        pose.pose.position.y = math.sin(theta_diff)*(xc_[i]) + math.cos(theta_diff)*(yc_[i]) + odom_pose.position.y
        pose.pose.position.z = 0

        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1

        lattice_path.poses.append(pose)

    for j in range(5):
        x_ = lattice_path.poses[-1].pose.position.x
        y_ = lattice_path.poses[-1].pose.position.y

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"

        pose.pose.position.x = math.cos(theta_global)*(j*0.1) - math.sin(theta_global)*(0) + x_
        pose.pose.position.y = math.sin(theta_global)*(j*0.1) + math.cos(theta_global)*(0) + y_
        pose.pose.position.z = 0

        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1

        lattice_path.poses.append(pose)
    return lattice_path

def publish_obstacle_msg():
    pub = rospy.Publisher('/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=1)
    marker1_pub = rospy.Publisher('visualization_marker_1', MarkerArray, queue_size = 100)
    marker2_pub = rospy.Publisher('visualization_marker_2', MarkerArray, queue_size = 100)
    marker3_pub = rospy.Publisher('visualization_marker_3', MarkerArray, queue_size = 100)
    bool_pub = rospy.Publisher('/adaptive_clustering/is_bool',Bool,queue_size = 1)
    path1_pub = rospy.Publisher('/adaptive_clustering/Path_1',Path,queue_size = 1)
    path2_pub = rospy.Publisher('/adaptive_clustering/Path_2',Path,queue_size = 1)
    path3_pub = rospy.Publisher('/adaptive_clustering/Path_3',Path,queue_size = 1)
    finalpath_pub = rospy.Publisher('/adaptive_clustering/path_final',Path,queue_size = 1)
    #pub = rospy.Publisher('/p3dx/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=1)
    bboxes_sub = rospy.Subscriber('/adaptive_clustering/bboxes_2d',Bboxes2d,bboxes_callback)
    path_sub = rospy.Subscriber('/global_path',Path,path_callback)
    step_sub = rospy.Subscriber('/current_step', PoseStamped, step_callback, queue_size=1)
    odom_sub = rospy.Subscriber('odom', Odometry, odom_callback, queue_size=1)

    rospy.init_node("path_obstacle")


    obstacle_msg = ObstacleArrayMsg() 
    obstacle_msg.header.stamp = rospy.Time.now()
    obstacle_msg.header.frame_id = "map" # CHANGE HERE: odom/map

    listener = tf.TransformListener()
    is_bool = Bool()
    is_bool.data = False
    box_bool = False

    global_path_box_0 = 0
    global_path_box_5 = 0

    r_gpath_list = [0,0,0]
    r_cpath_list = [0,0,0]

    lattice_path = Path()
    path_1 = Path()
    path_2 = Path()
    path_3 = Path()
    path_score = [0,0,0]
    
    r = rospy.Rate(144) # 10hz
    path_change = True

    xc_1 = []
    yc_1 = []
    xc_2 = []
    yc_2 = []
    xc_3 = []
    yc_3 = []

    k0 = 0.0

    l_center = 0.0
    l_heading = np.deg2rad(0.0)
    l_width = 5.0
    v_width = 3.0
    d = 3
    nxy = 3
    states = calc_lane_states(l_center, l_heading, l_width, v_width, d, nxy)
    result = generate_path(states, k0)

    for num,table in enumerate(result):
        lattice_path = Path()
        if(num == 0):
            xc_1, yc_1, yawc = motion_model.generate_trajectory(table[3], table[4], table[5], k0)
        elif(num == 1):
            xc_2, yc_2, yawc = motion_model.generate_trajectory(table[3], table[4], table[5], k0)
        else:
            xc_3, yc_3, yawc = motion_model.generate_trajectory(table[3], table[4], table[5], k0)

    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/velodyne', rospy.Time(0))
            euler = tf.transformations.euler_from_quaternion(rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        path_score = [0,0,0]

        lattice_path = Path()

        dx_global = global_path.poses[1].pose.position.x - global_path.poses[0].pose.position.x
        dy_global = global_path.poses[1].pose.position.y - global_path.poses[0].pose.position.y
        theta_global = math.atan2(dy_global,dx_global)

        for i in range(0,3):
            dx = global_path.poses[i+1].pose.position.x - global_path.poses[i].pose.position.x
            dy = global_path.poses[i+1].pose.position.y - global_path.poses[i].pose.position.y
            theta = math.atan2(dy,dx)

            cx = (global_path.poses[i+1].pose.position.x + global_path.poses[i].pose.position.x)/2
            cy = (global_path.poses[i+1].pose.position.y + global_path.poses[i].pose.position.y)/2

            height = math.sqrt(dx**2+dy**2)
            r_gpath = RotatedRect(cx, cy, height, 2, theta)

            r_gpath_list[i] = r_gpath
            if(step_num > 170):
              cx_ = (r_gpath.get_contour().exterior.coords[1][0] + r_gpath.get_contour().exterior.coords[2][0])/2
              cy_ = (r_gpath.get_contour().exterior.coords[1][1] + r_gpath.get_contour().exterior.coords[2][1])/2
            else:
              cx_ = (r_gpath.get_contour().exterior.coords[0][0] + r_gpath.get_contour().exterior.coords[3][0])/2
              cy_ = (r_gpath.get_contour().exterior.coords[0][1] + r_gpath.get_contour().exterior.coords[3][1])/2          

            r_cpath = RotatedRect(cx_, cy_, height, 5, theta)        

            r_cpath_list[i] = r_cpath

        dy = (yc_1[1] - yc_1[0])
        dx = (xc_1[1] - xc_1[0])
        theta = math.atan2(dy,dx)

        theta_diff = theta_global - theta

        
        for num in range(3):
            lattice_path = Path()
                
            if(num == 0):
                lattice_path = state_lattice_2_path(lattice_path,xc_1,yc_1,theta_diff,theta_global)
                path_1 = lattice_path
                path1_pub.publish(lattice_path)
            elif(num == 1):
                lattice_path = state_lattice_2_path(lattice_path,xc_2,yc_2,theta_diff,theta_global)
                path_2 = lattice_path
                path2_pub.publish(lattice_path)
            else:
                lattice_path = state_lattice_2_path(lattice_path,xc_3,yc_3,theta_diff,theta_global)
                path_3 = lattice_path
                path3_pub.publish(lattice_path)
        
        marker_array = MarkerArray()
        for i in range(len(path_1.poses)-1):
            dx = path_1.poses[i+1].pose.position.x - path_1.poses[i].pose.position.x
            dy = path_1.poses[i+1].pose.position.y - path_1.poses[i].pose.position.y
            theta = math.atan2(dy,dx)

            cx = (path_1.poses[i+1].pose.position.x + path_1.poses[i].pose.position.x)/2
            cy = (path_1.poses[i+1].pose.position.y + path_1.poses[i].pose.position.y)/2

            height = math.sqrt(dx**2+dy**2)
            r_gpath = RotatedRect(cx, cy, height, 1.5, theta)

            # marker = Marker()
            # marker.id = i
            # marker.header.frame_id = "/map"
            # marker.type = marker.LINE_STRIP
            # marker.scale.x = 1
            # marker.scale.y = 1
            # marker.scale.z = 1
            # marker.color.a = 1.0
            # marker.pose.orientation.w = 1.0
            # marker.lifetime = rospy.Duration(0.01)
            # marker.pose.orientation.x = 0
            # marker.pose.orientation.y = 0
            # marker.pose.orientation.z = 0
            # marker.pose.orientation.w = 0

            # for k in range(len(r_gpath.get_contour().exterior.coords)):
            #   p = Point32()
            #   p.x = r_gpath.get_contour().exterior.coords[k][0]#math.cos(-euler[2])*(r1.get_contour().exterior.coords[k][0]- trans[0])-math.sin(-euler[2])*(r1.get_contour().exterior.coords[k][1]- trans[1])
            #   p.y = r_gpath.get_contour().exterior.coords[k][1]#math.sin(-euler[2])*(r1.get_contour().exterior.coords[k][0]- trans[0])+math.cos(-euler[2])*(r1.get_contour().exterior.coords[k][1]- trans[1])
            #   # print(k,p)
            #   marker.points.append(p)

            # marker_array.markers.append(marker)  
            
            point_obj = shapely.geometry.Point(cx,cy)
            for box in bboxes:
                # x_ = math.cos(euler[2])*box.center.x-math.sin(euler[2])*box.center.y+trans[0]
                # y_ = math.sin(euler[2])*box.center.x+math.cos(euler[2])*box.center.y+trans[1]

                # dist = math.sqrt(math.pow((cx-x_),2) + math.pow((cy-y_),2))
                # if(dist < 0.5):
                #     path_score[0] -= 2
                
                r2 = RotatedRect(math.cos(euler[2])*box.center.x-math.sin(euler[2])*box.center.y+trans[0], math.sin(euler[2])*box.center.x+math.cos(euler[2])*box.center.y+trans[1], box.size_y, box.size_x,euler[2]-math.pi/2)
                if(r_gpath.intersection(r2).area != 0):
                    rospy.logwarn("1 is obstacle")
                    path_score[0] -= 1000

            path_out = True
            for l in range(len(r_cpath_list)):
                if(r_cpath_list[l].get_contour().contains(point_obj)):
                    path_out = False
            if(path_out):
                path_score[0] -= 5
            for l in range(len(r_gpath_list)):
                if(r_gpath_list[l].get_contour().contains(point_obj)):
                    path_score[0] += 0.1

        marker1_pub.publish(marker_array)

        marker_array = MarkerArray()
        for i in range(len(path_2.poses)-1):
            dx = path_2.poses[i+1].pose.position.x - path_2.poses[i].pose.position.x
            dy = path_2.poses[i+1].pose.position.y - path_2.poses[i].pose.position.y
            theta = math.atan2(dy,dx)

            cx = (path_2.poses[i+1].pose.position.x + path_2.poses[i].pose.position.x)/2
            cy = (path_2.poses[i+1].pose.position.y + path_2.poses[i].pose.position.y)/2

            height = math.sqrt(dx**2+dy**2)
            r_gpath = RotatedRect(cx, cy, height, 1.5, theta)

            marker = Marker()
            marker.id = i
            marker.header.frame_id = "/map"
            marker.type = marker.LINE_STRIP
            marker.scale.x = 1
            marker.scale.y = 1
            marker.scale.z = 1
            marker.color.a = 1.0
            marker.pose.orientation.w = 1.0
            marker.lifetime = rospy.Duration(0.01)
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 0

            for k in range(len(r_gpath.get_contour().exterior.coords)):
              p = Point32()
              p.x = r_gpath.get_contour().exterior.coords[k][0]#math.cos(-euler[2])*(r1.get_contour().exterior.coords[k][0]- trans[0])-math.sin(-euler[2])*(r1.get_contour().exterior.coords[k][1]- trans[1])
              p.y = r_gpath.get_contour().exterior.coords[k][1]#math.sin(-euler[2])*(r1.get_contour().exterior.coords[k][0]- trans[0])+math.cos(-euler[2])*(r1.get_contour().exterior.coords[k][1]- trans[1])
              # print(k,p)
              marker.points.append(p)

            marker_array.markers.append(marker)  
            point_obj = shapely.geometry.Point(cx,cy)
            for box in bboxes:
                # x_ = math.cos(euler[2])*box.center.x-math.sin(euler[2])*box.center.y+trans[0]
                # y_ = math.sin(euler[2])*box.center.x+math.cos(euler[2])*box.center.y+trans[1]

                # dist = math.sqrt(math.pow((cx-x_),2) + math.pow((cy-y_),2))
                # if(dist < 0.5):
                #     path_score[0] -= 2
                r2 = RotatedRect(math.cos(euler[2])*box.center.x-math.sin(euler[2])*box.center.y+trans[0], math.sin(euler[2])*box.center.x+math.cos(euler[2])*box.center.y+trans[1], box.size_y, box.size_x,euler[2]-math.pi/2)
                if(r_gpath.intersection(r2).area != 0):
                    # rospy.logwarn("2 is obstacle")
                    path_score[1] -= 1000

            path_out = True
            for l in range(len(r_cpath_list)):
                if(r_cpath_list[l].get_contour().contains(point_obj)):
                    path_out = False
            if(path_out):
                path_score[1] -= 5
            for l in range(len(r_gpath_list)):
                if(r_gpath_list[l].get_contour().contains(point_obj)):
                    path_score[1] += 1
        marker2_pub.publish(marker_array)

        marker_array = MarkerArray()
        for i in range(len(path_3.poses)-1):
            dx = path_3.poses[i+1].pose.position.x - path_3.poses[i].pose.position.x
            dy = path_3.poses[i+1].pose.position.y - path_3.poses[i].pose.position.y
            theta = math.atan2(dy,dx)

            cx = (path_3.poses[i+1].pose.position.x + path_3.poses[i].pose.position.x)/2
            cy = (path_3.poses[i+1].pose.position.y + path_3.poses[i].pose.position.y)/2

            height = math.sqrt(dx**2+dy**2)
            r_gpath = RotatedRect(cx, cy, height, 1.5, theta)

            marker = Marker()
            marker.id = i
            marker.header.frame_id = "/map"
            marker.type = marker.LINE_STRIP
            marker.scale.x = 1
            marker.scale.y = 1
            marker.scale.z = 1
            marker.color.a = 1.0
            marker.pose.orientation.w = 1.0
            marker.lifetime = rospy.Duration(0.01)
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 0
            
            point_obj = shapely.geometry.Point(cx,cy)

            for k in range(len(r_gpath.get_contour().exterior.coords)):
              p = Point32()
              p.x = r_gpath.get_contour().exterior.coords[k][0]#math.cos(-euler[2])*(r1.get_contour().exterior.coords[k][0]- trans[0])-math.sin(-euler[2])*(r1.get_contour().exterior.coords[k][1]- trans[1])
              p.y = r_gpath.get_contour().exterior.coords[k][1]#math.sin(-euler[2])*(r1.get_contour().exterior.coords[k][0]- trans[0])+math.cos(-euler[2])*(r1.get_contour().exterior.coords[k][1]- trans[1])
              # print(k,p)
              marker.points.append(p)

            marker_array.markers.append(marker)  

            for box in bboxes:
                # x_ = math.cos(euler[2])*box.center.x-math.sin(euler[2])*box.center.y+trans[0]
                # y_ = math.sin(euler[2])*box.center.x+math.cos(euler[2])*box.center.y+trans[1]

                # dist = math.sqrt(math.pow((cx-x_),2) + math.pow((cy-y_),2))
                
                # if(dist < 0.5):
                #     path_score[0] -= 2
                #     print(dist)
                r2 = RotatedRect(math.cos(euler[2])*box.center.x-math.sin(euler[2])*box.center.y+trans[0], math.sin(euler[2])*box.center.x+math.cos(euler[2])*box.center.y+trans[1], box.size_y, box.size_x,euler[2]-math.pi/2)
                if(r_gpath.intersection(r2).area != 0):
                    # rospy.logwarn("3 is obstacle")
                    path_score[2] -= 1000

            path_out = True
            for l in range(len(r_cpath_list)):
                if(r_cpath_list[l].get_contour().contains(point_obj)):
                    path_out = False
            if(path_out):
                path_score[2] -= 5

            for l in range(len(r_gpath_list)):
                if(r_gpath_list[l].get_contour().contains(point_obj)):
                    path_score[2] += 0.1
        marker3_pub.publish(marker_array)

        path_score = np.array(path_score)
        max_score = np.argmax(path_score)
        rospy.logwarn(path_score)
        print(path_score)
        if(max_score == 0):
            finalpath_pub.publish(path_1)
        elif(max_score == 1):
            finalpath_pub.publish(path_2)
        else:
            finalpath_pub.publish(path_3)
        # for i in range(len(path_score)):
            # if(step_num > 170):
            #     if(not path_bool[1] and not path_bool[0]):
            #         finalpath_pub.publish(path_3)
            #     elif(not path_bool[1] and path_bool[0]):
            #         finalpath_pub.publish(path_1)
            #     else:
            #         finalpath_pub.publish(path_2)
            # else:
            #     if(not path_bool[1] and not path_bool[2]):
            #         finalpath_pub.publish(path_1)
            #     elif(not path_bool[1] and path_bool[2]):
            #         finalpath_pub.publish(path_3)
            #     else:
            #         finalpath_pub.publish(path_2)
            # print(step_num)


        r.sleep()



if __name__ == '__main__': 
  try:
    publish_obstacle_msg()
  except rospy.ROSInterruptException:
    pass