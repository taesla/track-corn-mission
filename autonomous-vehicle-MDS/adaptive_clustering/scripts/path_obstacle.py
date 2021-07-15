#!/usr/bin/env python
# -*- coding: utf-8 -*-

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
import shapely.geometry
import shapely.affinity


bboxes = []
global_path = Path()
euler = []

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

def publish_obstacle_msg():
  pub = rospy.Publisher('/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=1)
  marker_pub = rospy.Publisher('visualization_marker', MarkerArray, queue_size = 100)
  bool_pub = rospy.Publisher('/adaptive_clustering/is_bool',Bool,queue_size = 1)
  #pub = rospy.Publisher('/p3dx/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=1)
  bboxes_sub = rospy.Subscriber('/adaptive_clustering/bboxes_2d',Bboxes2d,bboxes_callback)
  path_sub = rospy.Subscriber('/global_path',Path,path_callback)
  step_sub = rospy.Subscriber('current_step', PoseStamped, step_callback, queue_size=1)
  
  rospy.init_node("path_obstacle")


  obstacle_msg = ObstacleArrayMsg() 
  obstacle_msg.header.stamp = rospy.Time.now()
  obstacle_msg.header.frame_id = "map" # CHANGE HERE: odom/map

  listener = tf.TransformListener()
  is_bool = Bool()
  is_bool.data = False
  box_bool = False
  is_bool_num = 0
  
  global_path_box_0 = 0
  global_path_box_5 = 0

  r_gpath_list = [0,0,0,0,0,0]
  r_cpath_list = [0,0,0,0,0,0]
  
  r = rospy.Rate(144) # 10hz

  while not rospy.is_shutdown():
    try:
        (trans,rot) = listener.lookupTransform('/map', '/velodyne', rospy.Time(0))
        euler = tf.transformations.euler_from_quaternion(rot)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue

    marker_array = MarkerArray()
    box_num = 0
    if(len(global_path.poses) > 4):
      for i in range(0,6):
        dx = global_path.poses[i+1].pose.position.x - global_path.poses[i].pose.position.x
        dy = global_path.poses[i+1].pose.position.y - global_path.poses[i].pose.position.y
        theta = math.atan2(dy,dx)

        cx = (global_path.poses[i+1].pose.position.x + global_path.poses[i].pose.position.x)/2
        cy = (global_path.poses[i+1].pose.position.y + global_path.poses[i].pose.position.y)/2

        height = math.sqrt(dx**2+dy**2)

        r_gpath = RotatedRect(cx, cy, height, 3, theta) # global path를 중심으로한 영역
        r_gpath_list[i] = r_gpath

        if(step_num < 15): # 동적장애물 step 체크
          cx_ = (r_gpath.get_contour().exterior.coords[1][0] + r_gpath.get_contour().exterior.coords[2][0])/2
          cy_ = (r_gpath.get_contour().exterior.coords[1][1] + r_gpath.get_contour().exterior.coords[2][1])/2
        else:
          cx_ = (r_gpath.get_contour().exterior.coords[0][0] + r_gpath.get_contour().exterior.coords[3][0])/2
          cy_ = (r_gpath.get_contour().exterior.coords[0][1] + r_gpath.get_contour().exterior.coords[3][1])/2          

        r_cpath = RotatedRect(cx_, cy_, height, 8.5, theta) # 도로 중심선을 중심으로한 영역       
        r_cpath_list[i] = r_cpath

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

        for k in range(len(r_cpath.get_contour().exterior.coords)): # marker 표시용
          p = Point32()
          p.x = r_cpath.get_contour().exterior.coords[k][0]#math.cos(-euler[2])*(r1.get_contour().exterior.coords[k][0]- trans[0])-math.sin(-euler[2])*(r1.get_contour().exterior.coords[k][1]- trans[1])
          p.y = r_cpath.get_contour().exterior.coords[k][1]#math.sin(-euler[2])*(r1.get_contour().exterior.coords[k][0]- trans[0])+math.cos(-euler[2])*(r1.get_contour().exterior.coords[k][1]- trans[1])
          marker.points.append(p)

        marker_array.markers.append(marker)  

        if(i == 0): # 첫번째 r_cpath
          global_path_box_0 = r_cpath
        if(i == 4): # 마지막 r_cpath
          global_path_box_5 = r_cpath

    for box in bboxes:
      if(len(global_path.poses) > 4):
        
        for j in range(0 , 6):
          r_gpath = r_gpath_list[j]

          cx_obj = math.cos(euler[2])*box.center.x-math.sin(euler[2])*box.center.y+trans[0]
          cy_obj = math.sin(euler[2])*box.center.x+math.cos(euler[2])*box.center.y+trans[1]
          # r_obj = RotatedRect(cx_obj, cy_obj, box.size_y, box.size_x,euler[2]-math.pi/2)
          point_obj = shapely.geometry.Point(cx_obj,cy_obj) # clustering된 object의 중심점

          r_cpath = r_cpath_list[j]
          r_gpath = r_gpath_list[j]
          if(step_num <= 15 and step_num >= 5):
            if(r_gpath.get_contour().contains(point_obj)):
              is_bool.data = True
              box_bool = True
          else:
            if(r_cpath.get_contour().contains(point_obj)):
              is_bool.data = True
              box_bool = True            

        if(box_bool):
          obstacle_msg.obstacles.append(ObstacleMsg())
          obstacle_msg.obstacles[box_num].id = box_num
          v1 = Point32()
          v1.x = box.center.x - box.size_x/2
          v1.y = box.center.y - box.size_y/2
          v2 = Point32()
          v2.x = box.center.x - box.size_x/2
          v2.y = box.center.y + box.size_y/2
          v3 = Point32()
          v3.x = box.center.x + box.size_x/2
          v3.y = box.center.y + box.size_y/2
          v4 = Point32()
          v4.x = box.center.x + box.size_x/2
          v4.y = box.center.y - box.size_y/2
          obstacle_msg.obstacles[box_num].polygon.points = [v1, v2, v3, v4]

          box_num += 1
          box_bool = False

    if(step_num > 15): # 낭떨어지 및 버스 전용 도로 방지
      obstacle_msg.obstacles.append(ObstacleMsg())
      obstacle_msg.obstacles[box_num].id = box_num
      v1 = Point32()
      v1.x = math.cos(-euler[2])*(global_path_box_0.get_contour().exterior.coords[2][0]- trans[0])-math.sin(-euler[2])*(global_path_box_0.get_contour().exterior.coords[2][1]- trans[1])
      v1.y = math.sin(-euler[2])*(global_path_box_0.get_contour().exterior.coords[2][0]- trans[0])+math.cos(-euler[2])*(global_path_box_0.get_contour().exterior.coords[2][1]- trans[1])
      v2 = Point32()
      v2.x = math.cos(-euler[2])*(global_path_box_5.get_contour().exterior.coords[1][0]- trans[0])-math.sin(-euler[2])*(global_path_box_5.get_contour().exterior.coords[1][1]- trans[1])
      v2.y = math.sin(-euler[2])*(global_path_box_5.get_contour().exterior.coords[1][0]- trans[0])+math.cos(-euler[2])*(global_path_box_5.get_contour().exterior.coords[1][1]- trans[1])
      obstacle_msg.obstacles[box_num].polygon.points = [v1, v2]
      box_num += 1
    
    else:
      obstacle_msg.obstacles.append(ObstacleMsg())
      obstacle_msg.obstacles[box_num].id = box_num
      v3 = Point32()
      v3.x = math.cos(-euler[2])*(global_path_box_0.get_contour().exterior.coords[3][0]- trans[0])-math.sin(-euler[2])*(global_path_box_0.get_contour().exterior.coords[3][1]- trans[1])
      v3.y = math.sin(-euler[2])*(global_path_box_0.get_contour().exterior.coords[3][0]- trans[0])+math.cos(-euler[2])*(global_path_box_0.get_contour().exterior.coords[3][1]- trans[1])
      v4 = Point32()
      v4.x = math.cos(-euler[2])*(global_path_box_5.get_contour().exterior.coords[0][0]- trans[0])-math.sin(-euler[2])*(global_path_box_5.get_contour().exterior.coords[0][1]- trans[1])
      v4.y = math.sin(-euler[2])*(global_path_box_5.get_contour().exterior.coords[0][0]- trans[0])+math.cos(-euler[2])*(global_path_box_5.get_contour().exterior.coords[0][1]- trans[1])
      v3_ = Point32()
      v3_.x = v3.x + 0.01
      v3_.y = v3.y + 0.01
      v4_ = Point32()
      v4_.x = v4.x + 0.01
      v4_.y = v4.y + 0.01
      obstacle_msg.obstacles[box_num].polygon.points = [v3, v4,v3_,v4_]
      box_num += 1


    marker_pub.publish(marker_array)
    # if(is_bool_num > 2):
    #   is_bool.data = True
    #   is_bool_num = 0
    # else:
    #   is_bool.data = False
    bool_pub.publish(is_bool)
    is_bool.data = False
    pub.publish(obstacle_msg)
    print("obs:",len(obstacle_msg.obstacles))
    obstacle_msg = ObstacleArrayMsg() 
    obstacle_msg.header.stamp = rospy.Time.now()
    obstacle_msg.header.frame_id = "velodyne" # CHANGE HERE: odom/map
    print("num",step_num)
    print("len",len(global_path.poses))
    # print("ok")
    r.sleep()



if __name__ == '__main__': 
  try:
    publish_obstacle_msg()
  except rospy.ROSInterruptException:
    pass