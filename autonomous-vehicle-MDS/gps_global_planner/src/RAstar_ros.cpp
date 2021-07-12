/* iPath: A C++ Library of Intelligent Global Path Planners for Mobile Robots with ROS Integration. 

 * Website: http://www.iroboapp.org/index.php?title=IPath
 * Contact: 
 *
 * Copyright (c) 2014
 * Owners: Al-Imam University/King AbdulAziz Center for Science and Technology (KACST)/Prince Sultan University
 * All rights reserved.
 *
 * License Type: GNU GPL
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>

#include "RAstar_ros.h"

#include <pluginlib/class_list_macros.h>
//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(RAstar_planner::RAstarPlannerROS, nav_core::BaseGlobalPlanner)


int value;
int mapSize;
bool* OGM;
static const float INFINIT_COST = INT_MAX; //!< cost of non connected nodes
float infinity = std::numeric_limits< float >::infinity();
float tBreak;  // coefficient for breaking ties
ofstream MyExcelFile ("RA_result.xlsx", ios::trunc);

int clock_gettime(clockid_t clk_id, struct timespect *tp);

timespec diff(timespec start, timespec end)
{
  timespec temp;
	if ((end.tv_nsec-start.tv_nsec)<0) {
		temp.tv_sec = end.tv_sec-start.tv_sec-1;
		temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
	} else {
		temp.tv_sec = end.tv_sec-start.tv_sec;
		temp.tv_nsec = end.tv_nsec-start.tv_nsec;
	}
	return temp;
}

inline vector <int> findFreeNeighborCell (int CellID);

template<typename Out>
void split(const string &s, char delim, Out result) 
{
    stringstream ss(s);
    string item;
 
    while (getline(ss, item, delim)) 
        *(result++) = item;
}
 
vector<string> split(const string &s, const char delim)
{
    vector<string> elems;
    split(s, delim, back_inserter(elems));
 
    return elems;
}

namespace RAstar_planner
{

//Default Constructor
RAstarPlannerROS::RAstarPlannerROS()
{

}
RAstarPlannerROS::RAstarPlannerROS(ros::NodeHandle &nh)
{
  ROSNodeHandle = nh;
}

RAstarPlannerROS::RAstarPlannerROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  ROS_INFO("xxx");
  GPS_Path_sub = ROSNodeHandle.subscribe<nav_msgs::Path>("/global_path", 1, boost::bind(&RAstarPlannerROS::CB_GPS_path, this, _1));
  initialize(name, costmap_ros);
}

void RAstarPlannerROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  ROS_INFO("zzzzz");
  ros::NodeHandle private_nh("~/" + name);
  static ros::Subscriber costmapUpdateSubscriber = private_nh.subscribe<nav_msgs::Path>("/global_path", 1, boost::bind(&RAstarPlannerROS::CB_GPS_path, this, _1));
}

bool RAstarPlannerROS::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,std::vector<geometry_msgs::PoseStamped>& plan)
{
    for(int i=0; i < global_path_position.size() - 1; i++) {
      // global_path_position[i].pose.position.x -= 6763.45482065;
      // global_path_position[i].pose.position.y += 40939.2641813;
      for(int j=0; j < 10; j++) {
        geometry_msgs::PoseStamped new_goal = goal;
        new_goal.pose.position.x = global_path_position[i].pose.position.x + (global_path_position[i+1].pose.position.x - global_path_position[i].pose.position.x)*j/5;
        new_goal.pose.position.y = global_path_position[i].pose.position.y + (global_path_position[i+1].pose.position.y - global_path_position[i].pose.position.y)*j/5;
        new_goal.pose.orientation.x = 0;
        new_goal.pose.orientation.y = 0;
        new_goal.pose.orientation.z = 0;
        new_goal.pose.orientation.w = 1;
        // std::cout << "pose" << new_goal << std::endl;
        plan.push_back(new_goal);
      }
      // plan.push_back(global_path_position[i]);
    }
    cout << plan.size() << endl;
    // string in_line;
    // ifstream in("/home/plaif/global_path.txt");
    // double x_ = 0;
    // double y_ = 0;
    // while(getline(in,in_line)){
    //   vector<string> xy = split(in_line, ',');
    //   double x = stod(xy[0]);
    //   double y = stod(xy[1]);
    //   // cout<< x << y << x_ << y_ << endl;
      

    //   geometry_msgs::PoseStamped new_goal = goal;
    //   tf::Quaternion goal_quat = tf::createQuaternionFromYaw(atan2(x-x_,y-y_)+M_PI-M_PI/2 -M_PI/6);

    //   new_goal.pose.position.x = x;
    //   new_goal.pose.position.y = y;

    //   new_goal.pose.orientation.x = 0;
    //   new_goal.pose.orientation.y = 0;
    //   new_goal.pose.orientation.z = 0;
    //   new_goal.pose.orientation.w = 0;

    //   plan.push_back(new_goal);
    //   x_ = x;
    //   y_ = y;
    // }
    // in.close();
  //   plan.push_back(start);
  //  for (int i=0; i<20; i++){
     
  //    geometry_msgs::PoseStamped new_goal = goal;
  //    tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);

  //     new_goal.pose.position.x = -2.5+(0.05*i);
  //     new_goal.pose.position.y = -3.5+(0.05*i);

  //     new_goal.pose.orientation.x = goal_quat.x();
  //     new_goal.pose.orientation.y = goal_quat.y();
  //     new_goal.pose.orientation.z = goal_quat.z();
  //     new_goal.pose.orientation.w = goal_quat.w();

  //     plan.push_back(new_goal);
  //  }
  //  plan.push_back(goal);
  return true;
 
}

void RAstarPlannerROS::CB_GPS_path(const nav_msgs::Path::ConstPtr& global_path_) {
  global_path_position = global_path_->poses;
  // cout << global_path_position.size() << endl;
  // ROS_INFO("ok");
}
 
}
