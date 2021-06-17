/**
 * \file avt_341_global_follower_path_node.cpp
 * 
 * ROS Node that publishes follower global path from lead vehicle odometry.
 * 
 * \author Mahyar Bayran, Chris Goodin
 *
 * \contact mahyar.bayran@naisense.ai, cgoodin@cavs.msstate.edu
 * 
 * \date 04/09/2021
 */

// ros includes
#include "ros/ros.h"
#include <nav_msgs/Path.h>
#include "nav_msgs/Odometry.h"
// local includes
#include "avt_341/avt_341_utils.h"

nav_msgs::Odometry odom, lead_odom;
bool odom_rcvd = false, lead_odom_rcvd = false;
void OdometryCallback(const nav_msgs::Odometry::ConstPtr &rcv_odom){
  odom = *rcv_odom;
  odom_rcvd = true;
}

void LeadOdomteryCallback(const nav_msgs::Odometry::ConstPtr &rcv_odom){
  lead_odom = *rcv_odom;
  lead_odom_rcvd = true;
}

int main(int argc, char *argv[]){
  ros::init(argc, argv, "avt_341_global_follower_path_node");
  ros::NodeHandle n;

  ros::Publisher path_pub = n.advertise<nav_msgs::Path>("avt_341/global_path", 10);
  ros::Subscriber odometry_sub = n.subscribe("avt_341/odometry", 10, OdometryCallback);
  
  // subscribe to lead vehicle 
  ros::Subscriber lead_odometry_sub = n.subscribe("/fed_alpha/avt_341/odometry", 10, LeadOdomteryCallback);

  float goal_dist = 3.0f;
  if (ros::param::has("~goal_dist")){
    ros::param::get("~goal_dist", goal_dist);
   }
 
	
  std::vector<double> waypoints_x_list, waypoints_y_list;
  if (!(n.hasParam("/waypoints_x") && "/waypoints_y")){
    std::cerr<<"ERROR: NO WAYPOINTS WERE PROVIDED, EXITING."<<std::endl;
    return 1;
  }
  n.getParam("/waypoints_x", waypoints_x_list);
  n.getParam("/waypoints_y", waypoints_y_list);
  
  if (waypoints_x_list.size()!=waypoints_y_list.size()){
    std::cerr<<"WARNING: "<<waypoints_x_list.size()<<" X COORDINATES WERE PROVIDED FOR "<<waypoints_y_list.size()<<" Y COORDINATES."<<std::endl;
  }
  if (waypoints_x_list.size()==0 || waypoints_y_list.size()==0){
    std::cerr<<"ERROR: NO WAYPOINTS WERE LISTED IN /waypoints_x OR /waypoints_y, EXITING."<<std::endl;
    return 2;
  }
  
  /*  nav_msgs::Path ros_path;
  ros_path.header.frame_id = "odom"; 
  ros_path.poses.clear();
  float last_x = 1000000.0f;
  float last_y = 1000000.0f;*/
  
  int num_waypoints = std::min(waypoints_x_list.size(),waypoints_y_list.size());
  nav_msgs::Path ros_path;
  ros_path.header.frame_id = "odom"; 
  ros_path.poses.clear();
  float last_x = 0.0f;
  float last_y = 0.0f;
  for (int32_t i=0;i<4;i++){
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = static_cast<float>(waypoints_x_list[i]);
    pose.pose.position.y = static_cast<float>(waypoints_y_list[i]);
    pose.pose.position.z = 0.0f;
    pose.pose.orientation.w = 1.0f;
    pose.pose.orientation.x = 0.0f;
    pose.pose.orientation.y = 0.0f;
    pose.pose.orientation.z = 0.0f;
    ros_path.poses.push_back(pose);
  }


  ros::Rate r(50.0f); // Hz
  bool goal_reached = false;
  int nl = 0;
  
  bool path_empty = true;
  int numPathPoses = 0;
  lead_odom_rcvd = false;
  odom_rcvd = false;
  geometry_msgs::PoseStamped lastpose;
  
  while (ros::ok() && !goal_reached){
    
    if (lead_odom_rcvd)
    {
      geometry_msgs::PoseStamped pose;
      pose.pose = lead_odom.pose.pose;
     
      if (pose.pose.position.x > 1 && pose.pose.position.y < -1)
      {
	      if (path_empty)
	      {
		ros_path.poses.push_back(pose);
		lastpose = pose;
		last_x = pose.pose.position.x;
		last_y = pose.pose.position.y;
	      	path_empty = false;
	      	numPathPoses++;
	      }
	      else
	      {
		avt_341::utils::vec2 dif(pose.pose.position.x - lastpose.pose.position.x, pose.pose.position.y - lastpose.pose.position.y);
	      	if (avt_341::utils::length(dif) > 1)
	      	{
		  ros_path.poses.push_back(pose);
		  lastpose = pose;
		  last_x = pose.pose.position.x;
		  last_y = pose.pose.position.y;
		  numPathPoses++;
		}
	      }
	      
	      lead_odom_rcvd = false;
      }
    }
    
    if (!path_empty)
    {
      ros_path.header.stamp = ros::Time::now();
      ros_path.header.seq += 1; 

      for (int i=0;i<ros_path.poses.size();i++){
        ros_path.poses[i].header = ros_path.header; 
      }

      //if (numPathPoses > 1)
      //{
      path_pub.publish(ros_path);
      //}
    }
    
    if (odom_rcvd && numPathPoses > 2){
      float dx = last_x - odom.pose.pose.position.x;
      float dy = last_y - odom.pose.pose.position.y;
      double d = sqrt(dx*dx + dy*dy);
      if (nl%100==0){ //update every 2 seconds
        std::cout<<"Distance to goal = "<<d<<std::endl;
        nl = 0;
      }
      if (d<goal_dist){
        goal_reached = true;
        std::cout << "goal reached! " << last_x << ", " << last_y << std::endl;
      }
    }

    ros::spinOnce();
    r.sleep();
    nl++;
  }

  return 0;
}
