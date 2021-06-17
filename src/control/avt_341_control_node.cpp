/**
 * \file avt_341_control_node.cpp
 *
 * ROS node to subsribe to a trajectory message and 
 * convert it to a driving command using the pure-pursuit algorithm
 * 
 * \author Chris Goodin
 *
 * \contact cgoodin@cavs.msstate.edu
 * 
 * \date 7/13/2018
 */

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
//Naisense includes
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "avt_341/avt_341_utils.h"
//avt_341 includes
#include "avt_341/control/pure_pursuit_controller.h"

nav_msgs::Path control_msg;
nav_msgs::Odometry state;

void OdometryCallback(const nav_msgs::Odometry::ConstPtr& rcv_state) {
	state = *rcv_state; 
}

geometry_msgs::PoseStamped transform;
std_msgs::Float64 speed;

void TransformCallback(const geometry_msgs::PoseStamped::ConstPtr& rcv_tf) {
	transform = *rcv_tf;
}

void SpeedCallback(const std_msgs::Float64& rcv_speed) {
	speed = rcv_speed;
}

void PathCallback(const nav_msgs::Path::ConstPtr& rcv_control){
  control_msg.poses = rcv_control->poses;
  control_msg.header = rcv_control->header;
}

int main(int argc, char *argv[]){
  ros::init(argc,argv,"avt_341_control_node");
  ros::NodeHandle n;

  ros::Publisher dc_pub =
    n.advertise<geometry_msgs::Twist>("avt_341/cmd_vel",1);

  ros::Subscriber path_sub =
    n.subscribe("avt_341/local_path",1, PathCallback);

  //ros::Subscriber state_sub =
  //  n.subscribe("avt_341/odometry",1, OdometryCallback);
    
  // extra publishers, and subscribers naisense
  ros::Subscriber transform_sub =
    n.subscribe("api_interface/transform",1, TransformCallback);
  
  ros::Subscriber speed_sub =
    n.subscribe("api_interface/speed",1, SpeedCallback);
  
  ros::Publisher odom_pub =
    n.advertise<nav_msgs::Odometry>("avt_341/odometry",1);
   
  ros::Publisher waypoint_pub =
    n.advertise<geometry_msgs::Vector3>("api_interface/waypoint",1);
    
  ros::Publisher brake_pub =
    n.advertise<std_msgs::Float64>("api_interface/brake_api",1);
   
  ros::Publisher throttle_pub =
    n.advertise<std_msgs::Float64>("api_interface/throttle_api",1);

  ros::Publisher steering_pub =
    n.advertise<std_msgs::Float64>("api_interface/steering_api",1);
 
  ros::Publisher engine_running_pub =
    n.advertise<std_msgs::Bool>("api_interface/engine_running_api",1);

  ros::Publisher api_controlled_pub =
    n.advertise<std_msgs::Bool>("api_interface/isapicontrolled",1);
    
  ros::Publisher gear_pub =
    n.advertise<std_msgs::Int32>("api_interface/gear_api",1);
  
  avt_341::control::PurePursuitController controller;
	// Set controller parameters
	float wheelbase = 2.6f;
	if (ros::param::has("~vehicle_wheelbase")) {
		ros::param::get("~vehicle_wheelbase", wheelbase);
	}
	float steer_angle = 25.0f;
	if (ros::param::has("~vehicle_max_steer_angle_degrees")) {
		ros::param::get("~vehicle_max_steer_angle_degrees", steer_angle);
	}
	float vehicle_speed = 5.0f;
	if (ros::param::has("~vehicle_speed")) {
		ros::param::get("~vehicle_speed", vehicle_speed);
	}
  if (ros::param::has("~steering_coefficient")) {
    float steering_coeff;
		ros::param::get("~steering_coefficient", steering_coeff);
    controller.SetSteeringParam(steering_coeff);
	}
  
	controller.SetWheelbase(wheelbase);
	controller.SetMaxSteering(steer_angle*3.14159 / 180.0);
	controller.SetDesiredSpeed(vehicle_speed);
	//naisense
	controller.SetMaxStableSpeed(vehicle_speed);

  float rate = 50.0f;
  float dt = 1.0f/rate;
  ros::Rate r(rate);

  while (ros::ok()){
    geometry_msgs::Twist dc;
    
    nav_msgs::Odometry na_state;
    na_state.pose.pose.position.x = transform.pose.position.x;
    na_state.pose.pose.position.y = transform.pose.position.y;
    na_state.pose.pose.orientation = transform.pose.orientation;
    na_state.twist.twist.linear.x = speed.data / 3.6;
    na_state.header = transform.header;
    odom_pub.publish(na_state);
    //controller.SetVehicleState(state);
    
    controller.SetVehicleState(na_state);
    
    dc = controller.GetDcFromTraj(control_msg);

    dc_pub.publish(dc);
    
    avt_341::utils::vec2 goal = controller.GetLatestGoal();
    geometry_msgs::Vector3 waypoint_msg;
    waypoint_msg.x = goal.x;
    waypoint_msg.y = goal.y;
    
    waypoint_pub.publish(waypoint_msg);
    
    //Naisense
    std_msgs::Float64 throttle_msg, brake_msg, steering_msg;
    std_msgs::Int32 gear_msg;
    std_msgs::Bool is_api_controlled_msg, engine_running_msg;
    
    throttle_msg.data = dc.linear.x;
    brake_msg.data = dc.linear.y;
    steering_msg.data = -dc.angular.z; // negate steering?
    is_api_controlled_msg.data = true;
    engine_running_msg.data = true;
    gear_msg.data = 6;
    
    throttle_pub.publish(throttle_msg);
    brake_pub.publish(brake_msg);
    steering_pub.publish(steering_msg);
    engine_running_pub.publish(engine_running_msg);
    api_controlled_pub.publish(is_api_controlled_msg);
    gear_pub.publish(gear_msg);
    
    //std::cout << "published controls: t = " << dc.linear.x  << " b = " << dc.linear.y << " a = " << dc.angular.z << std::endl;
    
    ros::spinOnce();

    r.sleep();
  }

  return 0;
}
