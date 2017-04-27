#pragma once

#include "arc_msgs/State.h"
#include "arc_tools/coordinate_transform.hpp"
#include "arc_tools/timing.hpp"
#include "ackermann_msgs/AckermannDrive.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "math.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <sys/time.h>
#include <algorithm>
#include <Eigen/Dense>

class MPC{

public:
	MPC(ros::NodeHandle* n, std::string PATH_NAME);
	~MPC();
	void stateCallback(const arc_msgs::State::ConstPtr& incoming_state);
	void obstacleCallback(const std_msgs::Float64::ConstPtr& msg);
	void guiStopCallback(const std_msgs::Bool::ConstPtr& msg);
	
	float distanceIJ(int from_i , int to_i );

	geometry_msgs::Vector3 indexOfDistanceFront(int i, float d);
	geometry_msgs::Vector3 indexOfDistanceBack(int i, float d);

	Eigen::Vector4d calculateParamFun(float lad);
	Eigen::MatrixXd pathToMatrix(float lad);
	void writeTxt(float lad);

	void readPathFromTxt(std::string inFileName);
	float curveRadius(int i);
	void findReferencePoints();
	geometry_msgs::Point pointAtDistanceLinear(int i, float distance);	//summs up linearly the distance to the points and returns the exact point at certain (summed up) distance

private:
	// 1. ROS setup.
	ros::NodeHandle* n_;
	// Publishers.
	ros::Publisher pub_stellgroessen_;
	// Subscribers.
	ros::Subscriber sub_state_;
	ros::Subscriber distance_to_obstacle_sub_;
	ros::Subscriber gui_stop_sub_;

	nav_msgs::Path path_;
	nav_msgs::Path path_diff_;
	std::vector<float> teach_vel_;
	std::vector<float> ref_xy_;
	int steps_in_horizon_;
	// Number of path points.
	int n_poses_path_;
	int slow_down_index_;
	//VectorPath
	Eigen::MatrixXd d_;
	// 3. Variables (will be changed during process.)
	// Variable which saves the incoming state from L&M.
	arc_msgs::State state_;
	float obstacle_distance_;
	ackermann_msgs::AckermannDrive u_;
	float tracking_error_;
	//easy access to absolute velocity
	float v_abs_;
	//Reference velocity array
	std::vector<float> v_ref_;
	//Shut down from graphical user interface
	bool gui_stop_;
	//Time
	arc_tools::Clock BigBen_;
	//
	std_msgs::Float32MultiArray pure_pursuit_gui_msg_;
	float rest_;	//rest when getting reference point (point is exact, but for next one you have to start from full integer index and otherwise would lose that distance to the next index you take)
	//Polynome coeffitients y=poly_a_*x³+poly_b_*x²+poly_c_*x+poly_d
	float poly_a_;
	float ploy_b_;
	float poly_c_;
	float poly_d_;
};
