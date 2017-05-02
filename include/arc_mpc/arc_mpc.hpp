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
#include "../arc_solver/arc_solver/include/arc_solver.h"

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

	void calculateParamFun(float lad_interpolation);
	Eigen::MatrixXd pathToMatrix(float lad);
	void writeTxt(float lad); 
	float yPoly(float x);
	void findReferencePointsLinear();
	void findReferencePointsPoly();
	float nextReferenceXPolynomial(float  x_start, float step);
	void setSolverParam();
	void getOutputAndReact();
	float vRef(int index);
	float vRef(geometry_msgs::Point local, int i_start, int i_end);
	int localPointToPathIndex(geometry_msgs::Point p, int i_start, int i_end);
	void readPathFromTxt(std::string inFileName);
	float curveRadius(int i);

	geometry_msgs::Point localToGlobal(geometry_msgs::Point p_local, arc_msgs::State state_);
	geometry_msgs::Point pointAtDistanceLinear(int i, float distance);	//summs up linearly the distance to the points and returns the exact point at certain (summed up) distance
	float linearInterpolation(float a_lower, float a_upper ,float b_lower, float b_upper, float b_middle);

	
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
	std::vector<float> ref_x_;
	std::vector<float> ref_y_;
	std::vector<float> ref_v_;
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
	float rest_linear_interpolation_;	//rest when getting reference point (point is exact, but for next one you have to start from full integer index and otherwise would lose that distance to the next index you take)
	//Polynome coeffitients y=poly_a_*x³+poly_b_*x²+poly_c_*x+poly_d_
	float poly_a_;
	float poly_b_;
	float poly_c_;
	float poly_d_;
	//Solver
	arc_solver_params solver_param_;
	arc_solver_output solver_output_;
	arc_solver_info solver_info_;
	FILE fs_;
	arc_solver_ExtFunc arc_solver_evalExtFunctions_;

};
