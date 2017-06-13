#pragma once

#include "arc_msgs/State.h"
#include "arc_tools/coordinate_transform.hpp"
#include "arc_tools/timing.hpp"
#include "ackermann_msgs/AckermannDrive.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"
#include "math.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/OccupancyGrid.h"
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
#include <../arc_solver/arc_solver/include/arc_solver.h>
#include <../arc_solver/arc_solver_casadi2forces.c>
#include <../alglib/src/interpolation.h>
#include <../alglib/src/stdafx.h>
#include <stdlib.h>
#include <stdio.h>
class MPC{
	
private:
	// 1. ROS setup.
	ros::NodeHandle* n_;
	// Publishers.
	ros::Publisher pub_stellgroessen_;
	ros::Publisher pub_output_1_;
	ros::Publisher pub_output_2_;
	ros::Publisher pub_clustered_grid_;
	// Subscribers.
	ros::Subscriber sub_state_;
	ros::Subscriber distance_to_obstacle_sub_;
	ros::Subscriber gui_stop_sub_;
	ros::Subscriber sub_state_matlab_;
	ros::Subscriber grid_map_sub_;

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
	//Spline
	alglib::spline1dinterpolant c_x_;
	alglib::spline1dinterpolant c_y_;
	alglib::real_1d_array t_;
	alglib::real_1d_array x_;
	alglib::real_1d_array y_;
	//Simulation 
	std_msgs::Float32MultiArray matlab_output_1_;
	std_msgs::Float32MultiArray matlab_output_2_;
	//Solver
	bool first_flag_;
	arc_solver_params solver_param_;
	arc_solver_output solver_output_;
	arc_solver_info solver_info_;
	FILE fs_;
	arc_solver_ExtFunc pt2Function;
	//Clustering
	nav_msgs::OccupancyGrid obstacle_map_;
	nav_msgs::OccupancyGrid clustered_map_;
	int grid_width_;
	int grid_height_;
	float grid_resolution_;
	struct cluster
	{
		bool flag;		//tells if cluster is empty (0) or not (1)
		std::vector<int> body;	//contains grid index
		std::vector<int> temp;  //when building cluster fills with cell idices to be compared 
		float x_center;
		float y_center;
		float radius;
	};
	std::vector<int> all_cells_;	
	cluster cluster_1_;
	cluster cluster_2_;
	cluster cluster_3_;
	cluster cluster_4_;
	cluster cluster_5_;


public:
	MPC(ros::NodeHandle* n, std::string PATH_NAME, std::string MODE);
	~MPC();
	void stateCallback(const arc_msgs::State::ConstPtr& incoming_state);
	void obstacleCallback(const std_msgs::Float64::ConstPtr& msg);
	void guiStopCallback(const std_msgs::Bool::ConstPtr& msg);
	void stateMatlabCallback(const geometry_msgs::Quaternion::ConstPtr& incoming_state);
	void gridmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& grid_map);
	float distanceIJ(int from_i , int to_i );

	geometry_msgs::Vector3 indexOfDistanceFront(int i, float d);
	geometry_msgs::Vector3 indexOfDistanceBack(int i, float d);

	void calculateParamFun(float lad_interpolation);
	void generateSpline(float lad_interpolation);
	Eigen::MatrixXd pathToMatrix(float lad);
	void writeTxt(); 
	float yPoly(float x);
	float radiusPoly(float x);
	void findReferencePointsLinear();
	void findReferencePointsPoly();
	void findReferencePointsSpline();
	float nextReferenceXPolynomial(float  x_start, float step);
	void setSolverParam();
	float costWeight(int i);
	void getOutputAndReact();
	float vRef(int index);
	float vRef(geometry_msgs::Point local, int i_start, int i_end);
	int localPointToPathIndex(geometry_msgs::Point p, int i_start, int i_end);
	void readPathFromTxt(std::string inFileName);
	float curvatureSpline(float t);

	geometry_msgs::Point localToGlobal(geometry_msgs::Point p_local, arc_msgs::State state_);
	geometry_msgs::Point pointAtDistanceLinear(int i, float distance);	//summs up linearly the distance to the points and returns the exact point at certain (summed up) distance
	float linearInterpolation(float a_lower, float a_upper ,float b_lower, float b_upper, float b_middle);

	void clustering();
	void fillCluster(cluster &actual_cluster);
	void enframeCluster(cluster &actual_cluster);
	void emptyCluster(cluster &actual_cluster);
	void inflateClusterMap(cluster c);
	int convertIndex(int x, int y);
	geometry_msgs::Vector3 convertIndex(const int i);
	float distanceCells(int i_1, int i_2);
};
