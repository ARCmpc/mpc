#include "../include/arc_mpc/arc_mpc.hpp"

float SLOW_DOWN_DISTANCE=10; 
int QUEUE_LENGTH;
std::string STELLGROESSEN_TOPIC;
std::string TRACKING_ERROR_TOPIC;
std::string NAVIGATION_INFO_TOPIC;
std::string STATE_TOPIC;
std::string OBSTACLE_DISTANCE_TOPIC;
std::string SHUTDOWN_TOPIC;
std::string PATH_NAME_EDITED;

MPC::MPC(ros::NodeHandle* n, std::string PATH_NAME)
{
	n->getParam("/control/SLOW_DOWN_DISTANCE",SLOW_DOWN_DISTANCE);
	n->getParam("/topic/STELLGROESSEN",STELLGROESSEN_TOPIC);
	n->getParam("/topic/TRACKING_ERROR",TRACKING_ERROR_TOPIC);
	n->getParam("/topic/NAVIGATION_INFO",NAVIGATION_INFO_TOPIC);
	n->getParam("/topic/STATE",STATE_TOPIC);
	n->getParam("/topic/OBSTACLE_DISTANCE",OBSTACLE_DISTANCE_TOPIC);
	n->getParam("/topic/SHUTDOWN",SHUTDOWN_TOPIC);
	n->getParam("/general/QUEUE_LENGTH",QUEUE_LENGTH );
	PATH_NAME_EDITED =PATH_NAME + "_teach.txt";

	n_ = n;
	readPathFromTxt(PATH_NAME_EDITED);
	//Publisher
	pub_stellgroessen_ = n_->advertise<ackermann_msgs::AckermannDrive>(STELLGROESSEN_TOPIC, QUEUE_LENGTH);
	//Subscriber
	sub_state_ = n_->subscribe(STATE_TOPIC, QUEUE_LENGTH, &MPC::stateCallback,this);
	distance_to_obstacle_sub_=n_->subscribe(OBSTACLE_DISTANCE_TOPIC, QUEUE_LENGTH ,&MPC::obstacleCallback,this);
	gui_stop_sub_=n_->subscribe(SHUTDOWN_TOPIC, QUEUE_LENGTH ,&MPC::guiStopCallback,this);
	std::cout << std::endl << "MPC: Consturctor init, path lenght: " <<n_poses_path_<< " and slow_down_index: "<<slow_down_index_<<std::endl;
	pathToVector(); //useful vor Eigen
	calculateParamFun(d_);
	
}

void MPC::stateCallback(const arc_msgs::State::ConstPtr& incoming_state)
{
	state_ = *incoming_state;
}

void MPC::obstacleCallback(const std_msgs::Float64::ConstPtr& msg)
{
	obstacle_distance_=msg->data;
}

void MPC::guiStopCallback(const std_msgs::Bool::ConstPtr& msg)
{
	gui_stop_=msg->data;
	BigBen_.start();
}

float* MPC::findReferencePoints()
{
	
}
void MPC::readPathFromTxt(std::string inFileName)
{
	// Create an ifstream object.
	std::fstream fin;
	fin.open(inFileName.c_str());
	
	// Check if stream is open.
	if (!fin.is_open())
	{
		std::cout << "MPC: Error with opening of  " <<inFileName << std::endl;
	}

	// Truncate two lines of the file to get rid of the last '|'.
	fin.seekg (-2, fin.end);
	int length = fin.tellg();
	fin.seekg (0, fin.beg);
	//Stream erstellen mit chars von fin.
	char *file = new char [length];
	fin.read(file, length);
	std::istringstream stream(file, std::ios::in);
	delete[] file;
	fin.close();

	int i = 0;
	int j;
	geometry_msgs::PoseStamped temp_pose;

	// Save to path_ variable.
	while(!stream.eof() && i<length)
	{
		geometry_msgs::PoseStamped temp_pose;
		float temp_diff;
		path_.poses.push_back(temp_pose);
		teach_vel_.push_back(temp_diff);
		path_diff_.poses.push_back(temp_pose);
		stream>>j;
		stream>>path_.poses[j-1].pose.position.x;
		stream>>path_.poses[j-1].pose.position.y;
		stream>>path_.poses[j-1].pose.position.z;
		//Save orientation
		stream>>path_.poses[j-1].pose.orientation.x;
		stream>>path_.poses[j-1].pose.orientation.y;
		stream>>path_.poses[j-1].pose.orientation.z;
		stream>>path_.poses[j-1].pose.orientation.w;
		//Save teach_velocity
		stream>>teach_vel_[j-1];//path_diff_.poses[j-1].pose.position.x;
						//stream>>path_diff_.poses[j-1].pose.position.y;
						//stream>>path_diff_.poses[j-1].pose.position.z;

		stream.ignore (300, '|');
		i++;
	}
	n_poses_path_ = i;
	slow_down_index_=indexOfDistanceBack(i-1,SLOW_DOWN_DISTANCE);

}

float MPC::distanceIJ(int from_i , int to_i )
{	
	float d=0;
	for (int i =from_i; i<to_i; i++)
	{
		d += sqrt(	pow(path_.poses[i].pose.position.x - path_.poses[i+1].pose.position.x,2)+
				pow(path_.poses[i].pose.position.y - path_.poses[i+1].pose.position.y,2)+
				pow(path_.poses[i].pose.position.z - path_.poses[i+1].pose.position.z,2));
		if((i+1)>n_poses_path_-1){std::cout<<"MPC: LAUFZEITFEHLER distanceIJ"<<std::endl;}
	}
	return d;

}

void MPC::pathToVector()  //Let's see
{
	Eigen::MatrixXd d(2,n_poses_path_);
	for (int i=0; i<n_poses_path_; i++){
	d(0,i) = path_.poses[i].pose.position.x;
	d(1,i) = path_.poses[i].pose.position.y;	
}
	Eigen::MatrixXd d_ = d;
}

void MPC::calculateParamFun(Eigen::MatrixXd a)
{
	Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3,3);
	A << n_poses_path_, d_.row(0).sum(), d_.row(0).cwiseAbs2().sum(), d_.row(0).sum(), d_.row(0).cwiseAbs2().sum(), d_.row(0).cwiseAbs2().cwiseProduct(d_.row(0)).sum(), d_.row(0).cwiseAbs2().sum(), d_.row(0).cwiseAbs2().cwiseProduct(d_.row(0)).sum(), d_.row(0).cwiseAbs2().cwiseProduct(d_.row(0).cwiseAbs2()).sum();
//	std::cout << A << std::endl;

	Eigen::VectorXd rhs(3); 
	rhs << d_.row(1).sum(), d_.row(1).dot(d_.row(0)), d_.row(1).dot(d_.row(0).cwiseAbs2());
//	std::cout << rhs << std::endl;

	std::cout << A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(rhs) << std::endl; //!! Nico -> remember to put the coeff as private variables. 


//	std::cout << d(1,0);
	//fitting points path_.poses[i].position.x and path_.poses[i].position.y
//	Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3,3);
//	A << n, path_.poses[i].pose.position.x.segment(i, n);
//	
//	while(l<d &&j<n_poses_path_-1)
//	{	
//	
//	indexOfDistanceFront(j-1, 4); example with distance 4 meters
//	
//	}
}


int MPC::indexOfDistanceFront(int i, float d)
{
	int j=i;
	float l = 0;
	while(l<d &&j<n_poses_path_-1)
	{
		l += sqrt(	pow(path_.poses[j+1].pose.position.x - path_.poses[j].pose.position.x,2)+
				pow(path_.poses[j+1].pose.position.y - path_.poses[j].pose.position.y,2)+
				pow(path_.poses[j+1].pose.position.z - path_.poses[j].pose.position.z,2));
		if(j+1>n_poses_path_-1){std::cout<<"MPC: LAUFZEITFEHLER::indexOfDistanceFront"<<std::endl;}
		j ++;
	}
	return j+1;
}

int MPC::indexOfDistanceBack(int i, float d)
{
	int j=i;
	float l = 0;
	while(l<d && j>0)
	{
		l += sqrt(	pow(path_.poses[j-1].pose.position.x - path_.poses[j].pose.position.x,2)+
				pow(path_.poses[j-1].pose.position.y - path_.poses[j].pose.position.y,2)+
				pow(path_.poses[j-1].pose.position.z - path_.poses[j].pose.position.z,2));
		if(j>n_poses_path_-1){std::cout<<"MPC: LAUFZEITFEHLER indexOfDistanceBack"<<std::endl;}
		j --;
	}
	return j;
}
