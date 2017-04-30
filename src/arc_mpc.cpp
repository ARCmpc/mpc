#include "../include/arc_mpc/arc_mpc.hpp"

using namespace Eigen;

float TIME_HORIZON=10;
float SAMPLING_TIME=1;
int QUEUE_LENGTH;
float MAX_LATERAL_ACCELERATION=8;
float MAX_ABSOLUTE_VELOCITY=10;
float K1_LAD_V=1;
float K2_LAD_V=1;
float SLOW_DOWN_DISTANCE=10; 
float SLOW_DOWN_PUFFER=2;
float V_FREEDOM=0.2;
std::string STELLGROESSEN_TOPIC;
std::string TRACKING_ERROR_TOPIC;
std::string NAVIGATION_INFO_TOPIC;
std::string STATE_TOPIC;
std::string OBSTACLE_DISTANCE_TOPIC;
std::string SHUTDOWN_TOPIC;
std::string PATH_NAME_EDITED;

MPC::MPC(ros::NodeHandle* n, std::string PATH_NAME)
{
	n_ = n;
	n->getParam("/control/K1_LAD_V",K1_LAD_V);
	n->getParam("/control/K2_LAD_V",K2_LAD_V);
	n->getParam("/control/MAX_LATERAL_ACCELERATION",MAX_LATERAL_ACCELERATION);
	n->getParam("/control/SLOW_DOWN_DISTANCE",SLOW_DOWN_DISTANCE);
	n->getParam("/control/SLOW_DOWN_PUFFER",SLOW_DOWN_PUFFER);
	n->getParam("/control/V_FREEDOM",V_FREEDOM);
	n->getParam("/topic/STELLGROESSEN",STELLGROESSEN_TOPIC);
	n->getParam("/topic/TRACKING_ERROR",TRACKING_ERROR_TOPIC);
	n->getParam("/topic/NAVIGATION_INFO",NAVIGATION_INFO_TOPIC);
	n->getParam("/topic/STATE",STATE_TOPIC);
	n->getParam("/topic/OBSTACLE_DISTANCE",OBSTACLE_DISTANCE_TOPIC);
	n->getParam("/topic/SHUTDOWN",SHUTDOWN_TOPIC);
	n->getParam("/general/QUEUE_LENGTH",QUEUE_LENGTH );
	PATH_NAME_EDITED =PATH_NAME + "_teach.txt";
	//Publisher
	pub_stellgroessen_ = n_->advertise<ackermann_msgs::AckermannDrive>(STELLGROESSEN_TOPIC, QUEUE_LENGTH);
	//Subscriber
	sub_state_ = n_->subscribe(STATE_TOPIC, QUEUE_LENGTH, &MPC::stateCallback,this);
	distance_to_obstacle_sub_=n_->subscribe(OBSTACLE_DISTANCE_TOPIC, QUEUE_LENGTH ,&MPC::obstacleCallback,this);
	gui_stop_sub_=n_->subscribe(SHUTDOWN_TOPIC, QUEUE_LENGTH ,&MPC::guiStopCallback,this);


		//Initialisations.
		readPathFromTxt(PATH_NAME_EDITED);
	rest_=0;
	ref_xy_.clear();
	steps_in_horizon_=TIME_HORIZON/SAMPLING_TIME;	
	
//TEST

/*
state_.current_arrayposition=815;
findReferencePoints();
for(int i=0; i<2*steps_in_horizon_;i++)
{
std::cout<<ref_xy_[i]<<std::endl;
}
*/
	state_.current_arrayposition=394;
	state_.pose.pose.position.x=2.9;
	state_.pose.pose.position.y=-67;
	state_.pose.pose.position.z=1.6;
	state_.pose.pose.orientation.x=-0;	
	state_.pose.pose.orientation.y=0;
	state_.pose.pose.orientation.z=0;
	state_.pose.pose.orientation.w= 1;
	std::cout<<indexOfDistanceFront(394, 10).x<<std::endl;
	writeTxt(10);
	std::cout <<calculateParamFun(10)<<std::endl;

//	std::cout <<state_.current_arrayposition<<std::endl;
//	std::cout <<path_.poses[0].pose.position.x<<std::endl;	

//	std::cout <<d_(0,2)<<std::endl;	
	std::cout << std::endl << "MPC: Consturctor init, path lenght: " <<n_poses_path_<< " and slow_down_index: "<<slow_down_index_<<std::endl;
	//pathToVector(); 
	//useful vor Eigen
}

void MPC::stateCallback(const arc_msgs::State::ConstPtr& incoming_state)
{
	state_ = *incoming_state;
	rest_=0;
	ref_xy_.clear();
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

void MPC::findReferencePoints()		//Filling of array for MPC solver
{	
	int j_temp;
	int j=state_.current_arrayposition;
	float step=1;
	for(int i=0; i<steps_in_horizon_;i++)
	{	
		step=v_ref_[j]*SAMPLING_TIME-rest_;
		j_temp=indexOfDistanceFront(j,step).x;
		geometry_msgs::Point global;
		geometry_msgs::Point local;
		if(j_temp>=n_poses_path_-1) 
		{
			global=path_.poses[n_poses_path_-1].pose.position;
		}	
		else
		{
			global=pointAtDistanceLinear(j,step);
		}
		local=arc_tools::globalToLocal(global, state_);
		ref_xy_.push_back(local.x);
		ref_xy_.push_back(local.y);
		j=j_temp; 
	}
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
						//stream>>path_diff_.poses[j-1] .pose.position.y;
						//stream>>path_diff_.poses[j-1].pose.position.z;

		stream.ignore (300, '|');
		i++;
	}
	n_poses_path_ = i;
	slow_down_index_=indexOfDistanceBack(i-1,SLOW_DOWN_DISTANCE).x;

	//Calculate reference velocity. new, to implement also on pp
	v_ref_=teach_vel_;
	float n=n_poses_path_;
	for(int i_vel=0;i_vel<n_poses_path_;i_vel++)
	{
	    //First calculate physical limit velocity
		float lad_v= K2_LAD_V + K1_LAD_V*v_abs_;
		//find reference index for curvature
		int i_radius=indexOfDistanceFront(i_vel, lad_v).x;
		if(i_radius>=n_poses_path_) i_radius=n_poses_path_-1;
		float v_limit=sqrt(MAX_LATERAL_ACCELERATION*curveRadius(i_radius));
	    //Upper buonds
		//MAX_ABSOLUTE_VELOCITY
		float v_bounded=std::min(v_limit,MAX_ABSOLUTE_VELOCITY);
		v_bounded=std::min(v_bounded,teach_vel_[i_vel]+V_FREEDOM);
		float C=1;
	    //Slow down
		if (i_vel>=slow_down_index_)
			{
			//Lineares herrunterschrauben
			C=C*((distanceIJ(i_vel,n_poses_path_-1))-SLOW_DOWN_PUFFER)/(SLOW_DOWN_DISTANCE-SLOW_DOWN_PUFFER);
			}
		v_ref_[i_vel] =std::max(v_bounded * C,float(0));
	}

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


Eigen::MatrixXd MPC::pathToMatrix(float lad)  //Let's see

{
	int i_start = state_.current_arrayposition;
	int i_end = indexOfDistanceFront(i_start, lad).x;
	Eigen::MatrixXd d(2,i_end-i_start);
	for (int i=0; i<i_end-i_start; i++){
	geometry_msgs::Point p= arc_tools::globalToLocal(path_.poses[state_.current_arrayposition+i].pose.position, state_);
	d(0,i) = p.x;
	d(1,i) = p.y;	
}
std::cout<< d<<std::endl;
	return d;
}

void MPC::writeTxt(float lad)	//write for test
{
	std::string blabla;
	blabla = "ff.txt";
	int i_start = state_.current_arrayposition;
	int i_end = indexOfDistanceFront(i_start, lad).x;
	int lenght = i_end - i_start;
	std::ofstream stream(blabla.c_str(), std::ios::out);
	geometry_msgs::Point p;
	for (int i=i_start; i<i_end; i++)
	{
	p=arc_tools::globalToLocal(path_.poses[i].pose.position,state_);
  	stream <<p.x<<" "<<
           p.y<<"\r\n";
	}
	
stream.close();
}


Eigen::Vector4d MPC::calculateParamFun(float lad)
{
	Eigen::MatrixXd d_ = pathToMatrix(lad);
std::cout<<"ciao"<<std::endl;
	int i_start = state_.current_arrayposition;
	int i_end = indexOfDistanceFront(i_start, lad).x;
	int lenght = i_end - i_start;
	float sum1 = d_.row(0).sum();
	float sum2 = d_.row(0).cwiseAbs2().sum();
	float sum3 = 0;
	for (int i=0; i<lenght; i++)
	{
		sum3 += pow(d_(0, i), 3.0); 
	}

	float sum4 = 0;
	for (int i=0; i<lenght; i++)
	{
		sum4 += pow(d_(0, i), 4.0); 
	}

	float sum5 = 0;
	for (int i=0; i<lenght; i++)
	{
		sum5 += pow(d_(0, i), 5.0); 
	}

	float sum6 = 0;
	for (int i=0; i<lenght; i++)
	{
		sum6 += pow(d_(0, i), 6.0); 
	}
	Eigen::MatrixXd A = Eigen::MatrixXd::Zero(4,4);
	A << lenght, sum1, sum2, sum3, sum1, sum2, sum3, sum4, sum2, sum3, sum4, sum5, sum3, sum4, sum5, sum6;
	std::cout << A << std::endl;

	float sum_rhs = 0;
	for (int i=0; i<lenght; i++)
	{
		sum_rhs += pow(d_(0,i), 3.0)*d_(1,i); 
	}

	Eigen::VectorXd rhs(4); 
	rhs << d_.row(1).sum(), d_.row(1).dot(d_.row(0)), d_.row(1).dot(d_.row(0).cwiseAbs2()), sum_rhs;
//	std::cout << rhs << std::endl;

	Eigen::Vector4d a = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(rhs);//

	return a;

//	std::cout << d(1,0);
//	fitting points path_.poses[i].position.x and path_.poses[i].position.y

//	
//	while(l<d &&j<n_poses_path_-1)
//	{	
//	
//	indexOfDistanceFront(j-1, 4); example with distance 4 meters
//	
//	}
}

geometry_msgs::Vector3 MPC::indexOfDistanceFront(int i, float d)
{
	geometry_msgs::Vector3 vector;	//vector.x= index  vector.z=real distance upper  vector.z=real distance lower 
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
	vector.x=j+1;
	vector.y=l;
	vector.z=l-sqrt(	pow(path_.poses[j].pose.position.x - path_.poses[j-1].pose.position.x,2)+
				pow(path_.poses[j].pose.position.y - path_.poses[j-1].pose.position.y,2)+
				pow(path_.poses[j].pose.position.z - path_.poses[j-1].pose.position.z,2));
	return vector;
}

geometry_msgs::Vector3 MPC::indexOfDistanceBack(int i, float d)
{
	geometry_msgs::Vector3 vector;	//vector.x= index  vector.z=real distance upper  vector.z=real distance lower 
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
	vector.x=j;
	vector.y=l;
	vector.z=l-sqrt(	pow(path_.poses[j].pose.position.x - path_.poses[j+1].pose.position.x,2)+
				pow(path_.poses[j].pose.position.y - path_.poses[j+1].pose.position.y,2)+
				pow(path_.poses[j].pose.position.z - path_.poses[j+1].pose.position.z,2));
	return vector;
}

geometry_msgs::Point MPC::pointAtDistanceLinear(int i, float distance)		//TO TEST
{
	geometry_msgs::Point exact_point;
	geometry_msgs::Vector3 vector=indexOfDistanceFront(i,distance);
	int index=vector.x;
	float d_upper=vector.y;
	float d_lower=vector.z;
	rest_=d_upper-distance;
	float lambda=(distance-d_lower)/(d_upper-d_lower);	//lineare interpolation 
	float dx= path_.poses[index].pose.position.x - path_.poses[index-1].pose.position.x;
	float dy= path_.poses[index].pose.position.y - path_.poses[index-1].pose.position.y;
	exact_point.x=path_.poses[index-1].pose.position.x+lambda*dx;
	exact_point.y=path_.poses[index-1].pose.position.y+lambda*dy;
	exact_point.z=0;
	return exact_point;
}	

float MPC::curveRadius(int i)
{
float radius=10;
return radius;
}

MPC::~MPC()
{}
