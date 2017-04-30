#include "../include/arc_mpc/arc_mpc.hpp"
//momoentan erster referenzpunkt noch aktueller punkt,  zu korrigieren
//segmentation fault if polynomial is very wrong (b_poly=1 on gerade...)
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
	rest_linear_interpolation_=0;
	ref_xy_.clear();
	ref_xy_test_.clear();
	steps_in_horizon_=TIME_HORIZON/SAMPLING_TIME;	
	
//TEST

std::cout<<linearInterpolation(100,300,3,7,5.2)<<std::endl;
state_.current_arrayposition=0;
state_.pose.pose.orientation.w=1;
poly_c_=0;
poly_b_=1;
std::cout<<state_<<std::endl;
//findReferencePointsLinear();
std::cout<<"ciao"<<std::endl;
//findReferencePointsPoly();
std::cout<<nextReferenceXPolynomial(0,3)<<std::endl;
/*for(int i=0; i<2*steps_in_horizon_;i++)
{
std::cout<<"with linear "<<ref_xy_[i]<<" with poly "<<ref_xy_test_[i]<<std::endl;
}

geometry_msgs::Point p;
p.x=8;
p.y=-8;
state_=arc_tools::generate2DState(0,-4, -0.785);
localPointToPathIndex(p,0,600);
	std::cout << std::endl << "MPC: Consturctor init, path lenght: " <<n_poses_path_<< " and slow_down_index: "<<slow_down_index_<<std::endl;
<<<<<<< HEAD
//	pathToVector(); //useful vor Eigen
//	calculateParamFun(d_);
*/	
=======
	//pathToVector(); 
	//useful vor Eigen
	
>>>>>>> 1c49c800cb6bfffeb63b19d24168656df29d91e3
}

void MPC::stateCallback(const arc_msgs::State::ConstPtr& incoming_state)
{
	state_ = *incoming_state;
	rest_linear_interpolation_=0;
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

float MPC::vRef(int index)
{	
	return 3;//v_ref_[index];
}

float MPC::vRef(geometry_msgs::Point local, int i_start, int i_end)
{
	int index=localPointToPathIndex(local, i_start, i_end);
	return 3;//vRef(index);
}
void MPC::findReferencePointsPoly()
{
	float x_next;
	float x_curr=0;
	float v_ref=state_.pose_diff;	//v_ref_[state_.current_];weniger sinn 
	//Integers there to keep track of position in path. So to read out proper v_ref[].
	int j_start=state_.current_arrayposition;
	int j_end;
	int j_next;		
	float step;		//Meters to next ref_point
	geometry_msgs::Point ref_point;

	for(int i=0; i<steps_in_horizon_;i++)		//Where is first ref point?? at actual point or after first Ts?
	{	
		step=v_ref*SAMPLING_TIME;
		x_next= x_curr + step;		//Vorwäts nur entlang x achse 
		//Save next reference point.
		ref_point.x = x_next;
		ref_point.y = yPoly(x_next);
		ref_xy_test_.push_back(ref_point.x);
		ref_xy_test_.push_back(ref_point.y);
		//Find reference velocity at next point.
		j_end=indexOfDistanceFront(j_start,20).x;		//durch wieviele punkte nach vorne soll er durchsuchen, jetzt 20 m. Annahme, in einnem zeitschritt nie mehr als 20 m!!
		j_next=localPointToPathIndex(ref_point , j_start , j_end);
		//v_ref=v_ref_[j_next];
		v_ref=vRef(ref_point,j_start,j_end);
		//Actualisation.
		x_curr=x_next;
		j_start=j_next;
	}
}
void MPC::findReferencePointsLinear()		//Filling of array for MPC solver
{	
	int j_temp;
	int j=state_.current_arrayposition;
	float v_ref=state_.pose_diff;			//v_ref_[j]; hat weniger sinn da wir eigentlich eig genau wissen wie schnell wir im ersten punkt fahren
	float step;
	for(int i=0; i<steps_in_horizon_;i++)		//Where is first ref point?? at actual point or after first Ts?
	{	
		step=v_ref*SAMPLING_TIME-rest_linear_interpolation_;
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
		v_ref=vRef(j);		//replace j with localPointToPathIndex(local,j-30,j+30) how to make it independent of running index j (we do not have it with polyfit)
	}
}

float MPC::nextReferenceXPolynomial(float x_start, float step)		//interpolates linearly at the end.
{
	float x_run=x_start;
	float x_run_old=x_start;
	float sum=0;
	float sum_old=0;
	float dx=0.1;
	while (sum<step)
	{
		x_run_old=x_run;
		sum_old=sum;
		sum+=sqrt(pow(dx,2)+pow(yPoly(x_run)-yPoly(x_run+dx),2));
		x_run+=dx;
	}
	x_run=linearInterpolation( x_run_old, x_run, sum_old, sum, step );
	return x_run;
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


void MPC::pathToMatrix(float lad)  //Let's see

{
	int i_start = state_.current_arrayposition;
	int i_end = indexOfDistanceFront(i_start, lad);
	Eigen::MatrixXd d(2,n_poses_path_);
	for (int i=i_start; i<i_end; i++){
	d(0,i) = path_.poses[i].pose.position.x;
	d(1,i) = path_.poses[i].pose.position.y;	
}
	Eigen::MatrixXd d_ = d;
}

void MPC::calculateParamFun(Eigen::MatrixXd a)
{
	
//	Eigen::MatrixXd A = Eigen::MatrixXd::Zero(4,4);
//	A << n_poses_path_, d_.row(0).sum(), d_.row(0).cwiseAbs2().sum(), d_.row(0).cwiseProduct(d_.row(0)).sum, d_.row(0).sum(), d_.row(0).cwiseAbs2().sum(), d_.row(0).cwiseAbs2().cwiseProduct(d_.row(0)).sum(), d_.row(0).c d_.row(0).cwiseAbs2().sum(), d_.row(0).cwiseAbs2().cwiseProduct(d_.row(0)).sum(), d_.row(0).cwiseAbs2().cwiseProduct(d_.row(0).cwiseAbs2()).sum();
//	std::cout << A << std::endl;

//	Eigen::VectorXd rhs(3); 
//	rhs << d_.row(1).sum(), d_.row(1).dot(d_.row(0)), d_.row(1).dot(d_.row(0).cwiseAbs2());
//	std::cout << rhs << std::endl;

//	std::cout << A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(rhs) << std::endl; //

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
	rest_linear_interpolation_=d_upper-distance;
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

int MPC::localPointToPathIndex(geometry_msgs::Point p, int i_start, int i_end)
{	
	geometry_msgs::Point p_global=localToGlobal(p,state_);		//Transformation local to global
	int j;
	float distance_old=100;
	float distance_new;
	for(int i=i_start; i<i_end;i++)
	{
		distance_new=sqrt(	pow((path_.poses[i].pose.position.x-p_global.x),2) 
					+ 	pow((path_.poses[i].pose.position.y-p_global.y),2));
		
		if(distance_new<distance_old)
		{
			j=i;
			distance_old=distance_new;
			
		}	
	}
	std::cout<<"nearest index: "<<j<<std::endl<<path_.poses[j].pose.position<<std::endl;
	return j;
}

geometry_msgs::Point MPC::localToGlobal(geometry_msgs::Point p_local, arc_msgs::State state_)
{	
std::cout<<"Local point in local coordinate convention "<<p_local<<std::endl; 
	//Change local coordinates
	float temp=p_local.x;
	p_local.x=p_local.y;
	p_local.y=-temp;
std::cout<<"Local point in global coordinate convention "<<p_local<<std::endl;  
	//Rotate              
	geometry_msgs::Point p_add=arc_tools::rotationLocalToGlobal(p_local,state_);
std::cout<<"Local point rotated "<<p_add<<std::endl; 
	geometry_msgs::Point p_global=state_.pose.pose.position;
	p_global.x+=p_add.x;
	p_global.y+=p_add.y;
std::cout<<"Global position "<<p_global<<std::endl;
	return p_global;
}

float MPC::yPoly(float x)
{
	float y;
	y=(poly_a_ *x*x*x + poly_b_ *x*x + poly_c_ *x + poly_d_) ;
	return y;
}

float MPC::linearInterpolation(float a_lower, float a_upper ,float b_lower, float b_upper, float b_middle)
{	
	if(b_upper == b_lower) 
	{
		std::cout<<"Falsch interpoliert \n";
		return a_lower;
	}
	float a_middle =  a_lower + ( b_middle - b_lower ) * ( a_upper - a_lower )/( b_upper - b_lower );
	return a_middle;
}
MPC::~MPC()
{}
