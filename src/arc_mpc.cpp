#include "../include/arc_mpc/arc_mpc.hpp"
//momoentan erster referenzpunkt noch aktueller punkt,  zu korrigieren
//segmentation fault if polynomial is very wrong (b_poly=1 on gerade...)

int QUEUE_LENGTH;
float MAX_LATERAL_ACCELERATION=8;
float MAX_ABSOLUTE_VELOCITY=10;
float K1_LAD_V=1;
float K2_LAD_V=1;
float SLOW_DOWN_DISTANCE=10; 
float SLOW_DOWN_PUFFER=2;
float V_FREEDOM=0.2;
float INTERPOLATION_DISTANCE_FRONT=10;
float OBSTACLE_SLOW_DOWN_DISTANCE;
float OBSTACLE_PUFFER_DISTANCE;
float SHUT_DOWN_TIME;
std::string STELLGROESSEN_TOPIC;
std::string TRACKING_ERROR_TOPIC;
std::string NAVIGATION_INFO_TOPIC;
std::string STATE_TOPIC;
std::string OBSTACLE_DISTANCE_TOPIC;
std::string SHUTDOWN_TOPIC;
std::string PATH_NAME_EDITED;
//Solver constants
float TIME_HORIZON=10;
float SAMPLING_TIME=0.2;
int N_VAR=8;
int N_PARAM=11;	//x_ref y_ref v_ref
int N_STEPS=20;
int N_INIT=4;

MPC::MPC(ros::NodeHandle* n, std::string PATH_NAME, std::string MODE)
{
	n_ = n;
	n->getParam("/control/OBSTACLE_SLOW_DOWN_DISTANCE",OBSTACLE_SLOW_DOWN_DISTANCE);
	n->getParam("/control/OBSTACLE_PUFFER_DISTANCE",OBSTACLE_PUFFER_DISTANCE);
	n->getParam("/control/SHUT_DOWN_TIME",SHUT_DOWN_TIME);
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
	pub_output_1_ = n_->advertise<std_msgs::Float32MultiArray>("/matlab_output_1_topic", QUEUE_LENGTH);
	pub_output_2_ = n_->advertise<std_msgs::Float32MultiArray>("/matlab_output_2_topic", QUEUE_LENGTH);
	//Subscriber
	if(MODE=="simulation") sub_state_matlab_ =n->subscribe("/state_matlab", QUEUE_LENGTH, &MPC::stateMatlabCallback,this);
	else{
		sub_state_ = n_->subscribe(STATE_TOPIC, QUEUE_LENGTH, &MPC::stateCallback,this);
		distance_to_obstacle_sub_=n_->subscribe(OBSTACLE_DISTANCE_TOPIC, QUEUE_LENGTH ,&MPC::obstacleCallback,this);
		gui_stop_sub_=n_->subscribe(SHUTDOWN_TOPIC, QUEUE_LENGTH ,&MPC::guiStopCallback,this);
		}

	//Initialisations.
	readPathFromTxt(PATH_NAME_EDITED);
	rest_linear_interpolation_=0;
	obstacle_distance_=100;
	gui_stop_=0;
	ref_x_.clear();
	ref_y_.clear();
	ref_v_.clear();
	steps_in_horizon_=TIME_HORIZON/SAMPLING_TIME;
//Interface to casadi
	
	#ifdef _cplusplus
	extern "C" {
	#endif
	   	extern void arc_solver_casadi2forces(double *x, double *y, double *l, double *p,
	                                                double *f, double *nabla_f, double *c, double *nabla_c,
	                                                double *h, double *nabla_h, double *H, int stage);
	pt2Function = &arc_solver_casadi2forces;
	#ifdef _cplusplus
	}
	#endif 


	
//TEST
/*poly_a_=0;
poly_b_=0;
poly_c_=0.2;
poly_d_=0;
calculateParamFun(10);
findReferencePointsPoly();
for(int i=0;i<9;i++) std::cout<<"x-ref: "<<ref_x_[i]<<" y-ref: "<<ref_y_[i]<<" v-ref: "<<ref_v_[i]<<std::endl;
setSolverParam();
for (int i=0;i<27;i++) std::cout<<"all_param "<<i<<": "<<solver_param_.all_parameters[i]<<std::endl;
int ciao=arc_solver_solve(&solver_param_, &solver_output_, &solver_info_,stdout, pt2Function);
std::cout<<"int= "<<ciao<<std::endl;
std::cout<<std::endl;
std::cout<<"output0 "<<solver_output_.x01[0]<<std::endl;
std::cout<<"output0 "<<solver_output_.x02[0]<<std::endl;
std::cout<<"output0 "<<solver_output_.x03[0]<<std::endl;
std::cout<<"output0 "<<solver_output_.x04[0]<<std::endl;
std::cout<<"output0 "<<solver_output_.x05[0]<<std::endl;
std::cout<<"output0 "<<solver_output_.x06[0]<<std::endl;
std::cout<<"output0 "<<solver_output_.x07[0]<<std::endl;
std::cout<<"output0 "<<solver_output_.x08[0]<<std::endl;
std::cout<<"output0 "<<solver_output_.x09[0]<<std::endl;
std::cout<<std::endl;
std::cout<<"output1 "<<solver_output_.x01[1]<<std::endl;
std::cout<<"output1 "<<solver_output_.x02[1]<<std::endl;
std::cout<<"output1 "<<solver_output_.x03[1]<<std::endl;
std::cout<<"output1 "<<solver_output_.x04[1]<<std::endl;
std::cout<<"output1 "<<solver_output_.x05[1]<<std::endl;
std::cout<<"output1 "<<solver_output_.x06[1]<<std::endl;
std::cout<<"output1 "<<solver_output_.x07[1]<<std::endl;
std::cout<<"output1 "<<solver_output_.x08[1]<<std::endl;
std::cout<<"output1 "<<solver_output_.x09[1]<<std::endl;
std::cout<<std::endl;
std::cout<<"output2 "<<solver_output_.x01[2]<<std::endl;
std::cout<<"output2 "<<solver_output_.x02[2]<<std::endl;
std::cout<<"output2 "<<solver_output_.x03[2]<<std::endl;
std::cout<<"output2 "<<solver_output_.x04[2]<<std::endl;
std::cout<<"output2 "<<solver_output_.x05[2]<<std::endl;
std::cout<<"output2 "<<solver_output_.x06[2]<<std::endl;
std::cout<<"output2 "<<solver_output_.x07[2]<<std::endl;
std::cout<<"output2 "<<solver_output_.x08[2]<<std::endl;
std::cout<<"output2 "<<solver_output_.x09[2]<<std::endl;
std::cout<<std::endl;
std::cout<<"output3 "<<solver_output_.x01[3]<<std::endl;
std::cout<<"output3 "<<solver_output_.x02[3]<<std::endl;
std::cout<<"output3 "<<solver_output_.x03[3]<<std::endl;
std::cout<<"output3 "<<solver_output_.x04[3]<<std::endl;
std::cout<<"output3 "<<solver_output_.x05[3]<<std::endl;
std::cout<<"output3 "<<solver_output_.x06[3]<<std::endl;
std::cout<<"output3 "<<solver_output_.x07[3]<<std::endl;
std::cout<<"output3 "<<solver_output_.x08[3]<<std::endl;
std::cout<<"output3 "<<solver_output_.x09[3]<<std::endl;
std::cout<<std::endl;
std::cout<<"output4 "<<solver_output_.x01[4]<<std::endl;
std::cout<<"output4 "<<solver_output_.x02[4]<<std::endl;
std::cout<<"output4 "<<solver_output_.x03[4]<<std::endl;
std::cout<<"output4 "<<solver_output_.x04[4]<<std::endl;
std::cout<<"output4 "<<solver_output_.x05[4]<<std::endl;
std::cout<<"output4 "<<solver_output_.x06[4]<<std::endl;
std::cout<<"output4 "<<solver_output_.x07[4]<<std::endl;
std::cout<<"output4 "<<solver_output_.x08[4]<<std::endl;
std::cout<<"output4 "<<solver_output_.x09[4]<<std::endl;
std::cout<<std::endl;
std::cout<<"output5 "<<solver_output_.x01[5]<<std::endl;
std::cout<<"output5 "<<solver_output_.x02[5]<<std::endl;
std::cout<<"output5 "<<solver_output_.x03[5]<<std::endl;
std::cout<<"output5 "<<solver_output_.x04[5]<<std::endl;
std::cout<<"output5 "<<solver_output_.x05[5]<<std::endl;
std::cout<<"output5 "<<solver_output_.x06[5]<<std::endl;
std::cout<<"output5 "<<solver_output_.x07[5]<<std::endl;
std::cout<<"output5 "<<solver_output_.x08[5]<<std::endl;
std::cout<<"output5 "<<solver_output_.x09[5]<<std::endl;
std::cout<<std::endl;
std::cout<<"output6 "<<solver_output_.x01[6]<<std::endl;
std::cout<<"output6 "<<solver_output_.x02[6]<<std::endl;
std::cout<<"output6 "<<solver_output_.x03[6]<<std::endl;
std::cout<<"output6 "<<solver_output_.x04[6]<<std::endl;
std::cout<<"output6 "<<solver_output_.x05[6]<<std::endl;
std::cout<<"output6 "<<solver_output_.x06[6]<<std::endl;
std::cout<<"output6 "<<solver_output_.x07[6]<<std::endl;
std::cout<<"output6 "<<solver_output_.x08[6]<<std::endl;
std::cout<<"output6 "<<solver_output_.x09[6]<<std::endl;
std::cout<<std::endl;
std::cout<<poly_a_<<" "<<poly_b_<<" "<<poly_c_<<" "<<poly_d_<<std::endl;
//PLOT
writeTxt();*/
}

void MPC::stateCallback(const arc_msgs::State::ConstPtr& incoming_state)
{
	state_ = *incoming_state;
	v_abs_=state_.pose_diff;
	rest_linear_interpolation_=0;
	ref_x_.clear();
	ref_y_.clear();
	ref_v_.clear();
	//LOOP
std::cout<<"Arrayposition "<<state_.current_arrayposition<<std::endl;
	calculateParamFun(INTERPOLATION_DISTANCE_FRONT);
std::cout<<"Param calculated "<<std::endl;
	findReferencePointsPoly();
std::cout<<"Reference found "<<std::endl;
for(int i=0;i<9;i++) std::cout<<"x-ref: "<<ref_x_[i]<<" y-ref: "<<ref_y_[i]<<" v-ref: "<<ref_v_[i]<<std::endl;
	setSolverParam();
std::cout<<"Param setted "<<std::endl;
	getOutputAndReact();
	//END LOOP
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

void MPC::stateMatlabCallback(const geometry_msgs::Quaternion::ConstPtr& incoming_state)
{
	//set pose
	geometry_msgs::Pose2D pose;
	pose.x=incoming_state->x;
	pose.y=incoming_state->y;
	pose.theta=incoming_state->w;
	state_=arc_tools::generate2DState(pose);
	//set velocity	
	state_.pose_diff=incoming_state->z;
	v_abs_=state_.pose_diff;
	//find current arrayposition
	float distance_old=100;
	float distance_new=0;
	for(int i=0;i<n_poses_path_;i++)
	{
		distance_new=sqrt(pow((pose.x-path_.poses[i].pose.position.x),2)+pow(pose.y-path_.poses[i].pose.position.y,2));
		if(distance_new<distance_old)
		{
			state_.current_arrayposition=i;
			distance_old=distance_new;
		}
	}
	//Initialisation
	ref_x_.clear();
	ref_y_.clear();
	ref_v_.clear();
	//Loop
std::cout<<"Arrayposition "<<state_.current_arrayposition<<std::endl;
	calculateParamFun(INTERPOLATION_DISTANCE_FRONT);
std::cout<<"Param calculated "<<std::endl;
	findReferencePointsPoly();
std::cout<<"Reference found "<<std::endl;
for(int i=0;i<9;i++) std::cout<<"x-ref: "<<ref_x_[i]<<" y-ref: "<<ref_y_[i]<<" v-ref: "<<ref_v_[i]<<std::endl;
	setSolverParam();
std::cout<<"Param setted "<<std::endl;
	getOutputAndReact();
	//END LOOP

}
void MPC::setSolverParam()	//To test
{
	//Reference values
	int j=0;	
	for(int i=0;i<N_PARAM*N_STEPS;i+=N_PARAM, j++)
	{
	//p(1): Reference x
	solver_param_.all_parameters[i]=ref_x_[j];
	//p(2): Reference y
	solver_param_.all_parameters[i+1]=ref_y_[j];
	//p(3): Reference v
	solver_param_.all_parameters[i+2]=ref_v_[j];
	//p(4): Weight dx
	solver_param_.all_parameters[i+3]=1;
	//p(5): Weight dy
	solver_param_.all_parameters[i+4]=1;
	//p(6): Weight dv
	solver_param_.all_parameters[i+5]=0;
	//p(7): Weight change of acceleration
	solver_param_.all_parameters[i+6]=1;
	//p(8): Weight change of steer
	solver_param_.all_parameters[i+7]=1;
	//p(9): Weight acceleration
	solver_param_.all_parameters[i+8]=0;
	//p(10): Weight steer
	solver_param_.all_parameters[i+9]=0;	
	//p(11): Street slope, not implemented 
	solver_param_.all_parameters[i+10]=0;
	}

	//Booooooo
	for(int i=0;i<N_STEPS*N_VAR;i++) solver_param_.x0[i]=0;

	//Initial conditions
	solver_param_.xinit[0]=0;	//initial value x-position(local)
	solver_param_.xinit[1]=0;	//initial value y-position(local)
	solver_param_.xinit[2]=v_abs_; 	//initial value velocity.
	solver_param_.xinit[3]=0;	//initial value orientation (local)
}
float MPC::vRef(int index)	//For the moment const=3
{	
	geometry_msgs::Point local=arc_tools::globalToLocal(path_.poses[index].pose.position, state_);
	float v_limit=sqrt(MAX_LATERAL_ACCELERATION*radiusPoly(local.x));	
	float v_a_priori=v_ref_[index];
	float v_ref=std::min(v_a_priori,v_limit);	
	//Penalisation
	float C=1;
	//Obstacle slow down
	if(obstacle_distance_<OBSTACLE_SLOW_DOWN_DISTANCE) std::cout<<OBSTACLE_SLOW_DOWN_DISTANCE<<" PURE PURSUIT; Slow down for obstacle"<<std::endl;
	obstacle_distance_=std::max(obstacle_distance_,OBSTACLE_PUFFER_DISTANCE);
	obstacle_distance_=std::min(obstacle_distance_,OBSTACLE_SLOW_DOWN_DISTANCE);
	C=C * (obstacle_distance_ - OBSTACLE_PUFFER_DISTANCE) / (OBSTACLE_SLOW_DOWN_DISTANCE - OBSTACLE_PUFFER_DISTANCE);
	//Shutdown

	if(gui_stop_==1&&BigBen_.getTimeFromStart()<=SHUT_DOWN_TIME)//&& time zwischen 0 und pi/2
		{
		std::cout<<"PURE PURSUIT: Shutting down gradually"<<std::endl;
		C=C*cos(BigBen_.getTimeFromStart()*1.57079632679/(SHUT_DOWN_TIME));	//Zähler ist PI/2.
		}
	else if (gui_stop_==1 && BigBen_.getTimeFromStart()>SHUT_DOWN_TIME)
		{
		std::cout<<"PURE PURSUIT: Shutted down"<<std::endl;
		C=0;
		}
	
	v_ref*=C;
	return v_ref;
}

float MPC::vRef(geometry_msgs::Point local, int i_start, int i_end)	//For the moment const=3
{	
	//Physical limit
	float v_limit=sqrt(MAX_LATERAL_ACCELERATION*radiusPoly(local.x));
	int index=localPointToPathIndex(local, i_start, i_end);
	float v_a_priori=v_ref_[index];
	float v_ref=std::min(v_a_priori,v_limit);	
	//Penalisation
	float C=1;
	//Obstacle slow down
	if(obstacle_distance_<OBSTACLE_SLOW_DOWN_DISTANCE) std::cout<<OBSTACLE_SLOW_DOWN_DISTANCE<<" PURE PURSUIT; Slow down for obstacle"<<std::endl;
	obstacle_distance_=std::max(obstacle_distance_,OBSTACLE_PUFFER_DISTANCE);
	obstacle_distance_=std::min(obstacle_distance_,OBSTACLE_SLOW_DOWN_DISTANCE);
	C=C * (obstacle_distance_ - OBSTACLE_PUFFER_DISTANCE) / (OBSTACLE_SLOW_DOWN_DISTANCE - OBSTACLE_PUFFER_DISTANCE);
//std::cout<<"C obstacle "<<C<<std::endl;
	//Shutdown
	if(gui_stop_==1&&BigBen_.getTimeFromStart()<=SHUT_DOWN_TIME)//&& time zwischen 0 und pi/2
		{
		std::cout<<"PURE PURSUIT: Shutting down gradually"<<std::endl;
		C=C*cos(BigBen_.getTimeFromStart()*1.57079632679/(SHUT_DOWN_TIME));	//Zähler ist PI/2.
		}
	else if (gui_stop_==1 && BigBen_.getTimeFromStart()>SHUT_DOWN_TIME)
		{
		std::cout<<"PURE PURSUIT: Shutted down"<<std::endl;
		C=0;
		}
//std::cout<<"C obstacle "<<C<<std::endl;
	v_ref*=C;
//std::cout<<"Vpriori "<<v_a_priori<<" v_limit "<<v_limit<<" v_ref final "<<v_ref<<std::endl;
	return v_ref;
}
void MPC::findReferencePointsPoly()
{
	float x_next;
	float x_curr=0;
	float v_ref=state_.pose_diff;	//first reference velocity is actual velocity
	//Integers there to keep track of position in path. So to read out proper v_ref[].
	int j_start=state_.current_arrayposition;
	int j_end;
	int j_next;		
	float step;		//Meters to next ref_point
	geometry_msgs::Point ref_point;
	ref_point.x=x_curr;
	ref_point.y=yPoly(x_curr);
	ref_x_.push_back(ref_point.x);
	ref_y_.push_back(ref_point.y);	
	for(int i=0; i<N_STEPS;i++)		//Where is first ref point?? at actual point or after first Ts?
	{	
		step=v_ref*SAMPLING_TIME;
		x_next= nextReferenceXPolynomial(x_curr,step);		//Linear integration
		//Save next reference point.
		ref_point.x = x_next;
		ref_point.y = yPoly(x_next);
		ref_point.z =0;
		ref_x_.push_back(ref_point.x);
		ref_y_.push_back(ref_point.y);
		//Find reference velocity at next point.
		j_end=indexOfDistanceFront(j_start,20).x;		//durch wieviele punkte nach vorne soll er durchsuchen, jetzt 20 m. Annahme, in einnem zeitschritt nie mehr als 20 m!!
		j_next=localPointToPathIndex(ref_point , j_start , j_end);
		//v_ref=v_ref_[j_next];
		v_ref=vRef(ref_point,j_start,j_end);
		ref_v_.push_back(v_ref);

		//Actualisation.
		x_curr=x_next;
		j_start=j_next;
	}
}
/*
void MPC::findReferencePointsLinear()		//Filling of array for MPC solver
{	
	int j_temp;
	int j=state_.current_arrayposition;
	float v_ref=state_.pose_diff;			//v_ref_[j]; hat weniger sinn da wir eigentlich eig genau wissen wie schnell wir im ersten punkt fahren
	float step;
	for(int i=0; i<N_STEPS;i++)		//Where is first ref point?? at actual point or after first Ts?
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
		ref_x_.push_back(local.x);
		ref_y_.push_back(local.y);
		j=j_temp; 
		v_ref=vRef(j);
		ref_v_.push_back(v_ref);
	}
}
*/
float MPC::nextReferenceXPolynomial(float x_start, float step)		//interpolates linearly at the end.
{
	float x_run=x_start;
	float x_run_old=x_start;
	float sum=0;
	float sum_old=0;
	float dx=0.1;	//Finesse of integration 
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
	    //Upper buonds
		//MAX_ABSOLUTE_VELOCITY
		float v_bounded=std::min(teach_vel_[i_vel]+V_FREEDOM,MAX_ABSOLUTE_VELOCITY);
		float C=1;
	    //Slow down
		if (i_vel>=slow_down_index_)
			{
			//Lineares herrunterschrauben
			C=C*((distanceIJ(i_vel,n_poses_path_-1))-SLOW_DOWN_PUFFER)/(SLOW_DOWN_DISTANCE-SLOW_DOWN_PUFFER);
			}
		v_ref_[i_vel] =std::max(v_bounded * C,float(0));

	}
	//Write path into txt file
	std::string teachpoints= "/home/moritz/catkin_ws/src/arc_mpc/text/teach_global_points.txt";
	std::ofstream streamteachpoints(teachpoints.c_str(), std::ios::out);
	int i_start = 0;
	int i_end = n_poses_path_;
	geometry_msgs::Point p;
	for (int i=i_start; i<i_end; i++)
	{
		p=path_.poses[i].pose.position;
	  	streamteachpoints <<p.x<<" "<<
	           p.y<<"\r\n";
	}
	streamteachpoints.close();

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

void MPC::writeTxt()	//write for test
{
	std::string pointsinterpol= "/home/moritz/catkin_ws/src/arc_mpc/text/pointsinterpol.txt";
	std::ofstream streampinterp(pointsinterpol.c_str(), std::ios::out);
	int i_start = state_.current_arrayposition;
	int i_end = indexOfDistanceFront(i_start, INTERPOLATION_DISTANCE_FRONT).x;
	int lenght = i_end - i_start;
	geometry_msgs::Point p;
	for (int i=i_start; i<i_end; i++)
	{
		p=arc_tools::globalToLocal(path_.poses[i].pose.position,state_);
	  	streampinterp <<p.x<<" "<<
	           p.y<<"\r\n";
	}
	streampinterp.close();

	std::string pointsreference= "/home/moritz/catkin_ws/src/arc_mpc/text/pointsreference.txt";
	std::ofstream streamprefe(pointsreference.c_str(), std::ios::out);
	for (int i=0; i<N_STEPS; i++)
	{
		streamprefe <<ref_x_[i]<<" "<<ref_y_[i]<<"\r\n";
	}	
	streamprefe.close();

	std::string pointsplaned= "/home/moritz/catkin_ws/src/arc_mpc/text/pointsplaned.txt";
	std::ofstream streamplaned(pointsplaned.c_str(), std::ios::out);

	streamplaned <<solver_output_.x01[2]<<" "<<solver_output_.x01[3]<<"\r\n";	
	streamplaned <<solver_output_.x02[2]<<" "<<solver_output_.x02[3]<<"\r\n";	
	streamplaned <<solver_output_.x03[2]<<" "<<solver_output_.x03[3]<<"\r\n";	
	streamplaned <<solver_output_.x04[2]<<" "<<solver_output_.x04[3]<<"\r\n";	
	streamplaned <<solver_output_.x05[2]<<" "<<solver_output_.x05[3]<<"\r\n";	
	streamplaned <<solver_output_.x06[2]<<" "<<solver_output_.x06[3]<<"\r\n";	
	streamplaned <<solver_output_.x07[2]<<" "<<solver_output_.x07[3]<<"\r\n";	
	streamplaned <<solver_output_.x08[2]<<" "<<solver_output_.x08[3]<<"\r\n";	
	streamplaned <<solver_output_.x09[2]<<" "<<solver_output_.x09[3]<<"\r\n";
	streamplaned <<solver_output_.x10[2]<<" "<<solver_output_.x10[3]<<"\r\n";	
	streamplaned <<solver_output_.x11[2]<<" "<<solver_output_.x11[3]<<"\r\n";	
	streamplaned <<solver_output_.x12[2]<<" "<<solver_output_.x12[3]<<"\r\n";	
	streamplaned <<solver_output_.x13[2]<<" "<<solver_output_.x13[3]<<"\r\n";	
	streamplaned <<solver_output_.x14[2]<<" "<<solver_output_.x14[3]<<"\r\n";	
	streamplaned <<solver_output_.x15[2]<<" "<<solver_output_.x15[3]<<"\r\n";	
	streamplaned <<solver_output_.x16[2]<<" "<<solver_output_.x16[3]<<"\r\n";	
	streamplaned <<solver_output_.x17[2]<<" "<<solver_output_.x17[3]<<"\r\n";	
	streamplaned <<solver_output_.x18[2]<<" "<<solver_output_.x18[3]<<"\r\n";	
	streamplaned <<solver_output_.x19[2]<<" "<<solver_output_.x19[3]<<"\r\n";	
	streamplaned <<solver_output_.x20[2]<<" "<<solver_output_.x20[3]<<"\r\n";
	streamplaned.close();
	
	std::cout<<"writeTxt: "<<"f(x)="<<poly_a_<<"*x*x*x+"<<poly_b_<<"*x*x+"<<poly_c_<<"*x+"<<poly_d_<<std::endl;

}

Eigen::MatrixXd MPC::pathToMatrix(float lad)  //Let's see

{
	int i_start = state_.current_arrayposition;
	int i_end=indexOfDistanceFront(i_start, lad).x;
	Eigen::MatrixXd d(2,i_end-i_start);
	for (int i=0; i<i_end-i_start; i++){
	geometry_msgs::Point p= arc_tools::globalToLocal(path_.poses[state_.current_arrayposition+i].pose.position, state_);
	d(0,i) = p.x;
	d(1,i) = p.y;	
	}
	return d;
}

void MPC::calculateParamFun(float lad_interpolation)
{
	Eigen::MatrixXd d=pathToMatrix(lad_interpolation);
	int i_start = state_.current_arrayposition;
	int i_end = indexOfDistanceFront(i_start, lad_interpolation).x;
	int lenght = i_end - i_start;
	float sum1 = d.row(0).sum();
	float sum2 = d.row(0).cwiseAbs2().sum();
	float sum3 = 0;
	for (int i=0; i<lenght; i++)
	{
		sum3 += pow(d(0, i), 3.0); 
	}

	float sum4 = 0;
	for (int i=0; i<lenght; i++)
	{
		sum4 += pow(d(0, i), 4.0); 
	}

	float sum5 = 0;
	for (int i=0; i<lenght; i++)
	{
		sum5 += pow(d(0, i), 5.0); 
	}

	float sum6 = 0;
	for (int i=0; i<lenght; i++)
	{
		sum6 += pow(d(0, i), 6.0); 
	}
	Eigen::MatrixXd A = Eigen::MatrixXd::Zero(4,4);
	A << lenght, sum1, sum2, sum3, sum1, sum2, sum3, sum4, sum2, sum3, sum4, sum5, sum3, sum4, sum5, sum6;
//	std::cout << A << std::endl;

	float sum_rhs = 0;
	for (int i=0; i<lenght; i++)
	{
		sum_rhs += pow(d(0,i), 3.0)*d(1,i); 
	}

	Eigen::VectorXd rhs(4); 
	rhs << d.row(1).sum(), d.row(1).dot(d.row(0)), d.row(1).dot(d.row(0).cwiseAbs2()), sum_rhs;
//	std::cout << rhs << std::endl;

	Eigen::Vector4d a = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(rhs);//
	

	poly_a_ =a(3); 
	poly_b_= a(2);
	poly_c_= a(1);
	poly_d_= a(0);
	
}


void MPC::getOutputAndReact()
{
	int flag=arc_solver_solve(&solver_param_, &solver_output_, &solver_info_,stdout, pt2Function);
	//write txt file
	writeTxt();
	std::cout<<"Txt written "<<std::endl;
	if(flag==1)
	{
	//Set inputs
	u_.steering_angle=solver_output_.x01[1];
	u_.speed=solver_output_.x01[4];
	u_.acceleration=solver_output_.x01[0];
	pub_stellgroessen_.publish(u_);
/*	matlab_output_1_.data.clear();
	matlab_output_1_.data.push_back(solver_output_.x01[0]);
	matlab_output_1_.data.push_back(solver_output_.x02[0]);
	matlab_output_1_.data.push_back(solver_output_.x03[0]);
	matlab_output_1_.data.push_back(solver_output_.x04[0]);
	matlab_output_1_.data.push_back(solver_output_.x05[0]);
	matlab_output_1_.data.push_back(solver_output_.x06[0]);
	matlab_output_1_.data.push_back(solver_output_.x07[0]);
	matlab_output_1_.data.push_back(solver_output_.x08[0]);
	matlab_output_1_.data.push_back(solver_output_.x09[0]);
	pub_output_1_.publish(matlab_output_1_);
	matlab_output_2_.data.clear();
	matlab_output_2_.data.push_back(solver_output_.x01[1]);
	matlab_output_2_.data.push_back(solver_output_.x02[1]);
	matlab_output_2_.data.push_back(solver_output_.x03[1]);
	matlab_output_2_.data.push_back(solver_output_.x04[1]);
	matlab_output_2_.data.push_back(solver_output_.x05[1]);
	matlab_output_2_.data.push_back(solver_output_.x06[1]);
	matlab_output_2_.data.push_back(solver_output_.x07[1]);
	matlab_output_2_.data.push_back(solver_output_.x08[1]);
	matlab_output_2_.data.push_back(solver_output_.x09[1]);
	pub_output_2_.publish(matlab_output_2_);

std::cout<<"output0 "<<solver_output_.x01[0]<<std::endl;
std::cout<<"output0 "<<solver_output_.x02[0]<<std::endl;
std::cout<<"output0 "<<solver_output_.x03[0]<<std::endl;
std::cout<<"output0 "<<solver_output_.x04[0]<<std::endl;
std::cout<<"output0 "<<solver_output_.x05[0]<<std::endl;
std::cout<<"output0 "<<solver_output_.x06[0]<<std::endl;
std::cout<<"output0 "<<solver_output_.x07[0]<<std::endl;
std::cout<<"output0 "<<solver_output_.x08[0]<<std::endl;
std::cout<<"output0 "<<solver_output_.x09[0]<<std::endl;
std::cout<<std::endl;
std::cout<<"output1 "<<solver_output_.x01[1]<<std::endl;
std::cout<<"output1 "<<solver_output_.x02[1]<<std::endl;
std::cout<<"output1 "<<solver_output_.x03[1]<<std::endl;
std::cout<<"output1 "<<solver_output_.x04[1]<<std::endl;
std::cout<<"output1 "<<solver_output_.x05[1]<<std::endl;
std::cout<<"output1 "<<solver_output_.x06[1]<<std::endl;
std::cout<<"output1 "<<solver_output_.x07[1]<<std::endl;
std::cout<<"output1 "<<solver_output_.x08[1]<<std::endl;
std::cout<<"output1 "<<solver_output_.x09[1]<<std::endl;
std::cout<<std::endl;

std::cout<<"output2 "<<solver_output_.x1[2]<<std::endl;
std::cout<<"output2 "<<solver_output_.x2[2]<<std::endl;
std::cout<<"output2 "<<solver_output_.x3[2]<<std::endl;
std::cout<<"output2 "<<solver_output_.x4[2]<<std::endl;
std::cout<<"output2 "<<solver_output_.x5[2]<<std::endl;
std::cout<<"output2 "<<solver_output_.x6[2]<<std::endl;
std::cout<<"output2 "<<solver_output_.x7[2]<<std::endl;
std::cout<<"output2 "<<solver_output_.x8[2]<<std::endl;
std::cout<<"output2 "<<solver_output_.x9[2]<<std::endl;
std::cout<<std::endl;
std::cout<<"output3 "<<solver_output_.x1[3]<<std::endl;
std::cout<<"output3 "<<solver_output_.x2[3]<<std::endl;
std::cout<<"output3 "<<solver_output_.x3[3]<<std::endl;
std::cout<<"output3 "<<solver_output_.x4[3]<<std::endl;
std::cout<<"output3 "<<solver_output_.x5[3]<<std::endl;
std::cout<<"output3 "<<solver_output_.x6[3]<<std::endl;
std::cout<<"output3 "<<solver_output_.x7[3]<<std::endl;
std::cout<<"output3 "<<solver_output_.x8[3]<<std::endl;
std::cout<<"output3 "<<solver_output_.x9[3]<<std::endl;
std::cout<<std::endl;
std::cout<<"output4 "<<solver_output_.x1[4]<<std::endl;
std::cout<<"output4 "<<solver_output_.x2[4]<<std::endl;
std::cout<<"output4 "<<solver_output_.x3[4]<<std::endl;
std::cout<<"output4 "<<solver_output_.x4[4]<<std::endl;
std::cout<<"output4 "<<solver_output_.x5[4]<<std::endl;
std::cout<<"output4 "<<solver_output_.x6[4]<<std::endl;
std::cout<<"output4 "<<solver_output_.x7[4]<<std::endl;
std::cout<<"output4 "<<solver_output_.x8[4]<<std::endl;
std::cout<<"output4 "<<solver_output_.x9[4]<<std::endl;
std::cout<<std::endl;
std::cout<<"output5 "<<solver_output_.x1[5]<<std::endl;
std::cout<<"output5 "<<solver_output_.x2[5]<<std::endl;
std::cout<<"output5 "<<solver_output_.x3[5]<<std::endl;
std::cout<<"output5 "<<solver_output_.x4[5]<<std::endl;
std::cout<<"output5 "<<solver_output_.x5[5]<<std::endl;
std::cout<<"output5 "<<solver_output_.x6[5]<<std::endl;
std::cout<<"output5 "<<solver_output_.x7[5]<<std::endl;
std::cout<<"output5 "<<solver_output_.x8[5]<<std::endl;
std::cout<<"output5 "<<solver_output_.x9[5]<<std::endl;
std::cout<<std::endl;
std::cout<<"output6 "<<solver_output_.x1[6]<<std::endl;
std::cout<<"output6 "<<solver_output_.x2[6]<<std::endl;
std::cout<<"output6 "<<solver_output_.x3[6]<<std::endl;
std::cout<<"output6 "<<solver_output_.x4[6]<<std::endl;
std::cout<<"output6 "<<solver_output_.x5[6]<<std::endl;
std::cout<<"output6 "<<solver_output_.x6[6]<<std::endl;
std::cout<<"output6 "<<solver_output_.x7[6]<<std::endl;
std::cout<<"output6 "<<solver_output_.x8[6]<<std::endl;
std::cout<<"output6 "<<solver_output_.x9[6]<<std::endl;
std::cout<<std::endl;
*/
	}
	else if(flag==0)
	{
	u_.steering_angle=solver_output_.x01[1];
	u_.speed=solver_output_.x01[4];
	u_.acceleration=solver_output_.x01[0];
	pub_stellgroessen_.publish(u_);
	std::cout<<"Timeout"<<std::endl;
	}
	else if(flag==-6)
	{
	std::cout<<"NAN or inf occured"<<std::endl;
	}
	else if(flag==-7)
	{
	std::cout<<"Infeasible"<<std::endl;
	}
	else if(flag==-100)
	{
	std::cout<<"Licence error"<<std::endl;
	}
}

int MPC::localPointToPathIndex(geometry_msgs::Point p, int i_start, int i_end)
{	
	geometry_msgs::Point p_global=localToGlobal(p,state_);		//Transformation local to global
//std::cout<<"p global "<<p_global<<std::endl;
//std::cout<<"pathpoint start "<<path_.poses[i_start].pose.position<<std::endl;
	int j;
	float distance_old=100;
	float distance_new;
	for(int i=i_start; i<i_end;i++)
	{
		distance_new=sqrt(	pow((path_.poses[i].pose.position.x-p_global.x),2) 
					+ 	pow((path_.poses[i].pose.position.y-p_global.y),2));
//std::cout<<"distance_new "<<distance_new<<" d_old "<<distance_old<<std::endl;
		
		if(distance_new<distance_old)
		{
		
//std::cout<<"AAAAAAA";
			j=i;
			distance_old=distance_new;
			
		}	
	}
//	std::cout<<"nearest index: "<<j<<std::endl<<path_.poses[j-1].pose.position<<std::endl;
	return j;
}

geometry_msgs::Point MPC::localToGlobal(geometry_msgs::Point p_local, arc_msgs::State state_)
{	
//std::cout<<"Localtoglobal:local point "<<p_local<<std::endl; 
	//Change local coordinates
	float temp=p_local.x;
	p_local.x=p_local.y;
	p_local.y=-temp;
//std::cout<<"Local point in global coordinate convention "<<p_local<<std::endl;  
	//Rotate              
	geometry_msgs::Point p_add=arc_tools::rotationLocalToGlobal(p_local,state_);
//std::cout<<"Local point rotated "<<p_add<<std::endl; 
	geometry_msgs::Point p_global=state_.pose.pose.position;
	p_global.x+=p_add.x;
	p_global.y+=p_add.y;
//std::cout<<"Localtoglobal:Global position "<<p_global<<std::endl;
	return p_global;
}

float MPC::yPoly(float x)
{
	float y;
	y=(poly_a_ *x*x*x + poly_b_ *x*x + poly_c_ *x + poly_d_) ;
	return y;
}

float MPC::radiusPoly(float x)
{
	float radius;
	radius=fabs((pow((1+pow(3*poly_a_*x*x+2*poly_b_*x+poly_c_,2)),1.5))/(6*poly_a_*x+2*poly_b_));
	return radius;
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
{
}
geometry_msgs::Vector3 MPC::indexOfDistanceFront(int i, float d)
{
	if (i>=n_poses_path_-1) i=n_poses_path_-1;
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
//std::cout<<"Index "<<vector.x;
	vector.y=l;
	vector.z=l-sqrt(	pow(path_.poses[j].pose.position.x - path_.poses[j-1].pose.position.x,2)+
				pow(path_.poses[j].pose.position.y - path_.poses[j-1].pose.position.y,2)+
				pow(path_.poses[j].pose.position.z - path_.poses[j-1].pose.position.z,2));
	return vector;
}

geometry_msgs::Vector3 MPC::indexOfDistanceBack(int i, float d)
{
	geometry_msgs::Vector3 vector;	//vector.x= index  vector.z=real distance upp er  vector.z=real distance lower 
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

geometry_msgs::Point MPC::pointAtDistanceLinear(int i, float distance)
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

