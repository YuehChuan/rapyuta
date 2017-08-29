#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "navigation_controller/command.h"

class NavigationController {
public:
	NavigationController(tf::TransformListener& tf);
	~NavigationController();
	bool set_command(navigation_controller::command::Request &req,navigation_controller::command::Response &res);
	void run();
private:
	ros::NodeHandle nh_;
	ros::Publisher vel_pub_;
	ros::Subscriber scan_sub_;
	ros::Subscriber poscmd_sub_;
	ros::Publisher mark_array_pub_;

	tf::TransformListener& tf_;	
	sensor_msgs::LaserScan scan_;
	visualization_msgs::MarkerArray mark_array;

	enum NavigationState {
		CONTROLLING,
		APPROACHING,
		FINISH
	};
	NavigationState state_;
	unsigned int command_type_;
	geometry_msgs::PoseStamped goal_;

	void get_scan(const sensor_msgs::LaserScan::ConstPtr& scan);
	void get_pos(const std_msgs::Float64MultiArray::ConstPtr& array);

	void angle_control(double *cmd_w, double des_theta, double current_theta);
	void position_control(double *cmd_v, double *cmd_w, double desX, double desY, double currentX, double currentY, double current_theta);
	void avoid_radiate(double *V_o, double *W_o,double userVelocity,double userRotation,double *beam_minrange,double *beam_angle,double minrange);


	static const int beam_num;
	static const double kAvoidRadius_;

	static const double D2R;
	static const double R2D;
	static const double MaxRobotLinearVelocity;
	static const double MaxRobotAngularVelocity;
};

const double NavigationController::D2R = M_PI / 180.0;			// degrees to rad
const double NavigationController::R2D = 180.0 / M_PI;			// rad to degrees
const double NavigationController::MaxRobotLinearVelocity = 0.20;		// m/s
const double NavigationController::MaxRobotAngularVelocity = 80.0 * D2R;	// radian/s

const int NavigationController::beam_num = 10;
const double NavigationController::kAvoidRadius_ = 1.0;	//m

NavigationController::NavigationController(tf::TransformListener& tf) : tf_(tf)
{
	vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	scan_sub_ = nh_.subscribe("scan", 1, &NavigationController::get_scan, this);
	poscmd_sub_ = nh_.subscribe("pos_cmd", 1, &NavigationController::get_pos, this);
	mark_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>( "mark_array", 1 );

	command_type_ = 0;
	state_ = FINISH;

	ROS_INFO_STREAM("contrust the walk assist control class");
}

NavigationController::~NavigationController()
{
	ROS_INFO_STREAM("delete the navigation controller class");
}

void NavigationController::get_pos(const std_msgs::Float64MultiArray::ConstPtr& array)
{
	if (array->data.size() == 2) {
		goal_.header.stamp = ros::Time::now();
		goal_.pose.position.x = array->data[0];
		goal_.pose.position.y = array->data[1];
		goal_.pose.orientation.z = 0.0;
		state_ = CONTROLLING;
		command_type_ = 2;
		ROS_INFO_STREAM("Control postition, X = " << goal_.pose.position.x << " Y = " << goal_.pose.position.y );
	}
	else if(array->data.size() == 1) {
		goal_.header.stamp = ros::Time::now();
		goal_.pose.position.x = 0.0;
		goal_.pose.position.y = 0.0;
		goal_.pose.orientation.z = array->data[0];
		state_ = CONTROLLING;
		command_type_ = 1;
		ROS_INFO_STREAM("Control orientation, theta = " << goal_.pose.orientation.z );
	}
	
}
 
bool NavigationController::set_command(navigation_controller::command::Request &req,navigation_controller::command::Response &res)
{
	if (req.type == 2) {
		goal_.header.stamp = ros::Time::now();
		goal_.pose.position.x = req.x;
		goal_.pose.position.y = req.y;
		goal_.pose.orientation.z = 0.0;
		state_ = CONTROLLING;
		command_type_ = req.type;
		ROS_INFO_STREAM("Control postition, X = " << goal_.pose.position.x << " Y = " << goal_.pose.position.y );
	}
	else if(req.type == 1) {
		goal_.header.stamp = ros::Time::now();
		goal_.pose.position.x = 0.0;
		goal_.pose.position.y = 0.0;
		goal_.pose.orientation.z = req.theta;
		state_ = CONTROLLING;
		command_type_ = req.type;
		ROS_INFO_STREAM("Control orientation, theta = " << goal_.pose.orientation.z );
	}
	else if(req.type == 0) {
		if(state_ == CONTROLLING) {
			res.run_completed = false;
		}
		else {
			res.run_completed = true;
		}
	}
	return true;
}


void NavigationController::run()
{
	ros::Rate rate(10);

	//total command
	double v_t = 0.0;
	double w_t = 0.0;
	//user command
	double v_u = 0.0;
	double w_u = 0.0;
	//robot command
	double v_r = 0.0;
	double w_r = 0.0;
	//navigation command
	double v_n = 0.0;
	double w_n = 0.0;
	//avoid command
	double v_a = 0.0;
	double w_a = 0.0;

	double Es = 0.0;	//confidence factor of safety
	double Ea = 0.0;	//confidence factor of avoidance
	double Et = 0.0;	//confidence factor of time

	double G_u = 0.0;	//gain of user
	double G_r = 0.0;	//gain of robot
	geometry_msgs::Twist cmd;
	while (ros::ok())
	{
		//====================================================
			int beam_range = scan_.ranges.size() / beam_num;
			double beam_minrange[beam_num];		//m
			double beam_angle[beam_num];		//rad
			double minrange;			//m
			double adjust;				//m
			for(int i= 0; i < beam_num; i++) {
				if(i >= (beam_num / 3) && i <= (beam_num * 2 / 3))
					adjust = 0.2;
				else
					adjust = 0.3;

				beam_minrange[i] = scan_.range_max;
				for(int j = i * beam_range; j < (i+1) * beam_range; j++) { 
					if(std::isnormal(scan_.ranges[j]))
						beam_minrange[i] = std::min(beam_minrange[i],(double)std::max((scan_.ranges[j]-adjust), 0.0));
				}
				beam_angle[i] = scan_.angle_min + ((double)i + 0.5) * (double)beam_range * scan_.angle_increment;
				//ROS_INFO_STREAM("angle " << i << ": " << beam_angle[i] * R2D << " min : " << beam_minrange[i]);
			}

			minrange = beam_minrange[0];
			for(int i= 0; i < beam_num; i++) {
				if(beam_minrange[i] != 0.0)
					minrange = std::min(minrange,beam_minrange[i]);
			}
			
			mark_array.markers.clear();
			ros::Time current_time = ros::Time::now();
			for(int i = 0; i < beam_num; i++) {
				visualization_msgs::Marker marker;
				marker.header.frame_id = "laser";
				marker.header.stamp = current_time;
				marker.ns = "marker_test_arrow_by_points" + i;
				marker.type = visualization_msgs::Marker::ARROW;
				marker.action = visualization_msgs::Marker::ADD;
				marker.pose.position.x = 0.0;
				marker.pose.position.y = 0.0;
				marker.pose.position.z = 0.0;
				marker.pose.orientation.x = 0.0;
				marker.pose.orientation.y = 0.0;
				marker.pose.orientation.z = sin(beam_angle[i]/2);
				marker.pose.orientation.w = cos(beam_angle[i]/2);
				if(beam_minrange[i] <= 0)
					marker.scale.x = 0.00001;
				else
					marker.scale.x = beam_minrange[i];
				marker.scale.y = 0.01;
				marker.scale.z = 0.01;
				marker.color.r = 0;
				marker.color.g = 1.0;
				marker.color.b = 1.0;
				marker.color.a = 1.0;
				mark_array.markers.push_back(marker);
			}
			//==================================
		if(state_ == CONTROLLING || state_ == APPROACHING)
		{
			
			v_t = 0.0;
			w_t = 0.0;
			v_u = 0.0;
			w_u = 0.0;
			v_r = 0.0;
			w_r = 0.0;
			v_n = 0.0;
			w_n = 0.0;
			v_a = 0.0;
			w_a = 0.0;

			//get robot position
			tf::StampedTransform transform;
			tf::Quaternion q;
			try{
				tf_.lookupTransform("/world", "/base_link", ros::Time(0), transform);
				q = transform.getRotation(); 
			}
			catch (tf::TransformException ex){	
				ROS_ERROR("%s",ex.what());
				ros::Duration(1.0).sleep();
			}
			//ROS_INFO_STREAM("x = " << transform.getOrigin().x() * 100 << " y = " << transform.getOrigin().y() * 100 << " theta = " << tf::getYaw(q) * R2D);

			//calculate auto navigation v & w
			if(command_type_ == 2) {
				position_control(&v_n,&w_n,goal_.pose.position.x,goal_.pose.position.y,transform.getOrigin().x(),transform.getOrigin().y(),tf::getYaw(q));
				if(v_n == 0.0 && w_n == 0.0)
					state_ = FINISH;
				else if(sqrt(pow(goal_.pose.position.x-transform.getOrigin().x(),2)+pow(goal_.pose.position.y-transform.getOrigin().y(),2)) < 1.5)
					state_ = APPROACHING;
			}
			else if(command_type_ == 1) {
				angle_control(&w_n,goal_.pose.orientation.z,tf::getYaw(q));
				if(w_n == 0.0)
					state_ = FINISH;
				else if(fabs(goal_.pose.orientation.z-tf::getYaw(q)) < 10 * D2R)
					state_ = APPROACHING;
			}

			//Ea, confidence factor of avoidance
			Ea = std::min(minrange / kAvoidRadius_ , 1.0);  //obstacle avoid inside #kAvoidRadius_ m
			Ea = pow(Ea,1);			
			if (Ea >= 1.0)
				Ea = 1.0;
			else if (Ea <= 0.0)
				Ea = 0.0;

			G_u = Ea;

			G_u = 1;		//for no obstacle avoidance
			G_r = 1 - G_u;
			ROS_INFO_STREAM("G_u = " << G_u << " G_r = " << G_r);		

			//navigation controller join to control robot
			if (v_n != 0.0 || w_n != 0.0) {
				avoid_radiate(&v_a, &w_a, v_n, v_n / MaxRobotLinearVelocity * MaxRobotAngularVelocity,beam_minrange,beam_angle,minrange);

				v_r = v_a;
				w_r = w_a;

				//v_t = v_n * G_u + v_r * G_r;
				v_t = v_n * G_u;
				w_t = w_n * G_u + w_r * G_r;

				if(v_n != 0) {
					w_t = w_t *  v_n / v_t;
					v_t = v_n;
				}
				else if(v_n == 0) {
					w_t = w_n;
				}

			}
			else {
				;
			}

			//ROS_INFO_STREAM("v_t = " << v_t << " w_t = " << w_t);

			//send command to controller
			cmd.linear.x = v_t;
			cmd.angular.z = w_t;
			vel_pub_.publish(cmd);
		}
		else {
			cmd.linear.x = 0.0;
			cmd.angular.z = 0.0;
		}
		mark_array_pub_.publish(mark_array);
		//vel_pub_.publish(cmd);
		ros::spinOnce();
		rate.sleep();
	}
}

void NavigationController::get_scan(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	scan_ = *scan;
}

void NavigationController::avoid_radiate(double *V_o, double *W_o,double userVelocity,double userRotation,double *beam_minrange,double *beam_angle,double minrange)
{
	double obsVX = 0.0;
	double obsVY = 0.0;

	int count = 0;
	double tempV = 0.0;
	for (int i = 0; i < beam_num; i++) {
		tempV = 0.0;
		if (beam_minrange[i] < kAvoidRadius_) {
			tempV = userVelocity * (1.0 - pow((beam_minrange[i] / kAvoidRadius_),1));

			obsVX += tempV * cos(beam_angle[i]);
			obsVY += tempV * sin(beam_angle[i]);
			count++;
		}
	}
	if(count != 0) {
		obsVX /= count;
		obsVY /= count;
	}
	else {
		obsVX = 0;
		obsVY = 0;
	}

	double sumV;
	double sumTheta;

	if(obsVY != 0 || obsVX != 0) {
		sumV = sqrt(pow(obsVX, 2) + pow(obsVY, 2));
		sumTheta = atan2(obsVY, obsVX);
	}
	else {
		sumV = 0.0;
		sumTheta = 0.0;
	}

	visualization_msgs::Marker marker;
	marker.header.frame_id = "laser";
	marker.header.stamp = ros::Time::now();
	marker.ns = "marker_test_arrow_by_points obs";
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0.0;
	marker.pose.position.y = 0.0;
	marker.pose.position.z = 0.0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = sin(sumTheta/2);
	marker.pose.orientation.w = cos(sumTheta/2);
	if(sumV <= 0)
		marker.scale.x = 0.00001;
	else
		marker.scale.x = sumV;
	marker.scale.y = 0.01;
	marker.scale.z = 0.01;
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
	marker.color.a = 1.0;
	mark_array.markers.push_back(marker);

	*V_o = userVelocity;// * std::min(minrange / kAvoidRadius_ , 1.0);
	if (sumTheta >= 0.0 && count != 0)
		*W_o = (sumTheta - M_PI / 2.0) / 3.0;
	else if (sumTheta < 0.0)
		*W_o = (sumTheta + M_PI / 2.0) / 3.0;
	else
		*W_o = 0.0;
	//ROS_INFO_STREAM("*W_o = " << *W_o << " *W_o = " << *W_o);
}


//(rad/s ,rad ,rad)
void NavigationController::angle_control(double *cmdW, double desTheta, double currentTheta)
{
	double deltaTheta = 0.0;
	double kp = 2.0;
	double w = 0.0;

	//angle between -180 ~ 180 degree
	if (desTheta > M_PI)
		desTheta = desTheta - int((desTheta + M_PI) / (2 * M_PI)) * (2 * M_PI);
	else
		desTheta = desTheta - int((desTheta - M_PI) / (2 * M_PI)) * (2 * M_PI);

	//angle between -180 ~ 180 degree
	if (currentTheta > M_PI)
		currentTheta = currentTheta - int((desTheta + M_PI) / (2 * M_PI)) * (2 * M_PI);
	else
		currentTheta = currentTheta - int((desTheta - M_PI) / (2 * M_PI)) * (2 * M_PI);
	
	deltaTheta = desTheta - currentTheta;
	//in order not to turn opposite direction
	if (deltaTheta > M_PI)
		deltaTheta -= 2.0 * M_PI;
	else if (deltaTheta < -M_PI)
		deltaTheta += 2.0 * M_PI;

	w = kp * deltaTheta;

	if (fabs(w) > fabs(MaxRobotAngularVelocity))
		w = w / fabs(w) * fabs(MaxRobotAngularVelocity);
	else
		w = w / fabs(w) * std::max(5.0 * D2R, fabs(w));

	if (fabs(deltaTheta) <= 1.0 * D2R)
		w = 0.0;

	//return command
	*cmdW = w;
}

//(m/s ,rad/s ,m ,m ,m ,m ,rad)
void NavigationController::position_control(double *cmdV, double *cmdW, double desX, double desY, double currentX, double currentY, double currentTheta)
{
	double kp = 2.0;		//kp of rotation 
	double v = 0.0;
	double w = 0.0;

	double deltaX = 0.0;		//m
	double deltaY = 0.0;		//m
	double deltaR = 0.0;		//m
	double angle = 0.0;		//radian
	double deltaTheta = 0.0;	//radian

	deltaX = desX - currentX;
	deltaY = desY - currentY;
	deltaR = sqrt(pow(deltaX, 2) + pow(deltaY, 2));	
	//in radian between -pi ~ pi
	angle = atan2(deltaY, deltaX);	
	//angle between -180 ~ 180 degree
	if (currentTheta > M_PI)
		currentTheta = currentTheta - int((currentTheta + M_PI) / (2 * M_PI)) * (2 * M_PI);
	else
		currentTheta = currentTheta - int((currentTheta - M_PI) / (2 * M_PI)) * (2 * M_PI);

	deltaTheta = angle - currentTheta;
	//in order not to turn opposite direction
	if (deltaTheta > M_PI)
		deltaTheta -= 2.0 * M_PI;
	else if (deltaTheta < -M_PI)
		deltaTheta += 2.0 * M_PI;
/*
	if (fabs(deltaTheta) > 40 * D2R) {
		angle_control(&w, angle, currentTheta);
		*cmdV = 0.0;
		*cmdW = w;
		return;
	}
*/
	w = kp * deltaTheta;
	if (w == 0.0)	//not to divide by zero
		;
	else if (fabs(w) > MaxRobotAngularVelocity)
		w = w / fabs(w) * MaxRobotAngularVelocity;
	else
		w = w / fabs(w) * std::max(5.0 * D2R, fabs(w));	//adjustment for both of wheel speed not too low

	if (deltaR == 0.0)	//not to divide by zero
		;
	else if (fabs(deltaR) > fabs(MaxRobotLinearVelocity * 1.0))
		v = deltaR / fabs(deltaR) * fabs(MaxRobotLinearVelocity);
	else
		v = deltaR / fabs(deltaR) * std::max(0.10, fabs(deltaR / 1.0));	//low speed maybe cause robot can not move

	//reach the goal
	if (fabs(deltaTheta) <= 1.0 * D2R || fabs(deltaR) <= 0.2)
		w = 0.0;
	if (fabs(deltaR) <= 0.2)
		v = 0.0;

	//return command
	*cmdV = v;
	*cmdW = w;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "navigation_controller");
	ros::NodeHandle nh;
	tf::TransformListener tf(ros::Duration(10));
	NavigationController navigation_controller(tf);
	ros::ServiceServer service = nh.advertiseService("pos_cmd", &NavigationController::set_command, &navigation_controller);
	navigation_controller.run();

	ros::Rate rate(10);
	while (ros::ok()) {
		ros::spinOnce();
		rate.sleep();
	}
	ros::spin();
	ros::spin();
	return 0;
}
