#include "ros/ros.h"
#include <std_msgs/String.h>
#include "navigation_controller/command.h"
#include <iostream>
#include <fstream>

class CommandPubSrvGui {
public:
	CommandPubSrvGui();
	~CommandPubSrvGui();

private:
	ros::NodeHandle nh_;
	ros::Subscriber place_cmd_;
	ros::ServiceClient client_;
	ros::Publisher reach_pub_;

	void get_place(const std_msgs::String::ConstPtr& place);
};

CommandPubSrvGui::CommandPubSrvGui()
{
	place_cmd_ = nh_.subscribe<std_msgs::String>("place", 1000, boost::bind(&CommandPubSrvGui::get_place, this, _1));
	client_ = nh_.serviceClient<navigation_controller::command>("pos_cmd");
	reach_pub_ = nh_.advertise<std_msgs::String>("Goal", 1000);
}

CommandPubSrvGui::~CommandPubSrvGui()
{
	
}


void CommandPubSrvGui::get_place(const std_msgs::String::ConstPtr& place)
{
	int num = 0;
	double x = 0.0;
	double y = 0.0;
	double theta = 0.0;
	navigation_controller::command srv;

	std::string path = "/home/hosp/catkin_ws_jerry_test/src/navigation_controller/cmd_txt/cmd" + place->data + ".txt";
	std::fstream myfile;
	myfile.open(path.c_str());

	while (myfile >> num && ros::ok()) {
		x = 0.0;
		y = 0.0;
		theta = 0.0;
		
		if(num == 1)
			myfile >> theta;
		else if(num == 2)
			myfile >> x >> y;
		else if(num == 0) {
			continue;
		}
		ROS_INFO_STREAM("x = " << x << " y = " << y << " theta = " << theta);
		srv.request.type = num;
		srv.request.x = x;
		srv.request.y = y;
		srv.request.theta = theta;

		if(client_.call(srv)) {
			ROS_INFO("call service success");
			srv.request.type = 0;
			while(srv.response.run_completed == false) {
				usleep(100000);
				ROS_INFO("hello servie");
				client_.call(srv);
			}
		}
		else {
			ROS_INFO("call service fail");
		}
	}
	std_msgs::String temp;
	temp.data = "1";
	reach_pub_.publish(temp);
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "command_pub_srv_gui");
	CommandPubSrvGui commandpubsrvgui;

	ros::spin();	
	return 0;
}
