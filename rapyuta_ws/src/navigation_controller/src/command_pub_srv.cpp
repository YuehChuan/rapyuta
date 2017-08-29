#include "ros/ros.h"
#include "navigation_controller/command.h"
#include <iostream>
#include <fstream>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "command_pub");
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<navigation_controller::command>("pos_cmd");

	std::string file_path;
	ros::NodeHandle para_node("~");
	if(!para_node.getParam("file_path", file_path))
    		file_path = "/home/hosp/catkin_ws_jerry_test/src/navigation_controller/cmd_txt/cmd.txt";
	ROS_INFO_STREAM(file_path);
	int num = 0;
	double x = 0.0;
	double y = 0.0;
	double theta = 0.0;
	navigation_controller::command srv;
	std::fstream myfile(file_path.c_str(), std::ios_base::in);

	while (myfile >> num && ros::ok()) {
		x = 0.0;
		y = 0.0;
		theta = 0.0;
		
		if(num == 1)
			myfile >> theta;
		else if(num == 2)
			myfile >> x >> y;
		else if(num == 0) {
			std::cin.get();
			continue;
		}

		srv.request.type = num;
		srv.request.x = x;
		srv.request.y = y;
		srv.request.theta = theta;

		if(client.call(srv)) {
			ROS_INFO("call service success");
			srv.request.type = 0;
			while(srv.response.run_completed == false) {
				sleep(1);
				client.call(srv);
			}
		}
		else {
			ROS_INFO("call service fail");
		}
	}
	return 0;
}
