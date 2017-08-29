#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "command_pub");
	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("pos_cmd", 1000);

	ros::Rate loop_rate(1);

	std_msgs::Float64MultiArray array;

	int num = 0;
	double x = 0.0;
	double y = 0.0;
	double theta = 0.0;
	while (ros::ok()) {
		x = 0.0;
		y = 0.0;
		theta = 0.0;
		scanf("%d ",&num);
		array.data.clear();
		if(num == 2) {
			scanf("%lf %lf",&x,&y);
			array.data.push_back(x);
			array.data.push_back(y);
			ROS_INFO("x = %.3lf y = %.3lf",x,y);
			pub.publish(array);
			}
		else if(num == 1) {
			scanf("%lf",&theta);
			array.data.push_back(theta);
			ROS_INFO("theta = %.3lf",theta);
			pub.publish(array);
		}
    
    

		//ros::spinOnce();
		//loop_rate.sleep();
	}

	return 0;
}
