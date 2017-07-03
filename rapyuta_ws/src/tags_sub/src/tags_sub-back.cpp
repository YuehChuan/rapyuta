#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <ros/ros.h>

#include "rapyuta_msgs/AprilTagDetection.h"
#include "rapyuta_msgs/AprilTagDetections.h"


void tags_cb(const rapyuta_msgs::AprilTagDetections::ConstPtr& msg)
{
//        int count = sizeof(msg);
//        ROS_INFO("rpt01 = %d", count);
		ROS_INFO("x = %f", msg->pose.position.x);
		ROS_INFO("y = %f", msg->pose.position.y);
		ROS_INFO("z = %f", msg->pose.position.z);

		ROS_INFO("qx = %f", msg->pose.orientation.x);
		ROS_INFO("qy = %f", msg->pose.orientation.y);
		ROS_INFO("qz = %f", msg->pose.orientation.z);
		ROS_INFO("qw = %f", msg->pose.orientation.w);
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "tags_sub");
	ros::NodeHandle node;
	ros::Subscriber pose1_sub = node.subscribe("/rapyuta01/usb_cam/apriltags/detections", 10, tags_cb);
	ros::Subscriber pose2_sub = node.subscribe("/rapyuta02/usb_cam/apriltags/detections", 10, tags_cb);
	ros::Subscriber pose3_sub = node.subscribe("/rapyuta03/usb_cam/apriltags/detections", 10, tags_cb);
	ros::spin();
	return 0;
}
