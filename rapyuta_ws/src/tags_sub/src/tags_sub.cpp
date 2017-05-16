#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <ros/ros.h>

//PoseArray
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Pose.h"

//apriltags
#include "rapyuta_msgs/AprilTagDetection.h"
#include "rapyuta_msgs/AprilTagDetections.h"

geometry_msgs::PoseArray tagArray;

void tags_cb1(const rapyuta_msgs::AprilTagDetections::ConstPtr& msg)
{
        ROS_INFO_STREAM("Receive callbck");
        int tagsNum = msg->detections.size();
        
        ROS_INFO("rpt01 tagsNum = %d", tagsNum);
//		ROS_INFO("x = %f", msg->detections[0].pose.position.x);
//		ROS_INFO("y = %f", msg->pose.position.y);
//		ROS_INFO("z = %f", msg->pose.position.z);

//		ROS_INFO("qx = %f", msg->pose.orientation.x);
//		ROS_INFO("qy = %f", msg->pose.orientation.y);
//		ROS_INFO("qz = %f", msg->pose.orientation.z);
//		ROS_INFO("qw = %f", msg->pose.orientation.w);
}

//void tags_cb2(const rapyuta_msgs::AprilTagDetections::ConstPtr& msg)
//{
//        int count = sizeof(msg->detections);
//        ROS_INFO("rpt02 = %d", count);
//		ROS_INFO("x = %f", msg->pose.position.x);
//		ROS_INFO("y = %f", msg->pose.position.y);
//		ROS_INFO("z = %f", msg->pose.position.z);
//
//		ROS_INFO("qx = %f", msg->pose.orientation.x);
//		ROS_INFO("qy = %f", msg->pose.orientation.y);
//		ROS_INFO("qz = %f", msg->pose.orientation.z);
//		ROS_INFO("qw = %f", msg->pose.orientation.w);
//}
//void tags_cb3(const rapyuta_msgs::AprilTagDetections::ConstPtr& msg)
//{
//        int count = sizeof(msg->detections);
//        ROS_INFO("rpt03 = %d", count);
//		ROS_INFO("x = %f", msg->pose.position.x);
//		ROS_INFO("y = %f", msg->pose.position.y);
//		ROS_INFO("z = %f", msg->pose.position.z);
//
//		ROS_INFO("qx = %f", msg->pose.orientation.x);
//		ROS_INFO("qy = %f", msg->pose.orientation.y);
//		ROS_INFO("qz = %f", msg->pose.orientation.z);
//		ROS_INFO("qw = %f", msg->pose.orientation.w);
//}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "tags_sub");
	ros::NodeHandle node;
	ros::Subscriber pose1_sub = node.subscribe("/rapyuta01/usb_cam/apriltags/detections", 10, tags_cb1);
//	ros::Subscriber pose2_sub = node.subscribe("/rapyuta02/usb_cam/apriltags/detections", 10, tags_cb2);
//	ros::Subscriber pose3_sub = node.subscribe("/rapyuta03/usb_cam/apriltags/detections", 10, tags_cb3);
	ros::spin();
	return 0;
}
