#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <ros/ros.h>

//syncronize apriltags subscription
#include<message_filters/subscriber.h>
#include<message_filters/synchronizer.h>
#include<message_filters/sync_policies/approximate_time.h>

//apriltags message
#include "rapyuta_msgs/AprilTagDetection.h"
#include "rapyuta_msgs/AprilTagDetections.h"

#include "tags_sub/tags_sub.h"
void callback(const rapyuta_msgs::AprilTagDetections::ConstPtr& msg)
{

//        int count = sizeof(msg);
//        ROS_INFO("rpt01 = %d", count);
//		ROS_INFO("x = %f", msg->pose.position.x);
//		ROS_INFO("y = %f", msg->pose.position.y);
//		ROS_INFO("z = %f", msg->pose.position.z);
//
//		ROS_INFO("qx = %f", msg->pose.orientation.x);
//		ROS_INFO("qy = %f", msg->pose.orientation.y);
//		ROS_INFO("qz = %f", msg->pose.orientation.z);
//		ROS_INFO("qw = %f", msg->pose.orientation.w);
}

int main(int argc, char **argv)
{
/*
	ros::init(argc, argv, "tags_sub");
    ros::NodeHandle node;
    ROS_INFO("start apriltags subscribe");
    message_filters::Subscriber<rapyuta_msgs::AprilTagDetections>camera1_sub(node, "/rapyuta01/usb_cam/apriltags/detections", 1);
    message_filters::Subscriber<rapyuta_msgs::AprilTagDetections>camera2_sub(node, "/rapyuta02/usb_cam/apriltags/detections", 1);
    message_filters::Subscriber<rapyuta_msgs::AprilTagDetections>camera3_sub(node, "/rapyuta03/usb_cam/apriltags/detections", 1);
	
    typedef message_filters::sync_policies::ApproximateTime<rapyuta_msgs::AprilTagDetections,rapyuta_msgs::AprilTagDetections,rapyuta_msgs::AprilTagDetections> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), camera1_sub,camera2_sub, camera3_sub);
    sync.registerCallback(boost::bind(&tags_c, _1, _2, _3));
    //ros::Subscriber pose1_sub = node.subscribe("/rapyuta01/usb_cam/apriltags/detections", 10, tags_cb);
	//ros::Subscriber pose2_sub = node.subscribe("/rapyuta02/usb_cam/apriltags/detections", 10, tags_cb);
	//ros::Subscriber pose3_sub = node.subscribe("/rapyuta03/usb_cam/apriltags/detections", 10, tags_cb);
	ros::spin();
	return 0;
    */
    return 0;
}


