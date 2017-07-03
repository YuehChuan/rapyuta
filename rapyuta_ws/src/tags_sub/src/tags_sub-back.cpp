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

//TF
#include <tf/transform_broadcaster.h>

void tags_sub(const rapyuta_msgs::AprilTagDetections::ConstPtr& msg)
{
//  static tf::TransformBroadcaster br;
//  tf::Transform transform;
//  transform.setOrigin(tf::Vector3(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z) );
//  transform.setRotation(tf::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w) );
//  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "camera_link"));
//        int count = sizeof(msg);
//        ROS_INFO("rpt01 = %d", count);
		ROS_INFO("x = %f", msg->detections[0].pose.position.x);
		ROS_INFO("y = %f", msg->detections[0].pose.position.y);
		ROS_INFO("z = %f", msg->detections[0].pose.position.z);

		ROS_INFO("qx = %f", msg->detections[0].pose.orientation.x);
		ROS_INFO("qy = %f", msg->detections[0].pose.orientation.y);
		ROS_INFO("qz = %f", msg->detections[0].pose.orientation.z);
		ROS_INFO("qw = %f", msg->detections[0].pose.orientation.w);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "tags_sub");
    ros::NodeHandle node;
    ROS_INFO("start apriltags subscribe");

/*
    message_filters::Subscriber<rapyuta_msgs::AprilTagDetections>camera1_sub(node, "/rapyuta01/usb_cam/apriltags/detections", 1);
    message_filters::Subscriber<rapyuta_msgs::AprilTagDetections>camera2_sub(node, "/rapyuta02/usb_cam/apriltags/detections", 1);
    message_filters::Subscriber<rapyuta_msgs::AprilTagDetections>camera3_sub(node, "/rapyuta03/usb_cam/apriltags/detections", 1);
	
    typedef message_filters::sync_policies::ApproximateTime<rapyuta_msgs::AprilTagDetections,rapyuta_msgs::AprilTagDetections,rapyuta_msgs::AprilTagDetections> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), camera1_sub,camera2_sub, camera3_sub);
    sync.registerCallback(boost::bind(&callback, this, _1, _2, _3));
  */    
    ros::Subscriber pose1_sub = node.subscribe("/rapyuta01/usb_cam/apriltags/detections", 10, tags_sub);
	ros::Subscriber pose2_sub = node.subscribe("/rapyuta02/usb_cam/apriltags/detections", 10, tags_sub);
	ros::Subscriber pose3_sub = node.subscribe("/rapyuta03/usb_cam/apriltags/detections", 10, tags_sub);
	ros::spin();    
    return 0;

}
