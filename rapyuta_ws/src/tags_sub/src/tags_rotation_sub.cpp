#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <ros/ros.h>

//syncronize apriltags subscription in tags_sub.h
//apriltags message
#include "tags_sub/tags_sub.h"

//TF
#include <tf/transform_broadcaster.h>

//use sophus for transformation
#include "sophus/so3.h"
#include "sophus/se3.h"

void tags_sub1(const rapyuta_msgs::AprilTagDetections::ConstPtr& msg)
{
    if( (msg->detections.empty()) == false )
    {
    ROS_INFO("detections not empty!");
/*    Eigen::Quaterniond q;
    q.x() = msg->detections[0].pose.orientation.x;
    q.y() = msg->detections[0].pose.orientation.y;
    q.z() = msg->detections[0].pose.orientation.z;
    q.w() = msg->detections[0].pose.orientation.w;
    
    Eigen::Matrix3d R = q.toRotationMatrix();
    Eigen::Matrix3d RInv;
    RInv=R.inverse();
*/

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(msg->detections[0].pose.position.x,msg->detections[0].pose.position.y,msg->detections[0].pose.position.z) );
  transform.setRotation(tf::Quaternion(msg->detections[0].pose.orientation.x, msg->detections[0].pose.orientation.y, msg->detections[0].pose.orientation.z, msg->detections[0].pose.orientation.w) );

  //inverse of rotation
  br.sendTransform(tf::StampedTransform(transform.inverse(), ros::Time::now(), "map", "camera1"));

/*        int count = sizeof(msg->detections.size());
        ROS_INFO("detections = %d", count);
		ROS_INFO("x = %f", msg->detections[0].pose.position.x);
		ROS_INFO("y = %f", msg->detections[0].pose.position.y);
		ROS_INFO("z = %f", msg->detections[0].pose.position.z);

		ROS_INFO("qx = %f", msg->detections[0].pose.orientation.x);
		ROS_INFO("qy = %f", msg->detections[0].pose.orientation.y);
		ROS_INFO("qz = %f", msg->detections[0].pose.orientation.z);
		ROS_INFO("qw = %f", msg->detections[0].pose.orientation.w);
*/
        
    }

}

void tags_sub2(const rapyuta_msgs::AprilTagDetections::ConstPtr& msg)
{
    if( (msg->detections.empty()) == false )
    {
    ROS_INFO("detections not empty!");

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(msg->detections[0].pose.position.x,msg->detections[0].pose.position.y,msg->detections[0].pose.position.z) );
  transform.setRotation(tf::Quaternion(msg->detections[0].pose.orientation.x, msg->detections[0].pose.orientation.y, msg->detections[0].pose.orientation.z, msg->detections[0].pose.orientation.w) );
  br.sendTransform(tf::StampedTransform(transform.inverse(), ros::Time::now(), "map", "camera2"));
/*        int count = sizeof(msg->detections.size());
        ROS_INFO("detections = %d", count);
		ROS_INFO("x = %f", msg->detections[0].pose.position.x);
		ROS_INFO("y = %f", msg->detections[0].pose.position.y);
		ROS_INFO("z = %f", msg->detections[0].pose.position.z);

		ROS_INFO("qx = %f", msg->detections[0].pose.orientation.x);
		ROS_INFO("qy = %f", msg->detections[0].pose.orientation.y);
		ROS_INFO("qz = %f", msg->detections[0].pose.orientation.z);
		ROS_INFO("qw = %f", msg->detections[0].pose.orientation.w);
*/
        
    }

}
void tags_sub3(const rapyuta_msgs::AprilTagDetections::ConstPtr& msg)
{
    if( (msg->detections.empty()) == false )
    {
    ROS_INFO("detections not empty!");

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(msg->detections[0].pose.position.x,msg->detections[0].pose.position.y,msg->detections[0].pose.position.z) );
  transform.setRotation(tf::Quaternion(msg->detections[0].pose.orientation.x, msg->detections[0].pose.orientation.y, msg->detections[0].pose.orientation.z, msg->detections[0].pose.orientation.w) );
  br.sendTransform(tf::StampedTransform(transform.inverse(), ros::Time::now(), "map", "camera3"));
/*        int count = sizeof(msg->detections.size());
        ROS_INFO("detections = %d", count);
		ROS_INFO("x = %f", msg->detections[0].pose.position.x);
		ROS_INFO("y = %f", msg->detections[0].pose.position.y);
		ROS_INFO("z = %f", msg->detections[0].pose.position.z);

		ROS_INFO("qx = %f", msg->detections[0].pose.orientation.x);
		ROS_INFO("qy = %f", msg->detections[0].pose.orientation.y);
		ROS_INFO("qz = %f", msg->detections[0].pose.orientation.z);
		ROS_INFO("qw = %f", msg->detections[0].pose.orientation.w);
*/
        
    }

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
    ros::Subscriber pose1_sub = node.subscribe("/rapyuta01/usb_cam/apriltags/detections", 10, tags_sub1);
	ros::Subscriber pose2_sub = node.subscribe("/rapyuta02/usb_cam/apriltags/detections", 10, tags_sub2);
	ros::Subscriber pose3_sub = node.subscribe("/rapyuta03/usb_cam/apriltags/detections", 10, tags_sub3);
	ros::spin();    
    return 0;

}
