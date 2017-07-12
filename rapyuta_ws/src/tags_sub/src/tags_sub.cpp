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
namespace tags_sub
{

TSNode::TSNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) :nh_(nh), nh_private_(nh_private), cam1_initialize(false), cam2_initialize(false), cam3_initialize(false)
{
    cam1_pose_sub_ = nh_.subscribe("/rapyuta01/usb_cam/apriltags/detections", 10, &TSNode::tags_sub1, this);
    cam2_pose_sub_= nh_.subscribe("/rapyuta02/usb_cam/apriltags/detections", 10, &TSNode::tags_sub2, this);
    cam3_pose_sub_ = nh_.subscribe("/rapyuta03/usb_cam/apriltags/detections", 10, &TSNode::tags_sub3, this);

}

TSNode::~TSNode()
{


}



void TSNode::tags_sub1(const rapyuta_msgs::AprilTagDetections::ConstPtr& msg)
{
    if( (msg->detections.empty()) == false )//detection got value
    {


  static tf::TransformBroadcaster br;
  static tf::TransformBroadcaster br_static_cam1;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(msg->detections[0].pose.position.x,msg->detections[0].pose.position.y,msg->detections[0].pose.position.z) );
  transform.setRotation(tf::Quaternion(msg->detections[0].pose.orientation.x, msg->detections[0].pose.orientation.y, msg->detections[0].pose.orientation.z, msg->detections[0].pose.orientation.w) );





  //broadcast transform
  //inverse transform
  tf::Transform InvTransform_cam1;
  static tf::Transform static_pose_cam1;
  InvTransform_cam1 = transform.inverse();

  if(cam1_initialize == false)//next subscriber come change the pose_cam1 value, and next time static frame no value
  {
  static_pose_cam1 = InvTransform_cam1;
  cam1_initialize = true;
  ROS_INFO("cam1_initialize =%d ",cam1_initialize);

  }
  br.sendTransform(tf::StampedTransform(InvTransform_cam1, ros::Time::now(), "map", "camera1"));
  br_static_cam1.sendTransform(tf::StampedTransform(static_pose_cam1, ros::Time::now(), "map", "cam1_static"));


    }

}

void TSNode::tags_sub2(const rapyuta_msgs::AprilTagDetections::ConstPtr& msg)
{
    if( (msg->detections.empty()) == false )
    {
//    ROS_INFO("detections not empty!");

  static tf::TransformBroadcaster br;
  static tf::TransformBroadcaster br_static_cam2;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(msg->detections[0].pose.position.x,msg->detections[0].pose.position.y,msg->detections[0].pose.position.z) );
  transform.setRotation(tf::Quaternion(msg->detections[0].pose.orientation.x, msg->detections[0].pose.orientation.y, msg->detections[0].pose.orientation.z, msg->detections[0].pose.orientation.w) );

  tf::Transform InvTransform_cam2;
  static tf::Transform static_pose_cam2;
  InvTransform_cam2 = transform.inverse();

  if(cam2_initialize == false)//next subscriber come change the pose_cam1 value, and next time static frame no value
  {
  static_pose_cam2 = InvTransform_cam2;
  cam2_initialize = true;
  ROS_INFO("cam2_initialize =%d ",cam2_initialize);
  }



  br.sendTransform(tf::StampedTransform(InvTransform_cam2, ros::Time::now(), "map", "camera2"));
  br_static_cam2.sendTransform(tf::StampedTransform(static_pose_cam2, ros::Time::now(), "map", "cam2_static"));
    }

}
void TSNode::tags_sub3(const rapyuta_msgs::AprilTagDetections::ConstPtr& msg)
{
    if( (msg->detections.empty()) == false )
    {
//    ROS_INFO("detections not empty!");

  static tf::TransformBroadcaster br;
  static tf::TransformBroadcaster br_static_cam3;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(msg->detections[0].pose.position.x,msg->detections[0].pose.position.y,msg->detections[0].pose.position.z) );
  transform.setRotation(tf::Quaternion(msg->detections[0].pose.orientation.x, msg->detections[0].pose.orientation.y, msg->detections[0].pose.orientation.z, msg->detections[0].pose.orientation.w) );

  tf::Transform InvTransform_cam3;
  static tf::Transform static_pose_cam3;
  InvTransform_cam3 = transform.inverse();

  if(cam3_initialize == false)//next subscriber come change the pose_cam1 value, and next time static frame no value
  {
  static_pose_cam3 = InvTransform_cam3;
  cam3_initialize = true;
  ROS_INFO("cam3_initialize =%d ",cam3_initialize);

  }

  br.sendTransform(tf::StampedTransform(InvTransform_cam3, ros::Time::now(), "map", "camera3"));
  br_static_cam3.sendTransform(tf::StampedTransform(static_pose_cam3, ros::Time::now(), "map", "cam3_static"));
    }

}

//get and set cam1 pose
void TSNode::setInitPose_cam1(const Eigen::Matrix4d & pose, double time)
{
  cam1_initialize_pose = pose;
  cam1_initialize_time_ = time;

}

Eigen::Matrix4d TSNode::getInitPose_cam1()
{
  return cam1_initialize_pose;
}

//get and set cam2 pose
void TSNode::setInitPose_cam2(const Eigen::Matrix4d & pose, double time)
{
  cam2_initialize_pose = pose;
  cam2_initialize_time_ = time;

}

Eigen::Matrix4d TSNode::getInitPose_cam2()
{
  return cam2_initialize_pose;
}
//get and set cam3 pose
void TSNode::setInitPose_cam3(const Eigen::Matrix4d & pose, double time)
{
  cam3_initialize_pose = pose;
  cam3_initialize_time_ = time;

}

Eigen::Matrix4d TSNode::getInitPose_cam3()
{
  return cam3_initialize_pose;
}


}//end name space
