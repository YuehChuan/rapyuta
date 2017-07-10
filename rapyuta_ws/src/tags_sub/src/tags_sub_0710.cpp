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


/*    Eigen::Quaterniond q;
    q.x() = msg->detections[0].pose.orientation.x;
    q.y() = msg->detections[0].pose.orientation.y;
    q.z() = msg->detections[0].pose.orientation.z;
    q.w() = msg->detections[0].pose.orientation.w;

    Eigen::Matrix3d R = q.toRotationMatrix();
    Eigen::Matrix3d RInv;
    Eigen::Matrix4d R4 = Eigen::Matrix4d::
    RInv=R.inverse();
*/


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

  cam1_initialize = true;
  ROS_INFO("cam1_initialize =%d ",cam1_initialize);
  static_pose_cam1 = InvTransform_cam1;
  }
  br.sendTransform(tf::StampedTransform(InvTransform_cam1, ros::Time::now(), "map", "camera1"));
  br_static_cam1.sendTransform(tf::StampedTransform(static_pose_cam1, ros::Time::now(), "map", "cam1_static"));




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


}//end name space
