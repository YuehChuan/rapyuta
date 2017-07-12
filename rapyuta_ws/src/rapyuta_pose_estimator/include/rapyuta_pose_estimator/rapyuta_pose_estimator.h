#ifndef TAGS_SUB_H_
#define TAGS_SUB_H_

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

//Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

//for math calculating
#include<vector>
#include <tf/transform_broadcaster.h>

namespace rapyuta_pose_estimator
{

class TSNode
{
    private:
       bool cam1_initialize;
       bool cam2_initialize;
       bool cam3_initialize;
       ros::NodeHandle nh_;
       ros::NodeHandle nh_private_;
       ros::Publisher frame_pub_;
       ros::Subscriber cam1_pose_sub_;
       ros::Subscriber cam2_pose_sub_;
       ros::Subscriber cam3_pose_sub_;

       //initial
       Eigen::Matrix4d cam1_initialize_pose;
       Eigen::Matrix4d cam2_initialize_pose;
       Eigen::Matrix4d cam3_initialize_pose;
       double cam1_initialize_time_;//Get the time at which first apriltag arrive
       double cam2_initialize_time_;
       double cam3_initialize_time_;
    public:
       TSNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
       TSNode() : TSNode( ros::NodeHandle(), ros::NodeHandle("~") ){}
       ~TSNode();

       void tags_sub1(const rapyuta_msgs::AprilTagDetections::ConstPtr& msg);
       void tags_sub2(const rapyuta_msgs::AprilTagDetections::ConstPtr& msg);
       void tags_sub3(const rapyuta_msgs::AprilTagDetections::ConstPtr& msg);

       //set and get the pose
       Eigen::Matrix4d getInitPose_cam1();
       void setInitPose_cam1(const Eigen::Matrix4d & pose, double time);
       Eigen::Matrix4d getInitPose_cam2();
       void setInitPose_cam2(const Eigen::Matrix4d & pose, double time);
       Eigen::Matrix4d getInitPose_cam3();
       void setInitPose_cam3(const Eigen::Matrix4d & pose, double time);

};//end of class

}//tags_sub namespace

#endif /*TAGS_SUB_H_*/
