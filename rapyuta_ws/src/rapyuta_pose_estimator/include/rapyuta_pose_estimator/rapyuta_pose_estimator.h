#ifndef RAPYUTA_POSE_ESTIMATOR_
#define RAPYUTA_POSE_ESTIMATOR_

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
#include <tf_conversions/tf_eigen.h> //tf eigen convert
#include <tf/transform_datatypes.h>
#include "rapyuta_pose_estimator_lib/pose_estimator.h"

using namespace std;
namespace rapyuta_pose_estimator
{

class TSNode
{
    private:
       ros::NodeHandle nh_;
       ros::NodeHandle nh_private_;
       ros::Publisher frame_pub_;
       ros::Subscriber cam1_pose_sub_;
       ros::Subscriber cam2_pose_sub_;
       ros::Subscriber cam3_pose_sub_;
       PoseEstimator trackable_object_;

    public:
       TSNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
       TSNode() : TSNode( ros::NodeHandle(), ros::NodeHandle("~") ){}
       ~TSNode();

       void tags_sub1(const rapyuta_msgs::AprilTagDetections::ConstPtr& msg);
       void tags_sub2(const rapyuta_msgs::AprilTagDetections::ConstPtr& msg);
       void tags_sub3(const rapyuta_msgs::AprilTagDetections::ConstPtr& msg);


       //object to track  can it put in private????

       //matrix transform utility
       Eigen::Matrix4d poselistToTransform( const rapyuta_msgs::AprilTagDetections::ConstPtr& msg);

       tf::Transform matrixToTf( const Eigen::Matrix4d eigenMatrix);
       tf::Transform matrixToTf( const Eigen::Matrix3d rot, const Eigen::Vector3d pos);

       Eigen::Quaterniond poseToQuaterniond(const rapyuta_msgs::AprilTagDetections::ConstPtr& msg);

};//end of class

}//tags_sub namespace

#endif /*RAPYUTA_POSE_ESTIMATOR_*/
