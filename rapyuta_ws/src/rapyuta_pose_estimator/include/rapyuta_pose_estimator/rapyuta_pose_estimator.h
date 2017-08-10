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
#include <message_filters/time_synchronizer.h>
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

//time
#include <ctime>

using namespace std;
typedef message_filters::sync_policies::ApproximateTime<rapyuta_msgs::AprilTagDetections, rapyuta_msgs::AprilTagDetections> Cam12_SyncPolicy;

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

       //syncronize Subscriber pair
       void cam12_sub_callback(const rapyuta_msgs::AprilTagDetections::ConstPtr& cam1_msg, const rapyuta_msgs::AprilTagDetections::ConstPtr& cam2_msg);

       //object to track  can it put in private????

       //TF-Eigen transform utility
       Eigen::Matrix4d poselistToTransform( const rapyuta_msgs::AprilTagDetections::ConstPtr& msg);

       tf::Transform matrixToTf( const Eigen::Matrix4d eigenMatrix);
       tf::Transform matrixToTf( const Eigen::Matrix3d rot, const Eigen::Vector3d pos);
       tf::Transform setTFfromMsg(const rapyuta_msgs::AprilTagDetections::ConstPtr& msg);

       Eigen::Vector3d poseMsgToEigenPos(const rapyuta_msgs::AprilTagDetections::ConstPtr& msg);
       Eigen::Quaterniond poseToQuaterniond(const rapyuta_msgs::AprilTagDetections::ConstPtr& msg);

       //Matrix and vector arithmetic
       Eigen::Matrix4d  getMatrixInverse( const Eigen::Matrix4d inputMat4x4);
       Eigen::Matrix4d  getMatrixInverse( const Eigen::Matrix3d inputRot3x3, const Eigen::Vector3d inputVec);
       void InverseTimeBenchmark(const Eigen::Matrix4d inputMat4x4);


};//end of class

}//tags_sub namespace

#endif /*RAPYUTA_POSE_ESTIMATOR_*/
