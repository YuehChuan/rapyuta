#ifndef POSEESTIMATOR_H_
#define POSEESTIMATOR_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <math.h>
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <algorithm>
#include "rapyuta_pose_estimator_lib/datatypes.h"


namespace rapyuta_pose_estimator
{

class PoseEstimator
{
    private:
        Eigen::Matrix4d current_pose_;
        Eigen::Matrix4d previous_pose_;
        Eigen::Matrix4d predicted_pose_;
        Matrix6d pose_covariance_;         
        double current_time_;//!< Stores  the time of the current pose
        double previous_time_;//!< Stores the time of previous pose
        double predicted_time_;//!< Stores the time of predicted pose
        unsigned it_since_initialized_;//!< Counter to determine whether the system has been initialised already
        bool pose_updated_;
    public:
        cv::Mat camera_matrix_K_; //!< Variable to store the camera matrix as an OpenCV matrix
        std::vector<double>camera_distortion_coeffs_;//!< Variable to store the camera distortion parameters

    private:
        /*Computes the exponential map from a twist to a homogeneous transformation matrix
         *
         * Given a twist vector
         *
         */
        
        Eigen::Matrix4d exponentialMap(const Vector6d & twist);
        /*Computes the logarithm map from a homogeneous transformation matrix to twist coordinates.
         *
         * Given a homogeneous transformation matrix
         *
         */
        Vector6d logarithmMap(const Eigen::Matrix4d & trans);
        /*Creates a skew symmetric matrix from a vector of length 3.
         *
         *
         *
         */
        Eigen::Matrix3d skewSymmetricMatrix(const Eigen::Vector3d w);
        /*Finds the elements with the largest absolute value in a vector
         *
         *
         */
        double norm_max(const Eigen::VectorXd & v);
    public:
        /**
         *constructor
         *
         */
        PoseEstimator();
        /**
         *Sets the time which the pose will bee calculated. E_CXX_F
         */
        void setPredictedTime(double time);
        /**
         *Returns the time for which the predicted pose was calculate
         *
         */
        double getPredictedTime();
        void setPredictedPose(const Eigen::Matrix4d & pose, double time);
        Eigen::Matrix4d getPredictedPose();
        Matrix6d getPoseCovariance();

        void predictPose(double time_to_predict);

        unsigned initialise();
        void updatePose();

        void optimisePose();
        void optimiseAndUpdatePose(double& time_to_predict);
        
}; //class definition need ";"
}//namespace


#endif /* POSEESTIMATOR_H_*/
