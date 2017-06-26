#include "rapyuta_pose_estimator_lib/pose_estimator.h"

namespace rapyuta_pose_estimator
{

PoseEstimator::PoseEstimator()
{
    it_since_initialized_=0;
}

void PoseEstimator::setPredictedPose(const Eigen::Matrix4d & pose, double time)
{
    predicted_pose_= pose;
    predicted_time_=time;
}

Eigen::Matrix4d PoseEstimator::getPredictedPose()
{
return predicted_pose_;
}

Matrix6d PoseEstimator::getPoseCovariance()
{
    return pose_covariance_;
}

void PoseEstimator::predictPose(double time_to_predict)
{
    predicted_time_= time_to_predict;
    
    //get difference of poses, then its twist coordinates
    Vector6d delta = logarithmMap(previous_pose_.inverse()* current_pose_);
    
    //extrapolate
    Vector6d delta_hat = delta / ( current_time_ - previous_time_)*(predicted_time_ - current_time_);
    
    //Predict new pose
    predicted_pose_ =current_pose_ * exponentialMap(delta_hat);
}

void PoseEstimator::setPredictedTime(double time)
{
    predicted_time_ = time;
}

double PoseEstimator::getPredictedTime()
{
    return predicted_time_;
}

void PoseEstimator::optimisePose()
{

}

void PoseEstimator::updatePose()
{
    previous_pose_ = current_pose_;
    current_pose_ = predicted_pose_;
    previous_time_ = current_time_;
    current_time_ = predicted_time_;
}

void PoseEstimator::optimiseAndUpdatePose(double & time_to_predict)
{
    optimisePose();
    
    if(it_since_initialized_ <2)
    {
        it_since_initialized_++;    
    }
    updatePose();
    pose_updated_ = true;
}

Eigen::Matrix4d PoseEstimator::exponentialMap(const Vector6d & twist)
{
    Eigen::Vector3d upsilon = twist.head<3>();
    Eigen::Vector3d omega = twist.tail<3>();
    
    double theta = omega.norm();
    double theta_squared = theta * theta;

    Eigen::Matrix3d Omega = skewSymmetricMatrix(omega);
    Eigen::Matrix3d Omega_squared = Omega * Omega;
    Eigen::Matrix3d rotation;
    Eigen::Matrix3d V;

    if (theta ==0 )
    {
        rotation = Eigen::Matrix3d::Identity();
        V.setIdentity();
    }
    else
    {
        rotation = Eigen::Matrix3d::Identity() + Omega / theta * sin(theta)+ Omega_squared / theta_squared *(1 - cos(theta));

        V = (Eigen::Matrix3d::Identity() + ( 1- cos(theta)) / (theta_squared)*Omega + (theta - sin(theta)) / (theta_squared * theta) * Omega_squared);      
    }
    
    Eigen::Matrix4d transform;
    transform.setIdentity();
    transform.block<3,3>(0,0)= rotation;
    transform.block<3,1>(0,3)=V * upsilon;

    return transform;

}

Vector6d PoseEstimator::logarithmMap(const Eigen::Matrix4d & trans)
{
    Vector6d xi;
    Eigen::Matrix3d R = trans.block<3,3>(0,0);
    Eigen::Vector3d t = trans.block<3,1>(0,3);
    Eigen::Vector3d w, upsilon;
    Eigen::Matrix3d w_hat;
    Eigen::Matrix3d A_inv;
    double phi = 0;
    double w_norm;

    // Calculate w_hat
    if(R.isApprox(Eigen::Matrix3d::Identity(), 1e-10)==1)
    {
        //phi has already been set to 0;
        w_hat.setZero();
    }
    else
    {
        double temp = (R.trace() - 1)/ 2;
        
        // Force phi to be either 1 or -1 if necessary. Floating point errors can cause problems resulting in this not happening
        if(temp >1)
        {
            temp =1;
        }
        else if (temp < -1)
        {
            temp = -1;
        }
        phi = acos(temp);
        if(phi ==0)
        {
            w_hat.setZero();
        }
        else
        {
            w_hat = (R - R.transpose() ) / (2*sin(phi)) * phi;
        }
    
    }
    // Extract w from skew symmetrix matrix of w
    w <<w_hat(2,1), w_hat(0,2), w_hat(1,0);

    w_norm = w.norm();
    
    // Calculate upsilon
    if(t.isApproxToConstant(0, 1e-10)==1)
    {
        A_inv.setZero();
    }
    else if(w_norm == 0 || sin(w_norm)==0)
    {
        A_inv.setIdentity();
    }
    else
    {
        A_inv = Eigen::Matrix3d::Identity() - w_hat /2 + (2*sin(w_norm) - w_norm *(1+ cos(w_norm)))/ (2 * w_norm * w_norm * sin(w_norm)) * w_hat * w_hat;
    }

    upsilon = A_inv * t;

    // Compose twist coordinates vector
    xi.head<3>() = upsilon;
    xi.tail<3>() =w;
    
    return xi;
}

Eigen::Matrix3d PoseEstimator::skewSymmetricMatrix(const Eigen::Vector3d w)
{
    Eigen::Matrix3d Omega;
    Omega << 0, -w(2), w(1),w(2),0, -w(0), -w(1), w(0),0;
    return Omega;
}


}//namespace
