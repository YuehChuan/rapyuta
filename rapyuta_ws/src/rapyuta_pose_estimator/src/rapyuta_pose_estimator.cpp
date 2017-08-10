#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <ros/ros.h>

//syncronize apriltags subscription in tags_sub.h
//apriltags message
#include "rapyuta_pose_estimator/rapyuta_pose_estimator.h"

//TF
#include <tf/transform_broadcaster.h>

//use sophus for transformation
#include "sophus/so3.h"
#include "sophus/se3.h"

namespace rapyuta_pose_estimator
{

TSNode::TSNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) :nh_(nh), nh_private_(nh_private)
{

    trackable_object_.setInitialStatus_camera1(false);
    trackable_object_.setInitialStatus_camera2(false);
    trackable_object_.setInitialStatus_camera3(false);
    cam1_pose_sub_ = nh_.subscribe("/rapyuta01/usb_cam/apriltags/detections", 10, &TSNode::tags_sub1, this);
    cam2_pose_sub_= nh_.subscribe("/rapyuta02/usb_cam/apriltags/detections", 10, &TSNode::tags_sub2, this);
    cam3_pose_sub_ = nh_.subscribe("/rapyuta03/usb_cam/apriltags/detections", 10, &TSNode::tags_sub3, this);
}

TSNode::~TSNode()
{

}



void TSNode::tags_sub1(const rapyuta_msgs::AprilTagDetections::ConstPtr& msg)
{
    if( (msg->detections.empty()) == false )//if detections got value (prevent segmental fault)
    {
      double cam1_time_get_msg = msg->header.stamp.toSec();
      Eigen::Matrix4d transform_cam1;
      bool cam1status;
      cam1status=trackable_object_.getInitialStatus_camera1();
      transform_cam1 =poselistToTransform(msg); // read msg and convert  to 4x4 homogeniousmatrix

      //raw data
      tf::Transform transform2;
      transform2=setTFfromMsg(msg);
      //visualization
      static tf::TransformBroadcaster br;
      static tf::TransformBroadcaster br_cam1;
      static tf::TransformBroadcaster br_cam2;

      if(cam1status== false)
      {
//      trackable_object_.setInitPose_cam1(InitHomogenious_cam1,cam1_time_get_msg );
//      trackable_object_.setInitialStatus_camera1(true);
      }

      InverseTimeBenchmark(transform_cam1);

      tf::Transform transform;
      transform=matrixToTf(transform_cam1);

      br.sendTransform(tf::StampedTransform(transform.inverse(), ros::Time::now(), "apriltag", "camera1"));
      br_cam1.sendTransform(tf::StampedTransform(transform2.inverse(), ros::Time::now(),"apriltag","camera1_original"));
    }
}

void TSNode::tags_sub2(const rapyuta_msgs::AprilTagDetections::ConstPtr& msg)
{
    if( (msg->detections.empty()) == false )
    {
      double cam2_time_get_msg = msg->header.stamp.toSec();

      bool cam2status;
      //from camera2 to apriltag
      cam2status=trackable_object_.getInitialStatus_camera2();
      Eigen::Vector3d pos_cam2 = Eigen::Vector3d(msg->detections[0].pose.position.x,msg->detections[0].pose.position.y,msg->detections[0].pose.position.z);
      Eigen::Quaterniond q_cam2(msg->detections[0].pose.orientation.x, msg->detections[0].pose.orientation.y, msg->detections[0].pose.orientation.z, msg->detections[0].pose.orientation.w);
      Eigen::Matrix3d R_cam2 = q_cam2.toRotationMatrix();
      Eigen::Matrix4d InitHomogenious_cam2=Eigen::Matrix4d::Identity();;
      InitHomogenious_cam2.block(0,0,3,3)=R_cam2;
      InitHomogenious_cam2(0,3) = msg->detections[0].pose.position.x;
      InitHomogenious_cam2(1,3) = msg->detections[0].pose.position.y;
      InitHomogenious_cam2(2,3) = msg->detections[0].pose.position.z;





      //visualization
      static tf::TransformBroadcaster br;
      static tf::TransformBroadcaster br_cam2_1;

      Eigen::Matrix4d cam1_init=trackable_object_.getInitPose_cam1();
      //Get initial relation between cam1-->cam2 :get inverse  of (cam2 from apriltags) * cam1_init
      if(cam2status== false)
      {
      Eigen::Matrix4d cam2_init= cam1_init*InitHomogenious_cam2.inverse();
      trackable_object_.setInitPose_cam2(cam2_init,cam2_time_get_msg );
      trackable_object_.setInitialRelation_cam1Tocam2(cam2_init);//should record time stamp
      trackable_object_.setInitialStatus_camera2(true);

      //debug info
      double cam2_captime= trackable_object_.getInitTime_cam2();
      Eigen::Matrix4d cam2=trackable_object_.getInitPose_cam2();
      Eigen::Matrix4d cam1_to_cam2=trackable_object_.getInitialRelation_cam1Tocam2();

      }
      //visualization
      tf::Transform transform;
      Eigen::Matrix4d cam2=trackable_object_.getInitPose_cam2();
      Eigen::Matrix3d cam2_rot;
      tf::Quaternion cam2_q;
      for(int i=0; i<3; i++)//for rotation
      {
        for(int j=0; j<3; j++)
        {
          cam2_rot(i,j)=cam2(i,j);
        }
      }
            tf::quaternionEigenToTF( Eigen::Quaterniond(cam2_rot),cam2_q);

            transform.setOrigin(tf::Vector3( cam2(0,3),cam2(1,3),cam2(2,3) ) );
            transform.setRotation(tf::Quaternion(cam2_q));

    //  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera1", "camera2"));
    }

}
void TSNode::tags_sub3(const rapyuta_msgs::AprilTagDetections::ConstPtr& msg)
{
    if( (msg->detections.empty()) == false )
    {


  tf::Transform transform;
  transform.setOrigin(tf::Vector3(msg->detections[0].pose.position.x,msg->detections[0].pose.position.y,msg->detections[0].pose.position.z) );
  transform.setRotation(tf::Quaternion(msg->detections[0].pose.orientation.x, msg->detections[0].pose.orientation.y, msg->detections[0].pose.orientation.z, msg->detections[0].pose.orientation.w) );

  tf::Transform InvTransform_cam3;

  InvTransform_cam3 = transform.inverse();


  //br.sendTransform(tf::StampedTransform(InvTransform_cam3, ros::Time::now(), "map", "camera3"));
    }

}

//-------------------------------------------------
// Transformation utility
//-------------------------------------------------

//read pose and convert to 4x4Matrix
Eigen::Matrix4d TSNode::poselistToTransform( const rapyuta_msgs::AprilTagDetections::ConstPtr& msg)
{
  Eigen::Vector3d pos_cam;
  pos_cam = poseMsgToEigenPos(msg);
  Eigen::Quaterniond q_cam;
  q_cam=poseToQuaterniond(msg);
  Eigen::Matrix3d R_cam = q_cam.normalized().toRotationMatrix();

  Eigen::Matrix4d Transform_cam=Eigen::Matrix4d::Identity();
  Transform_cam.block(0,0,3,3)=R_cam;
  Transform_cam(0,3) = msg->detections[0].pose.position.x;
  Transform_cam(1,3) = msg->detections[0].pose.position.y;
  Transform_cam(2,3) = msg->detections[0].pose.position.z;

/* debug info
  cout<<"pose.orientation.x"<<endl;
  cout<<msg->detections[0].pose.orientation.x<<","<<msg->detections[0].pose.orientation.y<<","<<msg->detections[0].pose.orientation.z<<","<<msg->detections[0].pose.orientation.w<<endl;
  cout<<q_cam.coeffs()<<endl;
*/
  return Transform_cam;
}

//ref: http://docs.ros.org/hydro/api/tf_conversions/html/c++/tf__eigen_8h.html
//4x4matrix to TF
tf::Transform TSNode::matrixToTf( const Eigen::Matrix4d inputMatrix_4x4)
{
  Eigen::Matrix3d rot_3x3;
  Eigen::Vector3d pos;
//tf data type
  tf::Transform transform;
  tf::Vector3  v_cam;
  tf::Matrix3x3 R_matrix_cam;

  pos(0)=inputMatrix_4x4(0,3);
  pos(1)=inputMatrix_4x4(1,3);
  pos(2)=inputMatrix_4x4(2,3);
  rot_3x3=inputMatrix_4x4.block(0,0,3,3);

  tf::matrixEigenToTF(rot_3x3,R_matrix_cam);
  tf::vectorEigenToTF (pos,v_cam);
  transform.setOrigin(v_cam);
  transform.setBasis(R_matrix_cam);

  return transform;
}

//3x3matrix and vector to construct TF (overload in c++)
tf::Transform TSNode::matrixToTf( const Eigen::Matrix3d rot, const Eigen::Vector3d pos)
{
  tf::Transform transform;
  tf::Vector3 v_cam;
  tf::Matrix3x3 matrix_cam;
  vectorEigenToTF(pos,v_cam);
  matrixEigenToTF(rot,matrix_cam);

  transform.setOrigin(v_cam);
  transform.setBasis(matrix_cam);

  return transform;
}

//turn pose to quaterniond
Eigen::Quaterniond TSNode::poseToQuaterniond(const rapyuta_msgs::AprilTagDetections::ConstPtr& msg)
{
//be carefule the order is (W,X,Y,Z)!
  Eigen::Quaterniond q;
  q.x()=msg->detections[0].pose.orientation.x;
  q.y()=msg->detections[0].pose.orientation.y;
  q.z()=msg->detections[0].pose.orientation.z;
  q.w()=msg->detections[0].pose.orientation.w;

  return q;
}

//setTF transfrom
tf::Transform TSNode::setTFfromMsg(const rapyuta_msgs::AprilTagDetections::ConstPtr& msg)
{
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(msg->detections[0].pose.position.x,msg->detections[0].pose.position.y,msg->detections[0].pose.position.z) );
  transform.setRotation(tf::Quaternion(msg->detections[0].pose.orientation.x, msg->detections[0].pose.orientation.y, msg->detections[0].pose.orientation.z, msg->detections[0].pose.orientation.w) );
  return transform;
}

Eigen::Vector3d TSNode::poseMsgToEigenPos(const rapyuta_msgs::AprilTagDetections::ConstPtr& msg)
{
  Eigen::Vector3d pos;
  pos(0) = msg->detections[0].pose.position.x;
  pos(1) = msg->detections[0].pose.position.y;
  pos(2) = msg->detections[0].pose.position.z;
  return pos;
}

//-----------------TF-Eigen transformation utility(above part)--------------------------------

Eigen::Matrix4d  TSNode::getMatrixInverse( const Eigen::Matrix4d inputMat4x4)
{
  /* 4x4 homogenious Matrix inverse
    R.transpose()      | -R.transpose()*t
    ---------------------------------------
    0     0       0    |        1
  */
//get value
  Eigen::Matrix3d original_Rotation;
  Eigen::Matrix3d transpose_Rotation;
  Eigen::Vector3d original_Translation;
  Eigen::Vector3d translation;
  Eigen::Matrix4d outputMat4x4;

  original_Rotation=inputMat4x4.block(0,0,3,3);
  original_Translation(0) = inputMat4x4(0,3);
  original_Translation(1) = inputMat4x4(1,3);
  original_Translation(2) = inputMat4x4(2,3);


  transpose_Rotation=original_Rotation.transpose();
  translation= -transpose_Rotation*original_Translation;
  outputMat4x4.block(0,0,3,3) = transpose_Rotation;
  outputMat4x4(0,3) = translation(0);
  outputMat4x4(1,3) = translation(1);
  outputMat4x4(2,3) = translation(2);
  return outputMat4x4;
}


Eigen::Matrix4d  TSNode::getMatrixInverse( const Eigen::Matrix3d inputRot3x3, const Eigen::Vector3d inputVec)
{
  //overload version for different input combination
  /* 4x4 homogenious Matrix inverse
    R.transpose()      | -R.transpose()*t
    ---------------------------------------
    0     0       0    |        1
  */
//get value

  Eigen::Matrix3d transpose_Rotation;
  Eigen::Vector3d translation;
  Eigen::Matrix4d outputMat4x4;

  transpose_Rotation=inputRot3x3.transpose();
  translation= -transpose_Rotation*inputVec;
  outputMat4x4.block(0,0,3,3) = transpose_Rotation;
  outputMat4x4(0,3) = translation(0);
  outputMat4x4(1,3) = translation(1);
  outputMat4x4(2,3) = translation(2);
  return outputMat4x4;
}

//for testing the speed of inverse
void TSNode::InverseTimeBenchmark(const Eigen::Matrix4d inputMat4x4)
{

        Eigen::Matrix4d inv;
        Eigen::Matrix4d inv_1;
        Eigen::Matrix4d inv_2;
        clock_t time_stt = clock();//start
        inv=inputMat4x4.inverse();
        cout<<"time use in normal inverse is" << 1000*(clock()-time_stt)/(double)CLOCKS_PER_SEC<<"ms"<<endl;
        cout<<"normal inverse result is :"<<endl;
        cout<<inv<<endl;
        clock_t time_stt1 = clock();//start
        inv_1=getMatrixInverse(inputMat4x4);
        cout<<"time use in transpose inverse is" << 1000*(clock()-time_stt1)/(double)CLOCKS_PER_SEC<<"ms"<<endl;
        cout<<"transpose inverse result is :"<<endl;
        cout<<inv_1<<endl;


        Eigen::Matrix4d Identity=Eigen::Matrix4d::Identity();;
        clock_t time_stt2 = clock();//start

        inv_2= inputMat4x4.colPivHouseholderQr().solve(Identity);
        cout<<"time use in Qr composition is" << 1000*(clock()-time_stt2)/(double)CLOCKS_PER_SEC<<"ms"<<endl;
        cout<<"Qr composition inverse result is :"<<endl;
        cout<<inv_2<<endl;

}


//---------------Matrix and vector arithmetic(above part)------------------------------------








}//end name space
