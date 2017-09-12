#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <ros/ros.h>

//syncronize apriltags subscription in tags_sub.h
//apriltags message
#include "rapyuta_pose_estimator/rapyuta_pose_estimator.h"
#include <rapyuta_pose_estimator/ros/params.h>
//TF
#include <tf/transform_broadcaster.h>

//use sophus for transformation
#include "sophus/so3.h"
#include "sophus/se3.h"

//delete
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>


namespace rapyuta_pose_estimator
{

TSNode::TSNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) :nh_(nh), nh_private_(nh_private)
{

    trackable_object_.setInitialStatus_camera1(false);
    trackable_object_.setInitialStatus_camera2(false);
    trackable_object_.setInitialStatus_camera3(false);

    //initialize check msg empty flag
    trackable_object_.setEmptyflag_camera1(false);
    trackable_object_.setEmptyflag_camera2(false);
    trackable_object_.setEmptyflag_camera3(false);

//Cam123_sync
    subscribeToCam123();
    posPublisher = nh_.advertise<geometry_msgs::Pose>("target_pose", 10);
//subscriber
//    cam1_pose_sub_ = nh_.subscribe("/rapyuta01/usb_cam/apriltags/detections", 10, &TSNode::tags_sub1, this);
//    cam2_pose_sub_= nh_.subscribe("/rapyuta02/usb_cam/apriltags/detections", 10, &TSNode::tags_sub2, this);
//    cam3_pose_sub_ = nh_.subscribe("/rapyuta03/usb_cam/apriltags/detections", 10, &TSNode::tags_sub3, this);

    //syncronizer subscriber pair
/*
    message_filters::Subscriber<rapyuta_msgs::AprilTagDetections> cam12_cam1_sub(nh_,"/rapyuta01/usb_cam/apriltags/detections", 10);
    message_filters::Subscriber<rapyuta_msgs::AprilTagDetections> cam12_cam2_sub(nh_,"/rapyuta02/usb_cam/apriltags/detections", 10);
    message_filters::Synchronizer<Cam12_SyncPolicy> sync(Cam12_SyncPolicy(10), cam12_cam1_sub, cam12_cam2_sub);
   sync.registerCallback(boost::bind(&rapyuta_pose_estimator::TSNode::cam12_sub_callback,this, _1, _2));
   cout<<"YOOOOOOOOOOOOOOOOOOOOOOOO"<<endl;
*/
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
      tf::Transform transform1;
      transform1=setTFfromMsg(msg);
      //visualization
      static tf::TransformBroadcaster br;
      static tf::TransformBroadcaster br_cam1;
      static tf::TransformBroadcaster br_cam2;//test eigen inverse

      if(cam1status== false)
      {
      trackable_object_.setInitPose_cam1(transform_cam1,cam1_time_get_msg );
      trackable_object_.setInitialStatus_camera1(true);

      }

      Eigen::Matrix4d cam1_inverse=getMatrixInverse(transform_cam1);
      tf::Transform transform2;
      transform2=matrixToTf(cam1_inverse);

    //  InverseTimeBenchmark(transform_cam1); //for testing inverse speed

      tf::Transform transform;
      transform=matrixToTf(transform_cam1);

      br.sendTransform(tf::StampedTransform(transform.inverse(), ros::Time::now(), "apriltag", "camera1"));
      br_cam1.sendTransform(tf::StampedTransform(transform1.inverse(), ros::Time::now(),"apriltag","camera1_original"));
      br_cam2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(),"apriltag","camera1_eigen"));
    }
}

void TSNode::tags_sub2(const rapyuta_msgs::AprilTagDetections::ConstPtr& msg)
{
  if( (msg->detections.empty()) == false )//if detections got value (prevent segmental fault)
  {
    double cam2_time_get_msg = msg->header.stamp.toSec();
    Eigen::Matrix4d transform_cam2;
    bool cam2status;
    cam2status=trackable_object_.getInitialStatus_camera2();
    transform_cam2 =poselistToTransform(msg); // read msg and convert  to 4x4 homogeniousmatrix

    //raw data
    tf::Transform transform1;
    transform1=setTFfromMsg(msg);
    //visualization
    static tf::TransformBroadcaster br;
    static tf::TransformBroadcaster br_cam;

    if(cam2status== false)
    {
//      trackable_object_.setInitPose_cam2(InitHomogenious_cam2,cam2_time_get_msg );
//      trackable_object_.setInitialStatus_camera1(true);
    }

    Eigen::Matrix4d cam2_inverse=getMatrixInverse(transform_cam2);
    tf::Transform transform2;
    transform2=matrixToTf(cam2_inverse);

  //  InverseTimeBenchmark(transform_cam1); //for testing inverse speed

    tf::Transform transform;
    transform=matrixToTf(transform_cam2);

    br.sendTransform(tf::StampedTransform(transform.inverse(), ros::Time::now(), "apriltag", "camera2"));
//    br_cam.sendTransform(tf::StampedTransform(transform1.inverse(), ros::Time::now(),"apriltag","camera2_original"));
  }

}
void TSNode::tags_sub3(const rapyuta_msgs::AprilTagDetections::ConstPtr& msg)
{

  if( (msg->detections.empty()) == false )//if detections got value (prevent segmental fault)
  {
    double cam3_time_get_msg = msg->header.stamp.toSec();
    Eigen::Matrix4d transform_cam3;
    bool cam3status;
    cam3status=trackable_object_.getInitialStatus_camera3();
    transform_cam3 =poselistToTransform(msg); // read msg and convert  to 4x4 homogeniousmatrix

    //raw data
    tf::Transform transform1;
    transform1=setTFfromMsg(msg);
    //visualization
    static tf::TransformBroadcaster br;
    static tf::TransformBroadcaster br_cam;

    if(cam3status== false)
    {
//      trackable_object_.setInitPose_cam3(InitHomogenious_cam3,cam3_time_get_msg );
//      trackable_object_.setInitialStatus_camera1(true);
    }

    Eigen::Matrix4d cam3_inverse=getMatrixInverse(transform_cam3);
    tf::Transform transform2;
    transform2=matrixToTf(cam3_inverse);

  //  InverseTimeBenchmark(transform_cam1); //for testing inverse speed

    tf::Transform transform;
    transform=matrixToTf(transform_cam3);

    br.sendTransform(tf::StampedTransform(transform.inverse(), ros::Time::now(), "apriltag", "camera3"));
//    br_cam.sendTransform(tf::StampedTransform(transform1.inverse(), ros::Time::now(),"apriltag","camera3_original"));
  }
}


//----------------syncronize subscriber----------------

void TSNode::subscribeToCam123()
{
cout <<"from cam123_sub callback!!!!!!!!!" <<endl;

 Cam123_Cam1_Subscriber.reset( new message_filters::Subscriber<rapyuta_msgs::AprilTagDetections>(nh_, "/rapyuta01/usb_cam/apriltags/detections", 10) );
 Cam123_Cam2_Subscriber.reset (new message_filters::Subscriber<rapyuta_msgs::AprilTagDetections>(nh_, "/rapyuta02/usb_cam/apriltags/detections", 10) );
 Cam123_Cam3_Subscriber.reset (new message_filters::Subscriber<rapyuta_msgs::AprilTagDetections>(nh_, "/rapyuta03/usb_cam/apriltags/detections", 10) );
 Cam123_inputSynchronizer.reset( new Synchronizer(Cam123_SyncPolicy(10), *Cam123_Cam1_Subscriber, *Cam123_Cam2_Subscriber, *Cam123_Cam3_Subscriber) );
 Cam123_inputSynchronizer->registerCallback(boost::bind(&rapyuta_pose_estimator::TSNode::cam123_sub_callback, this, _1,_2,_3));

 int circular_buffer_size = 10;//not using now
 BufferType dataBuffer(circular_buffer_size);//not using now
 cam123_bufferVector.push_back(dataBuffer);//not using now


}

void TSNode::cam123_sub_callback(const rapyuta_msgs::AprilTagDetections::ConstPtr& cam1_msg, const rapyuta_msgs::AprilTagDetections::ConstPtr& cam2_msg, const rapyuta_msgs::AprilTagDetections::ConstPtr& cam3_msg)
{
  //----------cam1--------------(set as original global (0,0,0) )


  if( (cam1_msg->detections.empty()) == false )//check cam1_msg got detection or not
  {
    static tf::TransformBroadcaster br1_origin;//original cam1_pose for visualizing camera position
    static tf::TransformBroadcaster br1;//current cam1(mother) <--tags

    if( (trackable_object_.getInitialStatus_camera1())== true)
    {
      //visualize
      Eigen::Matrix4d transform_cam1_static;
      Eigen::Matrix4d transform_cam1;//current pose
      tf::Transform transform1_static;
      tf::Transform transform1;
      double cam1_time_get_msg = cam1_msg->header.stamp.toSec();

      transform_cam1_static=trackable_object_.getInitPose_cam1();

      transform_cam1 = transform_cam1_static*poselistToTransform(cam1_msg);
      trackable_object_.setPreviousPose_cam1(trackable_object_.getCurrentPose_cam1(),cam1_time_get_msg );
      trackable_object_.setCurrentPose_cam1(transform_cam1,cam1_time_get_msg );

      //visualize
      transform1=matrixToTf(transform_cam1);
      transform1_static=matrixToTf(transform_cam1_static);//init
      br1.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "map","apriltag" ));
      br1_origin.sendTransform(tf::StampedTransform(transform1_static, ros::Time::now(),"map", "camera1_static"));
    }
    else //If it is first cam1 than initialize
    {
      double cam1_time_get_msg = cam1_msg->header.stamp.toSec();
      Eigen::Matrix4d transform_cam1;
      transform_cam1 =getMatrixInverse( poselistToTransform(cam1_msg) ); // read msg and convert  to 4x4 homogeniousmatrix
      trackable_object_.setInitPose_cam1(transform_cam1,cam1_time_get_msg );
      trackable_object_.setInitialStatus_camera1(true);
    }
  }
  else//cam1 is empty
  {
    cout <<"cam1 is empty!" <<endl; //predict pose, recordtime
    trackable_object_.setEmptyflag_camera1(true);

  }//-----------cam1-------------

  //----------cam2--------------
  if( (cam2_msg->detections.empty()) == false )//check cam2_msg got detection or not
  {
    static tf::TransformBroadcaster br2_origin;//original cam2_pose relate to cam1   cam1<---cam2
    static tf::TransformBroadcaster br2;//current cam1(mother) <--cam2

    static tf::TransformBroadcaster br4;//current cam2(mother) <--tag // test
    geometry_msgs::Pose target_pose;//for rqt_value
    if( (trackable_object_.getInitialStatus_camera2())== true)
    {
      //visualize
      Eigen::Matrix4d transform_cam2_static;
      Eigen::Matrix4d transform_cam2;//current
      Eigen::Matrix4d cam2tag_observe;//apriltag observe from cam2
      tf::Transform transform2_static;
      tf::Transform transform2;
      double cam2_time_get_msg = cam2_msg->header.stamp.toSec();

      transform_cam2_static=trackable_object_.getInitPose_cam2();
      transform_cam2 = transform_cam2_static*poselistToTransform(cam2_msg);

      trackable_object_.setPreviousPose_cam2(trackable_object_.getCurrentPose_cam2(),cam2_time_get_msg );
      trackable_object_.setCurrentPose_cam2(transform_cam2,cam2_time_get_msg );

      //visualize
      transform2=matrixToTf(transform_cam2);
      transform2_static=matrixToTf(transform_cam2_static);//init
      br2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "map", "apriltag2"));// publish cam2 tag measurement from cam1 corordinate system
      br2_origin.sendTransform(tf::StampedTransform(transform2_static, ros::Time::now(), "map", "camera2_static"));

      //test
//      br4.sendTransform(tf::StampedTransform(transform4, ros::Time::now(), "camera2", "apriltag-test"));//publish cam2<---tag


    }
    else //If it is first cam2 than initialize
    {
      double cam2_time_get_msg = cam2_msg->header.stamp.toSec();
      Eigen::Matrix4d transform_cam2;
      transform_cam2 =getMatrixInverse(poselistToTransform(cam2_msg)); // read msg and convert  to 4x4 homogeniousmatrix
      trackable_object_.setInitPose_cam2(transform_cam2,cam2_time_get_msg );
      trackable_object_.setInitialStatus_camera2(true);
      setInit_cam1Tocam2(transform_cam2);
    }
  }
  else//cam2 is empty
  {
    cout <<"cam2 is empty!" <<endl;
    trackable_object_.setEmptyflag_camera2(true);
    bool msg2_empty = trackable_object_.getEmptyflag_camera2();


  }//-----------cam2-------------

  //----------cam3--------------
  if( (cam3_msg->detections.empty()) == false )//check cam3_msg got detection or not
  {
    static tf::TransformBroadcaster br3_origin;//original cam3_pose relate to cam1   cam1<---cam2
    static tf::TransformBroadcaster br3;//current cam1(mother) <--cam3
    if( (trackable_object_.getInitialStatus_camera3())== true)
    {
      //visualize
      Eigen::Matrix4d transform_cam3_static;
      Eigen::Matrix4d transform_cam3;//current
      Eigen::Matrix4d cam3tag_observe;//apriltag observe from cam2
      tf::Transform transform3_static;
      tf::Transform transform3;
      double cam3_time_get_msg = cam3_msg->header.stamp.toSec();

      transform_cam3_static=trackable_object_.getInitPose_cam3();

      transform_cam3 = transform_cam3_static*poselistToTransform(cam3_msg);
      trackable_object_.setPreviousPose_cam3(trackable_object_.getCurrentPose_cam3(),cam3_time_get_msg );
      trackable_object_.setCurrentPose_cam3(transform_cam3,cam3_time_get_msg );

      //visualize
      transform3=matrixToTf(transform_cam3);
      transform3_static=matrixToTf(transform_cam3_static);//init
      br3.sendTransform(tf::StampedTransform(transform3, ros::Time::now(), "map", "apriltag3"));// publish cam3 tag measurement from cam1 corordinate system
      br3_origin.sendTransform(tf::StampedTransform(transform3_static, ros::Time::now(), "map", "camera3_static"));//publish cam1<---cam3

    }
    else //If it is first cam3 than initialize
    {
      double cam3_time_get_msg = cam3_msg->header.stamp.toSec();
      Eigen::Matrix4d transform_cam3;
      transform_cam3 =getMatrixInverse(poselistToTransform(cam3_msg)); // read msg and convert  to 4x4 homogeniousmatrix
      trackable_object_.setInitPose_cam3(transform_cam3,cam3_time_get_msg );
      trackable_object_.setInitialStatus_camera3(true);
      setInit_cam1Tocam3(transform_cam3);
    }
  }
  else//cam3 is empty
  {
    cout<<"cam3 is empty!"<<endl;
    //get etime at which empty msg was receive. This time is used to calculate the estimated pose
    double time_to_predict = cam3_msg->header.stamp.toSec();
    trackable_object_.setEmptyflag_camera3(true);
//    trackable_object_.predictPose(time_to_predict);//specified

  }//-----------cam3-------------

  bool cam1_msg_empty = trackable_object_.getEmptyflag_camera1();
  bool cam2_msg_empty = trackable_object_.getEmptyflag_camera2();
  bool cam3_msg_empty = trackable_object_.getEmptyflag_camera3();

  int checksum = 4*cam1_msg_empty+2*cam2_msg_empty+1*cam3_msg_empty;//
  cout<<"checksum"<<endl;
  cout<<checksum<<endl;
  //visualization target
  static tf::TransformBroadcaster br_target;//current target
  tf::Transform target_TF;
  switch (checksum)
  {
    case 7: //111 all empty
    {
      //use predict
        break;
    }
    case 6: //110
    {
      Eigen::Matrix4d transform_target;
      transform_target=trackable_object_.getCurrentPose_cam3();
      trackable_object_.setCurrentPose_target(transform_target);
      tf::Transform target_TF=matrixToTf(transform_target);
      br_target.sendTransform(tf::StampedTransform(target_TF, ros::Time::now(), "map", "target"));
      break;
    }
    case 5: //100
    {
      Eigen::Matrix4d transform_target;
      //transform_target=(trackable_object_.getCurrentPose_cam2()+trackable_object_.getCurrentPose_cam3())/2.0;
      transform_target=trackable_object_.getCurrentPose_cam2();
      trackable_object_.setCurrentPose_target(transform_target);
      tf::Transform target_TF=matrixToTf(transform_target);
      br_target.sendTransform(tf::StampedTransform(target_TF, ros::Time::now(), "map", "target"));
      break;
    }

    case 4: //101
    {
      Eigen::Matrix4d transform_target;
      transform_target=trackable_object_.getCurrentPose_cam2();
      trackable_object_.setCurrentPose_target(transform_target);
      tf::Transform target_TF=matrixToTf(transform_target);
      br_target.sendTransform(tf::StampedTransform(target_TF, ros::Time::now(), "map", "target"));
      break;
    }

    case 3: //011
    {
      Eigen::Matrix4d transform_target;
      transform_target=trackable_object_.getCurrentPose_cam1();
      trackable_object_.setCurrentPose_target(transform_target);
      tf::Transform target_TF=matrixToTf(transform_target);
      br_target.sendTransform(tf::StampedTransform(target_TF, ros::Time::now(), "map", "target"));
      break;
    }
    case 2: //010
    {
      Eigen::Matrix4d transform_target;
      //transform_target=(trackable_object_.getCurrentPose_cam1()+trackable_object_.getCurrentPose_cam3())/2.0;
      transform_target=trackable_object_.getCurrentPose_cam1();
      trackable_object_.setCurrentPose_target(transform_target);
      tf::Transform target_TF=matrixToTf(transform_target);
      br_target.sendTransform(tf::StampedTransform(target_TF, ros::Time::now(), "map", "target"));
      break;
    }
    case 1: //001
    {
      Eigen::Matrix4d transform_target;
      //transform_target=(trackable_object_.getCurrentPose_cam1()+trackable_object_.getCurrentPose_cam2())/2.0;
      transform_target=trackable_object_.getCurrentPose_cam2();
      trackable_object_.setCurrentPose_target(transform_target);
      tf::Transform target_TF=matrixToTf(transform_target);
      br_target.sendTransform(tf::StampedTransform(target_TF, ros::Time::now(), "map", "target"));
      break;
    }
    case 0: //000
    {
      Eigen::Matrix4d transform_target;
      //transform_target=(trackable_object_.getCurrentPose_cam1()+trackable_object_.getCurrentPose_cam2()+trackable_object_.getCurrentPose_cam3())/3.0;
      transform_target=trackable_object_.getCurrentPose_cam3();
      trackable_object_.setCurrentPose_target(transform_target);
      tf::Transform target_TF=matrixToTf(transform_target);
      br_target.sendTransform(tf::StampedTransform(target_TF, ros::Time::now(), "map", "target"));
      break;
    }
    default:
    {
      break;
    }
  }//end switch


  //set back msg check value
    trackable_object_.setEmptyflag_camera1(false);
    trackable_object_.setEmptyflag_camera2(false);
    trackable_object_.setEmptyflag_camera3(false);

//FOR logging the data
// get publish topic euler angle
//Eigen::Vector3d euler_angles = ( cam3tag_observe.block(0,0,3,3) ).eulerAngles(2,1,0);//ZXY yaw,pitch roll
   geometry_msgs::Pose target_pose;
   Eigen::Quaterniond q;
   Eigen::Matrix4d target_;
   target_= trackable_object_.getCurrentPose_target();
   Eigen::Matrix3d rot=target_.block(0,0,3,3);
   q = Eigen::Quaterniond(rot);
   target_pose.position.x=target_(0,3);
   target_pose.position.y=target_(1,3);
   target_pose.position.z=target_(2,3);
   target_pose.orientation.x= q.x();
   target_pose.orientation.y= q.y();
   target_pose.orientation.z= q.z();
   target_pose.orientation.w= q.w();
   posPublisher.publish(target_pose);//publish pose


}//end cam123_sub_callback









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
  Eigen::Matrix4d outputMat4x4 = Eigen::Matrix4d::Identity();

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
  Eigen::Matrix4d outputMat4x4 = Eigen::Matrix4d::Identity();

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
        Eigen::Matrix4d Identity= Eigen::Matrix4d::Identity();
        clock_t time_stt2 = clock();//start

        inv_2= inputMat4x4.colPivHouseholderQr().solve(Identity);
        cout<<"time use in Qr composition is" << 1000*(clock()-time_stt2)/(double)CLOCKS_PER_SEC<<"ms"<<endl;
        cout<<"Qr composition inverse result is :"<<endl;
        cout<<inv_2<<endl;

}


//---------------Matrix and vector arithmetic(above part)------------------------------------


//set initial relationship between cam1_cam2 cam1_cam3
void TSNode::setInit_cam1Tocam2(const Eigen::Matrix4d inputMat4x4)//input cam2<--apriltag
{
    /*    Eigen::Matrix4d cam1_init=trackable_object_.getInitPose_cam1();//get cam1<---tag init
        //note: T(1<--2) = T(tag<---2)*T(1<---tag)
        Eigen::Matrix4d inv_cam2;
        Eigen::Matrix4d result;
        inv_cam2 = getMatrixInverse(inputMat4x4);
        result=inv_cam2*cam1_init;
        trackable_object_.setInitialRelation_cam1Tocam2( result); // T(1<--2) = ( T(2<---tag).inverse)*T(1<---tag)
      */
      Eigen::Matrix4d cam1_init=trackable_object_.getInitPose_cam1();
      //note: T(1<--2) = T(tag<---2)*T(1<---tag)

      trackable_object_.setInitialRelation_cam1Tocam2( getMatrixInverse(inputMat4x4)*cam1_init); // T(1<--2) = ( T(2<---tag).inverse)*T(1<---tag)
}
void TSNode::setInit_cam1Tocam3(const Eigen::Matrix4d inputMat4x4)
{
        Eigen::Matrix4d cam1_init=trackable_object_.getInitPose_cam1();//get cam3<---tag init
        //note: T(1<--3) = T(tag<---3)*T(1<---tag)

        Eigen::Matrix4d check=getMatrixInverse(inputMat4x4);

        trackable_object_.setInitialRelation_cam1Tocam3( getMatrixInverse(inputMat4x4)*cam1_init); // T(1<--3) = ( T(3<---tag).inverse)*T(1<---tag)
}
//---------------Set initialal relationship between cam1_cam2 cam1_cam3


//---------------Calculate  apriltag measurement from cam2,cam3(transform to cam1<--tag)
Eigen::Matrix4d  TSNode::cam2tagMeasurement(const Eigen::Matrix4d inputMat4x4)
{
        Eigen::Matrix4d cam2_tagMeasurement;
        //note measurement fromCam2  T(1<---2)*T(2<---tag)
        cam2_tagMeasurement = ( trackable_object_.getInitialRelation_cam1Tocam2() )*inputMat4x4;
        return cam2_tagMeasurement;
}
Eigen::Matrix4d  TSNode::cam3tagMeasurement(const Eigen::Matrix4d inputMat4x4)
{
        Eigen::Matrix4d cam3_tagMeasurement;
        //note measurement fromCam3  T(1<---3)*T(3<---tag)
        cam3_tagMeasurement = ( trackable_object_.getInitialRelation_cam1Tocam3() )*inputMat4x4;
        return cam3_tagMeasurement;
}

//------------------rviz ignore transform--nor,alization---------------------------------------------------
/*
ROS_DEBUG("Pre normalisation, norm=%e", (poseMsg.transform.rotation.x * poseMsg.transform.rotation.x + poseMsg.transform.rotation.y * poseMsg.transform.rotation.y + poseMsg.transform.rotation.z * poseMsg.transform.rotation.z + poseMsg.transform.rotation.w * poseMsg.transform.rotation.w));
		long double recipNorm = 1 / sqrt(poseMsg.transform.rotation.x * poseMsg.transform.rotation.x + poseMsg.transform.rotation.y * poseMsg.transform.rotation.y + poseMsg.transform.rotation.z * poseMsg.transform.rotation.z + poseMsg.transform.rotation.w * poseMsg.transform.rotation.w);
		poseMsg.transform.rotation.x *= recipNorm;
		poseMsg.transform.rotation.y *= recipNorm;
		poseMsg.transform.rotation.z *= recipNorm;
		poseMsg.transform.rotation.w *= recipNorm;
		ROS_DEBUG("Post normalisation, norm=%e", (poseMsg.transform.rotation.x * poseMsg.transform.rotation.x + poseMsg.transform.rotation.y * poseMsg.transform.rotation.y + poseMsg.transform.rotation.z * poseMsg.transform.rotation.z + poseMsg.transform.rotation.w * poseMsg.transform.rotation.w));
*/




}//end name space
