#include <ros/ros.h>
#include <tf/transform_listener.h>
//#include <geometry_msgs/P.h>
#include <math.h>

const double D2R = M_PI / 180.0;			// degrees to rad
const double R2D = 180.0 / M_PI;			// rad to degrees

int main(int argc, char** argv)
{
	ros::init(argc, argv, "transform");
	ros::NodeHandle nh;
    ros::Publisher posePublisher = nh.advertise<geometry_msgs::Pose>("robot_pose",10, true);
	tf::TransformListener tf(ros::Duration(10));
	ros::Rate rate(10);
	while (ros::ok()) {
			//get robot position
			tf::StampedTransform transform;
			tf::Quaternion q;
            geometry_msgs::Pose robot_pose;
			try {
				tf.lookupTransform("/world", "/base_link", ros::Time(0), transform);
				q = transform.getRotation(); 
			}
			catch (tf::TransformException ex) {	
				ROS_ERROR("%s",ex.what());
				ros::Duration(1.0).sleep();
			}
			ROS_INFO_STREAM("x = " << transform.getOrigin().x() * 100 << " y = " << transform.getOrigin().y() * 100 << " theta = " << tf::getYaw(q) * R2D);
            
               robot_pose.position.x=transform.getOrigin().x();
			   robot_pose.position.y=transform.getOrigin().y();
			   robot_pose.position.z=transform.getOrigin().z();
			   robot_pose.orientation.x= q.x();
			   robot_pose.orientation.y= q.y();
			   robot_pose.orientation.z= q.z();
			   robot_pose.orientation.w= q.w();
			   posePublisher.publish(robot_pose);//publish robot pose

		ros::spinOnce();	
	}
	ros::spin();
	return 0;
}
