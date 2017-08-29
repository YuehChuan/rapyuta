#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

main (int argc, char **argv)
{
	ros::init (argc, argv, "show_path");

	ros::NodeHandle ph;
	ros::Publisher path_pub = ph.advertise<nav_msgs::Path>("trajectory",1, true);
	ros::Publisher target_path_pub = ph.advertise<nav_msgs::Path>("target_trajectory",1, true);
	tf::TransformListener listener;
	tf::TransformListener listener2;

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	nav_msgs::Path path;
	nav_msgs::Path target_path;

	path.header.stamp=current_time;
	path.header.frame_id="map";
	
	target_path.header.stamp=current_time;
	target_path.header.frame_id="map";

	ros::Rate loop_rate(1);
	while (ros::ok()) {

		current_time = ros::Time::now();

		//get laser position
		tf::StampedTransform transform;
		tf::Quaternion q;
		//get target position
		tf::StampedTransform transform2;
		tf::Quaternion q2;
		try{
			listener.waitForTransform("/base_link", "/world", ros::Time(0), ros::Duration(10.0) );
			listener.lookupTransform("/world", "/base_link", ros::Time(0), transform);
			q = transform.getRotation(); 
			
			listener2.waitForTransform("/target", "/map", ros::Time(0), ros::Duration(10.0) );
			listener2.lookupTransform("/map", "/target", ros::Time(0), transform2);
			q2 = transform2.getRotation(); 
		}
		catch (tf::TransformException ex){	
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}

		geometry_msgs::PoseStamped this_pose_stamped;
		geometry_msgs::PoseStamped target_this_pose_stamped;
		this_pose_stamped.pose.position.x = transform.getOrigin().x();
		this_pose_stamped.pose.position.y = transform.getOrigin().y();

		target_this_pose_stamped.pose.position.x = transform2.getOrigin().x();
		target_this_pose_stamped.pose.position.y = transform2.getOrigin().y();

		geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(tf::getYaw(q));
		geometry_msgs::Quaternion goal2_quat = tf::createQuaternionMsgFromYaw(tf::getYaw(q2));
		this_pose_stamped.pose.orientation.x = goal_quat.x;
		this_pose_stamped.pose.orientation.y = goal_quat.y;
		this_pose_stamped.pose.orientation.z = goal_quat.z;
		this_pose_stamped.pose.orientation.w = goal_quat.w;
		
		target_this_pose_stamped.pose.orientation.x = goal2_quat.x;
		target_this_pose_stamped.pose.orientation.y = goal2_quat.y;
		target_this_pose_stamped.pose.orientation.z = goal2_quat.z;
		target_this_pose_stamped.pose.orientation.w = goal2_quat.w;

		this_pose_stamped.header.stamp=current_time;
		this_pose_stamped.header.frame_id="robot_path";
		path.poses.push_back(this_pose_stamped);
		
		target_this_pose_stamped.header.stamp=current_time;
		target_this_pose_stamped.header.frame_id="target_path";
		target_path.poses.push_back(target_this_pose_stamped);

		path_pub.publish(path);
		target_path_pub.publish(target_path);
		
		last_time = current_time;
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
