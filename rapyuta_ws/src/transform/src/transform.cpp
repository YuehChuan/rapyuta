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
	tf::TransformListener tf(ros::Duration(10));
	ros::Rate rate(10);
	while (ros::ok()) {
			//get robot position
			tf::StampedTransform transform;
			tf::Quaternion q;
			try {
				tf.lookupTransform("/world", "/base_link", ros::Time(0), transform);
				q = transform.getRotation(); 
			}
			catch (tf::TransformException ex) {	
				ROS_ERROR("%s",ex.what());
				ros::Duration(1.0).sleep();
			}
			ROS_INFO_STREAM("x = " << transform.getOrigin().x() * 100 << " y = " << transform.getOrigin().y() * 100 << " theta = " << tf::getYaw(q) * R2D);
		ros::spinOnce();	
	}
	ros::spin();
	return 0;
}
