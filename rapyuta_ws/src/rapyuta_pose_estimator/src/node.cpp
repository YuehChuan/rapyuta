#include "rapyuta_pose_estimator/rapyuta_pose_estimator.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "rapyuta_pose_estimator");
    rapyuta_pose_estimator::TSNode ts_node;
    ROS_INFO("start apriltags subscribe");

    ros::spin();
    return 0;

}
