#include "rapyuta_pose_estimator/rapyuta_pose_estimator.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "tags_sub");
    tags_sub::TSNode ts_node;
    ROS_INFO("start apriltags subscribe");

    ros::spin();
    return 0;

}
