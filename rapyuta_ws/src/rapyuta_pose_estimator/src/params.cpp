#include <rapyuta_pose_estimator/ros/params.h>

namespace rapyuta_pose_estimator {

Params* Params::s_instance = NULL;


Params::Params(ros::NodeHandle& privateNodeHandle)
    : m_privateNodeHandle(privateNodeHandle)
{
    assert(!s_instance);
    s_instance = this;
}

}
