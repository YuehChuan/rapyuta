
#ifndef TAGS_SUB_H_
#define TAGS_SUB_H_

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <ros/ros.h>

//syncronize apriltags subscription
#include<message_filters/subscriber.h>
#include<message_filters/synchronizer.h>
#include<message_filters/sync_policies/approximate_time.h>

//apriltags message
#include "rapyuta_msgs/AprilTagDetection.h"
#include "rapyuta_msgs/AprilTagDetections.h"
/*
typedef sync_policies::ApproximateTime<rapyuta_msgs::AprilTagDetections,rapyuta_msgs::AprilTagDetections,rapyuta_msgs::AprilTagDetections> MySyncPolicy;

namespace tags_sub
class tags_sub
{
    public:
        ros::NodeHandle node;
        void tags_cb(const rapyuta_msgs::AprilTagDetections::ConstPtr& msg);
    tags_sub();
    private:
    message_filters::Subscriber<rapyuta_msgs::AprilTagDetections> *camera1_sub;
    message_filters::Subscriber<rapyuta_msgs::AprilTagDetections> *camera2_sub;
    message_filters::Subscriber<rapyuta_msgs::AprilTagDetections> *camera3_sub;
    Synchronizer<MySyncPolicy> *sync;
};


}
*/
#endif /*TAGS_SUB_H_*/
