#include "querypose.h"

QueryPose::QueryPose(double timeout, QObject *parent) :
    QObject(parent), timeout_(timeout)
{
}

bool QueryPose::getPose(uint poseNumber, tf::StampedTransform &pose, double &timeDelayed)
{
    ros::Time now = ros::Time::now();
    ros::Duration delay = (now - pose_[poseNumber].stamp_);
    timeDelayed = delay.toSec();
    if (timeDelayed < timeout_)
    {
        pose = pose_[poseNumber];
        return true;
    }
    else
        return false;
}

void QueryPose::run()
{

    tf::TransformListener tf;
    try
    {
        tf.lookupTransform("map", "camera0_link", ros::Time(0), pose_[0]);
    }
    catch(tf::TransformException ex){}

    try
    {
        tf.lookupTransform("map", "camera1_link", ros::Time(0), pose_[1]);
    }
    catch(tf::TransformException ex){}
}

void QueryPose::start()
{
    timer_ = new QTimer();
    // 100Hz
    timer_->start(10);
    connect(timer_, SIGNAL(timeout()), this, SLOT(run()));
}
