#include "maplistener.h"

MapListener::MapListener(double inflationRadius, QObject *parent) :
    QObject(parent), inflationRadius_(inflationRadius)
{
}

gridmap_2d::GridMap2D MapListener::sharedMap()
{
    QMutexLocker locker(&mapListenerMutex_);
    return sharedMap_;
}

gridmap_2d::GridMap2D MapListener::frontierMap()
{
    QMutexLocker locker(&mapListenerMutex_);
    return frontierMap_;
}

gridmap_2d::GridMap2D MapListener::inflatedMap()
{
    QMutexLocker locker(&mapListenerMutex_);
    return inflatedMap_;
}

void MapListener::sharedMap_cb(const nav_msgs::OccupancyGrid::ConstPtr &new_map)
{
    QMutexLocker locker(&mapListenerMutex_);
    // Buffer new map
    sharedMap_ = gridmap_2d::GridMap2D(new_map);
    frontierMap_ = gridmap_2d::GridMap2D(new_map);
    inflatedMap_ = gridmap_2d::GridMap2D(new_map);
    // Inflate the maps
    frontierMap_.inflateMap(inflationRadius_);
    inflatedMap_.inflateMap(inflationRadius_);
    pub_inflatedMap_.publish(inflatedMap_.toOccupancyGridMsg());
    // Insert the poses in the frontier map
    uint mx, my;
    tf::StampedTransform transform;
    double timeDelayed;
    cv::Mat temp = frontierMap_.gridMap().clone();
    for (int i = 0; i < 2; i++)
    {
        if(queryPose.getPose(i, transform, timeDelayed))
        {
            sharedMap_.worldToMap(transform.getOrigin().x(),
                                  transform.getOrigin().y(),
                                  my, mx);
            cv::circle(temp, cv::Point(mx, my), 15, cv::Scalar(0), -1);
        }
    }
    frontierMap_.updateGridMap(temp, false);
}

void MapListener::start()
{
    ros::NodeHandle n;
    mapSub_ = n.subscribe("/projected_map", 1, &MapListener::sharedMap_cb, this);
    pub_inflatedMap_ = n.advertise<nav_msgs::OccupancyGrid>("MapListener/inflatedMap", 1);
    timer_ = new QTimer();
    timer_->start(1);
    connect(timer_, SIGNAL(timeout()), this, SLOT(run()));
}

void MapListener::run()
{
    ros::spinOnce();
}
