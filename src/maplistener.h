#ifndef MAPLISTENER_H
#define MAPLISTENER_H

#include <ros/ros.h>
#include "gridmap2d.h"
#include "querypose.h"
#include <nav_msgs/OccupancyGrid.h>
#include <QObject>
#include <QTimer>
#include <QMutex>
class MapListener : public QObject
{
    Q_OBJECT
public:
    explicit MapListener(double inflationRadius = 0.25, QObject *parent = 0);
    gridmap_2d::GridMap2D sharedMap();
    gridmap_2d::GridMap2D frontierMap();
    gridmap_2d::GridMap2D inflatedMap();

private:
    gridmap_2d::GridMap2D sharedMap_;
    gridmap_2d::GridMap2D frontierMap_;
    gridmap_2d::GridMap2D inflatedMap_;
    void sharedMap_cb(const nav_msgs::OccupancyGrid::ConstPtr &new_map);

    QTimer *timer_;
    ros::Publisher pub_inflatedMap_;
    double inflationRadius_;
    ros::Subscriber mapSub_;
    QMutex mapListenerMutex_;
Q_SIGNALS:
    
public Q_SLOTS:
    void start();

private Q_SLOTS:
    void run();
};

extern MapListener mapListener;
#endif // MAPLISTENER_H
