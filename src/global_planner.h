#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include "gridmap2d.h"
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include "dijkstra.h"
#include "merry_navigation_stack/QueryForFrontiers.h"
#include "qlogconsole.h"
#include "maplistener.h"
#include <QTimer>
#include <QImage>
#include <string>
#include <std_msgs/Bool.h>
// [X] 1. Create a class that listens to the RgbdslamClient/shared_map topic and buffer the map.
// [X] 2. Create the usual Qt-ROS framework (QTimer looping on ros::SpinOnce)
// [X] 3. Inflate the map and display the inflated map in a seperate topic.
// [X] 4. Create a Ros service client that calls a service from mission planner to retrieve a target for the current robot
// [X] 5. Place the dijsktra's algorithm here and retrieve the current robot's pose.
// [X] 6. Create a path and set the starting to robot's pose and the goal as the frontier target.
// [X] 7. Publish as marker points the path.
// [X] 8. Send only the line segment points

/// Program fully autonomous operation. Yikes~!
// [X] 1. Add a button in the GUI for autonomous navigation
// [] 2. Determine if each quadrotor had already reach the a certain radius of the frontier
// [] 3. Call the respective frontier callbacks. If call failed, then land.
// [] 4. Also add an if condition for toggling autonomous navigation. So use QTimer.
// [] 5. Create a subscription for the local planner which commands the quadrotor to land. Stop all operations then.
// [] 6. Connect the signal from widget1 to global planner.

class GlobalPlanner : public QObject
{
    Q_OBJECT

private:

    /// Fields
    static bool isInstantiated_;
    static ros::Publisher markerPub_;

    std::vector<cv::Point2i> path_;
    visualization_msgs::Marker markerPath_;
    ros::ServiceClient frontierServiceClient_;
    /// For threading
    QTimer *timer_;

    /// For autonomous operation
    QTimer *timerAutonomous_;
    bool isLand_;
    bool isAutonomousEnabled_;
    bool isGateTrue_;
    bool isFirstLoop_;
    geometry_msgs::Point currentFrontier_;
    ros::Publisher pubLand_;

private Q_SLOTS:
    /// ros spin
    void run();
    /// For autonomous operation
    void runAutonomous();

public Q_SLOTS:
    void start();
    void on_reqBtn_clicked();
    void toggleAutonomous();

public:
    GlobalPlanner(int id);
    QLogConsole log_;
};
#endif
