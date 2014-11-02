#ifndef MISSION_PLANNER_H
#define MISSION_PLANNER_H

/// ROS LIBRARIES
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
/// OPENCV LIBRARY
#include <opencv2/opencv.hpp>
/// USER-DEFINED LIBRARIES
#include "gridmap2d.h"
#include "merry_navigation_stack/QueryForFrontiers.h"
#include "hungarian.h"
#include "dijkstra.h"
#include "qlogconsole.h"
#include "maplistener.h"
#include "Timer.h"
/// QT LIBRARIES
#include <QObject>
#include <QTimer>
#include <QImage>
#include <QThread>

class MissionPlanner : public QObject
{
    Q_OBJECT
private:
    // Callback function for servicing a serviceclient.
    bool requestForfrontier_cb(merry_navigation_stack::QueryForFrontiersRequest &req, merry_navigation_stack::QueryForFrontiersResponse &res);

    // Some private methods for mission planning.
    /// Brief: Detects the frontiers in the ternary grid map.
    /// Param gridMap: Input gridMap
    /// Return edges: The frontier edges in the cv::Mat
    /// Return contours: Vector of vectors of points of contours
    /// Returns origCOG: Contains the original COG of the contours.
    /// Returns relocatedCOG: Relocated COG using Dijkstra's algorithm.
    /// Param dilate: To dilate or not?
    void detectFrontiers(const gridmap_2d::GridMap2D& gridMap, cv::Mat& edges,
                         std::vector<std::vector<cv::Point> >& contours, std::vector<cv::Point>& origCOG,
                         std::vector<cv::Point>& relocatedCOG, bool dilate = true);

    /// Brief: Canny edge filters the orig_img to cannyOP.
    void filterImage(const cv::Mat& orig_img, cv::Mat& cannyOP);

    /// For visualization purposes
    visualization_msgs::Marker points_;
    visualization_msgs::Marker chosenFrontier_;
    ros::Publisher markerPub_;
    /// ROS Service Server
    ros::ServiceServer frontierAssignerServer_;
    /// For threading
    QTimer *timer_;

private Q_SLOTS:
    void run();

public Q_SLOTS:
    void start();

public:
    /// By default, inflationRadius is 25 cm.
    explicit MissionPlanner(QObject *parent = 0);
    QLogConsole log_[2];
};
#endif
