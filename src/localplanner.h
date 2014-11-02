#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <merryMsg/Msg.h>
#include <QObject>
#include <QApplication>
#include <QTimer>
#include <QDebug>
#include "qlogconsole.h"
#include "querypose.h"
class LocalPlanner : public QObject
{
    Q_OBJECT

public:
    LocalPlanner(QString quad, QString camera);
    void moveToWayPoint_cb(geometry_msgs::PoseStamped::ConstPtr pose);
    void wayPoints_cb(visualization_msgs::Marker markerPath);
    QLogConsole log_;

private:
    tf::Transform waypointTransform;
    tf::Transform waypoint;
    tf::TransformListener tf;
    tf::StampedTransform sTransform;

    ros::NodeHandle n;
    ros::Subscriber subRviz_;
    ros::Subscriber sub_markerPath;
    ros::Publisher pub_yaw_goal;
    ros::Publisher pub_pitch_goal;
    ros::Publisher pub_roll_goal;
    ros::Publisher pub_waypoint_goal_;


    merryMsg::Msg yaw_goal, pitch_goal, roll_goal;

    geometry_msgs::Point p;
    visualization_msgs::Marker markerPath_;

    int wayPointIterator;
    double cYaw, cDistance;
    bool isPathReached, isWayPointsAvailable;

    QTimer *timer_;
    QString ns_quad, camerax_link;

    bool isControlledByRviz;

    void nextWayPoint();

public Q_SLOTS:
    void start();

private Q_SLOTS:
    void run();
    void send_yaw_goal(double yaw);
    void send_pitch_goal(double pitch);
    void send_roll_goal(double roll);

Q_SIGNALS:
    void goalYaw(double yaw);
    void goalPitch(double goal);
    void goalRoll(double roll);
};
