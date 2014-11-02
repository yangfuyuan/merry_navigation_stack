#include "localplanner.h"

LocalPlanner::LocalPlanner(QString quad, QString camera) : isControlledByRviz(false)
{
    ns_quad = quad;
    camerax_link = camera + "_link";

    if(quad == "quad1")
        markerPath_.id = 0;
    else if(quad == "quad2")
        markerPath_.id = 1;
}

void LocalPlanner::start()
{
    sub_markerPath = n.subscribe("global_planner/path", 1, &LocalPlanner::wayPoints_cb, this);
    subRviz_ = n.subscribe("move_base_simple/goal", 1, &LocalPlanner::moveToWayPoint_cb, this);

    pub_yaw_goal = n.advertise<merryMsg::Msg>(ns_quad.toStdString() + "/goal/yaw", 1);
    pub_pitch_goal = n.advertise<merryMsg::Msg>(ns_quad.toStdString() + "/goal/pitch", 1);
    pub_roll_goal = n.advertise<merryMsg::Msg>(ns_quad.toStdString() + "/goal/roll", 1);
    pub_waypoint_goal_ = n.advertise<geometry_msgs::Point>(ns_quad.toStdString() + "/goal/waypoint", 1);

    isPathReached = false;
    isWayPointsAvailable = false;
    wayPointIterator = 0;
    cYaw = 0;
    cDistance = 0;

    timer_ = new QTimer();
    timer_->start(100);

    QObject::connect(timer_, SIGNAL(timeout()), this, SLOT(run()));
    QObject::connect(this, SIGNAL(goalYaw(double)), this, SLOT(send_yaw_goal(double)));
    QObject::connect(this, SIGNAL(goalPitch(double)), this, SLOT(send_pitch_goal(double)));
    QObject::connect(this, SIGNAL(goalRoll(double)), this, SLOT(send_roll_goal(double)));
}

void LocalPlanner::run()
{
    ros::spinOnce();

    // Remember, the fixed frame is set to "map".
    // Hence, the retrieved pose can be directly related with camera0_link and camera1_link
    // as long as both of their fixed frames are set to "map" which it may be!

    // To get the camera0_link and camera1_link, just lookup for the transform of "map" to camera0_link
    // and camera1_link.
    tf::StampedTransform sTransform;
    double timeDelayed;
    if(!queryPose.getPose(markerPath_.id, sTransform, timeDelayed))
    {
        // This may be used when SLAM stops processing and localization becomes unavailable. (I think)
        log_ << "Pose is <font color = \"red\"> INVALID</font>. Outdated by <i>" << timeDelayed << "</i> seconds\n";
        log_ << "Yaw = 0    Roll = 0    Pitch = 0\n";
        Q_EMIT goalYaw(0);
        Q_EMIT goalRoll(0);
        Q_EMIT goalPitch(0);
        return;
    }
    else
    {
        log_ << "Pose is <font color = \"green\"> VALID</font>. Outdated by <i>" << timeDelayed << "</i> seconds\n";
    }

    // However, the goal is to find the euclidean distance and the bearing of the waypoint from the POV of the camerax_pose
    // with the X-axis as its reference axis. This can be done by computing by projecting the waypoint to the coordinate frame
    // of the camerax_pose.
    // T(camerax_link -> pose) = T(map->camerax_link).inverse * T(map->pose)
    waypoint = sTransform.inverseTimes(waypointTransform);

    if(isWayPointsAvailable || isControlledByRviz)
    {
        if (!isControlledByRviz)
            this->nextWayPoint();

        // This code retrieves the orientation of the waypoint with respect to the coordinate frame
        // of the camerax_link.
        //        yaw = tf::getYaw(waypoint.getRotation()) * 180 / M_PI;

        // Retrieve yaw in degrees [-180 -> 180]
        // This does not take into account the backview of the quadrotor.
        cYaw = atan2(waypoint.getOrigin().y(), waypoint.getOrigin().x()) * 180 / M_PI;
        // Retrieve the euclidean distance
        cDistance = std::sqrt(waypoint.getOrigin().x()*waypoint.getOrigin().x() +
                              waypoint.getOrigin().y()*waypoint.getOrigin().y());

        // Now, do your own thing.
        // Case 1: If the euclidean distance to the goalpoint is greater than 0.5 meters,
        // the yaw reading is acceptable.
        if (cDistance >= .5)
        {
            // If the angle is within [-10,10] degrees, consider the yaw-ing process finished.
            if(std::abs(cYaw) <= 10)
            {
                log_ << "Euclidean Distance = " << cDistance << " m\n";
                Q_EMIT goalPitch(cDistance);
            }
            else
            {
                log_ << "Delta yaw = " << -cYaw << " degrees\n";
                Q_EMIT goalYaw(-cYaw);
            }
        }
        //        // Case 2: If distance is less than 0.5, yaw readings cannot be trusted.
        //        // We will utilize the x-vector and y-vector for position stabilization.
        //        else
        //        {
        //            std::cerr << "Drift control: [roll,pitch]" << waypoint.getOrigin().y() << ", "
        //                      << waypoint.getOrigin().x() << std::endl;
        //            Q_EMIT goalRoll(waypoint.getOrigin().y());
        //            Q_EMIT goalPitch(waypoint.getOrigin().x());
        //        }

        // If the waypoint origin is within 20cm radius, then the waypoint
        // has been reached!
        //if(std::abs(waypoint.getOrigin().y()) < 0.5 &&
        //           std::abs(waypoint.getOrigin().x()) < 0.5 && !isControlledByRviz)
        else
        {
            isPathReached = true;
            log_ << "Waypoint has been reached!\n";
        }
    }
    //    else
    //    {
    //        std::cerr << "Drift control: [roll,pitch]" << waypoint.getOrigin().y() << ", "
    //                  << waypoint.getOrigin().x() << std::endl;
    //        Q_EMIT goalRoll(waypoint.getOrigin().y());
    //        Q_EMIT goalPitch(waypoint.getOrigin().x());
    //    }
}

void LocalPlanner::wayPoints_cb(visualization_msgs::Marker markerPath)
{
    isControlledByRviz = false;
    markerPath_.points.clear();
    markerPath_ = markerPath;

    wayPointIterator = 0;
    isPathReached = true;
    isWayPointsAvailable = true;
}

void LocalPlanner::nextWayPoint()
{
    if(isPathReached)
    {
        if(wayPointIterator < markerPath_.points.size())
        {
            p = markerPath_.points.at(wayPointIterator);
            waypointTransform.setOrigin(tf::Vector3(p.x, p.y, 0));
            waypointTransform.setRotation(tf::Quaternion(tf::Vector3(0,0,1), 0));
            wayPointIterator++;
            isPathReached = false;
            pub_waypoint_goal_.publish(p);
        }
        else
            isWayPointsAvailable = false;
    }
}

void LocalPlanner::send_yaw_goal(double yaw)
{
    yaw_goal.header.stamp = ros::Time::now();
    yaw_goal.data = yaw;

    pub_yaw_goal.publish(yaw_goal);
}

void LocalPlanner::send_pitch_goal(double pitch)
{
    pitch_goal.header.stamp = ros::Time::now();
    pitch_goal.data = pitch;

    pub_pitch_goal.publish(pitch_goal);
}

void LocalPlanner::send_roll_goal(double roll)
{
    roll_goal.header.stamp = ros::Time::now();
    roll_goal.data = roll;

    pub_roll_goal.publish(roll_goal);
}

void LocalPlanner::moveToWayPoint_cb(geometry_msgs::PoseStamped::ConstPtr pose)
{
    isControlledByRviz = true;
    // Only update the waypointTransform which will be periodically used in the main function.
    waypointTransform.setOrigin(tf::Vector3(pose->pose.position.x, pose->pose.position.y, pose->pose.position.z));
    waypointTransform.setRotation(tf::Quaternion(pose->pose.orientation.x, pose->pose.orientation.y,
                                                 pose->pose.orientation.z, pose->pose.orientation.w));
    geometry_msgs::Point p;
    p.x = pose->pose.position.x;
    p.y = pose->pose.position.y;
    pub_waypoint_goal_.publish(p);
}
