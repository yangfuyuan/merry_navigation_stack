#include "mission_planner.h"
MissionPlanner::MissionPlanner(QObject *parent):
    QObject(parent)
{
    points_.header.frame_id = "/map";
    points_.ns = "frontiers";
    points_.action = visualization_msgs::Marker::ADD;
    points_.type = visualization_msgs::Marker::POINTS;
    points_.scale.x = 0.2;
    points_.scale.y = 0.2;
    points_.scale.z = 0.2;
    points_.color.g = 1.0f;
    points_.color.a = 1.0f;
    points_.id = 0;

    chosenFrontier_.header.frame_id = "/map";
    chosenFrontier_.ns = "frontiers";
    chosenFrontier_.action = visualization_msgs::Marker::ADD;
    chosenFrontier_.type = visualization_msgs::Marker::SPHERE;
    chosenFrontier_.scale.x = 0.2;
    chosenFrontier_.scale.y = 0.2;
    chosenFrontier_.scale.z = 0.2;
    chosenFrontier_.color.a = 1.0f;
    chosenFrontier_.pose.orientation.w = 0;
    chosenFrontier_.pose.orientation.x = 0;
    chosenFrontier_.pose.orientation.y = 0;
    chosenFrontier_.pose.orientation.z = 0;
    chosenFrontier_.pose.position.z = 0;
}

void MissionPlanner::start()
{
    ros::NodeHandle n;
    frontierAssignerServer_ = n.advertiseService("mission_planner/request_for_frontier", &MissionPlanner::requestForfrontier_cb, this);
    markerPub_ = n.advertise<visualization_msgs::Marker>("mission_planner/frontierMarkers", 10);
    // QTimer loop
    timer_ = new QTimer();
    connect(timer_, SIGNAL(timeout()), this, SLOT(run()));
    // 1ms timeout
    timer_->start(1);
}

void MissionPlanner::run()
{
    ros::spinOnce();
}

void MissionPlanner::filterImage(const cv::Mat& orig_img, cv::Mat& cannyOP)
{
    double otsu_thresh_val = cv::threshold(
                orig_img, cannyOP, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU
                );
    double high_thresh_val  = otsu_thresh_val,
            lower_thresh_val = otsu_thresh_val * 0.5;
    cv::Canny(orig_img, cannyOP, lower_thresh_val, high_thresh_val);
}

void MissionPlanner::detectFrontiers(const gridmap_2d::GridMap2D& gridMap, cv::Mat& edges,
                                     std::vector<std::vector<cv::Point> >& contours, std::vector<cv::Point>& origCOG,
                                     std::vector<cv::Point>& relocatedCOG, bool dilate)
{
    // 0. Reset the output vectors
    contours.clear();
    origCOG.clear();
    relocatedCOG.clear();

    // 1. Apply edge filtering to both ternary map (Free, occupied, unknown) and binary map (free, occupied)
    cv::Mat binaryGrad, ternaryGrad;
    filterImage(gridMap.binaryMap(), binaryGrad);
    filterImage(gridMap.gridMap(), ternaryGrad);
    // 2. Edge Removal Filter (i.e. remove the occupied edges, leaving only the frontier edges behind) via
    // image subtraction
    ternaryGrad -= binaryGrad;

    // OPTIONAL: Dilate the edges. Eliminate false frontiers
    if (dilate)
    {
        cv::dilate(ternaryGrad, edges, cv::Mat());
        edges.copyTo(ternaryGrad);
    }
    else
        ternaryGrad.copyTo(edges);

    // 3. Find contours and filter out stray frontier cell edges by thresholding it with 1 meter.
    std::vector<cv::Vec4i> heirarchy;

    cv::findContours(edges, contours, heirarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

    for (int i = 0; i < contours.size(); i++)
    {
        double sumX = 0;
        double sumY = 0;
        for (int j = 0; j < contours[i].size(); j++)
        {
            sumX += contours[i][j].x;
            sumY += contours[i][j].y;
        }

        cv::Point2i cog(sumX/contours[i].size(), sumY/contours[i].size());
        // Find the nearest free cell for frontier.
        cv::Point2i p;
        Dijkstra::findNearestFreeCell(gridMap.gridMap(), cog, p, 1);
        relocatedCOG.push_back(p);
        origCOG.push_back(cog);
    }
}

/// Just a FUNCTION
int** vector_to_matrix(std::vector<std::vector<int> > m) {
    int i,j;
    int** r;
    r = (int**)calloc(m.size(),sizeof(int*));
    for(i=0;i<m.size();i++)
    {
        r[i] = (int*)calloc(m[i].size(),sizeof(int));
        for(j=0;j<m[i].size();j++)
            r[i][j] = m[i][j];
    }
    return r;
}


bool MissionPlanner::requestForfrontier_cb(merry_navigation_stack::QueryForFrontiersRequest &req, merry_navigation_stack::QueryForFrontiersResponse &res)
{
    assert (req.id == 0 || req.id == 1);

    /// Detect frontiers
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Point> relocatedCOG;
    std::vector<cv::Point> origCOG;
    cv::Mat edges;
    detectFrontiers(mapListener.frontierMap(), edges, contours, origCOG, relocatedCOG);

    /// For display purposes
    /// Display frontier map with original and relocated COG, as well as the contours.
    cv::Mat display0, display1;
    mapListener.frontierMap().gridMap().copyTo(display0);
    mapListener.frontierMap().gridMap().copyTo(display1);
    cv::cvtColor(display0, display0, CV_GRAY2RGBA);
    cv::cvtColor(display1, display1, CV_GRAY2RGBA);
    cv::drawContours(display0, contours, -1, cv::Scalar(0, 0, 255, 255));

    log_[0] << DisplayImage(display0, 0, "Frontier Edges");
    for (int i = 0; i < origCOG.size(); i++)
        cv::circle(display1, origCOG[i], 5, cv::Scalar(0,255,0,255));
    for (int i = 0; i < relocatedCOG.size(); i++)
        cv::circle(display1, relocatedCOG[i], 5, cv::Scalar(255,0,0,255));
//    Timer timer;
//    timer.start();
//    while(timer.getElapsedTimeInMilliSec() < 50);
//    timer.stop();
    log_[0] << DisplayImage(display1, 1, "Relocated and Orig COGs");

    /// Get localization
    bool isLocalizationValid[2] = {false};
    tf::StampedTransform sTransform[2];
    double timeDelayed;
    for (int i = 0; i < 2; i++)

        if(queryPose.getPose(i, sTransform[i], timeDelayed))
        {
            isLocalizationValid[i] = true;
            log_[req.id] << sTransform[i].getOrigin().x() << ",";
            log_[req.id] << sTransform[i].getOrigin().y() << ": " << timeDelayed << "\n";
        }

    /// Visualize frontiers in RVIZ as green points!!
    // Draw the markers
    points_.color.g = 1.0f;
    points_.color.r = 0.0f;
    points_.id = 0;
    points_.header.stamp = ros::Time::now();
    points_.points.clear();
    double wx, wy;
    for (int j = 0; j < relocatedCOG.size(); j++)
    {
        // Convert the frontiers to the world.
        mapListener.inflatedMap().mapToWorld(relocatedCOG[j].y, relocatedCOG[j].x,
                                          wx, wy);
        // Push to be visualized in RVIZ.
        geometry_msgs::Point p;
        log_[req.id] << "MissionPlanner: Frontiers[" << j << "] = " << wx << "," << wy << "\n";
        p.x = wx;
        p.y = wy;
        points_.points.push_back(p);
    }
    markerPub_.publish(points_);

    uint mx, my;
    /// Fail Safe Control: If the localization is invalid for the particular request, then return false.
    if (req.id == 0 && !isLocalizationValid[0] && !mapListener.inflatedMap().worldToMap(sTransform[0].getOrigin().x(), sTransform[0].getOrigin().y(),
                                                                                     my, mx))
    {
        log_[req.id] << "MissionPlanner: Failed to localize robot 0. Mission planner refuses to give a frontier assignment.\n";
        return false;
    }
    else if (req.id == 1 && !isLocalizationValid[1] && !mapListener.inflatedMap().worldToMap(sTransform[1].getOrigin().x(), sTransform[1].getOrigin().y(),
                                                                                          my, mx))
    {
        log_[req.id] << "MissionPlanner: Failed to localize robot 1. Mission planner refuses to give a frontier assignment.\n";
        return false;
    }

    /// Hungarian Algorithm
    // We need a NxN cost matrix. The cost metric would be the taxicab geometry cost derived from
    // dijkstra's from the starting pose to the frontiers.
    // Depending on the size of frontiers and robots, the NxN dimensions would possess the
    // greater size and fill the square matrix with dummy robots/frontiers for the hungarian
    // algorithm to work.
    int n = 2;
    if (relocatedCOG.size() > 2)
        n = relocatedCOG.size();
    std::vector<std::vector<int> > costMatrix(n, std::vector<int>(n, -1));

    bool isValid = false;
    log_[req.id] << "MissionPlanner: isLocalizationValid[0] = " << isLocalizationValid[0] << "\n";
    log_[req.id] << "MissionPlanner: isLocalizationValid[1] = " << isLocalizationValid[1] << "\n";

    // Create the cost matrix.
    int maxCost = 0;
    for (int i = 0; i < 2; i++)
        if (isLocalizationValid[i])
        {
            std::vector<int>costVector;
            uint mx, my;
            if(!mapListener.inflatedMap().worldToMap(sTransform[i].getOrigin().x(), sTransform[i].getOrigin().y(),
                                                  my, mx))
                break;
            Dijkstra::computeCostVector(mapListener.inflatedMap().gridMap(), cv::Point2i(mx, my), relocatedCOG, costVector);
            for (int j = 0; j < costVector.size(); j++)
            {
                if (i == req.id && costVector[j] != -1)
                    isValid = true;
                if (costVector[j] > maxCost)
                    maxCost = costVector[j];
                if (costVector[j] == -1)
                    costMatrix[i][j] = -2;
                else
                    costMatrix[i][j] = costVector[j];
            }
        }

    // Check if the client robot has any accessible target.
    if (!isValid)
    {
        log_[req.id] << "MissionPlanner: There is no reachable target for robot " << req.id << "\n";
        return false;
    }

    // Fill in the dummy values.
    for (int j = 0; j < n; j++)
        for (int i = 0; i < n; i++)
        {
            if (costMatrix[i][j] == -1)
                costMatrix[i][j] = maxCost;
            else if (costMatrix[i][j] == -2)
                costMatrix[i][j] = maxCost + 10;
        }

    int **costMat = vector_to_matrix(costMatrix);
    hungarian_problem_t p;
    int matrix_size = hungarian_init(&p, costMat, n,n, HUNGARIAN_MODE_MINIMIZE_COST) ;

    log_[req.id] << "MissionPlanner: Print cost matrix: " << "\n";
    log_[req.id] << hungarian_print_costmatrix(&p);
    hungarian_solve(&p);

    int i = 0;
    // Find the frontier assignment for the requested id.
    for(; i < n; i++)
    {
        if(p.assignment[req.id][i] == 1)
            break;
    }

    // The assigned frontier must not be a dummy frontier.
    if (i < relocatedCOG.size())
    {
        // Place the result to the response.
        res.frontier.pose.position.x = points_.points[i].x;
        res.frontier.pose.position.y = points_.points[i].y;
        res.frontier.header.frame_id = "\map";
        res.frontier.header.stamp = ros::Time::now();
        res.currentPose.pose.position.x = sTransform[req.id].getOrigin().x();
        res.currentPose.pose.position.y = sTransform[req.id].getOrigin().y();
        res.currentPose.header.frame_id = "\map";
        res.currentPose.header.stamp = ros::Time::now();

        // Visualize the chosen frontier as sphere in RVIZ
        // For robot 0, the color is blue.
        // For robot 1, the color is red.
        chosenFrontier_.id = req.id + 1;
        if (req.id == 0)
            chosenFrontier_.color.b = 1.0f;
        else
            chosenFrontier_.color.r = 1.0f;

        chosenFrontier_.pose.position.x = points_.points[i].x;
        chosenFrontier_.pose.position.y = points_.points[i].y;

        log_[req.id] << "MissionPlanner: Chosen frontier: " << i << "\n";
        log_[req.id] << "MissionPlanner: Print assignment: " << "\n";
        log_[req.id] << hungarian_print_assignment(&p);
        markerPub_.publish(chosenFrontier_);

        /* free used memory */
        hungarian_free(&p);
        free(costMat);
        return true;
    }
    else
    {
        log_[req.id] << "MissionPlanner: Assignment failure. Invalid target.\n";
        log_[req.id] << "MissionPlanner: Print assignment: " << "\n";
        log_[req.id] << hungarian_print_assignment(&p);

        /* free used memory */
        hungarian_free(&p);
        free(costMat);
        return false;
    }
}
