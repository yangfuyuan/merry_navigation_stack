#include "dijkstra.h"

namespace Dijkstra
{

/// Some helper functions for dijkstra's algorithm.
uint8_t get_cell_at(const cv::Mat & image, int x, int y)
{
    assert(x < image.cols);
    assert(y < image.rows);
    return image.data[y * image.cols + x];
}

/// Some helper functions for dijkstra's algorithm.
bool checkIfNotOutOfBounds(cv::Point2i current, int rows, int cols)
{
    return (current.x >= 0 && current.y >= 0 &&
            current.x < cols && current.y < rows);
}

bool operator<(const vertex& a, const vertex& b)
{
    return ( a.cost_ >= b.cost_);
}

/// Brief: Finds the shortest possible path from starting position to the goal position
/// Param gridMap: The stage where the tracing of the shortest possible path will be performed.
/// Param start: The starting position in the gridMap. It is assumed that start cell is a free cell.
/// Param goal: The goal position in the gridMap. It is assumed that the goal cell is a free cell.
/// Param path: Returns the sequence of free cells leading to the goal starting from the starting cell.
bool findPathViaDijkstra(const cv::Mat& gridMap, cv::Point2i start, cv::Point2i goal, std::vector<cv::Point2i>& path)
{
    //    cv::Mat image = gridMap.clone();

    // Clear the path just in case
    path.clear();
    // Create a visited set which contains a boolean value whether it is visited or not and a backpointer to the previous path.
    std::vector<std::pair<bool, cv::Point2i> > visited(gridMap.rows * gridMap.cols, std::make_pair(false, cv::Point2i(0,0)));
    // Priority queue was used which enables us to always get the lowest cost vertex without iteration.
    std::priority_queue <vertex, std::vector<vertex> > working;

    // Initialize working set. We are going to perform the djikstra's
    // backwards in order to get the actual path without reversing the path.
    working.push(vertex(goal, goal, 0));

    // Conditions in continuing
    // 1.) Working is empty implies all nodes are visited.
    // 2.) If the start is still not found in the working visited set.
    // The Dijkstra's algorithm
    while(!working.empty() && !visited[start.y*gridMap.cols+start.x].first)
    {

        // Get the top of the STL.
        // It is already given that the top of the multimap has the lowest cost.
        vertex currentCell = working.top();
        working.pop();
        // If the currentCell is already included in the visited set then abort.
        if (visited[currentCell.id_.y*gridMap.cols+currentCell.id_.x].first)
            continue;

        // Mark it visited
        visited[currentCell.id_.y*gridMap.cols+currentCell.id_.x].first = true;
        visited[currentCell.id_.y*gridMap.cols+currentCell.id_.x].second = currentCell.from_;

        // Check all arcs
        // Only insert the cells into working under these 3 conditions:
        // 1. The cell is not in visited cell
        // 2. The cell is not out of bounds
        // 3. The cell is free
        for (int x = currentCell.id_.x-1; x <= currentCell.id_.x+1; x++)
            for (int y = currentCell.id_.y-1; y <= currentCell.id_.y+1; y++)
            {
                // No one is neighbor with itself, right?
                if (x == currentCell.id_.x && y == currentCell.id_.y)
                    continue;

                if (checkIfNotOutOfBounds(cv::Point2i(x, y), gridMap.rows, gridMap.cols) &&
                        get_cell_at(gridMap, x, y) == gridmap_2d::GridMap2D::FREE)
                {
                    double cost = currentCell.cost_ + sqrt((x-currentCell.id_.x)*(x-currentCell.id_.x)
                                                           +(y-currentCell.id_.y)*(y-currentCell.id_.y));

                    /// Uncomment this to make it 4 children
                    //                    if (!(x == currentCell.id_.x || y == currentCell.id_.y))
                    //                        continue;
                    /// Ends here..
                    working.push(vertex(cv::Point2i(x,y), currentCell.id_, cost));
                }
            }
        //        image.data[currentCell.id_.y*image.cols+currentCell.id_.x] = 0;
        //        cv::imshow("Image", image);
        //        cv::waitKey(1);
    }

    // Now, recover the path.
    // Path is valid!
    if (visited[start.y*gridMap.cols+start.x].first)
    {
        for (cv::Point2i backPointer = start; backPointer.x != goal.x || backPointer.y != goal.y;
             backPointer = visited[backPointer.y*gridMap.cols+backPointer.x].second)
            path.push_back(backPointer);
        return true;
    }
    // Path is invalid!
    else
        return false;
}

bool discretizePathInLineSegments(const cv::Mat& gridMap, const std::vector<cv::Point2i>& path,
                                  std::vector<std::pair<cv::Point2i, cv::Point2i > > &discretizedPath)
{
    if (path.size() < 2)
    {
        discretizedPath.push_back(std::make_pair(path[0], path[0]));
        return false;
    }
    discretizedPath.clear();
    cv::Point2i p1, p2;
    p1 = p2 = cv::Point2i(path[0].x, path[0].y);

    for (int i = 0; i < path.size(); i++)
    {
        bool obstacle_found = false;

        // Normal Case: Valid slope
        if (p2.x != p1.x)
        {
            // Find the equation of the line of the two points
            double slope = (p2.y - p1.y)/(p2.x - p1.x);
            double b = p2.y - slope*p2.x;

            // Get the ROI to check if obstacle is present within the line
            cv::Point2i min(std::min(p1.x, p2.x), std::min(p1.y, p2.y));
            cv::Point2i max(std::max(p1.x, p2.x), std::max(p1.y, p2.y));
            for (int x = min.x; x < max.x; x++)
                for (int y = min.y; y < max.y; y++)
                {
                    // If y = mx+b, then the x and y is in the line.
                    //
                    if (std::abs(y - (slope*x+b)) < 1 && gridMap.data[y*gridMap.cols+x] != gridmap_2d::GridMap2D::FREE)
                        obstacle_found = true;
                }
        }
        // Special Case: Infinite Slope
        else
        {
            int y_max = std::max(p1.y, p2.y);
            int y_min = std::min(p1.y, p2.y);
            for (int y = y_min; y < y_max; y++)
                if (gridMap.data[y*gridMap.cols+p1.x] != gridmap_2d::GridMap2D::FREE)
                    obstacle_found = true;
        }

        if (obstacle_found)
        {
            discretizedPath.push_back(std::make_pair(p1, p2));
            p1 = p2;
        }
        p2 = cv::Point2i(path[i].x, path[i].y);

        // If no obstacle found and we are left with the last element of the path;
        // automatically make this the final line segment!
        if (i == path.size()-1)
            discretizedPath.push_back(std::make_pair(p1, p2));
    }

    return true;
}

void filterPointsViaDijkstra(const cv::Mat& gridMap, const cv::Point2i &start, const std::vector<cv::Point2i>& frontierSet,
                             std::vector<cv::Point2i>& filteredFrontiers)
{
    filteredFrontiers.clear();
    // Create a visited set which contains a boolean value whether it is visited or not and a backpointer to the previous path.
    std::vector<std::pair<bool, cv::Point2i> > visited(gridMap.rows * gridMap.cols, std::make_pair(false, cv::Point2i(0,0)));
    // Priority queue was used which enables us to always get the lowest cost vertex without iteration.
    std::priority_queue <vertex, std::vector<vertex> > working;
    // Create a boolean map of frontierSet
    std::vector<bool>isFrontierVisited(frontierSet.size(), false);

    // Initialize working set. We are going to perform the djikstra's
    // backwards in order to get the actual path without reversing the path.
    working.push(vertex(start, start, 0));

    bool isDone = false;
    // Conditions in continuing
    // 1.) Working is empty implies all nodes are visited.
    // 2.) If the frontier sets are still not found in the working visited set.
    // The Dijkstra's algorithm
    while(!working.empty() && !isDone)
    {

        // Get the top of the STL.
        // It is already given that the top of the multimap has the lowest cost.
        vertex currentCell = working.top();
        working.pop();
        // If the currentCell is already included in the visited set then abort.
        if (visited[currentCell.id_.y*gridMap.cols+currentCell.id_.x].first)
            continue;

        // Mark it visited
        visited[currentCell.id_.y*gridMap.cols+currentCell.id_.x].first = true;
        visited[currentCell.id_.y*gridMap.cols+currentCell.id_.x].second = currentCell.from_;

        isDone = true;
        for (int i = 0; i < isFrontierVisited.size(); i++)
        {
            if (frontierSet[i].x == currentCell.id_.x && frontierSet[i].y == currentCell.id_.y)
                isFrontierVisited[i] = true;
            if (isFrontierVisited[i] == false)
                isDone = false;
        }
        // Check all arcs
        // Only insert the cells into working under these 3 conditions:
        // 1. The cell is not in visited cell
        // 2. The cell is not out of bounds
        // 3. The cell is free
        for (int x = currentCell.id_.x-1; x <= currentCell.id_.x+1; x++)
            for (int y = currentCell.id_.y-1; y <= currentCell.id_.y+1; y++)
            {
                // No one is neighbor with itself, right?
                if (x == currentCell.id_.x && y == currentCell.id_.y)
                    continue;

                if (checkIfNotOutOfBounds(cv::Point2i(x, y), gridMap.rows, gridMap.cols) &&
                        get_cell_at(gridMap, x, y) == gridmap_2d::GridMap2D::FREE)
                {
                    double cost = currentCell.cost_ + sqrt((x-currentCell.id_.x)*(x-currentCell.id_.x)
                                                           +(y-currentCell.id_.y)*(y-currentCell.id_.y));

                    /// Uncomment this to make it 4 children
                    //                    if (!(x == currentCell.id_.x || y == currentCell.id_.y))
                    //                        continue;
                    /// Ends here..
                    working.push(vertex(cv::Point2i(x,y), currentCell.id_, cost));
                }
            }
    }

    // Only store the reachable frontiers.
    for (int i = 0; i < isFrontierVisited.size(); i++)
        if (isFrontierVisited[i] == true)
            filteredFrontiers.push_back(frontierSet[i]);
}

}

void Dijkstra::findNearestFreeCell(const cv::Mat& gridMap, const cv::Point2i &start, cv::Point2i &end, int numberOfFreeCells)
{

    cv::Mat image = gridMap.clone();
    int n = 0;
    // Create a visited set which contains a boolean value whether it is visited or not and a backpointer to the previous path.
    std::vector<std::pair<bool, cv::Point2i> > visited(gridMap.rows * gridMap.cols, std::make_pair(false, cv::Point2i(0,0)));
    // Priority queue was used which enables us to always get the lowest cost vertex without iteration.
    std::priority_queue <vertex, std::vector<vertex> > working;

    // Initialize working set. We are going to perform the djikstra's
    // backwards in order to get the actual path without reversing the path.
    working.push(vertex(start, start, 0));

    // Conditions in continuing
    // 1.) Working is empty implies all nodes are visited.
    // 2.) If a free cell is found
    // The Dijkstra's algorithm
    while(!working.empty())
    {

        // Get the top of the STL.
        // It is already given that the top of the multimap has the lowest cost.
        vertex currentCell = working.top();
        working.pop();

        if (gridMap.data[currentCell.id_.y*gridMap.cols+currentCell.id_.x] == gridmap_2d::GridMap2D::FREE
                && n++ == numberOfFreeCells)
        {
            end = cv::Point2i(currentCell.id_.x, currentCell.id_.y);
            break;
        }
        // If the currentCell is already included in the visited set then abort.
        if (visited[currentCell.id_.y*gridMap.cols+currentCell.id_.x].first)
            continue;

        // Mark it visited
        visited[currentCell.id_.y*gridMap.cols+currentCell.id_.x].first = true;
        visited[currentCell.id_.y*gridMap.cols+currentCell.id_.x].second = currentCell.from_;

        // Check all arcs
        // Only insert the cells into working under these 3 conditions:
        // 1. The cell is not in visited cell
        // 2. The cell is not out of bounds
        // 3. The cell is free or unknown
        for (int x = currentCell.id_.x-1; x <= currentCell.id_.x+1; x++)
            for (int y = currentCell.id_.y-1; y <= currentCell.id_.y+1; y++)
            {
                // No one is neighbor with itself, right?
                if (x == currentCell.id_.x && y == currentCell.id_.y)
                    continue;

                if (checkIfNotOutOfBounds(cv::Point2i(x, y), gridMap.rows, gridMap.cols) &&
                        (get_cell_at(gridMap, x, y) == gridmap_2d::GridMap2D::UNKNOWN ||
                        get_cell_at(gridMap, x, y) == gridmap_2d::GridMap2D::FREE))
                {
                    double cost = currentCell.cost_ + sqrt((x-currentCell.id_.x)*(x-currentCell.id_.x)
                                                           +(y-currentCell.id_.y)*(y-currentCell.id_.y));
                    working.push(vertex(cv::Point2i(x,y), currentCell.id_, cost));
                }
            }
    }
}


void Dijkstra::computeCostVector(const cv::Mat& gridMap, const cv::Point2i& start, const std::vector<cv::Point2i> &frontierSet,
                       std::vector<int> &costVector)
{
    costVector = std::vector<int>(frontierSet.size(), -1);
    // Create a visited set which contains a boolean value whether it is visited or not and a backpointer to the previous path.
    std::vector<std::pair<bool, int> > visited(gridMap.rows * gridMap.cols, std::make_pair(false, 0));
    // Priority queue was used which enables us to always get the lowest cost vertex without iteration.
    std::priority_queue <vertex, std::vector<vertex> > working;
    // Create a boolean map of frontierSet
    std::vector<bool>isFrontierVisited(frontierSet.size(), false);

    // Initialize working set. We are going to perform the djikstra's
    // backwards in order to get the actual path without reversing the path.
    working.push(vertex(start, start, 0));

    bool isDone = false;
    // Conditions in continuing
    // 1.) Working is empty implies all nodes are visited.
    // 2.) If the frontier sets are still not found in the working visited set.
    // The Dijkstra's algorithm
    while(!working.empty() && !isDone)
    {

        // Get the top of the STL.
        // It is already given that the top of the multimap has the lowest cost.
        vertex currentCell = working.top();
        working.pop();
        // If the currentCell is already included in the visited set then abort.
        if (visited[currentCell.id_.y*gridMap.cols+currentCell.id_.x].first)
            continue;

        // Mark it visited
        visited[currentCell.id_.y*gridMap.cols+currentCell.id_.x].first = true;
        visited[currentCell.id_.y*gridMap.cols+currentCell.id_.x].second = round(currentCell.cost_);

        isDone = true;
        for (int i = 0; i < isFrontierVisited.size(); i++)
        {
            if (frontierSet[i].x == currentCell.id_.x && frontierSet[i].y == currentCell.id_.y)
                isFrontierVisited[i] = true;
            if (isFrontierVisited[i] == false)
                isDone = false;
        }
        // Check all arcs
        // Only insert the cells into working under these 3 conditions:
        // 1. The cell is not in visited cell
        // 2. The cell is not out of bounds
        // 3. The cell is free
        for (int x = currentCell.id_.x-1; x <= currentCell.id_.x+1; x++)
            for (int y = currentCell.id_.y-1; y <= currentCell.id_.y+1; y++)
            {
                // No one is neighbor with itself, right?
                if (x == currentCell.id_.x && y == currentCell.id_.y)
                    continue;

                if (checkIfNotOutOfBounds(cv::Point2i(x, y), gridMap.rows, gridMap.cols) &&
                        get_cell_at(gridMap, x, y) == gridmap_2d::GridMap2D::FREE)
                {
                    double cost = currentCell.cost_ + sqrt((x-currentCell.id_.x)*(x-currentCell.id_.x)
                                                           +(y-currentCell.id_.y)*(y-currentCell.id_.y));
                    working.push(vertex(cv::Point2i(x,y), currentCell.id_, cost));
                }
            }
    }

    // Only store the reachable frontiers.
    for (int i = 0; i < isFrontierVisited.size(); i++)
        if (isFrontierVisited[i] == true)
            costVector[i] = visited[frontierSet[i].y*gridMap.cols+frontierSet[i].x].second;

}
