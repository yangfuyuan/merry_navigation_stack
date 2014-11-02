#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <opencv2/opencv.hpp>
#include <algorithm>
#include <queue>
#include <vector>
#include <math.h>
#include "gridmap2d.h"

namespace Dijkstra
{
/// STRUCTURES for easier management.
struct vertex {
    double cost_;
    cv::Point2i id_;
    cv::Point2i from_;

    vertex(cv::Point2i id = cv::Point2i(0,0), cv::Point2i from = cv::Point2i(0,0), double cost = 0)
    {
        cost_ = cost;
        id_ = id;
        from_ = from;
    }

    vertex& operator= (const vertex& rhs)
    {
        this->cost_ = rhs.cost_;
        this->from_ = rhs.from_;
        this->id_ = rhs.id_;
        return *this;
    }
};

/// Some helper functions for dijkstra's algorithm.
uint8_t get_cell_at(const cv::Mat & image, int x, int y);

/// Some helper functions for dijkstra's algorithm.
bool checkIfNotOutOfBounds(cv::Point2i current, int rows, int cols);

bool operator<(const vertex& a, const vertex& b);

/// Brief: Finds the shortest possible path from starting position to the goal position
/// Param gridMap: The stage where the tracing of the shortest possible path will be performed.
/// Param start: The starting position in the gridMap. It is assumed that start cell is a free cell.
/// Param goal: The goal position in the gridMap. It is assumed that the goal cell is a free cell.
/// Param path: Returns the sequence of free cells leading to the goal starting from the starting cell.
/// Returns: true if path is found.
bool findPathViaDijkstra(const cv::Mat& gridMap, cv::Point2i start, cv::Point2i goal, std::vector<cv::Point2i>& path);

/// Brief: Subdivides the path into line segments
/// Param gridMap: The same gridMap used in Dijkstra's algorithm which will be used as the criterion of in segmenting lines.
/// Param path: The path derived from the Dijkstra's.
/// Param discretizedPath: Contains pairs of start and end points of line segments that completes the path.
/// Returns: true if path is discretized.
bool discretizePathInLineSegments(const cv::Mat& gridMap, const std::vector<cv::Point2i>& path,
                                   std::vector<std::pair<cv::Point2i, cv::Point2i > > &discretizedPath);

/// Brief: Removes all frontier points that are unreachable.
/// Param gridMap: The stage where the tracing of the shortest possible path will be performed.
/// Param start: The starting position in the gridMap. It is assumed that start cell is a free cell.
/// Param frontierSet: Contains the unfiltered frontier set.
/// Returns filteredFrontiers: Contains the filtered frontier set.
void filterPointsViaDijkstra(const cv::Mat& gridMap, const cv::Point2i &start, const std::vector<cv::Point2i> &frontierSet,
                             std::vector<cv::Point2i> &filteredFrontiers);

/// Brief: Finds the nearest free cell from the current position.
/// Param gridMap: The stage where the tracing of the shortest possible path will be performed.
/// Param start: The starting position in the gridMap. It is assumed that start cell is a free cell.
/// Returns end: The nearest free cell coordinate
/// Param numberOfFreeCells: the end point will only be assigned to the numberOfFreeCells detected iteration.
void findNearestFreeCell(const cv::Mat& gridMap, const cv::Point2i& start, cv::Point2i &end, int numberOfFreeCells = 1);

/// Brief: Computes the cost vector from the starting position towards all frontiers.
/// Param gridMap: The stage where the tracing of the shortest possible path will be performed.
/// Param start: The starting position in the gridMap. It is assumed that start cell is a free cell.
/// Param frontierSet: Contains all the frontiers in the gridMap.
/// Returns costVector: Each element contains the rounded-to-nearest cost of each frontier from the starting position;
/// returns -1 if there is no path valid.
void computeCostVector(const cv::Mat& gridMap, const cv::Point2i& start, const std::vector<cv::Point2i> &frontierSet,
                       std::vector<int> &costVector);
}
#endif
