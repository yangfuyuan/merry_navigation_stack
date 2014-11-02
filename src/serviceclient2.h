/* File: serviceclient.h/.cpp
 * Description: The serviceclient creates N ros::ServiceClient that subscribes to the services
 * specified in the arguments in the constructor. The ServiceClient calls the services from two
 * nodes:
 *   RGBDSLAM Node (e.g. /rgbdslam/ros_ui)
 *   1. frame
 *   2. send_all
 *   3. reset
 *   Octomap Server (e.g. /octomap_server/reset)
 *   1. <No service name>
 * The frame service call captures a frame from the camera stream input while
 * the send_all service call sends the current available point clouds to
 * the octomap_server node. Resetting the system involves resetting both the RGBDSLAM
 * node and the Octomap Server. Both rgbdslam will be called to service at approximately
 * the same time (enqueued in QEventLoop).
 * Depending on the frequency specified by the user, the calling of the same services between
 * different service clients are time multiplexed (e.g. frame@10Hz with 2 clients would mean
 * each client calls 10 Hz alternatively with each other's call).
 */


/* Changelog v1.1 - 9/22/14
 * 1. Removed the octomap server reset service since the octomap is reset
      in the new rgbdslam itself.
 */

#ifndef SERVICECLIENT2_H
#define SERVICECLIENT2_H

#include "ros/ros.h"
#include <multi_rgbdslam/rgbdslam_ros_ui.h>
#include <multi_rgbdslam/rgbdslam_ros_ui_b.h>
#include <QObject>
#include <QTimer>
#include <string>
#include <std_srvs/Empty.h>

class ServiceClient2: public QObject
{
    Q_OBJECT
public:

    // Constructor which needs a valid ros::NodeHandle to work.
    ServiceClient2();
    // Adds a serviceClient which accepts the namespace of the Rgbdslam
    // and Octomap_server services
    // Example...
    // @param rgbdslamSrvNs: /rgbdslam/ before /ros_ui
    void addServiceClient(const char* rgbdslamSrvNs);
    // Set desired operating frequency for the frame and send_all.
    void setFreqOfSendAll(uint freqOfSendAll);
    // Apply new frequency. Call this only after start.
    void applyFrequency();
private:
    // Used in calling a service.
    ros::NodeHandle n_;
    // Used to play/pause the process.
    bool isRunning_;
    // The service clients for Rgbdslam and Octomap
    std::vector<ros::ServiceClient> clientRgbdslam_;
    ros::ServiceClient rgbdslamCaptureClient_;
    // Message for the service.
    multi_rgbdslam::rgbdslam_ros_ui rgbdslamSrv_;
    multi_rgbdslam::rgbdslam_ros_ui_b rgbdslamCaptureFrame_;
    // Since two services will be called regularly - frame and send_all,
    // we would need two timers.
    QTimer *timer_send_all_;
    // Holds the number of object instantiated.
    uint numOfServiceClient_;
    // The desired frequency rate of the calling of service of send_all.
    uint freqOfSendAll_;

public Q_SLOTS:
    // Allocates the timer and starts it.
    void start();
    // Start/pause the rgbdslam process
    void togglePause();
    // Reset all rgbdslam nodes and octomap_server nodes.
    void resetRgbdslamSystem();

private Q_SLOTS:
    // Execute this code every timeout for capturing frame.
    void run_send_all();
};

#endif // RGBDSLAMSERVICER_H
