#include "serviceclient2.h"

ServiceClient2::ServiceClient2(): isRunning_(false),
    numOfServiceClient_(0), freqOfSendAll_(5)
{
}

void ServiceClient2::addServiceClient(const char* rgbdslamSrvNs)
{
    std::string rgbdslamSrvString;
    rgbdslamSrvString = std::string(rgbdslamSrvNs) + "/rgbdslam";
    clientRgbdslam_.push_back(n_.serviceClient<multi_rgbdslam::rgbdslam_ros_ui>(rgbdslamSrvString.c_str() +
                                                                                std::string("/ros_ui")));
    rgbdslamCaptureClient_ = n_.serviceClient<multi_rgbdslam::rgbdslam_ros_ui_b>(rgbdslamSrvString.c_str() +
                                                                                 std::string("/ros_ui_b"));
    numOfServiceClient_++;
}

void ServiceClient2::setFreqOfSendAll(uint freqOfSendAll)
{
    freqOfSendAll_ = freqOfSendAll;
}

void ServiceClient2::applyFrequency()
{
    // Freq = Hz
    // Multiplexed Freq = numOfServiceClient2*Freq;
    // A = 1/Hz = Seconds where A < 1
    // A * 1000 = ms
    timer_send_all_->stop();
    timer_send_all_->start((1./(freqOfSendAll_ * numOfServiceClient_)) * 1000);
}

void ServiceClient2::start()
{
    timer_send_all_ = new QTimer();
    connect(timer_send_all_, SIGNAL(timeout()), this, SLOT(run_send_all()));
    applyFrequency();
}

void ServiceClient2::togglePause()
{
//    rgbdslamCaptureFrame_.request.command = "pause";
//    rgbdslamCaptureFrame_.request.value = isRunning_;
//    if(rgbdslamCaptureClient_.call(rgbdslamCaptureFrame_))
//        std::cerr << "ServiceClient2: Toggle pause" << std::endl;
//    else
//        std::cerr << "ServiceClient2: Toggle pause failed" << std::endl;

    isRunning_ = !isRunning_;
}

void ServiceClient2::run_send_all()
{
    static int i = 0;
    if (isRunning_)
    {
        rgbdslamSrv_.request.command = "send_all";
        clientRgbdslam_[i].call(rgbdslamSrv_);
        i = (i+1)%numOfServiceClient_;
    }
}

void ServiceClient2::resetRgbdslamSystem()
{
    for (uint i = 0; i < numOfServiceClient_; i++)
    {
        rgbdslamSrv_.request.command = "reset";
        clientRgbdslam_[i].call(rgbdslamSrv_);
    }
}
