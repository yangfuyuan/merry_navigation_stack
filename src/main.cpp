#include "mission_planner.h"
#include "global_planner.h"
#include "localplanner.h"
#include "mainwindow.h"
#include "querypose.h"
#include "serviceclient2.h"
#include "maplistener.h"
#include "qlogconsole.h"
#include <QtGui>
#include <QApplication>
#include <ros/ros.h>

/// Global Variable for QueryPose and MapListener
QueryPose queryPose(1); // 1 second timeout
MapListener mapListener;

int main(int argc, char **argv)
{

    /// ROS Node Initialization
    ros::init(argc, argv, "merry_navigation_stack");

    /// Initialize QApplication
    QApplication app(argc, argv);

    /// GUI Instantiation
    MainWindow w;
    QObject::connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

    /// Multi-RGBDSLAM Service Client to produce the 2D map
    ServiceClient2 *serviceClient2 = new ServiceClient2();
    QThread *thread0 = new QThread();
    serviceClient2->addServiceClient("");
    serviceClient2->setFreqOfSendAll(5);
    serviceClient2->moveToThread(thread0);
    QObject::connect(thread0, SIGNAL(started()), serviceClient2, SLOT(start()));
    QObject::connect(&w, SIGNAL(toggleGenerateMap()), serviceClient2, SLOT(togglePause()));
    QObject::connect(&w, SIGNAL(resetRgbdslamSystem()), serviceClient2, SLOT(resetRgbdslamSystem()));


    /// Mission Planner
    MissionPlanner *missionPlanner = new MissionPlanner();
    QThread *thread1 = new QThread();
    missionPlanner->moveToThread(thread1);
    QObject::connect(thread1, SIGNAL(started()), missionPlanner, SLOT(start()));
    QObject::connect(&missionPlanner->log_[0], SIGNAL(logConsoleToTextEdit(QString)),
                     &w, SLOT(writeToLogMgp0(QString)));
    QObject::connect(&missionPlanner->log_[0], SIGNAL(logConsoleToDisplayImage(QImage,QString,uint)),
                     &w, SLOT(writeToDisplayImage(QImage,QString,uint)));
    QObject::connect(&missionPlanner->log_[1], SIGNAL(logConsoleToTextEdit(QString)),
                     &w, SLOT(writeToLogMgp1(QString)));
    QObject::connect(&missionPlanner->log_[1], SIGNAL(logConsoleToDisplayImage(QImage,QString,uint)),
                     &w, SLOT(writeToDisplayImage(QImage,QString,uint)));

    /// Global Planner
    GlobalPlanner *globalPlanner0 = new GlobalPlanner(0);
    GlobalPlanner *globalPlanner1 = new GlobalPlanner(1);
    QThread *thread2 = new QThread();
    QThread *thread3 = new QThread();

    globalPlanner0->moveToThread(thread2);
    globalPlanner1->moveToThread(thread3);
    QObject::connect(thread2, SIGNAL(started()), globalPlanner0, SLOT(start()));
    QObject::connect(thread3, SIGNAL(started()), globalPlanner1, SLOT(start()));
    QObject::connect(&w, SIGNAL(reqFrontier0()), globalPlanner0, SLOT(on_reqBtn_clicked()));
    QObject::connect(&w, SIGNAL(reqFrontier1()), globalPlanner1, SLOT(on_reqBtn_clicked()));
    QObject::connect(&w, SIGNAL(toggleAutonomous()), globalPlanner0, SLOT(toggleAutonomous()));
    QObject::connect(&w, SIGNAL(toggleAutonomous()), globalPlanner1, SLOT(toggleAutonomous()));
    QObject::connect(&globalPlanner0->log_, SIGNAL(logConsoleToTextEdit(QString)),
                     &w, SLOT(writeToLogMgp0(QString)));
    QObject::connect(&globalPlanner0->log_, SIGNAL(logConsoleToDisplayImage(QImage,QString,uint)),
                     &w, SLOT(writeToDisplayImage(QImage,QString,uint)));
    QObject::connect(&globalPlanner1->log_, SIGNAL(logConsoleToTextEdit(QString)),
                     &w, SLOT(writeToLogMgp1(QString)));
    QObject::connect(&globalPlanner1->log_, SIGNAL(logConsoleToDisplayImage(QImage,QString,uint)),
                     &w, SLOT(writeToDisplayImage(QImage,QString,uint)));

    /// Local Planner
    LocalPlanner *localPlanner0 = new LocalPlanner("quad1", "camera0");
    LocalPlanner *localPlanner1 = new LocalPlanner("quad2", "camera1");
    QThread *thread4 = new QThread();
    QThread *thread5 = new QThread();
    localPlanner0->moveToThread(thread4);
    localPlanner1->moveToThread(thread5);
    QObject::connect(thread4, SIGNAL(started()), localPlanner0, SLOT(start()));
    QObject::connect(thread5, SIGNAL(started()), localPlanner1, SLOT(start()));
    QObject::connect(&localPlanner0->log_, SIGNAL(logConsoleToTextEdit(QString)),
                     &w, SLOT(writeToLogLp0(QString)));
    QObject::connect(&localPlanner1->log_, SIGNAL(logConsoleToTextEdit(QString)),
                     &w, SLOT(writeToLogLp1(QString)));

    /// Query Pose
    QThread *thread6 = new QThread();
    queryPose.moveToThread(thread6);
    QObject::connect(thread6, SIGNAL(started()), &queryPose, SLOT(start()));
    /// Map Listener
    mapListener.moveToThread(thread6);
    QObject::connect(thread6, SIGNAL(started()), &mapListener, SLOT(start()));

    thread0->start();
    thread1->start();
    thread2->start();
    thread3->start();
    thread4->start();
    thread5->start();
    thread6->start();

    w.show();
    int result = app.exec();
    return result;
}
