#ifndef QUERYPOSE_H
#define QUERYPOSE_H

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <QObject>
#include <QTimer>

class QueryPose : public QObject
{
    Q_OBJECT
public:
    /// Sets the time out in determining the validity of the pose.
    QueryPose(double timeout = 1, QObject *parent = 0);

    /// Param poseNumber: either 0 or 1
    /// Returns pose: The pose queried.
    /// Returns timeDelayed: The time elapsed from the recent pose to the current time.
    /// Returns true if the pose is valid within a timeout.
    bool getPose(uint poseNumber, tf::StampedTransform& pose, double &timeDelayed);

private:
    tf::StampedTransform pose_[2];
    QTimer *timer_;
    float timeout_;

Q_SIGNALS:
    
public Q_SLOTS:
    void start();

private Q_SLOTS:
    void run();
};

extern QueryPose queryPose;
#endif // QUERYPOSE_H
