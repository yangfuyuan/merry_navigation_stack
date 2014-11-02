#ifndef QLOGCONSOLE_H
#define QLOGCONSOLE_H

#include <QObject>
#include <opencv2/opencv.hpp>
#include <QImage>
struct DisplayImage{
    cv::Mat image_;
    uint channel_;
    QString name_;

    DisplayImage(const cv::Mat &image, uint channel, QString name)
    {
        image_ = image;
        channel_ = channel;
        name_ = name;
    }
};


class QLogConsole : public QObject
{
    Q_OBJECT
public:
    explicit QLogConsole(QObject *parent = 0);
    
    /// Overloaded insertion operator for easier logging.
    QLogConsole& operator<< (const std::string& rhs);
    QLogConsole& operator<< (const int& rhs);
    QLogConsole& operator<< (const size_t& rhs);
    QLogConsole& operator<< (const int64_t& rhs);
    QLogConsole& operator<< (const double& rhs);
    QLogConsole& operator<< (const float& rhs);
    QLogConsole& operator<< (const DisplayImage& rhs);

private:
    cv::Mat buffer[8];

Q_SIGNALS:
    void logConsoleToTextEdit(QString);
    // Display image mapping.
    void logConsoleToDisplayImage(QImage, QString, uint);

public Q_SLOTS:
    
};

#endif // QLOGCONSOLE_H
