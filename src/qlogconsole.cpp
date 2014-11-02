#include "qlogconsole.h"
#include <iostream>
QLogConsole::QLogConsole(QObject *parent) :
    QObject(parent)
{
}

QLogConsole& QLogConsole::operator <<(const std::string &rhs)
{
    Q_EMIT logConsoleToTextEdit(QString::fromStdString(rhs));
    return *this;
}

QLogConsole& QLogConsole::operator <<(const int &rhs)
{
    Q_EMIT logConsoleToTextEdit(QString::number(rhs));
    return *this;
}

QLogConsole &QLogConsole::operator <<(const size_t &rhs)
{
    Q_EMIT logConsoleToTextEdit(QString::number(rhs));
    return *this;
}

QLogConsole &QLogConsole::operator <<(const int64_t &rhs)
{
    Q_EMIT logConsoleToTextEdit(QString::number(rhs));
    return *this;
}

QLogConsole& QLogConsole::operator <<(const double &rhs)
{
    Q_EMIT logConsoleToTextEdit(QString::number(rhs));
    return *this;
}

QLogConsole& QLogConsole::operator <<(const float &rhs)
{
    Q_EMIT logConsoleToTextEdit(QString::number(rhs));
    return *this;
}

QLogConsole &QLogConsole::operator <<(const DisplayImage &rhs)
{
    buffer[rhs.channel_] = rhs.image_.clone();

    if (rhs.image_.channels() == 1)
        cv::cvtColor(buffer[rhs.channel_], buffer[rhs.channel_], CV_GRAY2RGBA);

    QImage qimage((uint8_t*) buffer[rhs.channel_].data, buffer[rhs.channel_].cols, buffer[rhs.channel_].rows, QImage::Format_ARGB32);
    Q_EMIT logConsoleToDisplayImage(qimage, rhs.name_, rhs.channel_);

    return *this;
}

