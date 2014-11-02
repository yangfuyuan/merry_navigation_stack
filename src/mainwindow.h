#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QScrollBar>
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;

public Q_SLOTS:
    void writeToLogLp0(QString);
    void writeToLogLp1(QString);
    void writeToLogMgp0(QString);
    void writeToLogMgp1(QString);
    void writeToDisplayImage(QImage, QString, uint);

private Q_SLOTS:
    void on_generateMapBtn_clicked();
    void on_resetBtn_clicked();

    void on_req0Btn_clicked();

    void on_req1Btn_clicked();

    void on_autonomousBtn_clicked();

Q_SIGNALS:
    void toggleGenerateMap();
    void resetRgbdslamSystem();
    void reqFrontier0();
    void reqFrontier1();
    void toggleAutonomous();

};

#endif // MAINWINDOW_H
