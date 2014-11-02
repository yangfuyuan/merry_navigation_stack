#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <iostream>
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::writeToLogLp0(QString text)
{
    QStringList list = text.split("\n");
    for (int i = 0; i < list.size()-1; i++)
    {
        ui->log_lp0->moveCursor(QTextCursor::End);
        ui->log_lp0->insertHtml(list.at(i));
        ui->log_lp0->append("");
    }
    ui->log_lp0->moveCursor(QTextCursor::End);
    ui->log_lp0->insertHtml(list.at(list.size()-1));
    ui->log_lp0->verticalScrollBar()->setValue(ui->log_lp0->verticalScrollBar()->maximum());
}

void MainWindow::writeToLogLp1(QString text)
{
    QStringList list = text.split("\n");
    for (int i = 0; i < list.size()-1; i++)
    {
        ui->log_lp1->moveCursor(QTextCursor::End);
        ui->log_lp1->insertHtml(list.at(i));
        ui->log_lp1->append("");
    }
    ui->log_lp1->moveCursor(QTextCursor::End);
    ui->log_lp1->insertHtml(list.at(list.size()-1));
    ui->log_lp1->verticalScrollBar()->setValue(ui->log_lp1->verticalScrollBar()->maximum());
}

void MainWindow::writeToLogMgp0(QString text)
{
    QStringList list = text.split("\n");
    for (int i = 0; i < list.size()-1; i++)
    {
        ui->log_mgp0->moveCursor(QTextCursor::End);
        ui->log_mgp0->insertHtml(list.at(i));
        ui->log_mgp0->append("");
    }
    ui->log_mgp0->moveCursor(QTextCursor::End);
    ui->log_mgp0->insertHtml(list.at(list.size()-1));
    ui->log_mgp0->verticalScrollBar()->setValue(ui->log_mgp0->verticalScrollBar()->maximum());
}

void MainWindow::writeToLogMgp1(QString text)
{
    QStringList list = text.split("\n");
    for (int i = 0; i < list.size()-1; i++)
    {
        ui->log_mgp1->moveCursor(QTextCursor::End);
        ui->log_mgp1->insertHtml(list.at(i));
        ui->log_mgp1->append("");
    }
    ui->log_mgp1->moveCursor(QTextCursor::End);
    ui->log_mgp1->insertHtml(list.at(list.size()-1));
    ui->log_mgp1->verticalScrollBar()->setValue(ui->log_mgp1->verticalScrollBar()->maximum());
}

void MainWindow::writeToDisplayImage(QImage image, QString name, uint channel)
{
    QImage scaled;
    switch(channel)
    {
    case 0:
        ui->extra00_lbl->setText("<b><i>" + name + "</b></i>");
        scaled = image.scaledToHeight(ui->extra00->height());
        ui->extra00->setPixmap(QPixmap::fromImage(scaled, Qt::AutoColor));
        break;
    case 1:
        ui->extra01_lbl->setText("<b><i>" + name + "</b></i>");
        scaled = image.scaledToHeight(ui->extra01->height());
        ui->extra01->setPixmap(QPixmap::fromImage(scaled, Qt::AutoColor));
        break;
    case 2:
        ui->extra02_lbl->setText("<b><i>" + name + "</b></i>");
        scaled = image.scaledToHeight(ui->extra02->height());
        ui->extra02->setPixmap(QPixmap::fromImage(scaled, Qt::AutoColor));
        break;
    case 3:
        ui->extra03_lbl->setText("<b><i>" + name + "</b></i>");
        scaled = image.scaledToHeight(ui->extra03->height());
        ui->extra03->setPixmap(QPixmap::fromImage(scaled, Qt::AutoColor));
        break;
    case 4:
        ui->extra10_lbl->setText("<b><i>" + name + "</b></i>");
        scaled = image.scaledToHeight(ui->extra10->height());
        ui->extra10->setPixmap(QPixmap::fromImage(scaled, Qt::AutoColor));
        break;
    case 5:
        ui->extra11_lbl->setText("<b><i>" + name + "</b></i>");
        scaled = image.scaledToHeight(ui->extra11->height());
        ui->extra11->setPixmap(QPixmap::fromImage(scaled, Qt::AutoColor));
        break;
    case 6:
        ui->extra12_lbl->setText("<b><i>" + name + "</b></i>");
        scaled = image.scaledToHeight(ui->extra12->height());
        ui->extra12->setPixmap(QPixmap::fromImage(scaled, Qt::AutoColor));
        break;
    case 7:
        ui->extra13_lbl->setText("<b><i>" + name + "</b></i>");
        scaled = image.scaledToHeight(ui->extra13->height());
        ui->extra13->setPixmap(QPixmap::fromImage(scaled, Qt::AutoColor));
        break;
    default:
        break;
    }
}

void MainWindow::on_generateMapBtn_clicked()
{
    static bool isGenerateMapEnabled = true;
    isGenerateMapEnabled = !isGenerateMapEnabled;
    if (isGenerateMapEnabled)
        ui->generateMapBtn->setText("Generate 2D Map");
    else
        ui->generateMapBtn->setText("Stop Generating");
    Q_EMIT toggleGenerateMap();
}

void MainWindow::on_resetBtn_clicked()
{
    Q_EMIT resetRgbdslamSystem();
}

void MainWindow::on_req0Btn_clicked()
{
    Q_EMIT reqFrontier0();
}

void MainWindow::on_req1Btn_clicked()
{
    Q_EMIT reqFrontier1();
}

void MainWindow::on_autonomousBtn_clicked()
{
    static bool isAutonomousEnabled = true;
    isAutonomousEnabled = !isAutonomousEnabled;
    if (isAutonomousEnabled)
        ui->autonomousBtn->setText("Start Autonomous");
    else
        ui->autonomousBtn->setText("Stop Autonomous");
    Q_EMIT toggleAutonomous();
}
