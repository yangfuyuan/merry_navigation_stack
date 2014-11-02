/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Sun Nov 2 23:52:08 2014
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QStatusBar>
#include <QtGui/QTabWidget>
#include <QtGui/QTextEdit>
#include <QtGui/QToolBar>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QHBoxLayout *horizontalLayout_3;
    QTabWidget *tabWidget;
    QWidget *tab;
    QVBoxLayout *verticalLayout_9;
    QVBoxLayout *verticalLayout_5;
    QHBoxLayout *horizontalLayout_2;
    QVBoxLayout *verticalLayout_2;
    QLabel *label;
    QTextEdit *log_mgp0;
    QVBoxLayout *verticalLayout;
    QLabel *label_2;
    QTextEdit *log_mgp1;
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *verticalLayout_3;
    QLabel *label_3;
    QTextEdit *log_lp0;
    QVBoxLayout *verticalLayout_4;
    QLabel *label_4;
    QTextEdit *log_lp1;
    QWidget *tab_2;
    QHBoxLayout *horizontalLayout_6;
    QVBoxLayout *verticalLayout_14;
    QHBoxLayout *horizontalLayout_5;
    QVBoxLayout *verticalLayout_10;
    QLabel *extra00_lbl;
    QLabel *extra00;
    QVBoxLayout *verticalLayout_11;
    QLabel *extra01_lbl;
    QLabel *extra01;
    QHBoxLayout *horizontalLayout_4;
    QVBoxLayout *verticalLayout_13;
    QLabel *extra02_lbl;
    QLabel *extra02;
    QVBoxLayout *verticalLayout_12;
    QLabel *extra03_lbl;
    QLabel *extra03;
    QWidget *tab_3;
    QHBoxLayout *horizontalLayout_10;
    QVBoxLayout *verticalLayout_21;
    QHBoxLayout *horizontalLayout_9;
    QVBoxLayout *verticalLayout_19;
    QLabel *extra10_lbl;
    QLabel *extra10;
    QVBoxLayout *verticalLayout_20;
    QLabel *extra11_lbl;
    QLabel *extra11;
    QHBoxLayout *horizontalLayout_8;
    QVBoxLayout *verticalLayout_17;
    QLabel *extra12_lbl;
    QLabel *extra12;
    QVBoxLayout *verticalLayout_18;
    QLabel *extra13_lbl;
    QLabel *extra13;
    QVBoxLayout *verticalLayout_8;
    QVBoxLayout *verticalLayout_7;
    QLabel *label_5;
    QSpacerItem *verticalSpacer;
    QPushButton *generateMapBtn;
    QPushButton *resetBtn;
    QVBoxLayout *verticalLayout_6;
    QLabel *label_6;
    QSpacerItem *verticalSpacer_2;
    QPushButton *req0Btn;
    QPushButton *req1Btn;
    QPushButton *autonomousBtn;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(710, 537);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        horizontalLayout_3 = new QHBoxLayout(centralWidget);
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        tabWidget = new QTabWidget(centralWidget);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        verticalLayout_9 = new QVBoxLayout(tab);
        verticalLayout_9->setSpacing(6);
        verticalLayout_9->setContentsMargins(11, 11, 11, 11);
        verticalLayout_9->setObjectName(QString::fromUtf8("verticalLayout_9"));
        verticalLayout_5 = new QVBoxLayout();
        verticalLayout_5->setSpacing(6);
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        label = new QLabel(tab);
        label->setObjectName(QString::fromUtf8("label"));

        verticalLayout_2->addWidget(label);

        log_mgp0 = new QTextEdit(tab);
        log_mgp0->setObjectName(QString::fromUtf8("log_mgp0"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(log_mgp0->sizePolicy().hasHeightForWidth());
        log_mgp0->setSizePolicy(sizePolicy);
        log_mgp0->setReadOnly(true);

        verticalLayout_2->addWidget(log_mgp0);


        horizontalLayout_2->addLayout(verticalLayout_2);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        label_2 = new QLabel(tab);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        verticalLayout->addWidget(label_2);

        log_mgp1 = new QTextEdit(tab);
        log_mgp1->setObjectName(QString::fromUtf8("log_mgp1"));
        log_mgp1->setReadOnly(true);

        verticalLayout->addWidget(log_mgp1);


        horizontalLayout_2->addLayout(verticalLayout);


        verticalLayout_5->addLayout(horizontalLayout_2);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        label_3 = new QLabel(tab);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        verticalLayout_3->addWidget(label_3);

        log_lp0 = new QTextEdit(tab);
        log_lp0->setObjectName(QString::fromUtf8("log_lp0"));
        log_lp0->setReadOnly(true);

        verticalLayout_3->addWidget(log_lp0);


        horizontalLayout->addLayout(verticalLayout_3);

        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        label_4 = new QLabel(tab);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        verticalLayout_4->addWidget(label_4);

        log_lp1 = new QTextEdit(tab);
        log_lp1->setObjectName(QString::fromUtf8("log_lp1"));
        log_lp1->setReadOnly(true);

        verticalLayout_4->addWidget(log_lp1);


        horizontalLayout->addLayout(verticalLayout_4);


        verticalLayout_5->addLayout(horizontalLayout);


        verticalLayout_9->addLayout(verticalLayout_5);

        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        horizontalLayout_6 = new QHBoxLayout(tab_2);
        horizontalLayout_6->setSpacing(6);
        horizontalLayout_6->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        verticalLayout_14 = new QVBoxLayout();
        verticalLayout_14->setSpacing(6);
        verticalLayout_14->setObjectName(QString::fromUtf8("verticalLayout_14"));
        verticalLayout_14->setSizeConstraint(QLayout::SetMaximumSize);
        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        verticalLayout_10 = new QVBoxLayout();
        verticalLayout_10->setSpacing(6);
        verticalLayout_10->setObjectName(QString::fromUtf8("verticalLayout_10"));
        extra00_lbl = new QLabel(tab_2);
        extra00_lbl->setObjectName(QString::fromUtf8("extra00_lbl"));
        QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(extra00_lbl->sizePolicy().hasHeightForWidth());
        extra00_lbl->setSizePolicy(sizePolicy1);

        verticalLayout_10->addWidget(extra00_lbl);

        extra00 = new QLabel(tab_2);
        extra00->setObjectName(QString::fromUtf8("extra00"));
        QSizePolicy sizePolicy2(QSizePolicy::Ignored, QSizePolicy::Ignored);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(extra00->sizePolicy().hasHeightForWidth());
        extra00->setSizePolicy(sizePolicy2);
        extra00->setMinimumSize(QSize(0, 0));
        extra00->setFrameShape(QFrame::StyledPanel);

        verticalLayout_10->addWidget(extra00);


        horizontalLayout_5->addLayout(verticalLayout_10);

        verticalLayout_11 = new QVBoxLayout();
        verticalLayout_11->setSpacing(6);
        verticalLayout_11->setObjectName(QString::fromUtf8("verticalLayout_11"));
        extra01_lbl = new QLabel(tab_2);
        extra01_lbl->setObjectName(QString::fromUtf8("extra01_lbl"));
        sizePolicy1.setHeightForWidth(extra01_lbl->sizePolicy().hasHeightForWidth());
        extra01_lbl->setSizePolicy(sizePolicy1);

        verticalLayout_11->addWidget(extra01_lbl);

        extra01 = new QLabel(tab_2);
        extra01->setObjectName(QString::fromUtf8("extra01"));
        sizePolicy2.setHeightForWidth(extra01->sizePolicy().hasHeightForWidth());
        extra01->setSizePolicy(sizePolicy2);
        extra01->setMinimumSize(QSize(0, 0));
        extra01->setFrameShape(QFrame::StyledPanel);

        verticalLayout_11->addWidget(extra01);


        horizontalLayout_5->addLayout(verticalLayout_11);


        verticalLayout_14->addLayout(horizontalLayout_5);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        verticalLayout_13 = new QVBoxLayout();
        verticalLayout_13->setSpacing(6);
        verticalLayout_13->setObjectName(QString::fromUtf8("verticalLayout_13"));
        extra02_lbl = new QLabel(tab_2);
        extra02_lbl->setObjectName(QString::fromUtf8("extra02_lbl"));
        sizePolicy1.setHeightForWidth(extra02_lbl->sizePolicy().hasHeightForWidth());
        extra02_lbl->setSizePolicy(sizePolicy1);

        verticalLayout_13->addWidget(extra02_lbl);

        extra02 = new QLabel(tab_2);
        extra02->setObjectName(QString::fromUtf8("extra02"));
        sizePolicy2.setHeightForWidth(extra02->sizePolicy().hasHeightForWidth());
        extra02->setSizePolicy(sizePolicy2);
        extra02->setMinimumSize(QSize(0, 0));
        extra02->setFrameShape(QFrame::StyledPanel);

        verticalLayout_13->addWidget(extra02);


        horizontalLayout_4->addLayout(verticalLayout_13);

        verticalLayout_12 = new QVBoxLayout();
        verticalLayout_12->setSpacing(6);
        verticalLayout_12->setObjectName(QString::fromUtf8("verticalLayout_12"));
        extra03_lbl = new QLabel(tab_2);
        extra03_lbl->setObjectName(QString::fromUtf8("extra03_lbl"));
        sizePolicy1.setHeightForWidth(extra03_lbl->sizePolicy().hasHeightForWidth());
        extra03_lbl->setSizePolicy(sizePolicy1);

        verticalLayout_12->addWidget(extra03_lbl);

        extra03 = new QLabel(tab_2);
        extra03->setObjectName(QString::fromUtf8("extra03"));
        sizePolicy2.setHeightForWidth(extra03->sizePolicy().hasHeightForWidth());
        extra03->setSizePolicy(sizePolicy2);
        extra03->setMinimumSize(QSize(0, 0));
        extra03->setFrameShape(QFrame::StyledPanel);

        verticalLayout_12->addWidget(extra03);


        horizontalLayout_4->addLayout(verticalLayout_12);


        verticalLayout_14->addLayout(horizontalLayout_4);


        horizontalLayout_6->addLayout(verticalLayout_14);

        tabWidget->addTab(tab_2, QString());
        tab_3 = new QWidget();
        tab_3->setObjectName(QString::fromUtf8("tab_3"));
        horizontalLayout_10 = new QHBoxLayout(tab_3);
        horizontalLayout_10->setSpacing(6);
        horizontalLayout_10->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_10->setObjectName(QString::fromUtf8("horizontalLayout_10"));
        verticalLayout_21 = new QVBoxLayout();
        verticalLayout_21->setSpacing(6);
        verticalLayout_21->setObjectName(QString::fromUtf8("verticalLayout_21"));
        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setSpacing(6);
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        verticalLayout_19 = new QVBoxLayout();
        verticalLayout_19->setSpacing(6);
        verticalLayout_19->setObjectName(QString::fromUtf8("verticalLayout_19"));
        extra10_lbl = new QLabel(tab_3);
        extra10_lbl->setObjectName(QString::fromUtf8("extra10_lbl"));
        sizePolicy1.setHeightForWidth(extra10_lbl->sizePolicy().hasHeightForWidth());
        extra10_lbl->setSizePolicy(sizePolicy1);

        verticalLayout_19->addWidget(extra10_lbl);

        extra10 = new QLabel(tab_3);
        extra10->setObjectName(QString::fromUtf8("extra10"));
        sizePolicy2.setHeightForWidth(extra10->sizePolicy().hasHeightForWidth());
        extra10->setSizePolicy(sizePolicy2);
        extra10->setMinimumSize(QSize(0, 0));
        extra10->setFrameShape(QFrame::StyledPanel);

        verticalLayout_19->addWidget(extra10);


        horizontalLayout_9->addLayout(verticalLayout_19);

        verticalLayout_20 = new QVBoxLayout();
        verticalLayout_20->setSpacing(6);
        verticalLayout_20->setObjectName(QString::fromUtf8("verticalLayout_20"));
        extra11_lbl = new QLabel(tab_3);
        extra11_lbl->setObjectName(QString::fromUtf8("extra11_lbl"));
        sizePolicy1.setHeightForWidth(extra11_lbl->sizePolicy().hasHeightForWidth());
        extra11_lbl->setSizePolicy(sizePolicy1);

        verticalLayout_20->addWidget(extra11_lbl);

        extra11 = new QLabel(tab_3);
        extra11->setObjectName(QString::fromUtf8("extra11"));
        sizePolicy2.setHeightForWidth(extra11->sizePolicy().hasHeightForWidth());
        extra11->setSizePolicy(sizePolicy2);
        extra11->setMinimumSize(QSize(0, 0));
        extra11->setFrameShape(QFrame::StyledPanel);

        verticalLayout_20->addWidget(extra11);


        horizontalLayout_9->addLayout(verticalLayout_20);


        verticalLayout_21->addLayout(horizontalLayout_9);

        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setSpacing(6);
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        verticalLayout_17 = new QVBoxLayout();
        verticalLayout_17->setSpacing(6);
        verticalLayout_17->setObjectName(QString::fromUtf8("verticalLayout_17"));
        extra12_lbl = new QLabel(tab_3);
        extra12_lbl->setObjectName(QString::fromUtf8("extra12_lbl"));
        sizePolicy1.setHeightForWidth(extra12_lbl->sizePolicy().hasHeightForWidth());
        extra12_lbl->setSizePolicy(sizePolicy1);

        verticalLayout_17->addWidget(extra12_lbl);

        extra12 = new QLabel(tab_3);
        extra12->setObjectName(QString::fromUtf8("extra12"));
        sizePolicy2.setHeightForWidth(extra12->sizePolicy().hasHeightForWidth());
        extra12->setSizePolicy(sizePolicy2);
        extra12->setMinimumSize(QSize(0, 0));
        extra12->setFrameShape(QFrame::StyledPanel);

        verticalLayout_17->addWidget(extra12);


        horizontalLayout_8->addLayout(verticalLayout_17);

        verticalLayout_18 = new QVBoxLayout();
        verticalLayout_18->setSpacing(6);
        verticalLayout_18->setObjectName(QString::fromUtf8("verticalLayout_18"));
        extra13_lbl = new QLabel(tab_3);
        extra13_lbl->setObjectName(QString::fromUtf8("extra13_lbl"));
        sizePolicy1.setHeightForWidth(extra13_lbl->sizePolicy().hasHeightForWidth());
        extra13_lbl->setSizePolicy(sizePolicy1);

        verticalLayout_18->addWidget(extra13_lbl);

        extra13 = new QLabel(tab_3);
        extra13->setObjectName(QString::fromUtf8("extra13"));
        sizePolicy2.setHeightForWidth(extra13->sizePolicy().hasHeightForWidth());
        extra13->setSizePolicy(sizePolicy2);
        extra13->setMinimumSize(QSize(0, 0));
        extra13->setFrameShape(QFrame::StyledPanel);

        verticalLayout_18->addWidget(extra13);


        horizontalLayout_8->addLayout(verticalLayout_18);


        verticalLayout_21->addLayout(horizontalLayout_8);


        horizontalLayout_10->addLayout(verticalLayout_21);

        tabWidget->addTab(tab_3, QString());

        horizontalLayout_3->addWidget(tabWidget);

        verticalLayout_8 = new QVBoxLayout();
        verticalLayout_8->setSpacing(6);
        verticalLayout_8->setObjectName(QString::fromUtf8("verticalLayout_8"));
        verticalLayout_7 = new QVBoxLayout();
        verticalLayout_7->setSpacing(6);
        verticalLayout_7->setObjectName(QString::fromUtf8("verticalLayout_7"));
        label_5 = new QLabel(centralWidget);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setWordWrap(true);

        verticalLayout_7->addWidget(label_5);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_7->addItem(verticalSpacer);

        generateMapBtn = new QPushButton(centralWidget);
        generateMapBtn->setObjectName(QString::fromUtf8("generateMapBtn"));

        verticalLayout_7->addWidget(generateMapBtn);

        resetBtn = new QPushButton(centralWidget);
        resetBtn->setObjectName(QString::fromUtf8("resetBtn"));

        verticalLayout_7->addWidget(resetBtn);


        verticalLayout_8->addLayout(verticalLayout_7);

        verticalLayout_6 = new QVBoxLayout();
        verticalLayout_6->setSpacing(6);
        verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
        label_6 = new QLabel(centralWidget);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        verticalLayout_6->addWidget(label_6);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_6->addItem(verticalSpacer_2);

        req0Btn = new QPushButton(centralWidget);
        req0Btn->setObjectName(QString::fromUtf8("req0Btn"));

        verticalLayout_6->addWidget(req0Btn);

        req1Btn = new QPushButton(centralWidget);
        req1Btn->setObjectName(QString::fromUtf8("req1Btn"));

        verticalLayout_6->addWidget(req1Btn);

        autonomousBtn = new QPushButton(centralWidget);
        autonomousBtn->setObjectName(QString::fromUtf8("autonomousBtn"));

        verticalLayout_6->addWidget(autonomousBtn);


        verticalLayout_8->addLayout(verticalLayout_6);


        horizontalLayout_3->addLayout(verticalLayout_8);

        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 710, 25));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MerryNavigationStack", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-weight:600; font-style:italic; text-decoration: underline;\">Mission and Global Planner 0</span></p></body></html>", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-weight:600; font-style:italic; text-decoration: underline;\">Mission and Global Planner 1</span></p></body></html>", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-weight:600; font-style:italic; text-decoration: underline;\">Local Planner 0</span></p></body></html>", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-weight:600; font-style:italic; text-decoration: underline;\">Local Planner 1</span></p></body></html>", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("MainWindow", "Log", 0, QApplication::UnicodeUTF8));
        extra00_lbl->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-weight:600; font-style:italic;\">Extra 00</span></p></body></html>", 0, QApplication::UnicodeUTF8));
        extra00->setText(QApplication::translate("MainWindow", "No Image Yet", 0, QApplication::UnicodeUTF8));
        extra01_lbl->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-weight:600; font-style:italic;\">Extra 01</span></p></body></html>", 0, QApplication::UnicodeUTF8));
        extra01->setText(QApplication::translate("MainWindow", "No Image Yet", 0, QApplication::UnicodeUTF8));
        extra02_lbl->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-weight:600; font-style:italic;\">Extra 02</span></p></body></html>", 0, QApplication::UnicodeUTF8));
        extra02->setText(QApplication::translate("MainWindow", "No Image Yet", 0, QApplication::UnicodeUTF8));
        extra03_lbl->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-weight:600; font-style:italic;\">Extra 03</span></p></body></html>", 0, QApplication::UnicodeUTF8));
        extra03->setText(QApplication::translate("MainWindow", "No Image Yet", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("MainWindow", "Extra0", 0, QApplication::UnicodeUTF8));
        extra10_lbl->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-weight:600; font-style:italic;\">Extra 10</span></p></body></html>", 0, QApplication::UnicodeUTF8));
        extra10->setText(QApplication::translate("MainWindow", "No Image Yet", 0, QApplication::UnicodeUTF8));
        extra11_lbl->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-weight:600; font-style:italic;\">Extra 11</span></p></body></html>", 0, QApplication::UnicodeUTF8));
        extra11->setText(QApplication::translate("MainWindow", "No Image Yet", 0, QApplication::UnicodeUTF8));
        extra12_lbl->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-weight:600; font-style:italic;\">Extra 12</span></p></body></html>", 0, QApplication::UnicodeUTF8));
        extra12->setText(QApplication::translate("MainWindow", "No Image Yet", 0, QApplication::UnicodeUTF8));
        extra13_lbl->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-weight:600; font-style:italic;\">Extra 13</span></p></body></html>", 0, QApplication::UnicodeUTF8));
        extra13->setText(QApplication::translate("MainWindow", "No Image Yet", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QApplication::translate("MainWindow", "Extra1", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-weight:600; text-decoration: underline;\">Multi-RGBDSLAM Service</span></p></body></html>", 0, QApplication::UnicodeUTF8));
        generateMapBtn->setText(QApplication::translate("MainWindow", "Generate 2D Map", 0, QApplication::UnicodeUTF8));
        resetBtn->setText(QApplication::translate("MainWindow", "Reset", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-weight:600; text-decoration: underline;\">Navigation Stack</span></p></body></html>", 0, QApplication::UnicodeUTF8));
        req0Btn->setText(QApplication::translate("MainWindow", "Request Frontier 0", 0, QApplication::UnicodeUTF8));
        req1Btn->setText(QApplication::translate("MainWindow", "Request Frontier 1", 0, QApplication::UnicodeUTF8));
        autonomousBtn->setText(QApplication::translate("MainWindow", "Start Autonomous", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
