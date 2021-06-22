/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.14.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>
#include "qcustomplot.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QCustomPlot *plot1;
    QCustomPlot *plot2;
    QCustomPlot *plot4;
    QCustomPlot *plot5;
    QCustomPlot *plot3;
    QCustomPlot *plot6;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1542, 839);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        plot1 = new QCustomPlot(centralwidget);
        plot1->setObjectName(QString::fromUtf8("plot1"));
        plot1->setGeometry(QRect(9, 9, 501, 371));
        plot2 = new QCustomPlot(centralwidget);
        plot2->setObjectName(QString::fromUtf8("plot2"));
        plot2->setGeometry(QRect(520, 10, 501, 371));
        plot4 = new QCustomPlot(centralwidget);
        plot4->setObjectName(QString::fromUtf8("plot4"));
        plot4->setGeometry(QRect(10, 390, 501, 371));
        plot5 = new QCustomPlot(centralwidget);
        plot5->setObjectName(QString::fromUtf8("plot5"));
        plot5->setGeometry(QRect(520, 390, 501, 371));
        plot3 = new QCustomPlot(centralwidget);
        plot3->setObjectName(QString::fromUtf8("plot3"));
        plot3->setGeometry(QRect(1030, 10, 501, 371));
        plot6 = new QCustomPlot(centralwidget);
        plot6->setObjectName(QString::fromUtf8("plot6"));
        plot6->setGeometry(QRect(1030, 390, 501, 371));
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 1542, 22));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
