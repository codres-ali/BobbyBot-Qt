#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "qcustomplot.h"
#include "TrajectoryPlanner.h"
#include "MyMath.h"
#include "RobotModel.h"
#include "PIDController.h"
#include "SMCKinematic.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT
    QCustomPlot* plots[6];
    int current_fig, n_color;
    int n_plots;
    bool hold;

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void figure(int fig);
    void hold_on();
    void hold_off();
    void setAxis(double x_min,double x_max,double y_min,double y_max);
    void plot(QVector<double> y);
    void plot(QVector<double> x, QVector<double> y);

    void TestCurvePlanner();
    void TestVelocityPlanner();
    void TestTrajectoryPlanner();
    void TestRobotModel();
    void TestPid();
    void TestSMC();

private:
    Ui::MainWindow *ui;
};
#endif // MAINWINDOW_H
