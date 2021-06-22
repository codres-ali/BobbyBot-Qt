#include "mainwindow.h"
#include "ui_mainwindow.h"

#define NP 1000

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    n_plots = 6;
    plots[0] = ui->plot1;
    plots[1] = ui->plot2;
    plots[2] = ui->plot3;
    plots[3] = ui->plot4;
    plots[4] = ui->plot5;
    plots[5] = ui->plot6;

    for(int i=0;i<n_plots;i++)
    {
        plots[i]->xAxis->setLabel("x");
        plots[i]->yAxis->setLabel("y");
        plots[i]->xAxis->setRange(-2, 2);
        plots[i]->yAxis->setRange(-1.5, 1.5);
        plots[i]->xAxis2->setVisible(true);
        plots[i]->xAxis2->setTickLabels(false);
        plots[i]->yAxis2->setVisible(true);
        plots[i]->yAxis2->setTickLabels(false);
        plots[i]->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);

    }

    current_fig = 0;
    n_color = 0;
    hold = false;


    //TestCurvePlanner();
    //TestVelocityPlanner();
    //TestTrajectoryPlanner();
    //TestRobotModel();
    //TestPid();
    TestSMC();

}

void MainWindow::TestCurvePlanner()
{
    CurvePlanner curve1,curve2;

    QVector<double> t(NP),x1(NP),y1(NP),x2(NP),y2(NP);
    QVector<double> s(NP),k(NP),k_dot(NP),ts(NP);

    curve1.Init(1,0.2,0,4);
    curve2.Init(1,0.2,M_PI,4);

    for(int i=0;i<NP;i++)
    {
        t[i] = (double)i*2*M_PI/NP;
        x1[i] = curve1.x(t[i]);
        y1[i] = curve1.y(t[i]);
        s[i] = curve1.s(t[i]);
        ts[i] = curve1.t(s[i]);
        k[i] = curve1.k(t[i]);
        k_dot[i] = curve1.k_dot(t[i]);
        x2[i] = curve2.x(t[i]);
        y2[i] = curve2.y(t[i]);
    }

    figure(0);
    plot(x1,y1);
    hold_on();
    plot(x2,y2);

    figure(1);
    setAxis(0,6.5,-5,5);
    plot(t,k);

    figure(2);
    setAxis(0,6.5,-20,20);
    plot(t,k_dot);

    figure(3);
    setAxis(0,6.5,0,10);
    plot(t,s);

    figure(4);
    setAxis(0,8,0,10);
    plot(s,ts);
}

void MainWindow::TestVelocityPlanner()
{
    VelocityPlanner vel1,vel2;

    QVector<double> sv1(NP),sv2(NP),tv1(NP),tv2(NP),v1(NP),a1(NP),v2(NP),a2(NP);

    vel1.Init(0,0.5,1);
    vel2.Init(0.5,0,0.5);

    for(int i=0;i<NP;i++)
    {
        tv1[i] = (double)i*vel1.tf()/NP;
        tv2[i] = (double)i*vel2.tf()/NP;

        v1[i] = vel1.v(tv1[i]);
        a1[i] = vel1.a(tv1[i]);
        sv1[i] = vel1.s(tv1[i]);
        v2[i] = vel2.v(tv2[i]);
        a2[i] = vel2.a(tv2[i]);
        sv2[i] = vel1.smax()+vel2.s(tv2[i]);
        tv2[i] += vel1.tf();
    }

    figure(0);
    setAxis(0,4,-1,1);
    plot(tv1,v1);
    hold_on();
    plot(tv2,v2);

    figure(1);
    setAxis(0,4,-1,1);
    plot(tv1,a1);
    hold_on();
    plot(tv2,a2);

    figure(2);
    setAxis(0,4,-1,1);
    plot(tv1,sv1);
    hold_on();
    plot(tv2,sv2);
}

void MainWindow::TestTrajectoryPlanner()
{
    TrajectoryPlanner traj;

    traj.Init(1,0.2,0,4,0.5,1,0,1);

    traj.PlanTraj_StartRunStop();

    QVector<double> Xd(traj.size()),Yd(traj.size()),Thetad(traj.size()),Vd(traj.size()),Wd(traj.size()),Vd_dot(traj.size()),Wd_dot(traj.size()),T(traj.size());

    for(unsigned long i=0;i<traj.size();i++)
    {
        vector<float> trj_tmp;

        trj_tmp = traj.GetTraj(i);

        Xd[i] = trj_tmp[0];
        Yd[i] = trj_tmp[1];
        Thetad[i] = trj_tmp[2];
        Vd[i] = trj_tmp[3];
        Wd[i] = trj_tmp[4];
        Vd_dot[i] = trj_tmp[5];
        Wd_dot[i] = trj_tmp[6];
        T[i] = trj_tmp[7];
    }

    figure(0);
    plot(Xd,Yd);

    figure(1);
    setAxis(0,16,0,7);
    plot(T,Thetad);

    figure(2);
    setAxis(0,16,0,1);
    plot(T,Vd);

    figure(3);
    setAxis(0,16,-2,2);
    plot(T,Wd);

    figure(4);
    setAxis(0,16,-2,2);
    plot(T,Vd_dot);

    figure(5);
    setAxis(0,16,-10,10);
    plot(T,Wd_dot);

}

void MainWindow::TestRobotModel()
{
    float ur,ul;
    float Tmax,dt;

    QVector<double> w_r(NP),w_l(NP),Ir(NP),Il(NP),Vr(NP),Wr(NP),t(NP);
    RobotModel rob;

    ur = 1;
    ul = 1;

    Tmax = 0.5;
    dt = Tmax/(float)NP;

    for(int i=0;i<NP;i++)
    {
        t[i] = (double)i*dt;

        rob.Update(ur,ul,dt);

        w_r[i] = rob.Get_w_r();
        w_l[i] = rob.Get_w_l();
        Ir[i] = rob.Get_Ir();
        Il[i] = rob.Get_Il();
        Vr[i] = rob.Get_Vr();
        Wr[i] = rob.Get_Wr();
    }

    figure(0);
    setAxis(0,0.5,0,20);
    plot(t,w_r);
    hold_on();
    plot(t,w_l);

    figure(1);
    setAxis(0,0.5,0,2);
    plot(t,Ir);
    hold_on();
    plot(t,Il);

    figure(2);
    setAxis(0,1,0,1);
    plot(t,Vr);

    figure(3);
    setAxis(0,1,0,2);
    plot(t,Wr);
}

void MainWindow::TestPid()
{
    float ur,ul;
    float wr,wl;
    float wr_set,wl_set;
    float Tmax,dt;

    Tmax = 0.5;
    dt = 0.001;

    int n_pid = 500;

    QVector<double> w_r(n_pid),w_l(n_pid),t(n_pid);
    RobotModel rob;
    PIDController pid_r,pid_l;

    pid_r.SetParameters(0.08,0.03,0);
    pid_l.SetParameters(0.08,0.03,0);

    wr = 0;
    wl = 0;

    wr_set = 10;
    wl_set = 13;

    for(int i=0;i<n_pid;i++)
    {
        t[i] = (double)i*dt;

        ur = pid_r.GetControl(wr_set,wr,dt);
        ul = pid_l.GetControl(wl_set,wl,dt);

        rob.Update(ur,ul,dt);

        w_r[i] = rob.Get_w_r();
        w_l[i] = rob.Get_w_l();

        wr = w_r[i];
        wl = w_l[i];
    }

    figure(0);
    setAxis(0,0.5,0,20);
    plot(t,w_r);
    hold_on();
    plot(t,w_l);

}

void MainWindow::TestSMC()
{
    float ur,ul;
    float w_r,w_l;
    float wr_set,wl_set;
    float xr,yr,thetar;
    float Vr,Wr;
    float R,L;
    float dt_pid,dt_smc;
    int k=0,N_pid,n_traj;
    float R_traj,dr_traj,xi_traj,v_max,a_max;

    R = 0.05;
    L = 0.055;

    dt_pid = 0.001;
    dt_smc = 0.05;

    N_pid = 50;

    R_traj = 1;
    dr_traj = 0.15;
    xi_traj = 0;
    v_max = 0.4;
    a_max = 0.5;
    n_traj = 4;

    RobotModel rob;
    PIDController pid_r,pid_l;
    SMCKinematic smck;
    TrajectoryPlanner traj;

    traj.Init(R_traj,dr_traj,xi_traj,n_traj,v_max,a_max,0.0f,1,dt_smc);

    traj.PlanTraj_StartRunStop();

    QVector<double> t_pid(N_pid*traj.size()),wr_pid(N_pid*traj.size()),wl_pid(N_pid*traj.size()),wr_set_pid(N_pid*traj.size());
    QVector<double> t(traj.size()),xd(traj.size()),yd(traj.size()),Vd(traj.size()),Wd(traj.size());
    QVector<double> xr_all(traj.size()),yr_all(traj.size()),thetar_all(traj.size()),Vr_all(traj.size()),Wr_all(traj.size());

    pid_r.SetParameters(0.08,0.03,0);
    pid_l.SetParameters(0.08,0.03,0);

    smck.SetK(10,14,20);
    smck.SetP(0.3,0.5);
    smck.SetQ(0.5,2);

    w_r = 0;
    w_l = 0;

    xr = traj.GetTraj(k)[0];
    yr = traj.GetTraj(k)[1];
    thetar = traj.GetTraj(k)[2];

    Vr = 0;
    Wr = 0;

    wr_set = 0;
    wl_set = 0;

    for(int i=0;i<N_pid*traj.size();i++)
    {
        if(i%N_pid==0)
        {
            float Vc,Wc;
            vector<float> trj_tmp;

            trj_tmp = traj.GetTraj(k);

            t[k] = trj_tmp[7];

            smck.Update(xr,yr,thetar,Vr,Wr,trj_tmp,dt_smc);

            smck.GetControl(Vc,Wc);

            wr_set = (Vc + L*Wc)/R;
            wl_set = (Vc - L*Wc)/R;

            xd[k] = trj_tmp[0];
            yd[k] = trj_tmp[1];
            Vd[k] = trj_tmp[3];
            Wd[k] = trj_tmp[4];

            xr_all[k] = xr;
            yr_all[k] = yr;
            thetar_all[k] = thetar;
            Vr_all[k] = Vr;
            Wr_all[k] = Wr;

            k++;
        }

        ur = pid_r.GetControl(wr_set,w_r,dt_pid);
        ul = pid_l.GetControl(wl_set,w_l,dt_pid);

        t_pid[i] = (double)i*dt_pid;
        wr_pid[i] = w_r;
        wl_pid[i] = w_l;
        wr_set_pid[i] = wr_set;

        rob.Update(ur,ul,dt_pid);

        w_r = rob.Get_w_r();
        w_l = rob.Get_w_l();

        Vr = (w_r+w_l)*R/2;
        Wr = (w_r-w_l)*R/(2*L);

        xr += dt_pid*Vr*cos(thetar+dt_pid*Wr/2);
        yr += dt_pid*Vr*sin(thetar+dt_pid*Wr/2);
        thetar = WrapTo2Pi(thetar + dt_pid*Wr);

    }

    figure(0);
    setAxis(-2,2,-1.5,1.5);
    plot(xd,yd);
    hold_on();
    plot(xr_all,yr_all);

    figure(1);
    setAxis(0,20,-0.5,0.5);
    plot(t,Vr_all);
    hold_on();
    plot(t,Vd);

    figure(2);
    setAxis(0,20,-1,1);
    plot(t,Wr_all);
    hold_on();
    plot(t,Wd);

    figure(3);
    setAxis(0,20,0,15);
    plot(t_pid,wr_pid);
    hold_on();
    plot(t_pid,wr_set_pid);

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::figure(int fig)
{
    if(fig>=0 && fig<n_plots)
    {
        current_fig = fig;
        n_color = 0;
    }
}
void MainWindow::hold_on()
{
    hold = true;
    n_color = 0;
}
void MainWindow::hold_off()
{
    hold = false;
    n_color = 0;
}

void MainWindow::setAxis(double x_min,double x_max,double y_min,double y_max)
{
    plots[current_fig]->xAxis->setRange(x_min, x_max);
    plots[current_fig]->yAxis->setRange(y_min, y_max);

}

void MainWindow::plot(QVector<double> y)
{

}

void MainWindow::plot(QVector<double> x, QVector<double> y)
{
    unsigned long colors[8] = {Qt::red, Qt::red, Qt::green, Qt::cyan, Qt::magenta, Qt::yellow, Qt::lightGray, Qt::darkBlue};

    if(!hold)
    {
        plots[current_fig]->clearGraphs();
        n_color = 0;
    }

    plots[current_fig]->addGraph();
    //plots[current_fig]->graph(0)->setPen(QPen(colors[n_color]));
    plots[current_fig]->graph()->setPen(QPen(Qt::red));
    n_color++;

    QCPCurve *curve = new QCPCurve(plots[current_fig]->xAxis, plots[current_fig]->yAxis);
    QVector<QCPCurveData> data_curve(x.size());
    for(int i=0;i<x.size();i++)
        data_curve[i] = QCPCurveData(i, x[i], y[i]);
    curve->data()->set(data_curve, true);

    ui->plot1->graph()->setPen(QPen(Qt::red));
    plots[current_fig]->replot();
}
