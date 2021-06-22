#ifndef TRAJECTORYPLANNER_H
#define TRAJECTORYPLANNER_H
#include <vector>

using namespace std;

class CurvePlanner
{
    float R;
    float dR;
    int n = 0;
    float xi;
    float dt;
    int dir;

    int s_size;
    float *s_vec;
    float s_max;

public:
    CurvePlanner()
    {
        R = 0;
        dR = 0;
        n = 0;
        xi = 0;
        s_max = 0;
        dt = 0.01;
        dir = 1;
        s_size = 0;
        s_vec = nullptr;
    }
    ~CurvePlanner()
    {
        if(s_vec)
            delete [] s_vec;
    }

    void Init(float R, float dR, float xi, int n, int dir=1);

    float x(float t);
    float y(float t);
    float x_dot(float t);
    float y_dot(float t);
    float x_dot_dot(float t);
    float y_dot_dot(float t);
    float theta(float t);
    float k(float t);
    float k_dot(float t);
    float s(float t);
    float t(float s);
    float smax();
};


class VelocityPlanner
{
    float b2;
    float _tf;
    float vf;
    float vi;
    float a_max;
    float s_max;

public:
    VelocityPlanner()
    {
        b2 = 0;
        _tf = 0;
        vf = 0;
        vi = 0;
        a_max = 0;
        s_max = 0;
    }
    ~VelocityPlanner(){}

    void Init(float vi, float vf, float a_max, float tf=0);

    float v(float t);
    float a(float t);
    float s(float t);
    float tf();
    float smax();

};

class TrajectoryPlanner
{
    CurvePlanner curve;
    VelocityPlanner vel_start;
    VelocityPlanner vel_stop;
    VelocityPlanner vel_stop_fast;
    vector<vector<float> > trajectory;
    float dt;
    float v_max;
    float a_max;
    int dir;
    float phase;
public:
    TrajectoryPlanner()
    {
        dt = 0.05;
        v_max = 0.5;
        a_max = 1;
        dir = 1;
        phase = 0;
    }
    ~TrajectoryPlanner(){}

    void Init(float R, float dR, float xi, int n,float v_max, float a_max,float phase,int dir,float dt=0.05);
    vector <float> GetTraj(unsigned long ind);
    unsigned long size();
    float GetDt();

    void PlanTraj_StartRunStop();
};

#endif // TRAJECTORYPLANNER_H
