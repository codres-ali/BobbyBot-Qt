#include "TrajectoryPlanner.h"
#include "MyMath.h"

void CurvePlanner::Init(float R, float dR, float xi, int n, int dir)
{
    this->R = R;
    this->dR = dR;
    this->xi = xi;
    this->n = n;
    this->dir = dir;

    s_size = (int)(2.0f*M_PI/dt)+1;
    s_vec = new float[s_size];

    if(R > 0 && dR>0 && n>0)
    {
        float t = 0;
        s_vec[0] = 0;

        float dsk = sqrt(powf(x_dot(t),2.0f) + powf(y_dot(t),2.0f));
        for(int i=1;i<s_size;i++)
        {
            float dsk_1 = dsk;
            t+=dt;
            dsk = sqrt(powf(x_dot(t),2.0f) + powf(y_dot(t),2.0f));
            s_vec[i] = s_vec[i-1]+dt*(dsk+dsk_1)/2.0f;
        }
        s_max = s_vec[s_size-1];//s(2.0f*M_PI-1e-6);
    }
}

float CurvePlanner::x(float t)
{
    float t1 = WrapTo2Pi(t);
    return dir*(R+dR*cos(xi+(float)n*t1))*sin(t1);
}

float CurvePlanner::y(float t)
{
    float t1 = WrapTo2Pi(t);
    return -(R+dR*cos(xi+(float)n*t1))*cos(t1);
}

float CurvePlanner::x_dot(float t)
{
    float t1 = WrapTo2Pi(t);
    return dir*(cos(t1)*(R + dR*cos(xi+(float)n*t1)) - dR*(float)n*sin(xi+(float)n*t1)*sin(t1));
}

float CurvePlanner::y_dot(float t)
{
    float t1 = WrapTo2Pi(t);
    return sin(t1)*(R + dR*cos(xi+(float)n*t1)) + dR*(float)n*sin(xi+(float)n*t1)*cos(t1);
}

float CurvePlanner::x_dot_dot(float t)
{
    float t1 = WrapTo2Pi(t);
    return dir*(- sin(t1)*(R + dR*cos(xi+(float)n*t1)) - dR*powf(n,2.0f)*cos(xi+(float)n*t1)*sin(t1) - 2.0f*dR*(float)n*sin(xi+(float)n*t1)*cos(t1));
}

float CurvePlanner::y_dot_dot(float t)
{
    float t1 = WrapTo2Pi(t);
    return cos(t1)*(R + dR*cos(xi+(float)n*t1)) + dR*powf(n,2.0f)*cos(xi+(float)n*t1)*cos(t1) - 2.0f*dR*(float)n*sin(xi+(float)n*t1)*sin(t1);
}

float CurvePlanner::theta(float t)
{
    float t1 = WrapTo2Pi(t);
    return WrapTo2Pi(atan2(y_dot(t1),x_dot(t1)));
}

float CurvePlanner::k(float t)
{
    float t1 = WrapTo2Pi(t);
    float x_d = x_dot(t1);
    float y_d = y_dot(t1);
    float x_dd = x_dot_dot(t1);
    float y_dd = y_dot_dot(t1);
    return (x_d*y_dd-x_dd*y_d)/powf(powf(x_d,2.0f)+powf(y_d,2.0f),1.5f);
}

float CurvePlanner::k_dot(float t)
{
    float t1 = WrapTo2Pi(t);
    return (k(t1+0.001f)-k(t1))/0.001f;
}

float CurvePlanner::s(float t)
{
    float t1 = WrapTo2Pi(t);
    int ind = floor(t1/dt);
    return s_vec[ind] + (s_vec[ind+1]-s_vec[ind])*(t1-(float)ind*dt)/dt;
}

float CurvePlanner::t(float s)
{
    if(s>=0 && s<=s_vec[s_size-1])
    {
        int indx = floor(s_size/2);
        int up = s_size;
        int low = 0;
        bool exit = false;
        while(!exit)
        {
            if(s>=s_vec[indx] && s<s_vec[indx+1])
            {
                return (indx)*dt + (s-s_vec[indx])*dt/(s_vec[indx+1]-s_vec[indx]);
            }
            else
                if(s<s_vec[indx])
                {
                    up = indx;
                    indx = floor((low+indx)/2.0f);
                }
                else
                {
                    low = indx;
                    indx = floor((up+indx+1)/2.0f);
                }
        }
    }
    else
    {
        if(s>s_vec[s_size-1])
           return 2.0f*M_PI;
    }

    return 0;
}

float CurvePlanner::smax()
{
    return s_max;
}



void VelocityPlanner::Init(float vi, float vf, float a_max, float tf)
{
    this->vi = vi;
    this->vf = vf;
    this->a_max = a_max;

    if(tf == 0)
    {
         if(a_max!=0)
         {
             _tf = 4.0f*((vf+vi)/2.0f-vi)/a_max;
             if(_tf<0)
             {
                 _tf = -_tf;
                 a_max = -a_max;
             }
             b2 = ((vf+vi)/2.0f-vi)/powf(_tf/2.0f,2.0f);
         }
    }
    else
    {
         b2 = ((vf+vi)/2.0f-vi)/powf(_tf/2.0f,2.0f);
         a_max = b2*_tf;
    }

     s_max = s(_tf);
}

float VelocityPlanner::v(float t)
{
    if(t>=0 && t<_tf/2.0f)
        return b2*powf(t,2.0f)+vi;
    else
         if(t>=_tf/2.0f && t<=_tf)
             return -b2*powf(t-_tf,2.0f)+vf;
    return 0;
}

float VelocityPlanner::a(float t)
{
    if(t>=0 && t<_tf/2.0f)
        return 2.0f*b2*t;
    else
         if(t>=_tf/2.0f && t<=_tf)
             return -2.0f*b2*(t-_tf);
    return 0;
}

float VelocityPlanner::s(float t)
{
    if(t>=0 && t<_tf/2)
    {
        return 1.0f/3.0f*b2*powf(t,3.0f)+vi*t;
    }
    else
         if(t>=_tf/2 && t<=_tf)
             return 1.0f/3.0f*b2*powf(_tf-t,3.0f)+vf*(t-_tf/2)+vi*_tf/2.0f;
    return 0;
}

float VelocityPlanner::tf()
{
    return _tf;
}

float VelocityPlanner::smax()
{
    return s_max;
}

void TrajectoryPlanner::Init(float R, float dR, float xi, int n,float v_max, float a_max,float phase,int dir,float dt)
{
    this->v_max = v_max;
    this->a_max = a_max;
    this->phase = phase;
    this->dt = dt;
    this->dir = dir;

    curve.Init(R,dR,xi,n,dir);
    vel_start.Init(0,v_max,a_max);
    vel_stop.Init(v_max,0,-a_max);
    vel_stop_fast.Init(v_max,0,-2*a_max);

}

unsigned long TrajectoryPlanner::size()
{
    return trajectory.size();
}

float TrajectoryPlanner::GetDt()
{
    return dt;
}

vector<float> TrajectoryPlanner::GetTraj(unsigned long ind)
{
    if(ind<trajectory.size())
        return trajectory[ind];

    return trajectory[0];
}

void TrajectoryPlanner::PlanTraj_StartRunStop()
{
    float t=0;
    float s=0;
    int k = 0;
    float tc = 0;
    bool first = true;
    bool exit = false;
    float s_start = curve.s(phase);
    float t_stop,s_stop;

    while(tc<2*M_PI && !exit)
    {
        vector<float> tmp_traj(8);
        t = k*dt;
        if( t < vel_start.tf())
        {
            tmp_traj[3] = vel_start.v(t);
            tmp_traj[5] = vel_start.a(t);
            s = s_start+vel_start.s(t);
        }
        else
            if(s<(curve.smax()-vel_stop.smax()-2*dt*v_max))
            {
                tmp_traj[3] = v_max;
                tmp_traj[5] = 0;
                s = s + dt*v_max;
            }
            else
            {
                if(first)
                {
                    t_stop = t;
                    s_stop = s+dt*v_max;
                    first = false;
                }

                float ts = t - t_stop;
                if(ts <= vel_stop.tf())
                {
                    tmp_traj[3] = vel_stop.v(ts);
                    tmp_traj[5] = vel_stop.a(ts);
                    s = s_stop+vel_stop.s(ts);
                }
                else
                    exit = true;
            }

        if(!exit)
        {
            tc = curve.t(s);
            tmp_traj[0] = curve.x(tc);
            tmp_traj[1] = curve.y(tc);
            tmp_traj[2] = curve.theta(tc);
            tmp_traj[4] = tmp_traj[3]*curve.k(tc);
            tmp_traj[6] = tmp_traj[3]*curve.k_dot(tc)+tmp_traj[5]*curve.k(tc);
            tmp_traj[7] = t;
            trajectory.push_back(tmp_traj);
        }
        k = k+1;
    }
}
