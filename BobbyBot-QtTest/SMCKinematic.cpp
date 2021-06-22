#include <math.h>
#include "SMCKinematic.h"
#include "MyMath.h"

void SMCKinematic::SetK(float k1,float k2,float k0)
{
    this->k0 = k0;
    this->k1 = k1;
    this->k2 = k2;
}

void SMCKinematic::SetP(float P1,float P2)
{
    this->P1 = P1;
    this->P2 = P2;
}

void SMCKinematic::SetQ(float Q1,float Q2)
{
    this->Q1 = Q1;
    this->Q2 = Q2;
}

void SMCKinematic::Update(float xr,float yr,float thetar,float vr,float wr,vector<float> traj,float dt)
{
    float x_d,y_d,theta_d,v_d,w_d,v_d_dot,w_d_dot;
    float x_e,y_e,theta_e;
    float x_e_dot,y_e_dot,theta_e_dot;
    float s1,s2,T1,T2;
    float vc_dot,wc_dot;

    x_d = traj[0];
    y_d = traj[1];
    theta_d = traj[2];
    v_d = traj[3];
    w_d = traj[4];
    v_d_dot = traj[5];
    w_d_dot = traj[6];

    // Compute the error signal of each state
    x_e = (xr-x_d)*cos(theta_d)+(yr-y_d)*sin(theta_d);
    y_e = -(xr-x_d)*sin(theta_d)+(yr-y_d)*cos(theta_d);
    theta_e = WrapToPi(thetar-theta_d);

    // Compute the error derivatives
    x_e_dot = vr*cos(theta_e) + y_e*w_d - v_d;
    y_e_dot = vr*sin(theta_e) - x_e*w_d;
    theta_e_dot = wr - w_d;

    // Compute sliding surfaces
    s1 = x_e_dot + k1*x_e;
    s2 = theta_e_dot + k2*theta_e + k0*y_e;

    // Compute control signals
    T1 = vr*theta_e_dot*sin(theta_e)-w_d_dot*y_e-w_d*y_e_dot+v_d_dot-k1*x_e_dot;
    T2 = w_d_dot - k2*theta_e_dot - k0*y_e_dot;

    vc_dot = (-Q1*s1-P1*sign(s1)+T1)/cos(theta_e);
    wc_dot = -Q2*s2-P2*sign(s2)+T2;

    vc = vc+dt*vc_dot;
    wc = wc+dt*wc_dot;
}

void SMCKinematic::GetControl(float &vc,float &wc)
{
    vc = this->vc;
    wc = this->wc;
}
