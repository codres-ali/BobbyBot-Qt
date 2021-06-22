#include <math.h>
#include "RobotModel.h"
#include "MyMath.h"

RobotModel::RobotModel()
{
    input_type = 1;

    // DC motor parameters
    params.Kt = 0.009;
    params.Ke = 0.009;
    params.Rar = 2.5;
    params.Ral = 2.5;
    params.La = 0.01;
    params.u_max = 5;

    // reduction gear parameters
    params.tr = 34;
    params.miu = 1;;
    params.tau_fr = 0;

    // robot parameters
    params.Rr = 0.05;
    params.Rl = 0.05;
    params.L = 0.055;
    params.d = 0;
    params.M = 0.6;
    params.mw = 0.05;

    // moment of inertia
    params.I0 = 0.001;
    params.Im = 0.0002;
    params.Iw = 0.0006;

    // friction parameters (static and viscous for each wheel)
    params.Cs_r = 0.05;
    params.Cs_l = 0.05;
    params.Cv_r = 0.0002;
    params.Cv_l = 0.0002;

    // output velocity for right and left wheel
    w_r = 0;
    w_l = 0;

    // dc motors current
    Ir = 0;
    Il = 0;

    Init(params);
}

void RobotModel::Init(RobotParams params)
{
    float m00,m01,m10,m11;
    float Mt,It;

    this->params = params;

    // Create M matrix for the Lagrange model (Inertia matrix)
    Mt = params.M+2*params.mw;
    It = params.I0+params.M*powf(params.d,2)+2*params.mw*powf(params.L,2)+2*params.Im;

    m00 = powf(params.Rr,2)/4*(Mt+It/powf(params.L,2))+params.Iw;
    m01 = params.Rr*params.Rl/4*(Mt-It/powf(params.L,2));
    m10 = m01;
    m11 = powf(params.Rl,2)/4*(Mt+It/powf(params.L,2))+params.Iw;

    M_inv[0][0] = m11/(m00*m11 - m01*m10);
    M_inv[0][1] = -m01/(m00*m11 - m01*m10);
    M_inv[1][0] = -m10/(m00*m11 - m01*m10);
    M_inv[1][1] = m00/(m00*m11 - m01*m10);

    V[0][0] = 0;
    V[0][1] = params.M*params.d*params.Rr*params.Rl/(2*params.L);
    V[1][0] = -params.M*params.d*params.Rr*params.Rl/(2*params.L);
    V[1][1] = 0;

}

void RobotModel::Update(float Ur,float Ul,float dt)
{
    float fr_r,fr_l;
    float Ir_dot,Il_dot;
    float tauR,tauL;
    float Tau[2],tmp[2];
    float w_r_dot,w_l_dot;

    // Friction for each wheel
    fr_r = params.Cs_r*sign(w_r)+params.Cv_r*w_r;
    fr_l = params.Cs_l*sign(w_l)+params.Cv_l*w_l;

    // simulate the DC motor for each wheel
    Ir_dot = (AdjustU(Ur,params.u_max) - params.Ke*params.tr*w_r - params.Rar*Ir)/params.La;
    Ir += dt*Ir_dot;
    Il_dot = (AdjustU(Ul,params.u_max) - params.Ke*params.tr*w_l - params.Ral*Il)/params.La;
    Il += dt*Il_dot;

    tauR = params.tr*params.miu*(params.Kt*Ir);
    tauL = params.tr*params.miu*(params.Kt*Il);

    // Select the input
    //  1 - torque from DC motor;
    //  2 - torque from model input;
    if (input_type == 1)
    {
        Tau[0] = tauR;
        Tau[1] = tauL;
    }
    if (input_type == 2)
    {
        Tau[0] = Ur;
        Tau[1] = Ul;
    }

    // Lagrange formula for the model - computes acceleration for each wheel
    tmp[0] = Tau[0] - V[0][1]*Wr*w_l - fr_r;
    tmp[1] = Tau[1] - V[1][0]*Wr*w_r - fr_l;

    w_r_dot = M_inv[0][0]*tmp[0] + M_inv[0][1]*tmp[1];
    w_l_dot = M_inv[1][0]*tmp[0] + M_inv[1][1]*tmp[1];

    //integrate to get the velocity for each wheel
    w_r += dt*w_r_dot;
    w_l += dt*w_l_dot;

    //compute robot velocities
    Vr = (params.Rr*w_r+params.Rl*w_l)/2;
    Wr = (params.Rr*w_r-params.Rl*w_l)/(2*params.L);

}

float RobotModel::Get_w_r()
{
    return w_r;
}

float RobotModel::Get_w_l()
{
    return w_l;
}

float RobotModel::Get_Ir()
{
    return Ir;
}

float RobotModel::Get_Il()
{
    return Il;
}

float RobotModel::Get_Vr()
{
    return Vr;
}

float RobotModel::Get_Wr()
{
    return Wr;
}

float AdjustU(float u, float u_max)
{
    float u_adj = u*u_max;

    if(u_adj>u_max)
        return u_max;
    if(u_adj<-u_max)
        return -u_max;

    return u_adj;
}

