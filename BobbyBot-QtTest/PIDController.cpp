/**
 * \file PIDController.cpp
 * \author Eduard Codres
 * \copyright Manchester Robotics
 * \date March, 2019
 * \brief PIDController class source file.
 */

#include "math.h"
#include "PIDController.h"

PIDController::PIDController()
{
  u = 0;

  Kp = 1;
  Ti = 1;
  Td = 0;

  ek_0 = 0;
  ek_1 = 0;

  yk_0 = 0;
  yk_1 = 0;
  yk_2 = 0;

  u_min = -1;
  u_max = 1;

  u_zero = 0;
}

void PIDController::SetParameters(double Kp,double Ti,double Td)
{
  this->Kp = Kp;
  this->Ti = Ti;
  this->Td = Td;
}

void PIDController::SetControlLimits(double u_min, double u_max)
{
  this->u_min = u_min;
  this->u_max = u_max;
}

void PIDController::SetControlZero(double u_zero)
{
  this->u_zero = u_zero;
  u = u_zero;
}

void PIDController::SetDeadZone(double dead_min,double dead_max)
{
  this->dead_min = dead_min;
  this->dead_max = dead_max;
}

double PIDController::GetControl(double desired,double system_,float dt)
{
  double du;

  ek_1 = ek_0;
  ek_0 = desired - system_;
    
  yk_2 = yk_1;
  yk_1 = yk_0;
  yk_0 = system_;

  du = Kp*(ek_0-ek_1) + Kp*dt/Ti*ek_0 + Kp*Td/dt*(yk_0-2*yk_1+yk_2);
  u += du;
    
  if(u<u_min)
    u=u_min;
  if(u>u_max)
    u=u_max;

  if(fabs(ek_0)<0.01)               // apply dead zone only when error is small
    if(u>dead_min && u<dead_max)
      u=u_zero;

  return u;
}
