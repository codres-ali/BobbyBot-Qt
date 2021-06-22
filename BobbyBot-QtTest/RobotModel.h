#ifndef ROBOTMODEL_H
#define ROBOTMODEL_H

struct RobotParams
{
    // DC motor parameters
    float Kt;
    float Ke;
    float Rar;
    float Ral;
    float La;
    float u_max;

    // reduction gear parameters
    float tr;
    float miu;
    float tau_fr;

    // robot parameters
    float Rr;          // wheel radius
    float Rl;
    float L;           // half of track width
    float d;           // distance between center of mass and center of rotation
    float M;           // mass of the robot
    float mw;          // mass of the wheel

    // moment of inertia
    float I0;
    float Im;
    float Iw;

    // friction parameters (static and viscous for each wheel)
    float Cs_r;
    float Cs_l;
    float Cv_r;
    float Cv_l;

};

class RobotModel
{
    RobotParams params;

    // Robot matrices
    float M_inv[2][2];     // robot mass and inertia matrix
    float V[2][2];     // robot coriolis matrix

    // output velocity for right and left wheel
    float w_r;
    float w_l;

    // set which input to use
    // 1 - Voltage input
    // 2 - Torque input
    int input_type;

    // wheel dc motor currents
    float Ir,Il;

    // robot linear and angular velocities
    float Vr,Wr;

public:

    RobotModel();
    ~RobotModel(){};

    void Init(RobotParams params);
    void Update(float Ur,float Ul,float dt);

    float Get_w_r();
    float Get_w_l();
    float Get_Ir();
    float Get_Il();
    float Get_Vr();
    float Get_Wr();
};

float AdjustU(float u, float u_max);


#endif // ROBOTMODEL_H
