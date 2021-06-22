#ifndef SMCKINEMATIC_H
#define SMCKINEMATIC_H

#include <vector>

using namespace std;

class SMCKinematic
{
    float k0,k1,k2;
    float P1,P2;
    float Q1,Q2;

    float vc,wc;

public:
    SMCKinematic()
    {
        k0 = 1;
        k1 = 1;
        k2 = 1;
        P1 = 0.1;
        P2 = 0.1;
        Q1 = 0.1;
        Q2 = 0.1;
        vc = 0;
        wc = 0;
    }
    ~SMCKinematic(){}

    void SetK(float k1,float k2,float k0);
    void SetP(float P1,float P2);
    void SetQ(float Q1,float Q2);

    void Update(float xr, float yr, float thetar, float vr, float wr, vector<float> traj, float dt);
    void GetControl(float &vc,float &wc);
};

#endif // SMCKINEMATIC_H
