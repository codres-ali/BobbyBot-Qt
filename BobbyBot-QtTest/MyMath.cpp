#include "MyMath.h"

int sign(float x)
{
    if (x>0)
        return 1;
    else if (x<0)
        return -1;
    else
        return 0;
}

float WrapToPi(float phi)
{
    phi = remainder(phi,2.0*M_PI);
    if (fabs(phi)>M_PI)
        phi = remainder(phi,2.0*M_PI) - sign(phi)*2.0*M_PI;
    return phi;
}

float WrapTo2Pi(float phi)
{
    phi = remainder(phi,2.0*M_PI);
    if (phi<0)
        phi = phi + 2.0*M_PI;
    return phi;
}
