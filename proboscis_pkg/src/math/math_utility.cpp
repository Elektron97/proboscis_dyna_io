/***Source Code for Math Utility Library***/
#include "proboscis_pkg/math/math_utility.h"

void cartesian2Polar(float x, float y, float& rho, float& theta)
{
    rho = sqrt(powf(x, 2.0) + powf(y, 2.0));
    theta = atan2(y, x) + PI;
    return;
}