#include "tk_odom.h"
#include "tk_global.h"
#include "math.h"
#include "iostream"
using namespace std;

void ODOM::odom_update(double L1, double L2, double x, double y, double theta, odom_res *odom)
{
    double rot_angle;

    // ?¨²?¡Â¨¨?¡Á?¡À¨º?¦Ì¡À??¡¥¨¢?
    rot_angle   =  (L2*k2 - L1*k1)/Len;
    if(L1*k1 == L2*k2) // ?¡À??
    {
        odom->dx = 0;
        odom->dy = L1*k1;
     }
    else
    {
        odom->dx    = -(L2*k2 + L1*k1)/2.0/(L2*k2-L1*k1)*Len*(1 - cos(rot_angle));
        odom->dy    =  (L2*k2 + L1*k1)/2.0/(L2*k2-L1*k1)*Len*sin(rot_angle);
    }
//    cout << theta << endl;
//    cout << rot_angle << endl;
    // ¨¨???¡Á?¡À¨º?¦Ì?????¡éo??¨°
    odom->x     =  x + (odom->dx*cos(theta) - odom->dy*sin(theta));
    odom->y     =  y + (odom->dx*sin(theta) + odom->dy*cos(theta));
    odom->theta =  theta + rot_angle;

}
