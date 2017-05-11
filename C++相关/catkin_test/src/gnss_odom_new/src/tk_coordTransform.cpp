#include "tk_coordTransform.h"
#include "cmath"
#include "tk_global.h"
#include <iostream>
using namespace std;

double CTF::deg2rad(const double dr, const int type)
{
    if(type == 1)
        return (dr/180.0*pi);
    else
        return (dr/pi*180);
}

void CTF::BLH_XYH(const double B, const double L, double *X, double *Y)
{
    CT Cs;
    int n;
    double L0;
    Cs.A    = 6378137.0;
    Cs.Alfa = 1.0/298.257223563;
    Cs.E2   = 0.00669437999014132;

    n = (int)(deg2rad(L,0)+1.5)/3;
    L0 = 3*n;

    double C    = Cs.A/sqrt(1-Cs.E2);
    double e2   = Cs.E2;
    double e4   = e2*e2;
    double e6   = e2*e4;
    double e8   = e4*e4;
    double e10  = e2*e8;

    double Ag   = 1+3.0/4*e2+45.0/64*e4+175.0/256*e6+11025.0/16384*e8+43659.0/65536*e10;
    double Bg   =   3.0/4*e2+15.0/16*e4+525.0/512*e6+  2205.0/2048*e8+72765.0/65536*e10;
    double Cg   =            15.0/64*e4+105.0/256*e6+  2205.0/4096*e8+10395.0/16384*e10;
    double Dg   =                        35.0/512*e6+   315.0/2048*e8+31185.0/13072*e10;

    double Alpha= Ag*Cs.A*(1-e2);
    double Beta = -0.5*Bg*Cs.A*(1-e2);
    double Gamma= 1.0/4*Cg*Cs.A*(1-e2);
    double Delta= -1.0/6*Dg*Cs.A*(1-e2);

    double C0   = Alpha;
    double C1   = 2*Beta+4*Gamma+6*Delta;
    double C2   = -8*Gamma-32*Delta;
    double C3   = 32*Delta;

    double l    = L - deg2rad(L0,1);
    double t    = tan(B);
    double m0   = l*cos(B);
    double Eta2 = e2/(1-e2)*cos(B)*cos(B);
    double N    = C/sqrt(1+Eta2);

    double m2   = m0*m0;
    double cB   = cos(B);
    double sB   = sin(B);
    double X0   = C0*B+cB*(C1*sB+C2*pow(sB,3)+C3*pow(sB,5));
    *X  = X0+0.5*N*t*m2+1.0/24*(5-t*t+9*Eta2+4*pow(Eta2,2))*N*t*m2*m2+1.0/720*(61-58*t*t+pow(t,4))*N*t*pow(m2,3);
    *Y  = 500000+N*m0+1.0/6*(1-t*t+Eta2)*N*m2*m0+1.0/120*(5-18*t*t+pow(t,4)+14*Eta2-58*Eta2*t*t)*N*pow(m2,3);

}


Vector3d CTF::coord_rot_translation(double rot_angle, Vector2d coord, double  RT_curr_offset_x, double RT_curr_offset_y, double RT_curr_std_error )//, Vector2d coord_res2)
{
    Vector2d coord_res1;
    Vector3d coord_res2;
    Matrix2d rot_matrix;
    rot_matrix << cos(rot_angle), -sin(rot_angle), sin(rot_angle), cos(rot_angle);
    coord_res1 = rot_matrix * coord;
    coord_res2(0)  = coord_res1(0) + RT_curr_offset_x;
    coord_res2(1)  = coord_res1(1) + RT_curr_offset_y;
    coord_res2(2)  = RT_curr_std_error;
    return coord_res2;
}

Vector3d CTF::coord_projection(Vector2d coord)
{
//std::cout << "coord::::" <<setiosflags(ios::fixed)<<setprecision(10)<< coord(0) << std::endl;
//std::cout << "coord::::" <<setiosflags(ios::fixed)<<setprecision(10)<< coord(1) << std::endl;
    int m_x, m_y;
    Vector2d coord_tr1;
    Vector3d coord_res;
    coord_tr1(0) = coord(0) - min_gpsx;
    coord_tr1(1) = coord(1) - min_gpsy;   // 整体一次平移

    m_y = (int)coord_tr1(0);// + 1;
    m_x = (int)coord_tr1(1);// + 1;  // 计算索引
 //   std::cout << " m_x " << m_x << " m_y " << m_y << std::endl;
 //   std::cout <<" offset : " <<  offset_x[m_x][m_y] << " " << offset_y[m_x][m_y] << std::endl;

// 判断索引  rot_angle  offset
    int x_inc, x_dec, y_inc, y_dec, step_search = 1;
//cout << "theta[m_x][m_y]" << theta[m_x][m_y] << endl;
    while(abs(theta[m_x][m_y]) < 0.00001 && step_search <= 132)
    {
        x_inc = m_x + step_search;
        x_dec = m_x - step_search;
        y_inc = m_y + step_search;
        y_dec = m_y - step_search;
        if(x_inc > x_max)
            x_inc = x_max;
        if(x_dec < x_min)
            x_dec = x_min;
        if(y_inc > y_max)
            y_inc = y_max;
        if(y_dec < y_min)
            y_dec = y_min;

        if(theta[x_inc][m_y] != 0.0000000)
        {
            theta[m_x][m_y]    = theta[x_inc][m_y];
            offset_x[m_x][m_y] = offset_x[x_inc][m_y];
            offset_y[m_x][m_y] = offset_y[x_inc][m_y];
            std_error[m_x][m_y] = std_error[x_inc][m_y];
        }
        else if(theta[m_x][y_dec] != 0.0000000)
        {
            theta[m_x][m_y]    = theta[m_x][y_dec];
            offset_x[m_x][m_y] = offset_x[m_x][y_dec];
            offset_y[m_x][m_y] = offset_y[m_x][y_dec];
            std_error[m_x][m_y] = std_error[m_x][y_dec];
        }
        else if(theta[x_dec][m_y] != 0.0000000)
        {
            theta[m_x][m_y]    = theta[x_dec][m_y];
            offset_x[m_x][m_y] = offset_x[x_dec][m_y];
            offset_y[m_x][m_y] = offset_y[x_dec][m_y];
            std_error[m_x][m_y] = std_error[x_dec][m_y];
        }
        else if(theta[m_x][y_inc] != 0.0000000)
        {
            theta[m_x][m_y]    = theta[m_x][y_inc];
            offset_x[m_x][m_y] = offset_x[m_x][y_inc];
            offset_y[m_x][m_y] = offset_y[m_x][y_inc];
            std_error[m_x][m_y] = std_error[m_x][y_inc];
        }
        else
            step_search = step_search + 1;
    }

    if(step_search <= 132)
    {
        RT_curr_theta    = theta[m_x][m_y];
        RT_curr_offset_x = offset_x[m_x][m_y];
        RT_curr_offset_y = offset_y[m_x][m_y];
        RT_curr_std_error = std_error[m_x][m_y];
    }

    Vector2d offset;
    offset << offset_x[m_x][m_y], offset_y[m_x][m_y];
 //   std::cout << " m_x " << m_x << " m_y " << m_y << std::endl;
 //   std::cout <<" offset : " <<  offset_x[m_x][m_y] << " " << offset_y[m_x][m_y] << std::endl;
//    std::cout <<" theta : " << theta[m_x][m_y] << std::endl;

    // 旋转 平移
//    CTF C;
//    coord_res = C.coord_rot_translation(RT_curr_theta , coord_tr1, RT_curr_offset_x, RT_curr_offset_y);
    coord_res = coord_rot_translation(RT_curr_theta , coord_tr1, RT_curr_offset_x, RT_curr_offset_y, RT_curr_std_error);
    return coord_res;
}


