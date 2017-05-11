#include "tk_kalman.h"
#include "tk_global.h"
#include <iostream>
using namespace std;

MatrixXd KF::kalman_initPk()
{
    // Initialise error covariance matrix Pk
    MatrixXd Pk;
    Pk.setZero(5,5);
    Pk = 1000 * MatrixXd::Identity(5,5);
    return Pk.matrix();
}

void KF::kalman_initHkRk_xy(double theta, double lat_std, double lon_std, kf_HkRk *HR)
{
    // Initialise measurement matrix Hk
    HR->Hk.setZero(2,5);
    HR->Hk.block<2,2>(0,0) = MatrixXd::Identity(2,2);
    HR->Hk(0,2) =  ly * cos(theta) - lx * sin(theta);
    HR->Hk(1,2) = -ly * sin(theta) - lx * cos(theta);

    // Initialise measurement noise covariance matrix Rk
    HR->Rk.setZero(2,2);
    HR->Rk(0,0) =  pow(lon_std,2);//pow(0.1,2);
    HR->Rk(1,1) =  pow(lat_std,2);//pow(0.1,2);
}
/*
void KF::kalman_initHkRk_xyh(double theta, kf_HkRk *HR)
{
    HR->Hk.setZero(3,5);
    HR->Hk.block<3,3>(0,0) = MatrixXd::Identity(3,3);
    HR->Hk(0,2) =  ly * cos(theta) - lx * sin(theta);
    HR->Hk(1,2) = -ly * sin(theta) - lx * cos(theta);


    HR->Rk.setZero(3,3);
    HR->Rk(0,0) =  pow(0.1,2);
    HR->Rk(1,1) =  pow(0.1,2);
    HR->Rk(2,2) =  pow(2.0/180*pi,2);
}
*/
void KF::kalman_initFtQt(const double L1, const double L2, const double dx, const double dy,
                   const double theta, kf_FtQt *FQ)
{
    // Construct transition matrix Ft
    FQ->Ft.setZero(5,5);
    FQ->Ft.block<5,5>(0,0) =  MatrixXd::Identity(5,5);
    FQ->Ft(0,2) = -(dx*sin(theta) + dy*cos(theta));
    FQ->Ft(0,3) =  k1*L1/2.0*(L1*k1*cos(theta)/Len - sin(theta));
    FQ->Ft(0,4) = -k2*L2/2.0*(L2*k2*cos(theta)/Len + sin(theta));
    FQ->Ft(1,2) =  dx*cos(theta) - dy*sin(theta);
    FQ->Ft(1,3) =  k1*L1/2.0*(L1/Len*k1*sin(theta) + cos(theta));
    FQ->Ft(1,4) = -k2*L2/2.0*(L2/Len*k2*sin(theta) - cos(theta));
    FQ->Ft(2,3) = -k1*L1/Len;
    FQ->Ft(2,4) =  k2*L2/Len;

    // Initialise system noise covariance matrix Qt
    FQ->Qt.setZero(5,5);
    FQ->Qt.block<2,2>(3,3) = 0.00002 * MatrixXd::Identity(2,2);
}
/*
void KF::kalman_update_xyh(MatrixXd Fk, MatrixXd Qk, MatrixXd Pk, MatrixXd Rk, Vector3d Zk, MatrixXd Hk, kf_XkPk *XP)
{
    MatrixXd Pkk_1, Pxz, Pzz, Kk;
    Kk.setZero(5,3);
    Pkk_1.setZero(5,5);
    Pxz.setZero(5,3);
    Pzz.setZero(3,3);

    Pkk_1 = Fk*Pk*Fk.transpose() + Qk; // prediction error covariance matrix

    Pxz = Pkk_1 * Hk.transpose();
    Pzz = Hk * Pxz + Rk;
    Kk = Pxz * Pzz.inverse();

    XP->Xk = Kk*Zk; // state vector update
    XP->Pk = Pkk_1 - Kk*Pzz*Kk.transpose();// error covariance matrix update
}
*/
void KF::kalman_update_xy(MatrixXd Fk, MatrixXd Qk, MatrixXd Pk, MatrixXd Rk, Vector2d Zk, MatrixXd Hk, kf_XkPk *XP)
{
    MatrixXd Pkk_1, Pxz, Pzz, Kk;
    Kk.setZero(5,2);
    Pkk_1.setZero(5,5);
    Pxz.setZero(5,2);
    Pzz.setZero(2,2);

    Pkk_1 = Fk*Pk*Fk.transpose() + Qk; // prediction error covariance matrix
//	cout << "Fk = \n" << Fk << endl;
//    cout << "Qk = \n" << Qk << endl;
//    cout << "Pk = \n" << Pk << endl;
//cout << " Pkk_1" << Pkk_1 << endl;
    Pxz = Pkk_1 * Hk.transpose();
//	cout << "Pxz " << Pxz << endl;
    Pzz = Hk * Pxz + Rk;
//cout << "Pzz " << Pzz << endl;
    Kk = Pxz * Pzz.inverse();
//cout << "Kk " << Kk << endl;
    XP->Xk = Kk*Zk; // state vector update
//cout << " Xk " << XP->Xk << endl;
    XP->Pk = Pkk_1 - Kk*Pzz*Kk.transpose();// error covariance matrix update
//cout << " Pk " << XP->Pk << endl;
}
