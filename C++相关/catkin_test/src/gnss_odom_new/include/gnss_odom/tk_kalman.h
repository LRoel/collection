#ifndef TK_KALMAN_H
#define TK_KALMAN_H
#include <iostream>
#include <Eigen/Eigen>
using namespace std;
using namespace Eigen;

typedef struct{
//    MatrixXd Hk;
    MatrixXd Ft;
    MatrixXd Qt;
//    MatrixXd Rk;

//    double Hk[3][5];
//    double Ft[5][5];
//    double Qt[5][5];
//    double Rk[3][3];
// //    double Pk[5][5];
}kf_FtQt;

typedef struct{
    MatrixXd Hk;
    MatrixXd Rk;
}kf_HkRk;

typedef struct{
    VectorXd Xk;
    MatrixXd Pk;

//    double Xk[5][1];
//    double Pk[5][5];
}kf_XkPk;

class KF{
private:
public:
    MatrixXd kalman_initPk();
    void 	 kalman_initHkRk_xy(double theta, double lat_std, double lon_std, kf_HkRk *HR);
    //void     kalman_initHkRk_xyh(double theta, kf_HkRk *HR);
    void     kalman_initFtQt(const double L1, const double L2, const double dx, const double dy,
                       const double theta, kf_FtQt *FQ);
    //void kalman_update_xyh(MatrixXd Fk, MatrixXd Qk, MatrixXd Pk, MatrixXd Rk, Vector3d Zk, MatrixXd Hk, kf_XkPk *XP);
    void kalman_update_xy(MatrixXd Fk, MatrixXd Qk, MatrixXd Pk, MatrixXd Rk, Vector2d Zk, MatrixXd Hk, kf_XkPk *XP);

};




#endif // TK_KALMAN_H
