#ifndef TK_COORDTRANSFORM_H
#define TK_COORDTRANSFORM_H
#include <Eigen/Eigen>
using namespace Eigen;

/**
  * @brief 此结构体用于存储参考椭球参数
  */
typedef struct CoordSys{
/// 椭球长半径
    double A;
/// 地球扁率
    double Alfa;
/// 第一偏心率
    double E2;
/// 投影方式选择 UTM投影和高斯投影
    double UTM;
}CT;

/**
 * @class CTF
 * @brief 此类封装了坐标转换、地图映射相关函数
 */
class CTF{
public:
  /**
    * @brief 高斯3度带投影
    * @param B 纬度（单位 rad）
    * @param L 经度（单位 rad）
    * @return X 投影后平面X坐标
    * @return Y 投影后平面Y坐标
    */
    void BLH_XYH(const double B, const double L, double *X, double *Y);

  /**
    * @brief 度、弧度互换
    * @param deg  度
    * @param type 开关，type == 1 度转弧度，type==0，弧度转度
    * @return 单位转换后变量
    */
    double deg2rad(const double deg, const int type);

  /**
    * @brief 坐标变换
    * @param rot_angle 旋转角
    * @param coord 坐标
    * @param RT_curr_offset_x X轴平移量
    * @param RT_curr_offset_y Y轴平移量
    * @return 变换后坐标
    */
    Vector3d coord_rot_translation(double rot_angle, Vector2d coord, double RT_curr_offset_x, double RT_curr_offset_y, double RT_curr_std_error);

  /**
    * @brief 地图映射
    * @param coord 映射前坐标
    * @return 映射后坐标
    */
    Vector3d coord_projection(Vector2d coord);
};
#endif // TK_COORDTRANSFORM_H
