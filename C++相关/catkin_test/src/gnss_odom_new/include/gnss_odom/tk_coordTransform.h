#ifndef TK_COORDTRANSFORM_H
#define TK_COORDTRANSFORM_H
#include <Eigen/Eigen>
using namespace Eigen;

/**
  * @brief �˽ṹ�����ڴ洢�ο��������
  */
typedef struct CoordSys{
/// ���򳤰뾶
    double A;
/// �������
    double Alfa;
/// ��һƫ����
    double E2;
/// ͶӰ��ʽѡ�� UTMͶӰ�͸�˹ͶӰ
    double UTM;
}CT;

/**
 * @class CTF
 * @brief �����װ������ת������ͼӳ����غ���
 */
class CTF{
public:
  /**
    * @brief ��˹3�ȴ�ͶӰ
    * @param B γ�ȣ���λ rad��
    * @param L ���ȣ���λ rad��
    * @return X ͶӰ��ƽ��X����
    * @return Y ͶӰ��ƽ��Y����
    */
    void BLH_XYH(const double B, const double L, double *X, double *Y);

  /**
    * @brief �ȡ����Ȼ���
    * @param deg  ��
    * @param type ���أ�type == 1 ��ת���ȣ�type==0������ת��
    * @return ��λת�������
    */
    double deg2rad(const double deg, const int type);

  /**
    * @brief ����任
    * @param rot_angle ��ת��
    * @param coord ����
    * @param RT_curr_offset_x X��ƽ����
    * @param RT_curr_offset_y Y��ƽ����
    * @return �任������
    */
    Vector3d coord_rot_translation(double rot_angle, Vector2d coord, double RT_curr_offset_x, double RT_curr_offset_y, double RT_curr_std_error);

  /**
    * @brief ��ͼӳ��
    * @param coord ӳ��ǰ����
    * @return ӳ�������
    */
    Vector3d coord_projection(Vector2d coord);
};
#endif // TK_COORDTRANSFORM_H
