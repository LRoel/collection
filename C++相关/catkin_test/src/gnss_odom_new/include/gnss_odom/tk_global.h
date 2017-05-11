#ifndef TK_GLOBAL_H
#define TK_GLOBAL_H

/// �־�
extern const double Len;
/// ���ֱ궨����
extern const double k1;
/// ���ֱ궨����
extern const double k2;
/// Բ����
extern const double pi;

/// �˱۲���ϵ��
extern const double lx;
/// �˱۲���ϵ��
extern const double ly;
/// �˱۲���ϵ��
extern const double lz;

/// ���������������ֵ
extern const int x_max;
/// ���������������ֵ
extern const int y_max;
/// ��������������Сֵ
extern const int x_min;
/// ��������������Сֵ
extern const int y_min;

/// ��˹ͶӰ��gps x����Сֵ�����ڳ�ʼ����һ��ƽ�ƣ�����RT��
extern const double min_gpsx;
/// ��˹ͶӰ��gps y����Сֵ�����ڳ�ʼ����һ��ƽ�ƣ�����RT��
extern const double min_gpsy;

/// RT������ת�Ƕ�
extern double theta[132][123];
/// RT����X��ƽ����
extern double offset_x[132][123];
/// RT����Y��ƽ����
extern double offset_y[132][123];
/// ��׼�����
extern double std_error[132][123];


///// RT������ת�Ƕ�
//extern double theta[117][55];
///// RT����X��ƽ����
//extern double offset_x[117][55];
///// RT����Y��ƽ����
//extern double offset_y[117][55];

extern double RT_curr_offset_x;
extern double RT_curr_offset_y;
extern double RT_curr_theta;
extern double RT_curr_std_error;

#endif // TK_GLOBAL_H
