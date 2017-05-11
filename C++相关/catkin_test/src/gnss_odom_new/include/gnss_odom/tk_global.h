#ifndef TK_GLOBAL_H
#define TK_GLOBAL_H

/// 轮距
extern const double Len;
/// 左轮标定因子
extern const double k1;
/// 右轮标定因子
extern const double k2;
/// 圆周率
extern const double pi;

/// 杆臂补偿系数
extern const double lx;
/// 杆臂补偿系数
extern const double ly;
/// 杆臂补偿系数
extern const double lz;

/// 数组索引中行最大值
extern const int x_max;
/// 数组索引中列最大值
extern const int y_max;
/// 数组索引中行最小值
extern const int x_min;
/// 数组索引中行最小值
extern const int y_min;

/// 高斯投影后gps x向最小值（用于初始整体一次平移，索引RT）
extern const double min_gpsx;
/// 高斯投影后gps y向最小值（用于初始整体一次平移，索引RT）
extern const double min_gpsy;

/// RT矩阵旋转角度
extern double theta[132][123];
/// RT矩阵X轴平移量
extern double offset_x[132][123];
/// RT矩阵Y轴平移量
extern double offset_y[132][123];
/// 标准差误差
extern double std_error[132][123];


///// RT矩阵旋转角度
//extern double theta[117][55];
///// RT矩阵X轴平移量
//extern double offset_x[117][55];
///// RT矩阵Y轴平移量
//extern double offset_y[117][55];

extern double RT_curr_offset_x;
extern double RT_curr_offset_y;
extern double RT_curr_theta;
extern double RT_curr_std_error;

#endif // TK_GLOBAL_H
