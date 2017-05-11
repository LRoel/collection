#ifndef TK_ODOM_H
#define TK_ODOM_H

typedef struct{
    double x;
    double y;
    double theta;
    double dx;
    double dy;
}odom_res;

class ODOM{
private:
public:
    void odom_update(double L1, double L2, double x, double y, double theta, odom_res *odom);

};

#endif // TK_ODOM_H
