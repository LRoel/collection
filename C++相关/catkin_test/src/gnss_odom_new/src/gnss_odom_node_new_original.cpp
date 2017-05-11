#include "ros/ros.h"
#include "af_msgs/Fuse_G_O.h"
#include "af_msgs/Gnss_Odom_lyz.h"
#include "Eigen/Eigen"
#include "main.h"
#include "iostream"

using namespace std;
using namespace Eigen;

af_msgs::Gnss_Odom_lyz output;
static int m = 2;

CTF CTF_;
KF K_;
MatrixXd Pk;

//MatrixXd Fk, Qk;
class feedback{
private:
    MatrixXd Fk, Qk;

public:
    feedback()
    {
        sub_ = n_.subscribe("/fuse",100,&feedback::callback,this);
        pub_ = n_.advertise<af_msgs::Gnss_Odom_lyz>("/Gnss_Odom_res1",100);
    }

    void callback(const af_msgs::Fuse_G_O & input)
    {
		double X, Y, X_1, Y_1, val_1, val_2;
        odom_res odom;
        ODOM O_;
        kf_FtQt FQ;
        kf_XkPk XP;
        kf_HkRk HR;

//        KF K_;
//        MatrixXd Pk;
        Vector2d Zk2;
        double Gpsx, Gpsy;
		Vector2d coord;
		Vector3d coord_res;
        
		if(m==2)
	    {
			CTF_.BLH_XYH(CTF_.deg2rad(input.lat,1), CTF_.deg2rad(input.lon,1), &Y, &X);
			output.Hdg = 2*pi - input.heading/180.0*pi;
			//	cout << " X " << X << " Y " << Y << endl;
			//	ROS_ERROR("output.heading :===============%lf", output.Hdg);

		    //CTF_.BLH_XYH(CTF_.deg2rad(39.95883266,1), CTF_.deg2rad(116.2957583,1), &Y, &X);   // cut_1_6000
			//output.Hdg = 2*pi - 79.83813477/180.0*pi;
			//cout << input.heading << endl;
		}
		// Odometry update----------------------------------------------------------
        O_.odom_update(input.left_encoder_val, input.right_encoder_val, X, Y, output.Hdg, &odom);
//		cout << " output.Hdg : " << output.Hdg << endl;
//		cout << " X : " << X << endl;

        XP.Xk.setZero(5,1);
//        Pk = K_.kalman_initPk();
        K_.kalman_initFtQt(input.left_encoder_val, input.right_encoder_val, odom.dx, odom.dy, output.Hdg, &FQ);
        if(m == 2)
        {
            Fk = FQ.Ft;
            Qk = FQ.Qt;
        }
        else
        {
            Fk = FQ.Ft * Fk;
            Qk = FQ.Ft * Qk * FQ.Ft.transpose() + FQ.Qt;
        }

        ROS_WARN("encoder_count = %d, gps_count = %d",input.encoder_time, input.gps_time);
        // GNSS/ODOM Integration ------------------------------------------------------
        if(input.Slo_stat == 0 && abs(input.encoder_time - input.gps_time) < 50 && input.lat_std < 1 && input.lon_std < 1) 
        {
           cout << "x y  **************************************************" << endl;
           CTF_.BLH_XYH(CTF_.deg2rad(input.lat,1), CTF_.deg2rad(input.lon,1), &Gpsy, &Gpsx);

            Zk2(0) = odom.x     - Gpsx + lx*cos(odom.theta) + ly*sin(odom.theta);
            Zk2(1) = odom.y     - Gpsy - lx*cos(odom.theta) + ly*sin(odom.theta);

            K_.kalman_initHkRk_xy(odom.theta,input.lat_std, input.lon_std, &HR);
			K_.kalman_update_xy(Fk,Qk,Pk,HR.Rk,Zk2,HR.Hk,&XP);
            Fk.setIdentity(5,5);
            Qk.setZero(5,5);

            Pk = (XP.Pk + XP.Pk.transpose())/2.0;

			X_1       = odom.x	  - XP.Xk(0,0);
        	Y_1       = odom.y 	  - XP.Xk(1,0);
			val_1 = sqrt(X_1*X_1 + Y_1*Y_1);

//			output.X_std = sqrt(XP.Pk(0,0));
//			output.Y_std = sqrt(XP.Pk(1,1));
//        	output.Hdg_std = sqrt(XP.Pk(2,2));
        } 
		val_2 = sqrt(odom.x*odom.x + odom.y*odom.y);
//		if(abs(val_1 - val_2) > 3)
//		{
//			Pk = Pk*2;
//		}
		X         = odom.x	  - XP.Xk(0,0);
        Y         = odom.y 	  - XP.Xk(1,0);
        output.Hdg 	 = odom.theta - XP.Xk(2,0);

        if(output.Hdg < 0)
            output.Hdg = output.Hdg + 2*pi;
        else if(output.Hdg > 2*pi)
            output.Hdg = output.Hdg - 2*pi;
//cout << " output.Hdg : " << output.Hdg << endl;
//cout << Pk << endl;
		coord(0) = X;
		coord(1) = Y;

        coord_res = CTF_.coord_projection(coord);

		output.X = coord_res(0);
		output.Y = coord_res(1);   // 映射后结果

 		output.X_std = sqrt(Pk(0,0)) + coord_res(2);
		output.Y_std = sqrt(Pk(1,1)) + coord_res(2);
       	output.Hdg_std = sqrt(Pk(2,2));

//		output.X = coord(0);
//		output.Y = coord(1);	   // 融合结果
//cout << "odom.x" << odom.x << endl;
//cout << " X " << X << endl;

//ROS_ERROR("output.x = %lf", output.X);
		m++;
        pub_.publish(output);
    }

private:
    ros::Publisher pub_;
    ros::Subscriber sub_;
    ros::NodeHandle n_;
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "gnss_odom_node_new");
    feedback FD;
    Pk = K_.kalman_initPk();
//    CTF_.BLH_XYH(CTF_.deg2rad(39.9590702847000,1), CTF_.deg2rad(116.296080722000,1), &Y, &X);
//    output.Hdg = 11.28284836/180.0*pi;
    ros::spin();
    return 0;
}


