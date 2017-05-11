#include "ActualOdom.h"

extern double left_fix_;
extern double right_fix_;
extern double length_;

ActualOdom::ActualOdom()
{
	m_tranX = 0.0;
	m_tranY = 0.0;
	m_rotation = 0.0;

	m_sumX = 0.0;
	m_sumY = 0.0;
	m_sumQ = 0.0;

	m_inverX = 0.0;
	m_inverY = 0.0;
	m_inverQ = 0.0;

    //m_radius = 0.1524/2;    //轮子直径120mm
	m_radius = 0.430/2;
	m_length = 0.710;   //中心轴距424mm
	//m_radius = 0.420/2;    //轮子直径120mm
	//m_length = 0.700;   //中心轴距424mm

	//m_angleVelThreshold = maxVelocity / m_radius;
}

ActualOdom::~ActualOdom()
{

}

void ActualOdom::inverse()
{
	_Matrix_Calc matrixCalc;
	_Matrix A =  _Matrix(3,3);
	_Matrix B = _Matrix(3,3);

         A.init_matrix();
	 B.init_matrix();

	 A.write(0, 0, cos(m_sumQ));
	 A.write(0, 1, -sin(m_sumQ));
	 A.write(0, 2, m_sumX);
	 A.write(1, 0, sin(m_sumQ));
	 A.write(1, 1, cos(m_sumQ));
	 A.write(1, 2, m_sumY);
	 A.write(2, 0, 0);
	 A.write(2, 1, 0);
	 A.write(2, 2, 1);

	 matrixCalc.inverse(&A, &B);


	 m_inverX = B.read(0,2);
	 m_inverY = B.read(1,2);
	 m_inverQ = -m_sumQ;
}

//计算运动位移
void ActualOdom::cal_motion(double thetaL, double thetaR, double thetaYaw)
{

  double l_tran = left_fix_ * thetaL * PulseToDistance;
  double r_tran = right_fix_ * thetaR * PulseToDistance;
  
	m_tranX = (l_tran+r_tran)*cos(+m_sumQ)/2.0;
	m_tranY = (l_tran+r_tran)*sin(+m_sumQ)/2.0;
	
	ROS_INFO("set configure: %f %f", 
            left_fix_, right_fix_);
    
	m_rotation = (r_tran - l_tran)/length_;

	/*if (m_rotation < IMU_ZERO_VALUE)  //若运动量过小，以里程计为准，否则以IMU为准。
	{
		m_rotation = m_rotation * 180.0/ PI;
	}
	else
	{
		m_rotation = thetaYaw;
	}*/
}

void ActualOdom::sumTranslation()
{
	m_sumX += m_tranX;
	m_sumY += m_tranY;

	m_sumQ += m_rotation;

        if(m_sumQ > PI)
        {
		m_sumQ -= 2.0*PI;
	}
        else
        {
		 if(m_sumQ <= -PI)
		 {
			m_sumQ += 2.0*PI;
		 }
        }

        inverse();
}

void ActualOdom::printModel()
{
	printf("org=%f %f %f", m_sumX, m_sumY, m_sumQ);
	printf("\r\n");
	printf("inv=%f %f %f", m_inverX, m_inverY, m_inverQ);
	printf("\r\n");

}
