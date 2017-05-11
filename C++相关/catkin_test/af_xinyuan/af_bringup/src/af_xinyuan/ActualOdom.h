#pragma once

#include "_Matrix.h"
#include "math.h"
#include <ros/ros.h> 

#define   IMU_ZERO_VALUE 0.5
#define   PI 3.14159265358979
#define   PulseToDistance  PI*0.1016/2000    //单位脉冲数所对应的距离，电机每转动一周所对应的脉冲数为16384
//#define PulseToDistance  PI*0.688/(4000*32) // 32 jiansubi 
//#define   PulseToDistance  PI*0.420/4096


//#define   PulsePerRad 2601.865


class ActualOdom
{
public:
	ActualOdom();
	virtual ~ActualOdom();

public:
        void cal_motion(double thetaL, double thetaR, double yaw);
	void inverse();
	void sumTranslation();
        void printModel();
	
public:
    //机器人运动位移
   	double m_tranX;
	double m_tranY;
	double m_rotation;

    //机器人实时位姿
        double m_sumX;
	double m_sumY;
	double m_sumQ;

    //机器人位姿取逆
        double m_inverX;
	double m_inverY;
	double m_inverQ;

        double m_radius;
        double m_length;
    
    //编码器角速度阈值
    //double m_angleVelThreshold;
};


