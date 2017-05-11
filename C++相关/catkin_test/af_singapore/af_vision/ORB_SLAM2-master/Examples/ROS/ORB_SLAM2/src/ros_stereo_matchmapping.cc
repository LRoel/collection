/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <tf/tf.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <iomanip>
#include <complex>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>
#include "Converter.h"

#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/PoseStamped.h"
#include"../../../include/System.h"

#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "Eigen/Eigen"  
#include "Eigen/Dense"

using namespace Eigen; 
using namespace std;

ros::Time current_time, last_time;

MatrixXd TpsCx(134, 2);
MatrixXd TpsC(134, 2);
MatrixXd TpsMpt(1, 2);
MatrixXd TpsD(3, 2);
double TpsScale;

MatrixXd Data_xy(1, 2);  // (Xi, Yi)
MatrixXd xy_a(1, 2);  
MatrixXd xy_A(134, 2);
MatrixXd xy_b(134, 2);
MatrixXd xy_B(134, 2);
MatrixXd xy_c(134, 1);
MatrixXd xy_C(134, 1);
MatrixXd xy_d(1, 134);
MatrixXd xy_E(1, 3);

MatrixXd xy_m(1, 2);
MatrixXd xy_g(1, 2);
MatrixXd OutPut(1, 2); // the output data x, y

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    void PublishPose(cv::Mat Tcw);

    ORB_SLAM2::System* mpSLAM;

    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;

    /******************* pub slam_odom topic ********************/
    ros::Publisher* pSlamPosPub;   

    tf::TransformBroadcaster odom_broadcaster;

    //test
    int sj_TR;
    int sj_TRp;
    double cov_err_TR;
    double cov_depth_TR;

    int sj_TM;
    int sj_TMp;
    double cov_err_TM;
    double cov_depth_TM;

    int sj_TLM;
    double cov_err_TLM;
    double cov_depth_TLM;
private:
    float x_x_tmp = 0.0;
    float x_y_tmp = 0.0;
    float y_tmp = 0.0;
    float z_tmp = 0.0;
    double yaw_tmp = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    double tmp_time = 0.0;
    double current_tmp_time = 0.0;

    double x_x = 0.0;
    double x_y = 0.0;
    double vx = 0.0;
    double vy = -0.0;
    double vth = 0.0;
    /******************* pub slam_odom topic  end ********************/
};


    /******************* pub slam_odom topic ********************/
 /******************* geometry_msgs::PoseStamped ********************/
/*
void ImageGrabber::PublishPose(cv::Mat Tcw)
{
    geometry_msgs::PoseStamped pose_slam;
    if(!Tcw.empty())
    {
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(1,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
    
        vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);

        pose_slam.pose.position.x = twc.at<float>(0);
        pose_slam.pose.position.y = twc.at<float>(2);
        pose_slam.pose.position.z = twc.at<float>(1);
        pose_slam.pose.orientation.x = q[0];
        pose_slam.pose.orientation.y = q[1];
        pose_slam.pose.orientation.z = q[2];
        pose_slam.pose.orientation.w = q[3];
        pose_slam.header.frame_id = "VSLAM";
        pose_slam.header.stamp = ros::Time::now();
        //cout << "PublishPose position.x = " << pose_slam.pose.position.x << endl;

        (pSlamPosPub)->publish(pose_slam);

    }
}
*/

 /******************* pub slam_odom topic  end ********************/



int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Stereo path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    ImageGrabber igb(&SLAM);

    if(argv[3])
    {
        igb.do_rectify = true;

        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/camera/right/image_raw", 1);


    /******************* pub slam_odom topic ********************/
    //ros::Publisher SlamPosPub = nh.advertise<geometry_msgs::PoseStamped>("ORB_SLAM/pose", 5);

    ros::Publisher SlamPosPub = nh.advertise<nav_msgs::Odometry>("/odom", 10);


    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));

    /******************* pub slam_odom topic ********************/
    igb.pSlamPosPub = &(SlamPosPub);
    /******************* pub slam_odom topic  end ********************/

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");


    SLAM.SaveMap("Slam_latest_Map.bin");
    //SLAM.LoadMap("Slam_Map.bin");
    ros::shutdown();

    return 0;
}



 /******************* nav_msgs::Odometry ********************/

void ImageGrabber::PublishPose(cv::Mat Tcw)
{

    nav_msgs::Odometry pose_slam;

 /******************* pub slam_odom topic  end ********************/
    current_time = ros::Time::now();

    cv::Mat A(3,3,CV_32F);

  /************ z(90)*x(90) ***********/

    A.at<float>(0,0) = 0;
    A.at<float>(1,0) = -1;
    A.at<float>(2,0) = 0;

    A.at<float>(0,1) = 0;
    A.at<float>(1,1) = 0;
    A.at<float>(2,1) = -1;

    A.at<float>(0,2) = 1;
    A.at<float>(1,2) = 0;
    A.at<float>(2,2) = 0;


  /************ z(-90)y(-90)*z(90) ***********/
/*
    A.at<float>(0,0) = 1;
    A.at<float>(1,0) = 0;
    A.at<float>(2,0) = 0;

    A.at<float>(0,1) = 0;
    A.at<float>(1,1) = 0;
    A.at<float>(2,1) = -1;

    A.at<float>(0,2) = 0;
    A.at<float>(1,2) = 1;
    A.at<float>(2,2) = 0;
*/

/**
*@brief  匹配映射-->将ORB轨迹映射到amcl;
*@params:  TpsCx, TpsC,,,, TpsMpt, TpsD, TpsScale;
*@input:  ORB:Data_xy的x, y;
*@output:  OutPut的x, y --> 映射匹配的结果;
*/
   TpsCx << 0.378372755625077, -0.234385464859299,
0.306596728565938, 0.125203786583652,
-0.106193653868241, 0.439706887843209,
0.241233075227177, 0.498086464506871,
0.229831953742726, -0.275279980806365,
0.0668270821117552, 0.476013154068399,
-0.0603581165425035, -0.309123743925241,
0.310507344970709, -0.268709417038349,
-0.324301247090343, -0.0360892799545780,
-0.434202855757533, 0.137847076777408,
0.276123924872487, 0.367347144107791,
0.0705807692847158, -0.294289044622264,
-0.355237442534059, 0.376659479941885,
0.328145890361391, -0.0988618990134073,
-0.323680387992795, -0.236759970587586,
-0.287767946564172, -0.321937341130282,
-0.400408070452602, 0.0849609242809641,
0.125335931420680, 0.483280604391473,
-0.146597762835233, -0.313510279669966,
0.340271539344980, -0.153966460205595,
0.310396017821810, 0.0778828402374170,
-0.0104279025913600, 0.468748006375878,
-0.390650088134792, 0.359484063501787,
0.296395527537983, 0.186051782485966,
0.316784614385928, 0.0191150201518692,
-0.358643757965561, 0.0312610173764611,
-0.432921692511437, 0.193166758664745,
-0.214096610296828, -0.322904252004382,
0.320301831480231, -0.0367538890203334,
-0.147119621361344, 0.423084402919023,
-0.0569663802619761, 0.455772136334276,
-0.310324045905989, 0.386440200729231,
-0.312600696121294, -0.0898188957901456,
0.268528600076760, 0.428637845634530,
-0.311547380604977, -0.173434532641397,
0.121284278935124, -0.288329275331844,
-0.199361738290961, 0.410522355514047,
0.260724937332885, 0.470024264018609,
-0.163505836436686, -0.315359716173115,
-0.0374496021983771, -0.306881778637520,
-0.426141148295207, 0.290183163342325,
-0.422442473953903, 0.116819710937378,
-0.232516318183899, -0.322068701511420,
0.0265055549641683, -0.298510487905384,
0.251277947707841, -0.268405038150165,
-0.0897437911003597, 0.445969941021222,
0.303411190288601, 0.149038809114162,
-0.369054473284231, 0.0455811477284037,
0.208933835519027, -0.275453926348366,
0.332793348330759, -0.138923942298652,
0.167014508487207, -0.283032441320540,
0.218099754126649, 0.499137615050119,
0.317797070735464, -0.0199201878610385,
0.291288744963985, 0.239536768925133,
0.362236626178618, -0.211935618120597,
-0.309440943544514, -0.156692711420879,
0.285439764708634, 0.285095953121467,
-0.215987652131224, 0.406110666823091,
-0.317920834874539, -0.0570806296605040,
0.346550310474778, -0.177144607503633,
-0.0347536230585604, 0.461833258882908,
0.350475243231661, -0.258901546492530,
-0.334742284540704, -0.0109306456969903,
-0.373325451267236, 0.371226374666857,
0.144121411751786, 0.485718138355671,
-0.329193271885422, 0.382955771081128,
-0.324014857498742, -0.273576870018837,
-0.408599413474780, 0.333497583742774,
0.312116582919272, 0.0501482389084796,
0.0994315704515749, -0.289538290551539,
-0.174876122702907, 0.416055324846353,
-0.112112551478235, -0.311050217632460,
-0.271195149639822, 0.395099390590011,
0.281657044375923, 0.326893390941185,
0.326118074345605, -0.0708474030104359,
0.0510145472504954, -0.299570148167318,
-0.315020121903534, -0.189993082043608,
0.00898219112538282, 0.472584738396099,
0.165371106519243, 0.489869429641400,
0.187970843200716, -0.279667316683834,
-0.184007298505079, -0.319835291502618,
-0.308410635555878, -0.132379567817179,
0.271406036462758, 0.402468356856073,
-0.249721880773216, 0.398761823539287,
0.0443972644988669, 0.475978664677807,
0.369742949323350, -0.249568924989763,
0.287673465870064, -0.271379998421130,
-0.430580543631395, 0.213426147380509,
-0.0915466877303699, -0.308690755649464,
0.278474008958133, 0.348650540284663,
0.331251937849744, -0.119740182082643,
0.149317446262816, -0.284798380114987,
-0.425235063434981, 0.254235857093234,
0.266252400376485, 0.451040382089105,
0.356415327538748, -0.195798950559488,
0.298858891002958, 0.168684597021701,
-0.435472806197225, 0.156484763310481,
-0.325365324898855, -0.254120523475979,
-0.293148291580234, 0.390735606610879,
0.308479979469954, 0.0975879959508620,
-0.379718540462267, 0.0587228071206252,
-0.347668485353760, 0.0137488413151493,
-0.00442036624496023, -0.302552532542843,
0.293384088103418, 0.208533759671661,
0.287998286855694, 0.268350427669903,
0.100651238462914, 0.480192076913384,
-0.266373996414490, -0.325224975896670,
-0.315520074160197, -0.299736125234105,
-0.128186866574747, -0.315356304464325,
-0.309453144979779, -0.106863738098575,
-0.123590423120552, 0.432167376793472,
-0.428321651197926, 0.234426282120089,
-0.249099002181727, -0.323034138339072,
0.282764208894155, 0.306565624129577,
-0.320558847809762, -0.216026027371374,
0.333369005802562, -0.264567880643846,
0.319934061441546, -0.00313303987093790,
-0.426177790144402, 0.272897470953697,
-0.420933125091549, 0.310313254384715,
0.188278344405075, 0.494099260629009,
-0.435428981143862, 0.174326640931037,
0.269649630431253, -0.267250776091265,
-0.411620851828965, 0.0980847633206613,
0.273457028948644, 0.385071337318546,
-0.303170951191546, -0.313564321861641,
0.324229517381057, -0.0533401325516700,
-0.0209061200055677, -0.305465459463579,
-0.390519426403502, 0.0717541655362053,
-0.232999869677410, 0.401937300324675,
0.253540431742985, 0.486945840542554,
0.0259724210593218, 0.474706061207439,
0.0842988370068503, 0.477640913169733,
-0.0740723423761076, 0.450895466577213,
-0.314115751500527, -0.0734402393480268;

   TpsC << -0.732214677821882, -0.224404819676537,
0.323192067227750, 0.176088753539958,
0.0157413268287493, -0.118512336891681,
-0.347043703222538, 0.244895201048489,
-1.00924662001814, -0.847829069714421,
-0.0409992466362696, -0.159330838776629,
0.00208664644516980, 0.203461294216826,
0.420846662595629, -0.377754973299618,
-0.366548675390132, 0.941268609312534,
2.67932698916446, -1.21888313148673,
-0.0657573534658305, 0.655030842429351,
0.232583080712013, -0.192874577121995,
-0.590360587730257, 0.765639874839137,
0.176832430302529, 0.0647595348201952,
-0.128602061486634, -0.276905684474849,
-0.318192225539832, -0.375445040709427,
-0.442162519010111, 1.84481776349654,
-0.139628011848094, -0.215072269715277,
-0.0377081377652013, 0.283466278420583,
0.0577770020140848, -0.0379866495560229,
0.200586657047042, -0.0560530462497870,
0.133387943526645, -0.208745115076902,
-0.391146787426717, 0.239719737226853,
0.179884334618991, 0.180986424079380,
0.255207759808020, -0.125559873006726,
-0.607457652186971, 3.01889573935549,
1.00095058588914, -3.20178985088354,
-0.0846329880158028, -0.139308260972267,
0.132474890304838, -0.0454946479231670,
-0.110912160567895, -0.211163231676160,
0.0152520441719755, -0.133782196882861,
-0.210925477693595, 0.217247917703028,
-0.138916847467831, 0.219433410131433,
-0.0497389472310619, -0.238678320105789,
-0.0430503932620412, -0.0233818190765695,
0.266780449136311, -0.212348401666481,
-0.235968207148601, -0.140782852199100,
-0.182830123640941, 0.147017202595163,
-0.0304635503551370, 0.142491914317790,
-0.0415849965932123, 0.0274029190418577,
0.326443172279484, -0.694158981292987,
0.368588163730046, 3.55008443307824,
-0.162822814474882, -0.160803179803796,
0.0941857568855716, -0.137274602228629,
-0.522805018536859, -0.450380621430566,
0.0108778013204863, -0.118591721436437,
0.448620772944365, 0.495306007412127,
-0.866339081043931, 3.44110957441330,
0.252016230855241, -0.175026622488284,
-0.0620766534801215, -0.130536303767066,
0.162010047183376, -0.105844702786384,
-0.348155715430778, -0.0827342648169388,
0.0648038463260067, -0.125769066608009,
0.215130322343127, 0.623252118944699,
-0.931439916014615, -0.548062965301335,
-0.0716552447473008, 0.138955269255183,
0.0645591011745549, 0.530062413919602,
-0.199371342936445, -0.0952746395411887,
-0.232106348555732, 0.689677731417164,
-0.467515501898849, -0.376302553398933,
0.0776709887971596, -0.217157081621698,
0.380051264868274, -0.181487064800256,
-0.182609078518624, 0.753769736131715,
-0.649749182321466, 0.593805238530330,
-0.155105017587877, -0.204086508129450,
-0.453598013363330, 0.464700097568602,
-0.0480794299034762, -0.434237092965462,
0.0370930044735138, -0.0654345661591924,
0.306500229462026, -0.116973114160477,
0.264150520484779, -0.172698019052019,
-0.220491200981013, -0.162878209314403,
0.00908349195020947, 0.337516286228734,
-0.294265247964223, 0.115977690380546,
0.0844022426747171, 0.867842388495695,
0.120667782024838, 0.0742057949724131,
0.159592684789012, -0.161634360405832,
-0.159916250358490, -0.132238756549358,
0.0519428344528686, -0.0870728223673738,
-0.190518081728797, -0.209326818115455,
0.281135354993138, -0.121322303227859,
-0.0192311023292589, 0.0407894156418428,
-0.145356438113672, 0.366314918471118,
-0.0408393489210287, -0.561397384061036,
-0.250971249583420, 0.0187967738435832,
-0.0226827283264737, -0.105559122554114,
0.321722531258545, -0.280370502537837,
0.321413900866328, -0.222059314751628,
0.953895560159091, -2.69443194031031,
0.0494682841145663, 0.386316221522447,
0.0431153632842814, 0.892827992188013,
0.180095442456435, -0.00570978435237938,
0.241566881778065, -0.211631313447701,
0.754157726342859, -1.31183592107415,
-0.0638642578942780, 0.0924633992609287,
-0.413039220753563, -0.318236967537888,
0.268585966387590, 0.280287724223832,
1.47856108650959, -4.28848490784375,
-0.0121291112604310, -0.329711916473092,
-0.304182389916258, 0.149481941602664,
0.217639562303417, 0.0174464158980300,
-0.409624390047980, 1.40376632705506,
-0.104233490503253, 0.842300951604706,
0.136768722449945, -0.142073141310432,
0.265038272759150, 0.412138628086302,
-0.0227783702343347, 0.569818336581200,
-0.106139648830881, -0.227837224759903,
-0.247024259359617, -0.194120098263048,
-0.422269788603142, -0.522300505039895,
0.00678269086507228, 0.339847891917560,
-0.0820491883921173, 0.545548177118432,
-0.00322778567083925, -0.173171210833914,
0.638202478083871, -1.83078872532879,
-0.137853888134174, -0.0569374764941622,
0.106189083521706, 0.673838343667014,
-0.231818560516098, -0.222388594011984,
0.345788385310251, -0.0760154526710143,
0.227562772122077, -0.0749433812460761,
0.472865920276277, -0.875687883815118,
0.234566409972980, -0.440924528044088,
-0.236325354190262, -0.175600191253226,
0.717069666209098, -2.73553647159936,
0.206610087051385, -0.138110693467468,
-0.785384252539008, 2.54826421367009,
-0.0920077318153888, -0.355551755109054,
-0.378521854718971, -0.436360421488717,
0.146624344362659, -0.00112806220893571,
0.0380008849534280, -0.0765721464830420,
-0.480517080030945, 1.33126803252702,
-0.208559843808916, -0.0472201942857686,
-0.209188763224359, 0.252370215116267,
0.00621990259200382, -0.0977378322770408,
-0.0688541861759683, -0.201299621128257,
0.0128354273743654, -0.114906474742915,
-0.200012127403170, 0.584010463382583;

   TpsMpt << 27.288017900710233, 39.504810980717490;

   TpsD << -0.0190910083463490, 0.304908730270642,
        -121.710273770359, 12.6101642982915,
        -12.4838184208816, -128.101137941808;

   TpsScale = 1.219308931753650e+02;
/********** matchmapping params end ************/

    if(!Tcw.empty())
    {
    	cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
        //A*Rwc;
        vector<float> q = ORB_SLAM2::Converter::toQuaternion(A*Rwc*A.t());
        //geometry_msgs::Quaternion p = ORB_SLAM2::Converter::toQuaternion(Rwc);
        geometry_msgs::Quaternion p;
        p.x = q[0];
        p.y = q[1];
        p.z = q[2];
        p.w = q[3];

        float lx = twc.at<float>(2)/10;
        float ly = -twc.at<float>(0)/10;
        float lz = -twc.at<float>(1)/10;

        Data_xy(0, 0) = lx;
        Data_xy(0, 1) = ly;

/******** begin to process data ********/
        xy_a = (Data_xy - TpsMpt) / TpsScale; //[(Xi, Yi) - Tps.mpt] / Tps.scale

        for(int i=0; i<134; i++)
        {
            xy_A.row(i) = xy_a;  // MATLAB: repmat(xy_a , 134, 1)
        }

       /**
        *@brief : xy_b
        *[xy_a.xi - Cx12, xy_a.yi - Cx13;
        * xy_a.xi - Cx22, xy_a.yi - Cx23;
        *       ......         ]
        */
        xy_b = xy_A - TpsCx;
        /********* all (col -1) is param col, due to Tps has 3 col ,but the first can be ignore **********/
        xy_B = xy_b.array().square(); //(xy_a.xi - Cx12)^2, (xy_a.yi - Cx13)^2; ...

        xy_c = xy_B.col(0) + xy_B.col(1); // (xy_a.xi-Cx12))^2 + (xy_a.yi - Cx13)^2; ...
        xy_C = xy_c.cwiseSqrt();  // MATLAB : xy_C = sqrt(xy_c)
        xy_d = xy_C.adjoint();  //MATLAB : xy_C'
        xy_m = xy_d * TpsC;  //(xy_C'.xi * C12 + xy_C'.xi * C22 + ... , xy_C'.yi * C13 + xy_C'.yi * C23 + ...)

        xy_E(0, 0) = 1;
        xy_E(0, 1) = xy_a(0, 0);
        xy_E(0, 2) = xy_a(0, 1);  //xy_E = (1, xy_a.xi, xy_a.yi)

       /**
        *@brief : xy_g  1 * 2
        *( D12 + (Xi - Tps.mpt.0) * D22 / Tps.scale + (Yi - Tps.mpt.0) * D32 / Tps.scale, 
        *  D13 + (Xi - Tps.mpt.0) * D23 / Tps.scale + (Yi - Tps.mpt.0) * D33 / Tps.scale)
        */
        xy_g = xy_E * TpsD;  //same as: (D12 + xy_a.xi * D22 + xy_a.yi * D32, D13 + xy_a.xi * D23 + xy_a.yi * D33)
        OutPut = xy_g - xy_m; //(x, y)

/******** end to process data ********/


        //first, we'll publish the transform over tf

        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_footprint";

        odom_trans.transform.translation.x = OutPut(0, 0);
        odom_trans.transform.translation.y = OutPut(0, 1);
        odom_trans.transform.translation.z = 0;
         
                /****Quaternion****/
        odom_trans.transform.rotation.x = q[0];
        odom_trans.transform.rotation.y = q[1];
        odom_trans.transform.rotation.z = q[2];
        odom_trans.transform.rotation.w = q[3];
        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        pose_slam.header.stamp = ros::Time::now();
        pose_slam.header.frame_id = "odom";

        //set the position   Point  and Quaternion
        pose_slam.pose.pose.position.x = OutPut(0, 0);
        pose_slam.pose.pose.position.y = OutPut(0, 1);
        pose_slam.pose.pose.position.z = 0;
        pose_slam.pose.pose.orientation = p;

        pose_slam.pose.covariance[0]  = 0.001;
        pose_slam.pose.covariance[7]  = 0.001;
        pose_slam.pose.covariance[14] = 99999;
        pose_slam.pose.covariance[21] = 99999;
        pose_slam.pose.covariance[28] = 99999;
        pose_slam.pose.covariance[35] = 0.1;

        /**
         * @brief test
         * sj_TR: trackreference -- nmatches
         * sj_TRp: trackreference -- nmatchesMap
         * sj_TM: trackmotion -- nmatches
         * sj_TMp: trackmotion -- nmatchesMap
         * sj_TLM: tracklocalmap -- mnMatchesInliers
         * */
        pose_slam.pose.covariance[2] = sj_TR;
        pose_slam.pose.covariance[3] = sj_TRp;
        pose_slam.pose.covariance[4] = cov_err_TR;
        pose_slam.pose.covariance[5] = cov_depth_TR/100;  //dm^2 / 100

        pose_slam.pose.covariance[9] = sj_TM;
        pose_slam.pose.covariance[10] = sj_TMp;
        pose_slam.pose.covariance[11] = cov_err_TM;
        pose_slam.pose.covariance[12] = cov_depth_TM/100;  //dm^2 / 100

        pose_slam.pose.covariance[17] = sj_TLM;
        pose_slam.pose.covariance[18] = cov_err_TLM;
        pose_slam.pose.covariance[19] = cov_depth_TLM/100;  //dm^2 / 100

	//print
        //ROS_WARN("x:%f", OutPut(0, 0));
        //ROS_WARN("y:%f", OutPut(0, 1));
        //ROS_WARN("z:%f",lz);

        x_x = (OutPut(0, 0) - x_x_tmp);
        x_y = (OutPut(0, 1) - x_y_tmp);
        
        current_tmp_time = ros::Time::now().toSec();
        //ROS_WARN("current_t:%f",current_tmp_time);
        //ROS_WARN("last_t:%f",tmp_time);

        double dt = current_tmp_time - tmp_time;
        //double dt = current_time - tmp_time;
        //double dt = 0.05; //s
        //print time
        //ROS_INFO("dt:%f",dt); 

        tmp_time = current_tmp_time;
        //ROS_WARN("last_t:%f",tmp_time);

        vx = sqrt(x_x * x_x + x_y * x_y)/dt;
        //vy = 0;
        //vz = 0
        //double vz = (twc.at<float>(1) - z_tmp) / dt;
       // double delta_th = vth * dt;
         
        x_x_tmp = OutPut(0, 0);
        x_y_tmp = OutPut(0, 1);
        //z_tmp = twc.at<float>(1);

        yaw = tf::getYaw(p);
        vth = (yaw - yaw_tmp) / dt;
        yaw_tmp = yaw;

        //print
        //ROS_INFO("yaw%f",yaw);
        //last_time = ros::Time::now();
        //set the velocity

        pose_slam.child_frame_id = "base_footprint";
        pose_slam.twist.twist.linear.x = vx;
        pose_slam.twist.twist.linear.y = 0;
        pose_slam.twist.twist.linear.z = 0;
        pose_slam.twist.twist.angular.z = vth;
        
        
        (pSlamPosPub)->publish(pose_slam);

    }
}
    /******************* pub slam_odom topic  end ********************/



void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(do_rectify)
    {
        cv::Mat imLeft, imRight, Tcw;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        Tcw = mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
        //mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    /*************** topic_slam_odom ***************/

        //test
        sj_TR = mpSLAM->mpTracker->mNum_TRKF;
        sj_TRp = mpSLAM->mpTracker->mNum_TRKFp;
        cov_err_TR = mpSLAM->mpTracker->cov_err_sj_TRKF;
        cov_depth_TR = mpSLAM->mpTracker->cov_depth_sj_TRKF;

        sj_TM = mpSLAM->mpTracker->mNum_TWMM;
        sj_TMp = mpSLAM->mpTracker->mNum_TWMMp;
        cov_err_TM = mpSLAM->mpTracker->cov_err_sj_TWMM;
        cov_depth_TM = mpSLAM->mpTracker->cov_depth_sj_TWMM;

        sj_TLM = mpSLAM->mpTracker->mNum_TLM;
        cov_err_TLM = mpSLAM->mpTracker->cov_err_sj_TLM;
        cov_depth_TLM = mpSLAM->mpTracker->cov_depth_sj_TLM;

        cout<<"--------------------------------------------------"<<endl;
        cout<<"sj_TR******************"<<sj_TR<<"***************"<<endl;
        cout<<"sj_TRp******************"<<sj_TRp<<"***************"<<endl;
        cout<<"cov_err_TR******************"<<cov_err_TR<<"***************"<<endl;
        cout<<"cov_depth_TR******************"<<cov_depth_TR<<"***************"<<endl;

        cout<<"--------------------------------------------------"<<endl;
        cout<<"sj_TM******************"<<sj_TM<<"***************"<<endl;
        cout<<"sj_TMp******************"<<sj_TMp<<"***************"<<endl;
        cout<<"cov_err_TM******************"<<cov_err_TM<<"***************"<<endl;
        cout<<"cov_depth_TM******************"<<cov_depth_TM<<"***************"<<endl;

        cout<<"--------------------------------------------------"<<endl;
        cout<<"sj_TLM******************"<<sj_TLM<<"***************"<<endl;
        cout<<"cov_err_TLM******************"<<cov_err_TLM<<"***************"<<endl;
        cout<<"cov_depth_TLM******************"<<cov_depth_TLM<<"***************"<<endl;
        cout<<"--------------------------------------------------"<<endl;

        //cv::Mat Tcw= mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    
        //tmp_time = ros::Time::now().toSec();
        PublishPose(Tcw);
    /******************* pub slam_odom topic  end ********************/

    }
    else
    {
        mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    }

}


