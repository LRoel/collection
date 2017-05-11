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

MatrixXd TpsCx(132, 2);
MatrixXd TpsC(132, 2);
MatrixXd TpsMpt(1, 2);
MatrixXd TpsD(3, 2);
double TpsScale;

MatrixXd Data_xy(1, 2);  // (Xi, Yi)
MatrixXd xy_a(1, 2);  
MatrixXd xy_A(132, 2);
MatrixXd xy_b(132, 2);
MatrixXd xy_B(132, 2);
MatrixXd xy_c(132, 1);
MatrixXd xy_C(132, 1);
MatrixXd xy_d(1, 132);
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


    //SLAM.SaveMap("Slam_latest_Map.bin");
    SLAM.LoadMap("Slam_Map.bin");
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
   TpsCx << 0.198271858228085, 0.495323959385912,
-0.00745538260761218, -0.304715050842621,
-0.112180741860372, -0.313659985809441,
-0.327910832181030, -0.0258651777714910,
0.291836570648358, 0.233378285177950,
0.199150411644781, -0.277634153936937,
-0.434569324309812, 0.139239867516305,
0.319695639134098, -0.00121610049066257,
0.304158443744450, 0.144487510971026,
0.308361644316949, 0.0995023824161878,
-0.310154502266460, -0.163043790134673,
-0.358013500634554, 0.0304322958739075,
-0.317397030375640, 0.384994950074448,
-0.350799010798620, 0.377886130928398,
0.351621452342908, -0.187541644699750,
0.283805297946274, 0.296616661189108,
0.153766978176109, 0.487428391675149,
-0.0944135995965632, 0.444274204280390,
-0.223800328435743, -0.323992189847249,
0.149497251623288, -0.283203530475689,
-0.398489691927616, 0.349358098402653,
-0.323675182047169, -0.236651415353918,
0.286256948032771, -0.271483591737565,
0.324682184382525, -0.0577221623694255,
-0.0391295584150957, 0.460504341095297,
0.235755118726528, -0.270227607248222,
0.275809716007102, 0.369374860008670,
-0.231919398146618, 0.402226380489529,
0.104302884091882, 0.480736048195581,
-0.308360497041833, -0.309061782532167,
0.116671210242262, -0.287791148224831,
-0.165921595531277, 0.417909943048606,
0.281898970685424, 0.324400944260237,
0.0326805075204259, -0.300929320822288,
-0.0681653333446547, -0.310383970759461,
-0.322274888990557, -0.282562564037531,
0.364352092301130, -0.254147767091715,
0.287722171497861, 0.269735259318411,
-0.384749536479148, 0.365043463020210,
-0.434487030319842, 0.183883506047444,
-0.320805754807784, -0.217529557087476,
0.267525404303200, 0.440488981089773,
0.336332840866742, -0.172697578449068,
-0.0330772459298698, -0.304755357815464,
0.330647697733221, -0.113832347107925,
0.313396094278839, 0.0383870311615841,
-0.425473635913558, 0.295398870321403,
0.0132321200671376, 0.473153688206876,
0.215435411118575, -0.273830919549627,
0.319842256589564, -0.0341461106163592,
-0.279785998947334, 0.393586412582587,
-0.419079745804193, 0.109123420826140,
-0.425985445463007, 0.246522046972552,
-0.313467667559352, -0.0810034274399378,
0.0847841913412509, -0.296464560578819,
0.311149378252524, 0.0718615381688422,
-0.407314996511847, 0.335483652077004,
0.177521608889215, 0.492504989774940,
-0.206811670467653, -0.319054640137561,
-0.240587160297593, -0.322461260213759,
-0.293910155472811, -0.319063655903553,
-0.373504993827204, 0.0511826453276847,
-0.334742284540704, -0.0109306456969903,
0.180399270533138, -0.281055861948211,
0.136367055322688, 0.484580739305370,
0.307094747364280, 0.118883067544126,
-0.308309363641285, -0.122582565935286,
0.310354820769371, -0.268746678345813,
-0.343646879684926, 0.00521627106977527,
0.268420076128847, -0.267265480385258,
0.0838284247380128, 0.477525581446707,
0.377096748267563, -0.226410550275499,
0.338307345994444, -0.147091783478718,
-0.141489747603160, -0.316394676350258,
-0.425688218493236, 0.264878962830462,
-0.257573660972374, 0.397390757612954,
0.345681117749785, -0.260475156248870,
-0.191718521296235, 0.412271753542267,
0.329366985002155, -0.0925498398645164,
-0.430487149463572, 0.215854921339165,
-0.320265544071320, -0.0494072156771064,
0.0442685675116528, 0.475982118621996,
0.297916164292717, 0.174616572068088,
-0.115124303770345, 0.435810587788909,
0.271631944478214, 0.400479635491986,
-0.277828694714709, -0.324536602776338,
-0.186926301115747, -0.320019126856533,
-0.313861924098840, -0.185208504967141,
-0.333576928560775, 0.382007888482304,
-0.147119621361344, 0.423084402919023,
0.222155636443436, 0.499617312923863,
-0.403202799876997, 0.0879737152589787,
-0.0511001052534657, -0.306959168588125,
0.261484955366762, 0.468068780616571,
0.0141616316843207, -0.303199033426639,
0.0499617197917859, -0.299640519164991,
-0.0891583349371105, -0.308616561535369,
-0.0135523210613542, 0.467964010955927,
-0.435548342469125, 0.157268758729612,
-0.366783054044305, 0.373430542097283,
0.293750856998777, 0.204285707873541,
-0.162945340030728, -0.315307678614505,
-0.308615400672821, -0.142430447158623,
0.360357129561872, -0.209734404063107,
0.252476216277808, 0.488517986126072,
0.289897656169482, 0.253184155920870,
0.278357425805744, 0.352604155800352,
-0.0645418323606623, 0.453647159348933,
0.324535366698886, -0.0754118661887671,
-0.297534588771798, 0.389554858051177,
0.0609683156794239, 0.476009099438049,
-0.385747764077834, 0.0657485563913037,
-0.259756735562972, -0.324508897989536,
0.334086475240550, -0.130584229820392,
-0.309792632716825, -0.105268666330173,
-0.325440197913087, -0.256313158985632,
-0.213553008968453, 0.406759958393408,
-0.419381778262424, 0.314008524838052,
0.101174385961787, -0.293179493002680,
0.316446378078246, 0.0212880520218066,
0.133284709904620, -0.286186496765265,
0.269353041690219, 0.420504956873732,
0.330141019044243, -0.265854644035912,
0.317591535989122, -0.0178824605252545,
0.253488372285084, -0.274735471409518,
0.239223379906997, 0.498773749470653,
-0.426265490308991, 0.124941136604914,
0.0659041029563565, -0.295060319262767,
-0.130417170105854, 0.429349107983567,
0.376476239570139, -0.242858961385844,
0.313041639447299, 0.0552652081174451,
-0.317614078270647, -0.201407343535142;

   TpsC << -0.213455391632108, -0.137396928225202, 
0.103429926969043, -0.146992189314483, 
0.00464846673705373, 0.515431928932632, 
-0.349883106781653, 0.869060733289388, 
0.227319681900694, 0.476054669847595, 
0.292965520133308, -0.0792072851600362, 
2.55690059235574, -1.87566106288167, 
0.192245929303852, -0.121213280516979, 
0.527808205928602, 0.553763430351323, 
0.217589084643724, 0.000307954854338514, 
-0.0619168825602437, 0.0586192829846109, 
-0.856055594253393, 4.15674045005012, 
-0.279071501044432, 0.253897144027327, 
-0.431603737470726, 0.516669645941333, 
-0.562304036753039, -0.431758076343257, 
0.127981348947770, 0.841393406803477, 
-0.172255260191236, -0.207742024800261, 
0.0229793990254468, -0.194120926133661, 
-0.256653221912680, -0.264650818869552, 
0.278809628605853, -0.181088711163783, 
-0.132545496897628, 0.0613253442471939, 
-0.138037440879946, -0.289740975541635, 
0.242289007033613, -0.276214094216078, 
0.161856035020691, 9.61123969023773e-05, 
0.0586348835048145, -0.210786123836417, 
-1.07311357542284, -0.782375883093738, 
-0.134989806846066, 0.289221346022700, 
-0.286947258986744, -0.0565429767287080, 
-0.145340022600248, -0.285090260227560, 
-0.563992560779883, -0.620355707007319, 
0.214603548619979, -0.111367086145812, 
-0.169420456486770, -0.140623608098771, 
0.112947522055067, 1.13576376425289, 
0.0825646075036086, -0.0694487092181514, 
-0.0254861659677953, 0.127552338872780, 
-0.210778136007825, -0.549268314249362, 
0.436642598537968, -0.304196066495208, 
-0.00657710453785137, 0.598551354382208, 
-0.505499283450584, 0.408879492133616, 
1.41827879580207, -4.79094506723803, 
-0.180652825020534, -0.172442601599595, 
-0.0638031774291047, 0.00107436991736595, 
-0.204626506943397, -0.193671201018851, 
0.00409523338778457, -0.0177379112487368, 
0.159012675528078, 0.00993854494108782, 
0.171547962177001, -0.0621517099496679, 
0.486513278263095, -0.959373624347509, 
0.0591860275607696, -0.163163521245568, 
-0.0802397731596316, -0.323909522986439, 
0.158543174134979, -0.0680377516663074, 
-0.262250283463604, 0.123008666403682, 
-0.488376476799059, 3.40774649703038, 
0.806403531742517, -1.86406132287816, 
-0.334147229907963, 0.842492841474203, 
0.147484742787714, -0.138889361662596, 
0.201353524642506, -0.0593475181632391, 
0.0411301569168550, -0.0803176785712101, 
-0.188645050527708, -0.169751033652574, 
0.0765384238986758, 0.0187060808821116, 
-0.0874888777907585, -0.0480919210240245, 
-0.272997337041907, -0.346678379923105, 
-0.940114392803769, 3.63970019045519, 
-0.0401410815246785, 0.333324851468205, 
0.243366676098958, -0.175104840658605, 
-0.177253505289636, -0.253737560792208, 
0.246947156855556, 0.108752509261780, 
-0.0954776862662234, 0.332851496484049, 
0.436127775716555, -0.350971455797462, 
-0.0532566423496039, 0.530916339858299, 
0.281957656517937, -0.0423684368135719, 
-0.0828883057480338, -0.252630885734070, 
-0.721892388486302, -0.290962053317417, 
0.0123978825761474, -0.0807795353962876, 
-0.0365321893561169, 0.449531794804417, 
0.726078754321853, -1.27028238160391, 
-0.328654572332876, 0.0530437640635667, 
0.303693438096317, -0.0718354112387357, 
-0.253935482094891, -0.158211278557254, 
0.146282429438428, 0.0567548626155728, 
1.40950916695554, -4.08762616011553, 
-0.356397785947831, 1.04326358823729, 
-0.0122091228035416, -0.121194330540932, 
0.416625817574777, 0.427309979061600, 
0.0135208219190258, -0.125461763611575, 
-0.0840269823576766, -0.714128258595469, 
-0.252340530669221, -0.257385264168890, 
-0.0699422904393637, -0.0147760332750698, 
-0.112865627628442, -0.0846838017986660, 
-0.304564900425649, 0.351166775463202, 
-0.0712479972033191, -0.145282798368048, 
-0.282352048864917, -0.0402852876134657, 
-0.928722029658794, 3.21127281755721, 
-0.0138155940134329, 0.0830483138361796, 
-0.206140339922996, 0.200466496030850, 
0.0961012895093378, -0.107871421316556, 
0.0507024818824714, -0.147710122046298, 
0.0799037331183679, 0.350755158355059, 
0.156822052315918, -0.239114682484823, 
1.44863605527125, -4.65188508311734, 
-0.593035726012109, 0.641339524555589, 
0.268192762988036, 0.392697959962829, 
-0.0185921749957377, 0.205102620490553, 
-0.101913185962000, 0.224755667263949, 
-0.797750871186700, -0.526971441080113, 
-0.274784734544884, 0.273813701247227, 
0.0299830793616261, 0.336348526190516, 
0.0620079000686556, 1.05153875638814, 
0.0209397409588577, -0.199515361196134, 
0.0726194208357569, 0.0679132044237712, 
-0.277447128810987, 0.179725305526287, 
-0.0425096071445701, -0.138880876239780, 
-0.550601806950836, 1.64253475840155, 
-0.216650808660622, -0.128595294475221, 
0.0576520625210410, -0.0722675479098953, 
-0.0757905858507897, 0.409225236123891, 
0.0178410547334519, -0.417958459545464, 
-0.234020436515785, -0.125614456456837, 
0.147803162773616, -0.239211447599776, 
0.152844642553738, -0.117541996957186, 
0.194684218775526, -0.0816131622798590, 
0.114911116437137, -0.153981225621413, 
-0.0249126486084304, -0.238931570254408, 
0.286139079436052, -0.105878096846481, 
0.111832554077986, -0.0805208725917690, 
-0.306261688171394, -0.357439223939977, 
-0.272674054193607, 0.209313681251970, 
0.739737730392454, 1.95220989604815, 
0.229719010612216, -0.120925541712188, 
-0.0179308665829048, -0.138633208726740, 
-0.239510369707045, -0.133539863000846, 
0.157037094750332, -0.0607594747774995, 
-0.116667862910878, -0.107445281698402;

   TpsMpt << 27.288017900710233, 39.504810980717490;

   TpsD << -4.40535190333951,58.7592346812598,
-121.710280308991,12.6102212169654,
-12.4838168507825,-128.101160918566;

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

        for(int i=0; i<132; i++)
        {
            xy_A.row(i) = xy_a;  // MATLAB: repmat(xy_a , 132, 1)
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

        odom_trans.transform.translation.x = OutPut(0, 0)-0.185;
        odom_trans.transform.translation.y = OutPut(0, 1)-1.265;
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
        pose_slam.pose.pose.position.x = OutPut(0, 0)-0.185;
        pose_slam.pose.pose.position.y = OutPut(0, 1)-1.265;
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
        
        ROS_WARN("lx:%f", lx);
        ROS_WARN("ly:%f", ly);
        ROS_WARN("x:%f", OutPut(0, 0));
        ROS_WARN("y:%f", OutPut(0, 1));
        
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


