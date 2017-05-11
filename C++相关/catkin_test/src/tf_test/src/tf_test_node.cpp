#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <af_msgs/Fuse_tf.h>
#include <std_msgs/Int8.h>

/// fuse/tf 回调函数
///
///     topic /fuse/tf 回调过程中调用,并且将fuse_tf_msg分解为map->base和odom->base两组数据,转换为map->odom并传递到main函数循环中
///     @param fuse_tf_msg 自定义Fuse_tf格式数据
///     @return    无返回值
///     @note    临时解决方案
class TfPubNode
{
public:
    TfPubNode(void)
    {
        switch_flag_ = 1;
        static tf::TransformBroadcaster br;
        m_tfServer = new tf::TransformBroadcaster();
        map_to_odom_ = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0.0, 0.0, 0.0));   ///< map->odom全局变量
        sub_ = nh.subscribe("/fuse/tf", 100, &TfPubNode::tfReceived, this);
        flag_sub_ = nh.subscribe("/switch", 1, &TfPubNode::switchReceived, this);
        update();
    }

    ~TfPubNode(void)
    {
        if (m_tfServer)
            delete m_tfServer;
    }

private:
    ros::NodeHandle nh;
    tf::Transform map_to_odom_;
    tf::Transform laser_to_map;
    tf::Transform odom_to_laser;
    tf::TransformBroadcaster       *m_tfServer;
    ros::Subscriber sub_;
    ros::Subscriber flag_sub_;
    bool switch_flag_;

public:
    void update(){
        ros::Rate rate(10.0);
        while (nh.ok()){
            if (switch_flag_ == 1){
                m_tfServer->sendTransform(tf::StampedTransform (map_to_odom_, ros::Time::now(), "map", "odom"));
            }
            ros::spinOnce();
            rate.sleep();
        }
    }

    void tfReceived(const af_msgs::Fuse_tf::ConstPtr& fuse_tf_msg){
        laser_to_map = tf::Transform(tf::createQuaternionFromRPY(0, 0, fuse_tf_msg->map_theta), tf::Vector3(fuse_tf_msg->map_x, fuse_tf_msg->map_y, 0.0)).inverse();
        odom_to_laser = tf::Transform(tf::createQuaternionFromRPY(0, 0, fuse_tf_msg->odom_theta), tf::Vector3(fuse_tf_msg->odom_x, fuse_tf_msg->odom_y, 0.0));

        map_to_odom_ = (odom_to_laser * laser_to_map).inverse();
        ROS_INFO("x=%f,y=%f",map_to_odom_.getOrigin().getX(),map_to_odom_.getOrigin().getY());
        ROS_INFO("processing tf");
    }

    void switchReceived(const std_msgs::Int8ConstPtr msg){
//        if (msg->data == 0) {
//            switch_flag_ = 0;
//        } else {
//            switch_flag_ = 1;
//        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "af_v2_tf_publish");

    TfPubNode tf_test;

    return 0;
}
