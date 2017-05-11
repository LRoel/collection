#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Empty


seq = 0

def amcl_update(input_x, input_y, input_yaw):
    global pub_amcl, seq
    amcl_init_pose.header.stamp = rospy.Time.now()
    amcl_init_pose.header.frame_id = "map"
    amcl_init_pose.header.seq = seq
    seq += 1

    amcl_init_pose.pose.pose.position.x = input_x
    amcl_init_pose.pose.pose.position.y = input_y
    amcl_init_pose.pose.pose.position.z = 0

    q = quaternion_from_euler(0, 0, input_yaw)
    amcl_init_pose.pose.pose.orientation.x = q[0]
    amcl_init_pose.pose.pose.orientation.y = q[1]
    amcl_init_pose.pose.pose.orientation.z = q[2]
    amcl_init_pose.pose.pose.orientation.w = q[3]

    amcl_init_pose.pose.covariance = [0.0] * 36
    amcl_init_pose.pose.covariance[0] = 2.5
    amcl_init_pose.pose.covariance[7] = 2.5
    amcl_init_pose.pose.covariance[35] = 3.14

    pub_amcl.publish(amcl_init_pose)
    print "pub amcl ok  " + str(input_yaw)

if __name__ == '__main__':
    try:
        rospy.init_node('initialpose_node', anonymous=True)
        pub_amcl = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=0)
        amcl_init_pose = PoseWithCovarianceStamped()
        n = 0
        time.sleep(1)
        rospy.wait_for_message('/abc', Empty)
        print "asdfdsdfsdds"
        amcl_update(0, 0, 0)

    except rospy.ROSInterruptException:
        pass