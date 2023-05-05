#include "beta_bot_localization/PoseRPY.h"
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/tf.h>

class ROSNode {
public:
  ROSNode(ros::NodeHandle nh) {
    _gt_sub =
        nh.subscribe("/ground_truth/state", 10, &ROSNode::callbackGT, this);
    _gt_pub = nh.advertise<beta_bot_localization::PoseRPY>(
        "/ground_truth/poseRPY", 10);
  }

  void callbackGT(const nav_msgs::Odometry::ConstPtr msg) {
    _gtTfMsg.x = msg->pose.pose.position.x;
    _gtTfMsg.y = msg->pose.pose.position.y;
    _gtTfMsg.z = msg->pose.pose.position.z;
    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                     msg->pose.pose.orientation.z,
                     msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    _gtTfMsg.roll = roll;
    _gtTfMsg.pitch = pitch;
    _gtTfMsg.yaw = yaw;
    _gt_pub.publish(_gtTfMsg);
  }

private:
  ros::Subscriber _gt_sub;
  ros::Publisher _gt_pub;
  beta_bot_localization::PoseRPY _gtTfMsg;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "gt_pub");
  ros::NodeHandle nh;
  ROSNode node(nh);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::waitForShutdown();
}