#include "beacons_gazebo/ReceiverIn.h"
#include <ros/ros.h>
#include <std_msgs/Float64.h>

void receiver_callback(const beacons_gazebo::ReceiverIn &msg);

ros::NodeHandlePtr nhPtr;
ros::Subscriber sub;
ros::Publisher pub1, pub2, pub3, pub4, pub5;

int beacon_id = 0;

void receiver_callback(const beacons_gazebo::ReceiverIn &msg) {
  beacon_id = std::stoi(msg.id.substr(msg.id.size() - 1));
  float measured_power = msg.m_rssi;
  std_msgs::Float64 distance;
  distance.data = pow(msg.rssi / measured_power, 10);
  switch (beacon_id) {
  case 1:
    pub1.publish(distance);
    break;
  case 2:
    pub2.publish(distance);
    break;
  case 3:
    pub3.publish(distance);
    break;
  case 4:
    pub4.publish(distance);
    break;
  case 5:
    pub5.publish(distance);
    break;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "rssi_to_distance");
  nhPtr = ros::NodeHandlePtr(new ros::NodeHandle("~"));
  sub = nhPtr->subscribe("/quadrotor/receiver__quadrotor/receiver_in_msgs", 10,
                         receiver_callback);
  pub1 =
      nhPtr->advertise<std_msgs::Float64>("/quadrotor/odom_rssi_beacon_1", 10);
  pub2 =
      nhPtr->advertise<std_msgs::Float64>("/quadrotor/odom_rssi_beacon_2", 10);
  pub3 =
      nhPtr->advertise<std_msgs::Float64>("/quadrotor/odom_rssi_beacon_3", 10);
  pub4 =
      nhPtr->advertise<std_msgs::Float64>("/quadrotor/odom_rssi_beacon_4", 10);
  pub5 =
      nhPtr->advertise<std_msgs::Float64>("/quadrotor/odom_rssi_beacon_5", 10);
  ros::spin();
  return 0;
}
