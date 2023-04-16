#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <beacon_gazebo_sim/ReceiverIn.h>

void receiver_callback(const beacon_gazebo_sim::ReceiverIn& msg) {
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Float64>("/quadrotor/odom_rssi_beacon", 10);
    //Typical values for BLE (bluetooth low energy) beacons
    float measured_power = -69;
    int N=2;
    std_msgs::Float64 distance;
    distance.data = pow(10,((measured_power -msg.m_rssi)/(10 * N)));
    pub.publish(distance);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rssi_to_distance");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/quadrotor/receiver__quadrotor/receiver_in_msgs", 10, receiver_callback);
    ros::spin();
    return 0;
}
