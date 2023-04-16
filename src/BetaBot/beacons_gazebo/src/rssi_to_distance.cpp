#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "beacons_gazebo/ReceiverIn.h"

void receiver_callback(const beacons_gazebo::ReceiverIn& msg);

ros::NodeHandlePtr nhPtr;
ros::Subscriber sub;
ros::Publisher pub;

void receiver_callback(const beacons_gazebo::ReceiverIn& msg) {
    //Typical values for BLE (bluetooth low energy) beacons
    if(msg.id=="beacon__beacon_1")
    {
    float measured_power = msg.m_rssi;
    int N=2;
    std_msgs::Float64 distance;
    // distance.data = pow(10,((measured_power -msg.rssi)/(10 * N)));
    distance.data = pow(msg.rssi/measured_power,10);
    ROS_INFO_STREAM("Potencia recibida: "<<msg.rssi);   
    ROS_INFO_STREAM("Distancia calculada: "<<distance.data);
    pub.publish(distance);
    } 

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rssi_to_distance");
    nhPtr = ros::NodeHandlePtr(new ros::NodeHandle("~"));
    sub = nhPtr->subscribe("/quadrotor/receiver__quadrotor/receiver_in_msgs", 10, receiver_callback);
    pub = nhPtr->advertise<std_msgs::Float64>("/quadrotor/odom_rssi_beacon_1", 10);
    ros::spin();
    return 0;
}
