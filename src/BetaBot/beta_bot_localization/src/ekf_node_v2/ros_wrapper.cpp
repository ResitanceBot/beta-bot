#include "beacons_gazebo/BeaconSimPose.h"
#include "beta_bot_localization/IniLocalization.h"
#include "beta_bot_localization/PoseRPYWithCovariance.h"
#include "ekf.hpp"
#include "std_msgs/Float64.h"
#include <beta_bot_localization/IniLocalization.h>
#include <boost/optional.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

class SensorMeasurementsMaintainer {
public:
  inline bool checkValidPredictValues() {
    return (_AccX.has_value() && _AccY.has_value() && _AccZ.has_value() &&
            _GyrX.has_value() && _GyrY.has_value() && _GyrZ.has_value());
  }
  inline bool checkValidUpdateValues() {
    return (_AccX.has_value() && _AccY.has_value() && _AccZ.has_value() &&
            _dist1.has_value() && _dist2.has_value() && _dist3.has_value() &&
            _dist4.has_value() && _dist1_ant.has_value() &&
            _dist2_ant.has_value() && _dist3_ant.has_value() &&
            _dist4_ant.has_value() && _magX.has_value() && _magY.has_value() &&
            _magZ.has_value());
  }

  boost::optional<double> _AccX;
  boost::optional<double> _AccY;
  boost::optional<double> _AccZ;
  boost::optional<double> _GyrX;
  boost::optional<double> _GyrY;
  boost::optional<double> _GyrZ;
  boost::optional<double> _dist1;
  boost::optional<double> _dist2;
  boost::optional<double> _dist3;
  boost::optional<double> _dist4;
  boost::optional<double> _dist1_ant;
  boost::optional<double> _dist2_ant;
  boost::optional<double> _dist3_ant;
  boost::optional<double> _dist4_ant;
  boost::optional<double> _magX;
  boost::optional<double> _magY;
  boost::optional<double> _magZ;
};

class EkfROSWrapper {
public:
  EkfROSWrapper(ros::NodeHandle &nh) {
    acc_gyr_sub =
        nh.subscribe("/raw_imu", 10, &EkfROSWrapper::callbackIMU, this);
    Magnetometer_sub = nh.subscribe("/magnetic/converted", 10,
                                    &EkfROSWrapper::callbackMag, this);
    beacons_dist_1 =
        nh.subscribe("quadrotor/odom_rssi_beacon_1", 10,
                     &EkfROSWrapper::beacons_dist_1_Callback, this);
    beacons_dist_2 =
        nh.subscribe("quadrotor/odom_rssi_beacon_2", 10,
                     &EkfROSWrapper::beacons_dist_2_Callback, this);
    beacons_dist_3 =
        nh.subscribe("quadrotor/odom_rssi_beacon_3", 10,
                     &EkfROSWrapper::beacons_dist_3_Callback, this);
    beacons_dist_4 =
        nh.subscribe("quadrotor/odom_rssi_beacon_4", 10,
                     &EkfROSWrapper::beacons_dist_4_Callback, this);
    beacons_pos = nh.subscribe("beacons_gazebo/beacons", 10,
                               &EkfROSWrapper::beacons_pos_Callback, this);

    init_sub = nh.subscribe("/iniLoc", 10, &EkfROSWrapper::callbackInit, this);
    _pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        "/estimated_localization/pose", 10);
    _poseRPY_pub = nh.advertise<beta_bot_localization::PoseRPYWithCovariance>(
        "/estimated_localization/poseRPY", 10);
  }
  void callbackIMU(const sensor_msgs::ImuConstPtr msg) {
    sensorValues._AccX = msg->linear_acceleration.x;
    sensorValues._AccY = msg->linear_acceleration.y;
    sensorValues._AccZ = msg->linear_acceleration.z;
    sensorValues._GyrX = msg->angular_velocity.x;
    sensorValues._GyrY = msg->angular_velocity.y;
    sensorValues._GyrZ = msg->angular_velocity.z;
    if (sensorValues.checkValidPredictValues() && _ekf.matrixInitialized &&
        beaconsPositionInitialized) {
      _ekf.EKFPrediction(sensorValues._GyrX.value(), sensorValues._GyrY.value(),
                         sensorValues._GyrZ.value(), msg->header.stamp.toSec());
    }
  }

  void callbackMag(const sensor_msgs::MagneticFieldConstPtr msg) {
    sensorValues._magX = msg->magnetic_field.x;
    sensorValues._magY = msg->magnetic_field.y;
    sensorValues._magZ = msg->magnetic_field.z;
    if (sensorValues.checkValidUpdateValues() && _ekf.matrixInitialized &&
        beaconsPositionInitialized) {
      _ekf.EKFUpdate(
          sensorValues._dist1.value(), sensorValues._dist2.value(),
          sensorValues._dist3.value(), sensorValues._dist4.value(),
          sensorValues._dist1_ant.value(), sensorValues._dist2_ant.value(),
          sensorValues._dist3_ant.value(), sensorValues._dist4_ant.value(),
          sensorValues._magX.value(), sensorValues._magY.value(),
          sensorValues._magZ.value(), sensorValues._AccX.value(),
          sensorValues._AccY.value(), sensorValues._AccZ.value(),
          msg->header.stamp.toSec());
    }
  }

  void beacons_dist_1_Callback(const std_msgs::Float64 msg) {
    sensorValues._dist1_ant = sensorValues._dist1;
    if ((ros::Time::now() - lastStampBeacon1).toSec() < 1 / 20.0)
      sensorValues._dist1 = msg.data;
    else
      sensorValues._dist1 = -1.0;
    lastStampBeacon1 = ros::Time::now();
  }

  void beacons_dist_2_Callback(const std_msgs::Float64 msg) {
    sensorValues._dist2_ant = sensorValues._dist2;
    if ((ros::Time::now() - lastStampBeacon2).toSec() < 1 / 20.0)
      sensorValues._dist2 = msg.data;
    else
      sensorValues._dist2 = -1.0;
    lastStampBeacon2 = ros::Time::now();
  }

  void beacons_dist_3_Callback(const std_msgs::Float64 msg) {
    sensorValues._dist3_ant = sensorValues._dist3;
    if ((ros::Time::now() - lastStampBeacon3).toSec() < 1 / 20.0)
      sensorValues._dist3 = msg.data;
    else
      sensorValues._dist3 = -1.0;
    lastStampBeacon3 = ros::Time::now();
  }

  void beacons_dist_4_Callback(const std_msgs::Float64 msg) {
    sensorValues._dist4_ant = sensorValues._dist4;
    if ((ros::Time::now() - lastStampBeacon4).toSec() < 1 / 20.0)
      sensorValues._dist4 = msg.data;
    else
      sensorValues._dist4 = -1.0;
    lastStampBeacon4 = ros::Time::now();
  }

  void beacons_pos_Callback(const beacons_gazebo::BeaconSimPose &msg) {
    int beacon_id = std::stoi(msg.id.substr(msg.id.size() - 1));
    switch (beacon_id) {
    case 1:
      this->contb[beacon_id - 1] = true;
      _ekf.SetBeaconPosition(msg.position.x, msg.position.y, msg.position.z, 1);
      break;
    case 2:
      this->contb[beacon_id - 1] = true;
      _ekf.SetBeaconPosition(msg.position.x, msg.position.y, msg.position.z, 2);
      break;
    case 3:
      this->contb[beacon_id - 1] = true;
      _ekf.SetBeaconPosition(msg.position.x, msg.position.y, msg.position.z, 3);
      break;
    case 4:
      this->contb[beacon_id - 1] = true;
      _ekf.SetBeaconPosition(msg.position.x, msg.position.y, msg.position.z, 4);
      break;
    }
    if ((contb[0] == true) && (contb[1] == true) && (contb[2] == true) &&
        (contb[3] == true)) {
      beaconsPositionInitialized = true;
      beacons_pos.shutdown();
    }
  }

  void callbackInit(const beta_bot_localization::IniLocalization msg) {
    pose InitialPose;
    InitialPose.x = msg.gps_bar.x;
    InitialPose.y = msg.gps_bar.y;
    InitialPose.z = msg.gps_bar.z;
    InitialPose.r = msg.orientation.x;
    InitialPose.p = msg.orientation.y;
    InitialPose.yaw = msg.orientation.z;
    _ekf.initMatrix(InitialPose);
  }

  void pubPose() {
    pose PoseEstimatedByEKF = _ekf.GetEstimatedPose();
    beta_bot_localization::PoseRPYWithCovariance msgPoseRPY;
    msgPoseRPY.x = PoseEstimatedByEKF.x;
    msgPoseRPY.y = PoseEstimatedByEKF.y;
    msgPoseRPY.z = PoseEstimatedByEKF.z;
    msgPoseRPY.roll = PoseEstimatedByEKF.r;
    msgPoseRPY.pitch = PoseEstimatedByEKF.p;
    msgPoseRPY.yaw = PoseEstimatedByEKF.yaw;
    for (int i = 0; i < 81; i++) {
      msgPoseRPY.covariance[i] = PoseEstimatedByEKF.covariance[i];
    }
    _poseRPY_pub.publish(msgPoseRPY);

    geometry_msgs::PoseWithCovarianceStamped msgPose;
    msgPose.header.frame_id = "world";
    msgPose.header.stamp = ros::Time::Time::now();
    msgPose.pose.pose.position.x = PoseEstimatedByEKF.x;
    msgPose.pose.pose.position.y = PoseEstimatedByEKF.y;
    msgPose.pose.pose.position.z = PoseEstimatedByEKF.z;
    tf::Quaternion ori;
    ori.setRPY(PoseEstimatedByEKF.r, PoseEstimatedByEKF.p,
               PoseEstimatedByEKF.yaw);
    ori.normalize();
    msgPose.pose.pose.orientation.x = ori.x();
    msgPose.pose.pose.orientation.y = ori.y();
    msgPose.pose.pose.orientation.z = ori.z();
    msgPose.pose.pose.orientation.w = ori.w();
    for (int i = 0; i < 36; i++) {
      msgPose.pose.covariance[i] = PoseEstimatedByEKF.covariance[i];
    }
    _pose_pub.publish(msgPose);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(PoseEstimatedByEKF.x, PoseEstimatedByEKF.y,
                                    PoseEstimatedByEKF.z));
    transform.setRotation(ori);
    _transform_broadcaster.sendTransform(tf::StampedTransform(
        transform, ros::Time::now(), "world", "base_link"));
  }

  // should be private, only for debug purposes
  ExtendedKalmanFilter _ekf;

private:
  SensorMeasurementsMaintainer sensorValues;
  ros::Subscriber acc_gyr_sub;
  ros::Subscriber Magnetometer_sub;
  ros::Subscriber beacons_dist_1;
  ros::Subscriber beacons_dist_2;
  ros::Subscriber beacons_dist_3;
  ros::Subscriber beacons_dist_4;
  ros::Subscriber beacons_pos;
  ros::Subscriber init_sub;
  ros::Publisher _pose_pub;
  ros::Publisher _poseRPY_pub;
  tf::TransformBroadcaster _transform_broadcaster;

  // Time variables for evaluating data losses from beacons
  ros::Time lastStampBeacon1;
  ros::Time lastStampBeacon2;
  ros::Time lastStampBeacon3;
  ros::Time lastStampBeacon4;

  // Variables for evaluating if beacons' position has been set
  bool contb[4] = {false, false, false, false};
  bool beaconsPositionInitialized = false;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "ekf");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  EkfROSWrapper ekf_ros_wrapper(nh);

  // Set the beacons' position: #MUST BE DONE BY SUBSCRIBING TO A SPECIFIC TOPIC
  // FOR THIS PURPOSE Beacons' position extracted directly from
  // beta-bot/src/BetaBot/beacons_gazebo/launch/spawn_beacons.launch
  // ekf_ros_wrapper._ekf.SetBeaconPosition(0.755,9.436,12.38,1);
  // ekf_ros_wrapper._ekf.SetBeaconPosition(24.83,-14.54,6.324,2);
  // ekf_ros_wrapper._ekf.SetBeaconPosition(-24.925,-8.906,12.42,3);
  // ekf_ros_wrapper._ekf.SetBeaconPosition(62.639,-13.663,5.179,4);

  // Debug msg
  ros::Rate rate(50);
  while (ros::ok()) {
    ekf_ros_wrapper.pubPose();
    rate.sleep();
  }

  ros::waitForShutdown();
}