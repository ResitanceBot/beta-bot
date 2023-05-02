#include "beta_bot_localization/IniLocalization.h"
#include "ekf.hpp"
#include <boost/optional.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

class SensorMeasurementsMaintainer {
public:
  inline bool checkValidPredictValues() {
    return (_AccX.has_value() && _AccY.has_value() && _AccZ.has_value() &&
            _GyrX.has_value() && _GyrY.has_value() && _GyrZ.has_value());
  }
  inline bool checkValidUpdateValues() {
    return (_AccX.has_value() && _AccY.has_value() && _AccZ.has_value() &&
            _gpsX.has_value() && _gpsY.has_value() && _gpsX_ant.has_value() &&
            _gpsY_ant.has_value() && _barZ.has_value() &&
            _barZ_ant.has_value() && _magX.has_value() && _magY.has_value());
  }
  boost::optional<double> _AccX;
  boost::optional<double> _AccY;
  boost::optional<double> _AccZ;
  boost::optional<double> _GyrX;
  boost::optional<double> _GyrY;
  boost::optional<double> _GyrZ;
  boost::optional<double> _gpsX;
  boost::optional<double> _gpsY;
  boost::optional<double> _gpsX_ant;
  boost::optional<double> _gpsY_ant;
  boost::optional<double> _barZ;
  boost::optional<double> _barZ_ant;
  boost::optional<double> _magX;
  boost::optional<double> _magY;
};

class EkfROSWrapper {
public:
  EkfROSWrapper(ros::NodeHandle &nh) {
    acc_gyr_sub =
        nh.subscribe("/raw_imu", 10, &EkfROSWrapper::callbackIMU, this);
    Magnetometer_sub = nh.subscribe("/magnetic/converted", 10,
                                    &EkfROSWrapper::callbackMag, this);
    GPS_sub =
        nh.subscribe("/odometry/gps", 10, &EkfROSWrapper::callbackGPS, this);
    Barometer_sub = nh.subscribe("/pose_height", 10,
                                 &EkfROSWrapper::callbackBarometer, this);

    init_sub = nh.subscribe("/iniLoc", 10, &EkfROSWrapper::callbackInit, this);
  }
  void callbackIMU(const sensor_msgs::ImuConstPtr msg) {
    sensorValues._AccX = msg->linear_acceleration.x;
    sensorValues._AccY = msg->linear_acceleration.y;
    sensorValues._AccZ = msg->linear_acceleration.z;
    sensorValues._GyrX = msg->angular_velocity.x;
    sensorValues._GyrY = msg->angular_velocity.y;
    sensorValues._GyrZ = msg->angular_velocity.z;
    if (sensorValues.checkValidPredictValues() && ekf.matrixInitialized) {
      ekf.EKFPrediction(sensorValues._AccX.value(), sensorValues._AccY.value(),
                        sensorValues._AccZ.value(), sensorValues._GyrX.value(),
                        sensorValues._GyrY.value(), sensorValues._GyrZ.value(),
                        msg->header.stamp.toSec());
    }
  }
  void callbackMag(const sensor_msgs::MagneticFieldConstPtr msg) {
    sensorValues._magX = msg->magnetic_field.x;
    sensorValues._magY = msg->magnetic_field.y;
    if (sensorValues.checkValidUpdateValues() && ekf.matrixInitialized) {
      ekf.EKFUpdate(sensorValues._gpsX.value(), sensorValues._gpsY.value(),
                    sensorValues._barZ.value(), sensorValues._gpsX_ant.value(),
                    sensorValues._gpsY_ant.value(),
                    sensorValues._barZ_ant.value(), sensorValues._AccX.value(),
                    sensorValues._AccY.value(), sensorValues._AccZ.value(),
                    sensorValues._magX.value(), sensorValues._magY.value(),
                    msg->header.stamp.toSec());
    }
  }

  void callbackGPS(const nav_msgs::OdometryConstPtr msg) {
    if (sensorValues._gpsX) {
      sensorValues._gpsX_ant = sensorValues._gpsX;
    }
    if (sensorValues._gpsY) {
      sensorValues._gpsY_ant = sensorValues._gpsY;
    }
    sensorValues._gpsX = msg->pose.pose.position.x;
    sensorValues._gpsY = msg->pose.pose.position.y;
    if (sensorValues.checkValidUpdateValues() && ekf.matrixInitialized) {
      ekf.EKFUpdate(sensorValues._gpsX.value(), sensorValues._gpsY.value(),
                    sensorValues._barZ.value(), sensorValues._gpsX_ant.value(),
                    sensorValues._gpsY_ant.value(),
                    sensorValues._barZ_ant.value(), sensorValues._AccX.value(),
                    sensorValues._AccY.value(), sensorValues._AccZ.value(),
                    sensorValues._magX.value(), sensorValues._magY.value(),
                    msg->header.stamp.toSec());
    }
  }

  void callbackBarometer(
      const geometry_msgs::PoseWithCovarianceStampedConstPtr msg) {
    if (sensorValues._barZ.has_value()) {
      sensorValues._barZ_ant = sensorValues._barZ;
    }
    sensorValues._barZ = msg->pose.pose.position.z;
    if (sensorValues.checkValidUpdateValues() && ekf.matrixInitialized) {
      ekf.EKFUpdate(sensorValues._gpsX.value(), sensorValues._gpsY.value(),
                    sensorValues._barZ.value(), sensorValues._gpsX_ant.value(),
                    sensorValues._gpsY_ant.value(),
                    sensorValues._barZ_ant.value(), sensorValues._AccX.value(),
                    sensorValues._AccY.value(), sensorValues._AccZ.value(),
                    sensorValues._magX.value(), sensorValues._magY.value(),
                    msg->header.stamp.toSec());
    }
  }

  void callbackInit(const beta_bot_localization::IniLocalization msg) {
    pose InitialPose;
    InitialPose.x = msg.gps_bar.x;
    InitialPose.y = msg.gps_bar.y;
    InitialPose.z = msg.gps_bar.z;
    InitialPose.r = 0;
    InitialPose.p = 0;
    InitialPose.yaw = 0;
    ekf.initMatrix(InitialPose);
  }

  // should be private, only for debug purposes
  ExtendedKalmanFilter ekf;

private:
  SensorMeasurementsMaintainer sensorValues;
  ros::Subscriber acc_gyr_sub;
  ros::Subscriber Magnetometer_sub;
  ros::Subscriber GPS_sub;
  ros::Subscriber Barometer_sub;
  ros::Subscriber init_sub;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "ekf");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  EkfROSWrapper ekf_ros_wrapper(nh);

  // Debug msg
  ros::Rate rate(0.5);
  while (ros::ok()) {
    pose PoseEstimatedByEKF = ekf_ros_wrapper.ekf.GetEstimatedPose();
    ROS_INFO_STREAM("x: " << PoseEstimatedByEKF.x
                          << ",y: " << PoseEstimatedByEKF.y
                          << ",z: " << PoseEstimatedByEKF.z);
    ROS_INFO_STREAM("r: " << PoseEstimatedByEKF.r
                          << ",p: " << PoseEstimatedByEKF.p
                          << ",yaw: " << PoseEstimatedByEKF.yaw);
    rate.sleep();
  }

  ros::waitForShutdown();
}