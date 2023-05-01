#ifndef EKF_HPP
#define EKF_HPP

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
// #include <iostream>

struct pose {
  double x;
  double y;
  double z;
  double r;
  double p;
  double yaw;
};

constexpr double desv_tip_sigma_inicial{1};
constexpr double desv_tip_R{1};
constexpr double desv_tip_Q{5};

class ExtendedKalmanFilter {
public:
  void initMatrix(pose InitialPose);
  void EKFPrediction(double LinAccX, double LinAccY, double LinAccZ,
                     double AngVelX, double AngVelY, double AngVelZ,
                     double currentTimeStamp);
  void EKFUpdate(double xGPS, double yGPS, double zBar, double xGPS_ant,
                 double yGPS_ant, double zBar_ant, double LinAccX,
                 double LinAccY, double LinAccZ, double magX, double magY,
                 double currentTimeStamp);
  inline pose GetEstimatedPose() {
    pose Pose;
    Pose.x = _nu(0);
    Pose.y = _nu(1);
    Pose.z = _nu(2);
    Pose.r = _nu(6);
    Pose.p = _nu(7);
    Pose.yaw = _nu(8);
    return Pose;
  };
  bool matrixInitialized{false};

private:
  // Matrix
  Eigen::Matrix<double, 9, 1>
      _nu; // variable state vector: [x, y, z, vx, vy, vz, roll, pitch, yaw]'
  Eigen::Matrix<double, 9, 9> _sigma;
  Eigen::Matrix<double, 9, 9> _G;
  Eigen::Matrix<double, 9, 9> _R;
  Eigen::Matrix<double, 9, 9> _H;
  Eigen::Matrix<double, 9, 9> _Q;

  // Internal variables
  double _lastPredTimeStamp{0};
  double _lastUpdTimeStamp{0};
};

#endif