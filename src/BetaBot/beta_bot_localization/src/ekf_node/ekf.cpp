#include "ekf.hpp"

void ExtendedKalmanFilter::initMatrix(pose InitialPose) {

  _nu(0) = InitialPose.x;
  _nu(1) = InitialPose.y;
  _nu(2) = InitialPose.z;

  _nu(6) = InitialPose.r;
  _nu(7) = InitialPose.p;
  _nu(8) = InitialPose.yaw;

  _sigma = Eigen::Matrix<double, 9, 9>::Identity() * desv_tip_sigma_inicial *
           desv_tip_sigma_inicial;

  _R = Eigen::Matrix<double, 9, 9>::Identity() * desv_tip_R * desv_tip_R;

  _Q = Eigen::Matrix<double, 9, 9>::Identity() * desv_tip_Q * desv_tip_Q;

  matrixInitialized = true;

  // std::cout << "x: " << _nu(0) << ",y: " << _nu(1) << ",z: " << _nu(2)
  //           << std::endl;
  // std::cout << "r: " << _nu(6) << ",p: " << _nu(7) << ",yaw: " << _nu(8)
  //           << std::endl;
}

void ExtendedKalmanFilter::EKFPrediction(double LinAccX, double LinAccY,
                                         double LinAccZ, double AngVelX,
                                         double AngVelY, double AngVelZ,
                                         double currentTimeStamp) {

  const double T = currentTimeStamp - _lastPredTimeStamp;
  _G << 1, 0, 0, T, 0, 0, 0, 0, 0, 0, 1, 0, 0, T, 0, 0, 0, 0, 0, 0, 1, 0, 0, T,
      0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 1;

  _nu(0) = _nu(0) + _nu(3) * T + 1 / 2 * LinAccX * T;
  _nu(1) = _nu(1) + _nu(4) * T + 1 / 2 * LinAccY * T;
  // _nu(2) = _nu(2) + _nu(5) * T + 1 / 2 * LinAccZ * T;
  _nu(2) = _nu(2);
  _nu(3) = _nu(3) + LinAccX * T;
  _nu(4) = _nu(4) + LinAccY * T;
  _nu(5) = _nu(5) + LinAccZ * T;
  _nu(6) = _nu(6) + AngVelX * T;
  _nu(7) = _nu(7) + AngVelY * T;
  _nu(8) = _nu(8) + AngVelZ * T;

  _sigma = _G * _sigma * _G.transpose() + _R;

  _lastPredTimeStamp = currentTimeStamp;
}

void ExtendedKalmanFilter::EKFUpdate(double xGPS, double yGPS, double zBar,
                                     double xGPS_ant, double yGPS_ant,
                                     double zBar_ant, double LinAccX,
                                     double LinAccY, double LinAccZ,
                                     double magX, double magY,
                                     double currentTimeStamp) {

  const double T = currentTimeStamp - _lastUpdTimeStamp;
  Eigen::Matrix<double, 9, 1> z;
  z << xGPS, yGPS, zBar, xGPS_ant, yGPS_ant, zBar_ant,
      atan2(LinAccX, sqrt(LinAccY * LinAccY + LinAccZ * LinAccZ)),
      atan2(LinAccY, sqrt(LinAccX * LinAccX + LinAccZ * LinAccZ)),
      atan2(-magY, magX);
  Eigen::Matrix<double, 9, 1> h;
  h << _nu(0), _nu(1), _nu(2), _nu(0) - T * _nu(3), _nu(1) - T * _nu(4),
      _nu(2) - T * _nu(5), _nu(6), _nu(7), _nu(8);

  _H << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
      0, 0, 0, 1, 0, 0, -T, 0, 0, 0, 0, 0, 0, 1, 0, 0, -T, 0, 0, 0, 0, 0, 0, 1,
      0, 0, -T, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 1;
  Eigen::Matrix<double, 9, 9> K;
  K = _sigma * _H.transpose() * (_H * _sigma * _H.transpose() + _Q).inverse();
  _nu = _nu + K * (z - h);
  _sigma = (Eigen::Matrix<double, 9, 9>::Identity() - K * _H) * _sigma;

  _lastUpdTimeStamp = currentTimeStamp;
}
