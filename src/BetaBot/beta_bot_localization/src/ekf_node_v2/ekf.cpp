#include "ekf.hpp"
#include "ros/ros.h"

void ExtendedKalmanFilter::initMatrix(pose InitialPose) {

  _nu(0) = InitialPose.x;
  _nu(1) = InitialPose.y;
  _nu(2) = InitialPose.z;

  _nu(6) = InitialPose.r;
  _nu(7) = InitialPose.p;
  _nu(8) = InitialPose.yaw;

  _sigma = Eigen::Matrix<double, 9, 9>::Identity() * desv_tip_sigma_inicial *
           desv_tip_sigma_inicial;

  _R << desv_tip_R_position * desv_tip_R_position, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      desv_tip_R_position * desv_tip_R_position, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      desv_tip_R_position * desv_tip_R_position, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      desv_tip_R_vel * desv_tip_R_vel, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      desv_tip_R_vel * desv_tip_R_vel, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      desv_tip_R_vel * desv_tip_R_vel, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      desv_tip_R_orientation * desv_tip_R_orientation, 0, 0, 0, 0, 0, 0, 0, 0,
      0, desv_tip_R_orientation * desv_tip_R_orientation, 0, 0, 0, 0, 0, 0, 0,
      0, 0, desv_tip_R_orientation * desv_tip_R_orientation;

   _Q << desv_tip_Q_beacons * desv_tip_Q_beacons, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, desv_tip_Q_beacons * desv_tip_Q_beacons, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, desv_tip_Q_beacons * desv_tip_Q_beacons, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, desv_tip_Q_beacons * desv_tip_Q_beacons, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, desv_tip_Q_beacons * desv_tip_Q_beacons, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, desv_tip_Q_beacons * desv_tip_Q_beacons, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, desv_tip_Q_beacons * desv_tip_Q_beacons, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, desv_tip_Q_beacons * desv_tip_Q_beacons, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, desv_tip_Q_or_rp * desv_tip_Q_or_rp,     0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, desv_tip_Q_or_rp * desv_tip_Q_or_rp,     0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, desv_tip_R_orientation * desv_tip_R_orientation;

  matrixInitialized = true;
}
// Same as in the "ekf_node" version
void ExtendedKalmanFilter::EKFPrediction(double LinAccX, double LinAccY,
                                         double LinAccZ, double AngVelX,
                                         double AngVelY, double AngVelZ,
                                         double currentTimeStamp) {

  const double T = currentTimeStamp - _lastPredTimeStamp;
  _G << 1, 0, 0, T, 0, 0, 0, 0, 0, 0, 1, 0, 0, T, 0, 0, 0, 0, 0, 0, 1, 0, 0, T,
      0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 1;

  _nu(0) = _nu(0) + _nu(3) * T + 1 / 2 * LinAccX * T * T;
  _nu(1) = _nu(1) + _nu(4) * T + 1 / 2 * LinAccY * T * T;
  _nu(2) = _nu(2) + _nu(5) * T + 1 / 2 * LinAccZ * T * T;
  _nu(3) = _nu(3) + LinAccX * T;
  _nu(4) = _nu(4) + LinAccY * T;
  _nu(5) = _nu(5) + LinAccZ * T;
  _nu(6) = _nu(6) + AngVelX * T;
  _nu(7) = _nu(7) + AngVelY * T;
  _nu(8) = _nu(8) + AngVelZ * T;

  _sigma = _G * _sigma * _G.transpose() + _R;

  _lastPredTimeStamp = currentTimeStamp;
}
// Changes compared to the "ekf_node" version
void ExtendedKalmanFilter::EKFUpdate(double dist1, double dist2, double dist3,             
                 double dist4, double dist1_ant, double dist2_ant, 
                 double dist3_ant, double dist4_ant, double magX, double magY, 
                 double magZ, double LinAccX, double LinAccY,
                 double LinAccZ, double currentTimeStamp) {

  const double T = currentTimeStamp - _lastUpdTimeStamp;
  Eigen::Matrix<double, 11, 1> z;
  double accEstimatedRoll =
      atan2(LinAccY, sqrt(LinAccX * LinAccX + LinAccZ * LinAccZ));
  double accEstimatedPitch =
      atan2(-LinAccX, sqrt(LinAccY * LinAccY + LinAccZ * LinAccZ));
  z << dist1, dist2, dist3, dist4, dist1_ant,
       dist2_ant, dist3_ant, dist4_ant, 
      accEstimatedRoll,  // REVIEWED (OK SUPOSSING STATIC)
      accEstimatedPitch, // REVIEWED (OK SUPOSSING STATIC)
      atan2(cos(accEstimatedRoll) * -magY - sin(accEstimatedRoll) * magZ,
            cos(accEstimatedPitch) * magX + sin(accEstimatedRoll) * magY +
            cos(accEstimatedRoll) * sin(accEstimatedPitch) * magZ); // WRONG

  Eigen::Matrix<double, 11, 1> h;
  h(0) = sqrt((_nu(0)-_xb[0])*(_nu(0)-_xb[0]) + (_nu(1)-_yb[0])*(_nu(1)-_yb[0]) + (_nu(2)-_zb[0])*(_nu(2)-_zb[0]));
  h(1) = sqrt((_nu(0)-_xb[1])*(_nu(0)-_xb[1]) + (_nu(1)-_yb[1])*(_nu(1)-_yb[1]) + (_nu(2)-_zb[1])*(_nu(2)-_zb[1]));
  h(2) = sqrt((_nu(0)-_xb[2])*(_nu(0)-_xb[2]) + (_nu(1)-_yb[2])*(_nu(1)-_yb[2]) + (_nu(2)-_zb[2])*(_nu(2)-_zb[2]));
  h(3) = sqrt((_nu(0)-_xb[3])*(_nu(0)-_xb[3]) + (_nu(1)-_yb[3])*(_nu(1)-_yb[3]) + (_nu(2)-_zb[3])*(_nu(2)-_zb[3]));

  h(4) = sqrt(((_nu(0)-T*_nu(3)-(1/2)*LinAccX*T*T)-_xb[0])*((_nu(0)-T*_nu(3)-(1/2)*LinAccX*T*T)-_xb[0]) + ((_nu(1)-T*_nu(4)-(1/2)*LinAccY*T*T)-_yb[0])*((_nu(1)-T*_nu(4)-(1/2)*LinAccY*T*T)-_yb[0]) + ((_nu(2)-T*_nu(5)-(1/2)*LinAccZ*T*T)-_zb[0])*((_nu(2)-T*_nu(5)-(1/2)*LinAccZ*T*T)-_zb[0]));
  h(5) = sqrt(((_nu(0)-T*_nu(3)-(1/2)*LinAccX*T*T)-_xb[1])*((_nu(0)-T*_nu(3)-(1/2)*LinAccX*T*T)-_xb[1]) + ((_nu(1)-T*_nu(4)-(1/2)*LinAccY*T*T)-_yb[1])*((_nu(1)-T*_nu(4)-(1/2)*LinAccY*T*T)-_yb[1]) + ((_nu(2)-T*_nu(5)-(1/2)*LinAccZ*T*T)-_zb[1])*((_nu(2)-T*_nu(5)-(1/2)*LinAccZ*T*T)-_zb[1]));
  h(6) = sqrt(((_nu(0)-T*_nu(3)-(1/2)*LinAccX*T*T)-_xb[2])*((_nu(0)-T*_nu(3)-(1/2)*LinAccX*T*T)-_xb[2]) + ((_nu(1)-T*_nu(4)-(1/2)*LinAccY*T*T)-_yb[2])*((_nu(1)-T*_nu(4)-(1/2)*LinAccY*T*T)-_yb[2]) + ((_nu(2)-T*_nu(5)-(1/2)*LinAccZ*T*T)-_zb[2])*((_nu(2)-T*_nu(5)-(1/2)*LinAccZ*T*T)-_zb[2]));
  h(7) = sqrt(((_nu(0)-T*_nu(3)-(1/2)*LinAccX*T*T)-_xb[3])*((_nu(0)-T*_nu(3)-(1/2)*LinAccX*T*T)-_xb[3]) + ((_nu(1)-T*_nu(4)-(1/2)*LinAccY*T*T)-_yb[3])*((_nu(1)-T*_nu(4)-(1/2)*LinAccY*T*T)-_yb[3]) + ((_nu(2)-T*_nu(5)-(1/2)*LinAccZ*T*T)-_zb[3])*((_nu(2)-T*_nu(5)-(1/2)*LinAccZ*T*T)-_zb[3]));

  h(8) = _nu(6);
  h(9) = _nu(7);
  h(10) = _nu(8);


  double H11, H12, H13;
  double H21, H22, H23;
  double H31, H32, H33;
  double H41, H42, H43;

  double H51, H52, H53, H54, H55, H56;
  double H61, H62, H63, H64, H65, H66;
  double H71, H72, H73, H74, H75, H76;
  double H81, H82, H83, H84, H85, H86;


  H11 = (_nu(0)-_xb[0])/h(0);  // h(i) = distance between robot and beacon i+1 at the current instant (i = 0,1,2,3)
  H12 = (_nu(1)-_xb[0])/h(0);
  H13 = (_nu(2)-_xb[0])/h(0);

  H21 = (_nu(0)-_xb[1])/h(1);  // h(i) = distance between robot and beacon i+1 at the current instant (i = 0,1,2,3)
  H21 = (_nu(1)-_xb[1])/h(1);
  H21 = (_nu(2)-_xb[1])/h(1);
  
  H31 = (_nu(0)-_xb[2])/h(2);  // h(i) = distance between robot and beacon i+1 at the current instant (i = 0,1,2,3)
  H32 = (_nu(1)-_xb[2])/h(2);
  H33 = (_nu(2)-_xb[2])/h(2);

  H41 = (_nu(0)-_xb[3])/h(3);  // h(i) = distance between robot and beacon i+1 at the current instant (i = 0,1,2,3)
  H42 = (_nu(1)-_xb[3])/h(3);
  H43 = (_nu(2)-_xb[3])/h(3);

  H51 = ((_nu(0)-T*_nu(3)-(1/2)*LinAccX*T*T)-_xb[0])/h(4);  // h(i) = distance between robot and beacon i+1 at the previous instant (i = 4,5,6,7)
  H52 = ((_nu(1)-T*_nu(4)-(1/2)*LinAccY*T*T)-_yb[0])/h(4);
  H53 = ((_nu(2)-T*_nu(5)-(1/2)*LinAccZ*T*T)-_zb[0])/h(4);
  H54 = -T*H51;
  H55 = -T*H52;
  H56 = -T*H53;

  H61 = ((_nu(0)-T*_nu(3)-(1/2)*LinAccX*T*T)-_xb[1])/h(5);  // h(i) = distance between robot and beacon i+1 at the previous instant (i = 4,5,6,7)
  H62 = ((_nu(1)-T*_nu(4)-(1/2)*LinAccY*T*T)-_yb[1])/h(5);
  H63 = ((_nu(2)-T*_nu(5)-(1/2)*LinAccZ*T*T)-_zb[1])/h(5);
  H64 = -T*H61;
  H65 = -T*H62;
  H66 = -T*H63;

  H71 = ((_nu(0)-T*_nu(3)-(1/2)*LinAccX*T*T)-_xb[2])/h(6);  // h(i) = distance between robot and beacon i+1 at the previous instant (i = 4,5,6,7)
  H72 = ((_nu(1)-T*_nu(4)-(1/2)*LinAccY*T*T)-_yb[2])/h(6);
  H73 = ((_nu(2)-T*_nu(5)-(1/2)*LinAccZ*T*T)-_zb[2])/h(6);
  H74 = -T*H71;
  H75 = -T*H72;
  H76 = -T*H73;

  H81 = ((_nu(0)-T*_nu(3)-(1/2)*LinAccX*T*T)-_xb[3])/h(7);  // h(i) = distance between robot and beacon i+1 at the previous instant (i = 4,5,6,7)
  H82 = ((_nu(1)-T*_nu(4)-(1/2)*LinAccY*T*T)-_yb[3])/h(7);
  H83 = ((_nu(2)-T*_nu(5)-(1/2)*LinAccZ*T*T)-_zb[3])/h(7);
  H84 = -T*H81;
  H85 = -T*H82;
  H86 = -T*H83;


  _H << H11, H12, H13, 0, 0, 0, 0, 0, 0,
        H21, H22, H23, 0, 0, 0, 0, 0, 0,
        H31, H32, H33, 0, 0, 0, 0, 0, 0,
        H41, H42, H43, 0, 0, 0, 0, 0, 0,
        H51, H52, H53, H54, H55, H56, 0, 0, 0, 
        H61, H62, H63, H64, H65, H66, 0, 0, 0,
        H71, H72, H73, H74, H75, H76, 0, 0, 0,
        H81, H82, H83, H84, H85, H86, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1;

  Eigen::Matrix<double, 9, 11> K;
  K = _sigma * _H.transpose() * (_H * _sigma * _H.transpose() + _Q).inverse();
  _nu = _nu + K * (z - h);
  _sigma = (Eigen::Matrix<double, 9, 9>::Identity() - K * _H) * _sigma;

  _lastUpdTimeStamp = currentTimeStamp;
}
