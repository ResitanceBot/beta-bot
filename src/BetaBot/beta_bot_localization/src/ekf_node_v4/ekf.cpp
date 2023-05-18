#include "ekf.hpp"
#include "ros/ros.h"
#include <iostream>

void ExtendedKalmanFilter::initMatrix(pose InitialPose) {

  _nu(0) = InitialPose.x;
  _nu(1) = InitialPose.y;
  _nu(2) = InitialPose.z;

  _nu(6) = InitialPose.r;
  _nu(7) = InitialPose.p;
  _nu(8) = InitialPose.yaw;

  _sigma = Eigen::Matrix<double, 9, 9>::Identity() * desv_tip_sigma_inicial *
           desv_tip_sigma_inicial;

   _Q << desv_tip_Q_beacons * desv_tip_Q_beacons, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, desv_tip_Q_beacons * desv_tip_Q_beacons, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, desv_tip_Q_beacons * desv_tip_Q_beacons, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, desv_tip_Q_beacons * desv_tip_Q_beacons, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, desv_tip_Q_beacons * desv_tip_Q_beacons, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, desv_tip_Q_beacons * desv_tip_Q_beacons, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, desv_tip_Q_beacons * desv_tip_Q_beacons, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, desv_tip_Q_beacons * desv_tip_Q_beacons, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, desv_tip_Q_gps * desv_tip_Q_gps,         0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, desv_tip_Q_gps * desv_tip_Q_gps,         0, 0, 0, 0, 0, 0, 0, 
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, desv_tip_Q_bar * desv_tip_Q_bar,         0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, desv_tip_Q_gps * desv_tip_Q_gps,         0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, desv_tip_Q_gps * desv_tip_Q_gps,         0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, desv_tip_Q_bar * desv_tip_Q_bar,         0, 0, 0,  
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, desv_tip_Q_or_rp * desv_tip_Q_or_rp,     0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, desv_tip_Q_or_rp * desv_tip_Q_or_rp,     0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, desv_tip_R_orientation * desv_tip_R_orientation;

  matrixInitialized = true;
}
// Same as in the "ekf_node" version
void ExtendedKalmanFilter::EKFPrediction(double AngVelX, double AngVelY, double AngVelZ,
                                         double VO_x, double VO_y, double VO_z,
                                         double VO_vx, double VO_vy, double VO_vz,
                                         double VO_r, double VO_p, double VO_yaw, 
                                         double VO_var_x, double VO_var_y, double VO_var_z,
                                         double VO_var_r, double VO_var_p, double VO_var_yaw,
                                          double VO_var_vx, double VO_var_vy, double VO_var_vz,
                                         double currentTimeStamp, int predictionModel) {

  const double T = currentTimeStamp - _lastPredTimeStamp;

  if(predictionModel == 1){  // Visual Odometry is currently available and not lost
    _R << 100*VO_var_x, 0, 0, 0, 0, 0, 0, 0, 0,
           0, 100*VO_var_y, 0, 0, 0, 0, 0, 0, 0,
           0, 0, 100*VO_var_z, 0, 0, 0, 0, 0, 0,
           0, 0, 0, 100*VO_var_vx, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 100*VO_var_vy, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 100*VO_var_vz, 0, 0, 0,
           0, 0, 0, 0, 0, 0, 100*VO_var_r, 0, 0,
           0, 0, 0, 0, 0, 0, 0, 100*VO_var_p, 0,
           0, 0, 0, 0, 0, 0, 0, 0, 100*VO_var_yaw;

     _nu(0) = VO_x;
     _nu(1) = VO_y;
     _nu(2) = VO_z;

     _nu(3) = VO_vx;
     _nu(4) = VO_vy;
     _nu(5) = VO_vz;

     _nu(6) = VO_p;
     _nu(7) = VO_r;
     _nu(8) = VO_yaw;

     //_sigma = _G * _sigma * _G.transpose() + _R;    // With VO, we do not have access to the matriz G
     _sigma = _sigma + _R;                            // If we assume that the sensor lectures are directly the 
                                                      // components of the state vector, we can also assume that 
                                                      // G is the identity, and then, the prediction equation of 
                                                      // sigma is an incremental equation like this

  }
  else{                                               // If Visual Odometry is lost, we will use MRU IMU model as backup
      
      std::cout << "[ekf.cpp]: using IMU MRU model for prediction" << std::endl;
      _R << desv_tip_R_position * desv_tip_R_position, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      desv_tip_R_position * desv_tip_R_position, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      desv_tip_R_position * desv_tip_R_position, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      desv_tip_R_vel * desv_tip_R_vel, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      desv_tip_R_vel * desv_tip_R_vel, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      desv_tip_R_vel * desv_tip_R_vel, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      desv_tip_R_orientation * desv_tip_R_orientation, 0, 0, 0, 0, 0, 0, 0, 0,
      0, desv_tip_R_orientation * desv_tip_R_orientation, 0, 0, 0, 0, 0, 0, 0,
      0, 0, desv_tip_R_orientation * desv_tip_R_orientation;
      
      _G << 1, 0, 0, T, 0, 0, 0, 0, 0, 0, 1, 0, 0, T, 0, 0, 0, 0, 0, 0, 1, 0, 0, T,
      0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 1;

      _nu(0) = _nu(0) + _nu(3) * T;
      _nu(1) = _nu(1) + _nu(4) * T;
      _nu(2) = _nu(2) + _nu(5) * T;
      _nu(3) = _nu(3);
      _nu(4) = _nu(4);
      _nu(5) = _nu(5);
      _nu(6) = _nu(6) + AngVelX * T;
      _nu(7) = _nu(7) + AngVelY * T;
      _nu(8) = _nu(8) + AngVelZ * T;

      _sigma = _G * _sigma * _G.transpose() + _R;
  }
 
  _lastPredTimeStamp = currentTimeStamp;
}
// Changes compared to the "ekf_node" version
void ExtendedKalmanFilter::EKFUpdate(double dist1, double dist2, double dist3,             
                                     double dist4, double dist1_ant, double dist2_ant, 
                                     double dist3_ant, double dist4_ant, double xGPS, 
                                     double yGPS, double zBar, double xGPS_ant, double yGPS_ant,
                                     double zBar_ant, double magX, double magY, 
                                     double magZ, double LinAccX, double LinAccY,
                                     double LinAccZ, double currentTimeStamp) {

  const double T = currentTimeStamp - _lastUpdTimeStamp;

  double accEstimatedRoll = atan2(LinAccY, sqrt(LinAccX * LinAccX + LinAccZ * LinAccZ));
  double accEstimatedPitch = atan2(-LinAccX, sqrt(LinAccY * LinAccY + LinAccZ * LinAccZ));
  
  Eigen::Matrix<double, 8, 1> dists;
  dists << dist1, dist2, dist3, dist4, dist1_ant,
         dist2_ant, dist3_ant, dist4_ant;

  Eigen::Matrix<double, 17, 1> z;
  z.setZero();  // Eigen does not initialize its matrices with zeros but with garbage!!!!
  int i=0, j=0;

  for(int i = 0; i<8; i++){
  	if(dists[i]!=-1.0){
  		z(j) = dists[i];
  		j++;
  	}
  }

  z(j)   = xGPS;    
  z(j+1) = yGPS;    
  z(j+2) = zBar;    
  z(j+3) = xGPS_ant;
  z(j+4) = yGPS_ant;
  z(j+5) = zBar_ant;
  z(j+6) = accEstimatedRoll;
  z(j+7) = accEstimatedPitch;
  z(j+8) = atan2(cos(accEstimatedRoll) * -magY - sin(accEstimatedRoll) * magZ, cos(accEstimatedPitch) * magX + sin(accEstimatedRoll) * magY + cos(accEstimatedRoll) * sin(accEstimatedPitch) * magZ); 

  int MatrixRowSize = j+9; // En este punto, tendré en j+2(r,p,y que siempre están)+1(para empezar a contar en 1), en j+3 el número de elementos real de los vectores

  Eigen::Matrix<double, 17, 1> h;
  h.setZero();   // Eigen does not initialize its matrices with zeros but with garbage!!!!
  j=0;
  for(i=0;i<4;i++){
  	if(dists[i]!=-1.0){
  		h(j) = sqrt((_nu(0)-_xb[i])*(_nu(0)-_xb[i]) + (_nu(1)-_yb[i])*(_nu(1)-_yb[i]) + (_nu(2)-_zb[i])*(_nu(2)-_zb[i]));
  		j++;
  	}
  }

  for(i=4;i<8;i++){
  	if(dists[i]!=-1.0){
  		h(j) = sqrt(((_nu(0)-T*_nu(3)-(1/2)*LinAccX*T*T)-_xb[i-4])*((_nu(0)-T*_nu(3)-(1/2)*LinAccX*T*T)-_xb[i-4]) + ((_nu(1)-T*_nu(4)-(1/2)*LinAccY*T*T)-_yb[i-4])*((_nu(1)-T*_nu(4)-(1/2)*LinAccY*T*T)-_yb[i-4]) + ((_nu(2)-T*_nu(5)-(1/2)*LinAccZ*T*T)-_zb[i-4])*((_nu(2)-T*_nu(5)-(1/2)*LinAccZ*T*T)-_zb[i-4]));
  		j++;
  	}
  }

  h(j)   = _nu(0);
  h(j+1) = _nu(1);
  h(j+2) = _nu(2);
  h(j+3) = _nu(0) - T* _nu(3);
  h(j+4) = _nu(1) - T* _nu(4);
  h(j+5) = _nu(2) - T* _nu(5);
  h(j+6) = _nu(6);
  h(j+7) = _nu(7);
  h(j+8) = _nu(8);

  // // Modified z-h table
  // // At this point, prove that h is almost equal to z
  // std::cout << "-----------------------------------------------------------------------" << std::endl;
  // std::cout << "h           " << "z           " << "abs(h-z)" << std::endl;

  // j=0;
  // for(i=0; i<8; i++){
  //   const Eigen::IOFormat fmt(6, 0, "\t", " ", "");    
  //   if(dists[i]!=-1.0){
  //      std::cout << h(j) << "   |" << z(j) << "   |" << abs(h(j)-z(j)) << std::endl;
  //      j++;
  //   }
  //   else std::cout << "NOCALC" << "   |" << -1 << "   |" << "NODIST" << std::endl;
  // }

  // for(int i=j; i<z.size(); i++){
  //   const Eigen::IOFormat fmt(6, 0, "\t", " ", "");    
  //   std::cout << h(i) << "   |" << z(i) << "   |" << abs(h(i)-z(i)) << std::endl;
  // } 

  // MODIFIED JACOBIAN
  // _H declared in "ekf.hpp"
  _H.setZero();    // Eigen does not initialize its matrices with zeros but with garbage!!!!
  j=0;
  double H_1, H_2, H_3, H_4, H_5, H_6;
  for(i=0;i<4;i++){
  	if(dists[i]!=-1.0){
  		H_1 = (_nu(0)-_xb[i])/h(j);
  		H_2 = (_nu(1)-_yb[i])/h(j);
  		H_3 = (_nu(2)-_zb[i])/h(j);		
  		_H.row(j) << H_1, H_2, H_3, 0, 0, 0, 0, 0, 0;
  		j++;
  	}
  }

  for(i=4;i<8;i++){
  	if(dists[i]!=-1.0){
  		 H_1 = ((_nu(0)-T*_nu(3)-(1/2)*LinAccX*T*T)-_xb[i-4])/h(j);	
  		 H_2 = ((_nu(1)-T*_nu(4)-(1/2)*LinAccY*T*T)-_yb[i-4])/h(j);	
  		 H_3 = ((_nu(2)-T*_nu(5)-(1/2)*LinAccZ*T*T)-_zb[i-4])/h(j);
  		 H_4 = -T*H_1;
  		 H_5 = -T*H_2;
  		 H_6 = -T*H_3;		
  		_H.row(j) << H_1, H_2, H_3, H_4, H_5, H_6, 0, 0, 0;
  		j++;
  	}
  }

  _H.row(j)   << 1, 0, 0, 0, 0, 0, 0, 0, 0;
  _H.row(j+1) << 0, 1, 0, 0, 0, 0, 0, 0, 0;
  _H.row(j+2) << 0, 0, 1, 0, 0, 0, 0, 0, 0;
  _H.row(j+3) << 1, 0, 0,-T, 0, 0, 0, 0, 0;
  _H.row(j+4) << 0, 1, 0, 0,-T, 0, 0, 0, 0;
  _H.row(j+5) << 0, 0, 1, 0, 0,-T, 0, 0, 0;
  _H.row(j+6) << 0, 0, 0, 0, 0, 0, 1, 0, 0;
  _H.row(j+7) << 0, 0, 0, 0, 0, 0, 0, 1, 0;
  _H.row(j+8) << 0, 0, 0, 0, 0, 0, 0, 0, 1;

  // MODIFIED Q: THE REST OF _Q_MODIF UNUSED WILL BE FILLED WITH 0's
  Eigen::Matrix<double, 17, 17> _Q_modif;    // Eigen does not initialize its matrices with zeros but with garbage!!!!
  _Q_modif.setZero();                        // Eigen does not initialize its matrices with zeros but with garbage!!!! 

  j=0;
  for(i=0;i<8;i++){
  	if(dists[i]!=-1.0){
  		_Q_modif(j,j) = _Q(i,i);
  		j++;
  	}
  }

  _Q_modif(j,j) = _Q(8,8);
  _Q_modif(j+1,j+1) = _Q(9,9);
  _Q_modif(j+2,j+2) = _Q(10,10);
  _Q_modif(j+3,j+3) = _Q(11,11);
  _Q_modif(j+4,j+4) = _Q(12,12);
  _Q_modif(j+5,j+5) = _Q(13,13);
  _Q_modif(j+6,j+6) = _Q(14,14);
  _Q_modif(j+7,j+7) = _Q(15,15);
  _Q_modif(j+8,j+8) = _Q(16,16);

  /* MODIFIED EQUATIONS
  Notes:
  matrix.block<p,q>(i,j); == Block of size (p,q), starting at (i,j) // Values must be known at compile time (cannot be variables)
  matrix.block(i,j,p,q);  == Block of size (p,q), starting at (i,j) // Related to dynamic memory, it is allowed to use variables
  */

  Eigen::Matrix<double, 9, 17> K;  
  K.setZero();    // Eigen does not initialize its matrices with zeros but with garbage!!!!
  K.block(0,0,9,MatrixRowSize) = _sigma * _H.block(0,0,MatrixRowSize,9).transpose() * (_H.block(0,0,MatrixRowSize,9) * _sigma * _H.block(0,0,MatrixRowSize,9).transpose() + _Q_modif.block(0,0,MatrixRowSize,MatrixRowSize)).inverse();
  _nu = _nu + K.block(0,0,9,MatrixRowSize) * (z.block(0,0,MatrixRowSize,1) - h.block(0,0,MatrixRowSize,1));                                                
  _sigma = (Eigen::Matrix<double, 9, 9>::Identity() - K.block(0,0,9,MatrixRowSize) * _H.block(0,0,MatrixRowSize,9)) * _sigma;    

    _lastUpdTimeStamp = currentTimeStamp;
  }
