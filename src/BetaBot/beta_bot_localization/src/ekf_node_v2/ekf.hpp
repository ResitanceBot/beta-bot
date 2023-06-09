#ifndef EKF_HPP
#define EKF_HPP

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

struct pose {
  double x{0};
  double y{0};
  double z{0};
  double r{0};
  double p{0};
  double yaw{0};
  long double covariance[81];
};

constexpr double desv_tip_sigma_inicial{1};
constexpr double desv_tip_R_position{1};
constexpr double desv_tip_R_vel{1};
constexpr double desv_tip_R_orientation{1};
constexpr double desv_tip_Q_beacons{
    6.0}; // Based on "/beacons_gazebo/src/rssi_noise.cpp", line 36
constexpr double desv_tip_Q_or_rp{0.005};   // aprox
constexpr double desv_tip_Q_or_yaw{1.3e-2}; // aprox
constexpr double desv_tip_Q_IMU{0.005};
constexpr double desv_tip_Q_mag{1.3e-2};

class ExtendedKalmanFilter {
public:
  void initMatrix(pose InitialPose);
  void EKFPrediction(double AngVelX, double AngVelY, double AngVelZ,
                     double currentTimeStamp);
  void EKFUpdate(double dist1, double dist2,
                 double dist3, // Changes compared to "ekf_node" version
                 double dist4, double dist1_ant, double dist2_ant,
                 double dist3_ant, double dist4_ant, double magX, double magY,
                 double magZ, double LinAccX, double LinAccY, double LinAccZ,
                 double currentTimeStamp);
  inline pose GetEstimatedPose() {
    pose Pose;
    Pose.x = _nu(0);
    Pose.y = _nu(1);
    Pose.z = _nu(2);
    Pose.r = _nu(6);
    Pose.p = _nu(7);
    Pose.yaw = _nu(8);
    for (int i=0; i<9; i++) {
      for(int j=0; j<9; j++){
        Pose.covariance[i*9 + j] = _sigma(i,j);
      }
    }
    return Pose;
  };
  bool matrixInitialized{false};

  // Store the beacon's position of the beacon i
  inline void SetBeaconPosition(double xb, double yb, double zb, int i) {
    if (i > 0 && i < 5) {
      _xb[i - 1] = xb;
      _yb[i - 1] = yb;
      _zb[i - 1] = zb;
    }
  };

private:
  // Matrix
  Eigen::Matrix<double, 9, 1>
      _nu; // variable state vector: [x, y, z, vx, vy, vz, roll, pitch, yaw]'
  Eigen::Matrix<double, 9, 9> _sigma;
  Eigen::Matrix<double, 9, 9> _G;
  Eigen::Matrix<double, 9, 9> _R;
  Eigen::Matrix<double, 11, 9> _H;
  Eigen::Matrix<double, 11, 11> _Q;

  // Internal variables
  double _lastPredTimeStamp{0};
  double _lastUpdTimeStamp{0};

  // Beacons' position
  double _xb[4];
  double _yb[4];
  double _zb[4];
};

#endif