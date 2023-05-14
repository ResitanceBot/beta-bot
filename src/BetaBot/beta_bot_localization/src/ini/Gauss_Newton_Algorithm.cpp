#include "beacons_gazebo/BeaconSimPose.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <beta_bot_localization/IniLocalization.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

// Subscriptores
ros::Subscriber accelerometer;
ros::Subscriber magnetometer;
ros::Subscriber gps;
ros::Subscriber barometer;
ros::Subscriber beacons_pos;
ros::Subscriber beacons_dist_1;
ros::Subscriber beacons_dist_2;
ros::Subscriber beacons_dist_3;
ros::Subscriber beacons_dist_4;

// Publicadores
ros::Publisher res_ini;

#define MAX_ITERATIONS 50
#define TOLERANCE 0.000000001
#define NUM_MUESTRAS 10
#define N_BALIZAS 4

// Cabeceras funciones
double distance(double x1, double y1, double z1, double x2, double y2,
                double z2);
void GN_ini_algorithm(void);

// Variables
double x, y, z, xini = 0, yini = 0, zini = 0, roll, pitch, yaw;
double d1, d2, d3, d4, dx, dy, dz, e0, e;
double xb1, yb1, zb1, xb2, yb2, zb2, xb3, yb3, zb3, xb4, yb4, zb4;
double db1 = -1, db2 = -1, db3 = -1, db4 = -1;
int contb1 = 0, contb2 = 0, contb3 = 0, contb4 = 0, contgps = 0, contbar = 0,
    contmag = 0, contacc = 0;
bool contb[N_BALIZAS] = {false, false, false, false};

Eigen::Matrix<double, 4, 3> J;
Eigen::Matrix<double, 4, 1> E;
Eigen::Matrix<double, 3, 1> Inc;

int i = 0;

void accelerometer_Callback(const sensor_msgs::Imu &msg) {
  static double accX;
  static double accY;
  static double accZ;
  accX += (double)msg.linear_acceleration.x;
  accY += (double)msg.linear_acceleration.y;
  accZ += (double)msg.linear_acceleration.z;
  contacc++;
  if (contacc >= NUM_MUESTRAS) {
    accX = accX / NUM_MUESTRAS;
    accY = accY / NUM_MUESTRAS;
    accZ = accZ / NUM_MUESTRAS;
    roll = atan2(accY, sqrt(accX * accX + accZ * accZ));
    pitch = atan2(-accX, sqrt(accY * accY + accZ * accZ));
    std::cout << "Acelerometro: Obtenidos roll: " << roll
              << "y pitch: " << pitch << std::endl;
    accelerometer.shutdown();
  }
}

void magnetometer_Callback(const sensor_msgs::MagneticField &msg) {
  static double compX;
  static double compY;
  compX += (double)msg.magnetic_field.x;
  compY += (double)msg.magnetic_field.y;
  contmag++;
  if (contmag >= NUM_MUESTRAS) {
    compX = compX / NUM_MUESTRAS;
    compY = compY / NUM_MUESTRAS;
    yaw = atan2(compY, compX);
    std::cout << "Magnetometro: Obtenido yaw: " << yaw << std::endl;
    magnetometer.shutdown();
  }
}

void gps_Callback(const nav_msgs::Odometry &msg) {
  xini += (double)msg.pose.pose.position.x;
  yini += (double)msg.pose.pose.position.y;
  contgps++;
  if (contgps >= NUM_MUESTRAS) {
    xini = xini / NUM_MUESTRAS;
    yini = yini / NUM_MUESTRAS;
    std::cout << "GPS: Obtenidas Xini: " << xini << " ||  Yini:" << yini
              << std::endl;
    gps.shutdown();
  }
}

void barometer_Callback(const geometry_msgs::PoseWithCovarianceStamped &msg) {
  zini += (double)msg.pose.pose.position.z;
  contbar++;
  if (contbar >= NUM_MUESTRAS) {
    zini = zini / NUM_MUESTRAS;
    std::cout << "Barómetro: Obtenida Zini: " << zini << std::endl;
    barometer.shutdown();
  }
}

void beacons_pos_Callback(const beacons_gazebo::BeaconSimPose &msg) {
  int beacon_id = std::stoi(msg.id.substr(msg.id.size() - 1));
  switch (beacon_id) {
  case 1:
    xb1 = msg.position.x;
    yb1 = msg.position.y;
    zb1 = msg.position.z;
    std::cout << "Baliza 1: Coordenadas: x:" << xb1 << " y:" << yb1
              << " z:" << zb1 << std::endl;
    contb[beacon_id - 1] = true;
    break;
  case 2:
    xb2 = msg.position.x;
    yb2 = msg.position.y;
    zb2 = msg.position.z;
    std::cout << "Baliza 2: Coordenadas: x:" << xb2 << " y:" << yb2
              << " z:" << zb2 << std::endl;
    contb[beacon_id - 1] = true;
    break;
  case 3:
    xb3 = msg.position.x;
    yb3 = msg.position.y;
    zb3 = msg.position.z;
    std::cout << "Baliza 3: Coordenadas: x:" << xb3 << " y:" << yb3
              << " z:" << zb3 << std::endl;
    contb[beacon_id - 1] = true;
    break;
  case 4:
    xb4 = msg.position.x;
    yb4 = msg.position.y;
    zb4 = msg.position.z;
    std::cout << "Baliza 4: Coordenadas: x:" << xb4 << " y:" << yb4
              << " z:" << zb4 << std::endl;
    contb[beacon_id - 1] = true;
    break;
  }
  if ((contb[0] == true) && (contb[1] == true) && (contb[2] == true) &&
      (contb[3] == true)) {
    beacons_pos.shutdown();
  }
}

void beacons_dist_1_Callback(const std_msgs::Float64::ConstPtr &msg) {
  db1 += (double)msg->data;
  contb1++;
  if (contb1 >= NUM_MUESTRAS) {
    db1 = db1 / NUM_MUESTRAS;
    std::cout << "Baliza 1: Recibida distancia:" << db1 << std::endl;
    beacons_dist_1.shutdown();
  }
}

void beacons_dist_2_Callback(const std_msgs::Float64::ConstPtr &msg) {
  db2 += (double)msg->data;
  contb2++;
  if (contb2 >= NUM_MUESTRAS) {
    db2 = db2 / NUM_MUESTRAS;
    std::cout << "Baliza 2: Recibida distancia:" << db2 << std::endl;
    beacons_dist_2.shutdown();
  }
}

void beacons_dist_3_Callback(const std_msgs::Float64::ConstPtr &msg) {
  db3 += (double)msg->data;
  contb3++;
  if (contb3 >= NUM_MUESTRAS) {
    db3 = db3 / NUM_MUESTRAS;
    std::cout << "Baliza 3: Recibida distancia:" << db3 << std::endl;
    beacons_dist_3.shutdown();
  }
}

void beacons_dist_4_Callback(const std_msgs::Float64::ConstPtr &msg) {
  db4 += (double)msg->data;
  contb4++;
  if (contb4 >= NUM_MUESTRAS) {
    db4 = db4 / NUM_MUESTRAS;
    std::cout << "Baliza 4: Recibida distancia:" << db4 << std::endl;
    beacons_dist_4.shutdown();
  }
}

void GN_ini_algorithm(void) {

  while (i <= MAX_ITERATIONS) {

    d1 = distance(x, y, z, xb1, yb1, zb1);
    d2 = distance(x, y, z, xb2, yb2, zb2);
    d3 = distance(x, y, z, xb3, yb3, zb3);
    d4 = distance(x, y, z, xb4, yb4, zb4);

    // Calculo Jacobiano
    J(0, 0) = (x - xb1) / d1;
    J(0, 1) = (y - yb1) / d1;
    J(0, 2) = (z - zb1) / d1;
    J(1, 0) = (x - xb2) / d2;
    J(1, 1) = (y - yb2) / d2;
    J(1, 2) = (z - zb2) / d2;
    J(2, 0) = (x - xb3) / d3;
    J(2, 1) = (y - yb3) / d3;
    J(2, 2) = (z - zb3) / d3;
    J(3, 0) = (x - xb4) / d4;
    J(3, 1) = (y - yb4) / d4;
    J(3, 2) = (z - zb4) / d4;

    // Error anterior
    e0 = pow((d1 - db1), 2) + pow((d2 - db2), 2) + pow((d3 - db3), 2) +
         pow((d4 - db4), 2);

    // Matriz de errores
    E << (db1 - d1), (db2 - d2), (db3 - d3), (db4 - d4);

    // Solucion ecuacion para inc
    Inc = -(((J.transpose()) * J).inverse()) * (J.transpose()) * E;

    dx = Inc(0);
    dy = Inc(1);
    dz = Inc(2);

    x = x - dx;
    y = y - dy;
    z = z - dz;

    d1 = distance(x, y, z, xb1, yb1, zb1);
    d2 = distance(x, y, z, xb2, yb2, zb2);
    d3 = distance(x, y, z, xb3, yb3, zb3);
    d4 = distance(x, y, z, xb4, yb4, zb4);

    e = pow((d1 - db1), 2) + pow((d2 - db2), 2) + pow((d3 - db3), 2) +
        pow((d4 - db4), 2);

    if (abs(e - e0) < TOLERANCE) {
      i = MAX_ITERATIONS;
      std::cout << "Alcanzada Tolerancia " << std::endl;
    } else {
      std::cout << "iteracion: " << i << std::endl;
    }
    i++;
  }
}

double distance(double x1, double y1, double z1, double x2, double y2,
                double z2) {
  return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "gn_ini");
  ros::NodeHandle nh;

  beta_bot_localization::IniLocalization msg;

  magnetometer = nh.subscribe("/magnetic/converted", 10, magnetometer_Callback);
  accelerometer = nh.subscribe("/raw_imu", 10, accelerometer_Callback);
  gps = nh.subscribe("odometry/gps", 10, gps_Callback);
  barometer = nh.subscribe("pose_height", 10, barometer_Callback);
  beacons_pos =
      nh.subscribe("beacons_gazebo/beacons", 10, beacons_pos_Callback);
  beacons_dist_1 =
      nh.subscribe("quadrotor/odom_rssi_beacon_1", 10, beacons_dist_1_Callback);
  beacons_dist_2 =
      nh.subscribe("quadrotor/odom_rssi_beacon_2", 10, beacons_dist_2_Callback);
  beacons_dist_3 =
      nh.subscribe("quadrotor/odom_rssi_beacon_3", 10, beacons_dist_3_Callback);
  beacons_dist_4 =
      nh.subscribe("quadrotor/odom_rssi_beacon_4", 10, beacons_dist_4_Callback);

  res_ini = nh.advertise<beta_bot_localization::IniLocalization>("iniLoc", 1);

  bool flagRes = false;

  ros::Rate rate(50);

  ros::AsyncSpinner spinner(4);
  spinner.start();

  while (ros::ok()) {
    if ((contb1 < NUM_MUESTRAS) || (contb2 < NUM_MUESTRAS) ||
        (contb3 < NUM_MUESTRAS) || (contb4 < NUM_MUESTRAS) ||
        (contb[0] == false) || (contb[1] == false) || (contb[2] == false) ||
        (contb[3] == false) || (contgps < NUM_MUESTRAS) ||
        (contbar < NUM_MUESTRAS) || (contmag < NUM_MUESTRAS) ||
        (contacc < NUM_MUESTRAS)) {
      // std::cout << "Esperando distancias de balizas... " << std::endl;
    } else if (!flagRes) {
      x = xini;
      y = yini;
      z = zini;
      GN_ini_algorithm();

      // RESULTADOS
      std::cout << std::endl;
      std::cout << "----------------RESULTADOS---------------" << std::endl;
      std::cout << std::endl;
      std::cout << "INICIALIZACION GPS + BAROMETRO " << std::endl;
      std::cout << "x: " << xini << std::endl;
      std::cout << "y: " << yini << std::endl;
      std::cout << "z: " << zini << std::endl;
      std::cout << std::endl;
      std::cout << "INICIALIZACION BALIZAS " << std::endl;
      std::cout << "x: " << x << std::endl;
      std::cout << "y: " << y << std::endl;
      std::cout << "z: " << z << std::endl;
      std::cout << std::endl;
      std::cout << "INICIALIZACION ORIENTACIÓN " << std::endl;
      std::cout << "roll: " << roll << std::endl;
      std::cout << "pitch: " << pitch << std::endl;
      std::cout << "yaw: " << yaw << std::endl;
      std::cout << std::endl;

      // PUBLICACION RESULTADOS
      msg.gps_bar.x = xini;
      msg.gps_bar.y = yini;
      msg.gps_bar.z = zini;
      msg.beacons.x = x;
      msg.beacons.y = y;
      msg.beacons.z = z;
      msg.orientation.x = roll;
      msg.orientation.y = pitch;
      msg.orientation.z = yaw;

      flagRes = true;

      res_ini.publish(msg);
      break;
    }
    // res_ini.publish(msg);
    // ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
