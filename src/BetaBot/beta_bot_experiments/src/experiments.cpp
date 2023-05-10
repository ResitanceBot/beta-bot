/* LIBRERÍAS */
#include <ros/ros.h>
#include <math.h>
#include "beta_bot_experiments/PoseRPYWithCovariance.h"
#include <iostream>
#include <fstream>
#include <string>

/* VARIABLES GLOBALES */
float x_GT, y_GT, z_GT, roll_GT, pitch_GT, yaw_GT;       /* Coordenadas del robot reales (Ground Truth) */
float x_LOC, y_LOC, z_LOC, roll_LOC, pitch_LOC, yaw_LOC; /* Coordenadas del robot calculadas por el EKF */

// Apertura de fichero
// std::ofstream ficheroDatosExp("//home/aglora/beta-bot/src/BetaBot/beta_bot_experiments/tests/"+nombreFile+".txt",std::ios::app); //fichero datos experimento en formato CSV

/* CALLBACKS */
void GTCallback(const beta_bot_experiments::PoseRPYWithCovariance &msg)
{
  x_GT = msg.x;
  y_GT = msg.y;
  z_GT = msg.z;
  roll_GT = msg.roll;
  pitch_GT = msg.pitch;
  yaw_GT = msg.yaw;
}

void LOCCallback(const beta_bot_experiments::PoseRPYWithCovariance &msg)
{
  x_LOC = msg.x;
  y_LOC = msg.y;
  z_LOC = msg.z;
  roll_LOC = msg.roll;
  pitch_LOC = msg.pitch;
  yaw_LOC = msg.yaw;
}

/* MAIN */

int main(int argc, char **argv)
{
  ros::init(argc, argv, "experiments");

  ros::NodeHandle n;
  ros::Subscriber gt_sub = n.subscribe("/ground_truth/poseRPY", 10, GTCallback);
  ros::Subscriber loc_sub = n.subscribe("/estimated_localization/poseRPY", 10, LOCCallback);

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  /* BUCLE INFINITO */
  int bucle = 1;
  ros::Rate r(50.0);
  ros::Time contadorTiempo = ros::Time::now();

  std::string nombreFile;
  nombreFile = argv[1];

  // Apertura de fichero
  std::ofstream ficheroDatosExp("//home/aglora/beta-bot/src/BetaBot/beta_bot_experiments/tests/" + nombreFile + ".txt", std::ios::app); // fichero datos experimento en formato CSV
  //std::ofstream ficheroDatosExp("//home/aglora/beta-bot/src/BetaBot/beta_bot_experiments/tests/" + nombreFile + ".txt", std::ios::app); // fichero datos experimento en formato CSV

  while (n.ok())
  {

    ros::spinOnce();
    current_time = ros::Time::now();

    ficheroDatosExp << ros::Time::now() << "," << x_GT << "," << y_GT << "," << z_GT << "," << roll_GT << "," << pitch_GT << "," << yaw_GT << "," << x_LOC << "," << y_LOC << "," << z_LOC << "," << roll_LOC << "," << pitch_LOC << "," << yaw_LOC << std::endl;

    last_time = current_time;
    r.sleep();
  }
}