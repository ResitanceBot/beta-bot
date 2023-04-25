/* NODO QUE DECIDE SI USAREMOS EL GROUND TRUTH O LA LOCALIZACIÓN IMPLEMENTADA PARA CONTROLAR EL UAV
   Grupo Trabajo Localización ARP */


/* CABECERAS */
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>                                      //Para convertir los cuaternios en rpy (y quedarnos solamente con yaw)
#include <nav_msgs/Odometry.h>                          //Tipo de dato al que queremos convertir las lecturas del sónar
#include <geometry_msgs/Vector3Stamped.h>               //Tipo de dato original de las lecturas del magnetómetro
#include <sensor_msgs/Range.h>                          //Tipo de dato original de las lecturas del altímetro sónar
#include <sensor_msgs/MagneticField.h>                  //Tipo de dato al que queremos convertir las lecturas del magnetómetro
#include <sensor_msgs/Imu.h>                            //Tipo de dato de la IMU
#include <geometry_msgs/PoseWithCovarianceStamped.h>    //Tipo de dato de actualización de pose
#include <geometry_msgs/PoseStamped.h>                  //Tipo de dato de pose para la GUI del quadrotor
#include <cmath>                                        //Para el uso de las funciones "abs", "pow" y "fmod"
#include <iostream>                                     //Para el uso de malloc
#include <cstdlib>                                      //Para el uso de malloc
#include <vector>                                       //Para "size()" => cálculo del número de elementos de un vector


/* VARIABLES GLOBALES */
nav_msgs::Odometry echoLocalization;
sensor_msgs::Imu echoImu;


/* CALLBACKS */
void localizationCallback(const nav_msgs::Odometry& msg){  
  echoLocalization = msg;
}

void imuCallback(const sensor_msgs::Imu& msg){  
  echoImu = msg;
}


/* MAIN */
int main(int argc, char** argv){
  ros::init(argc, argv, "topicAdapterControls_publisher");

  //PUBLICACIONES Y SUSCRIPCIONES
  ros::NodeHandle n;
  ros::Subscriber localization_sub = n.subscribe("/ground_truth/state", 50, localizationCallback);         /* Nombre del topic donde vamos a suscribirnos DEBERÍA SER /odometry/filtered/global */ 
  ros::Subscriber imu_sub          = n.subscribe("/raw_imu3", 50, imuCallback);                            /* Nombre del topic donde vamos a suscribirnos */

  ros::Publisher localization_pub = n.advertise<nav_msgs::Odometry>("/state", 50);                         /* Nombre del topic donde vamos a publicar */
  ros::Publisher imu_pub          = n.advertise<sensor_msgs::Imu>("/imu", 50);                             /* Nombre del topic donde vamos a publicar */

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Time contadorTiempo = ros::Time::now();
  /* BUCLE INFINITO */
  ros::Rate r(50.0);
  while(n.ok()){   

    ros::spinOnce();               
    current_time = ros::Time::now();

    //PUBLICACIONES
    localization_pub.publish(echoLocalization);
    imu_pub.publish(echoImu);

    last_time = current_time;
    r.sleep();
  }
}

