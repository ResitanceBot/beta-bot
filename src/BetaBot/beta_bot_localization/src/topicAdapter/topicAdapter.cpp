/* NODO QUE:
   - CONVIERTE EL TOPIC /pressure_height, DE TIPO geometry_msgs/PointStamped A geometry_msgs/PoseWithCovarianceStamped (/pose_height)
   - CONVIERTE EL TOPIC /magnetic, DE TIPO /geometry_msgs/Vector3Stamped A sensor_msgs/MagneticField (/magnetic/converted)
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
#include <math.h>                                       //Para el uso de las funciones "abs", "pow" y "fmod"; definición de pi = M_PI
#include <iostream>                                     //Para el uso de malloc
#include <cstdlib>                                      //Para el uso de malloc
#include <vector>                                       //Para "size()" => cálculo del número de elementos de un vector


/* VARIABLES GLOBALES */
geometry_msgs::PoseWithCovarianceStamped altura;
geometry_msgs::PoseWithCovarianceStamped GT_xy;
geometry_msgs::PoseWithCovarianceStamped poseGPS;
sensor_msgs::MagneticField lecturaMagnetometro;
sensor_msgs::Imu CorrectedRawImu;
sensor_msgs::Imu RawImuBias;
sensor_msgs::Imu CorrectedMagImu;
geometry_msgs::PoseStamped EKFpose;

/* variables globales para el cálculo de la covarianza */
int tamColaMag = 0;
int tamColaImu = 0;
int tamColaAltura = 0;
int tamColaGT = 0;
int maxTamCola = 50;
double roll, pitch, yaw;

//Vectores de muestras para el cálculo de covarianzas:
//EL CÁLCULO DE LAS COVARIANZAS DEBE HACERSE EN LAS CALLBACKS, DADO QUE SE METE UN NUEVO ELEMENTO EN LAS COLAS CADA VEZ QUE LLEGA UNA NUEVA MUESTRA (CADA VEZ QUE 
//SALTA UNA VEZ MÁS EL CALLBACK CONCRETO)
std::vector<float> colaX, colaY, colaZ, colaAltura, colaGTx, colaGTy, cola_orR, cola_orP, cola_orY, cola_avX, cola_avY, cola_avZ, cola_laX, cola_laY, cola_laZ;
float varX, varY, varZ, varAltura, varGTx, varGTy, var_orR, var_orP, var_orY, var_avX, var_avY, var_avZ, var_laX, var_laY, var_laZ;


/* FUNCIONES */
float calcularMedia(std::vector<float> v){
    float suma = 0;
    for(int i=0; i<v.size(); i++){
        suma += v[i];
    }
    return suma/v.size();
}

float calcularVarianza(std::vector<float> v){
    float media = calcularMedia(v);
    float suma = 0;
    for(int i=0; i<v.size()-1; i++){
        suma += pow(v[i]-media, 2);
    }
    return suma/(v.size()-1);                         //Varianza como estimador insesgado (dividiendo por n-1 en lugar de n)
}


/* CALLBACKS */
void alturaCallback(const geometry_msgs::PointStamped& msg) {  
    altura.header = msg.header;
    altura.pose.pose.position.z = msg.point.z;

    //CÁLCULO DE LAS COVARIANZAS
    if(tamColaAltura >= maxTamCola){
      //Eliminar el elemento más antiguo de la cola para hacer espacio al nuevo
      colaAltura.erase(colaAltura.begin());          
      tamColaAltura--;                              
    }
     //Añadir nuevo valor leído a la cola
     tamColaAltura++;
     colaAltura.push_back(altura.pose.pose.position.z);

     if(tamColaAltura >= maxTamCola){
      //Calcular la varianza de las medidas sólo cuando la cola esté llena
      //MÉTODO 1: CÁLCULO DE LA VARIANZA DE LAS 50 MEDIDAS ANTERIOR A ESTA                           
      varAltura = calcularVarianza(colaAltura); 

      //RECONSTRUCCIÓN DE LAS COVARIANZAS
      altura.pose.covariance[14] = varAltura;
    }
}

void magnetometroCallback(const geometry_msgs::Vector3Stamped& msg) {  
    lecturaMagnetometro.header = msg.header;
    lecturaMagnetometro.magnetic_field.x = msg.vector.x;
    lecturaMagnetometro.magnetic_field.y = msg.vector.y;
    lecturaMagnetometro.magnetic_field.z = msg.vector.z;

    //CÁLCULO DE LAS COVARIANZAS
    if(tamColaMag >= maxTamCola){
      //Eliminar el elemento más antiguo de la cola para hacer espacio al nuevo
      colaX.erase(colaX.begin()); 
      colaY.erase(colaY.begin());
      colaZ.erase(colaZ.begin());           
      tamColaMag--;                              
    }
     //Añadir nuevo valor leído a la cola
     tamColaMag++;
     colaX.push_back(lecturaMagnetometro.magnetic_field.x);
     colaY.push_back(lecturaMagnetometro.magnetic_field.y);
     colaZ.push_back(lecturaMagnetometro.magnetic_field.z);

     if(tamColaMag >= maxTamCola){
      //Calcular la varianza de las medidas sólo cuando la cola esté llena
      //MÉTODO 1: CÁLCULO DE LA VARIANZA DE LAS 50 MEDIDAS ANTERIOR A ESTA                           
      varX = calcularVarianza(colaX);
      varY = calcularVarianza(colaY);
      varZ = calcularVarianza(colaZ);  

      //RECONSTRUCCIÓN DE LAS COVARIANZAS
      lecturaMagnetometro.magnetic_field_covariance[0] = varX;
      lecturaMagnetometro.magnetic_field_covariance[4] = varY;
      lecturaMagnetometro.magnetic_field_covariance[8] = varZ;
    }
} 

void gtCallback(const nav_msgs::Odometry& msg){  
  GT_xy.header = msg.header;
  GT_xy.pose = msg.pose;

  //CÁLCULO DE LAS COVARIANZAS
  if(tamColaGT >= maxTamCola){
    //Eliminar el elemento más antiguo de la cola para hacer espacio al nuevo
    colaGTx.erase(colaGTx.begin());
    colaGTy.erase(colaGTy.begin());           
    tamColaGT--;                              
  }
   //Añadir nuevo valor leído a la cola
   tamColaGT++;
   colaGTx.push_back(GT_xy.pose.pose.position.x);
   colaGTy.push_back(GT_xy.pose.pose.position.y);
 if(tamColaGT >= maxTamCola){
    //Calcular la varianza de las medidas sólo cuando la cola esté llena
    //MÉTODO 1: CÁLCULO DE LA VARIANZA DE LAS 50 MEDIDAS ANTERIOR A ESTA                           
    varGTx = calcularVarianza(colaGTx);
    varGTy = calcularVarianza(colaGTy);  
  //RECONSTRUCCIÓN DE LAS COVARIANZAS
    GT_xy.pose.covariance[0] = varGTx;
    GT_xy.pose.covariance[7] = varGTy;
  }
}

void imuBiasCallback(const sensor_msgs::Imu& msg){  
  RawImuBias = msg;
}

void imuCallback(const sensor_msgs::Imu& msg){  
  CorrectedRawImu = msg;

  CorrectedRawImu.orientation.x = msg.orientation.x - RawImuBias.orientation.x;
  CorrectedRawImu.orientation.y = msg.orientation.y - RawImuBias.orientation.y;
  CorrectedRawImu.orientation.z = msg.orientation.z - RawImuBias.orientation.z;
  CorrectedRawImu.orientation.w = msg.orientation.w - RawImuBias.orientation.w;

  CorrectedRawImu.angular_velocity.x = msg.angular_velocity.x - RawImuBias.angular_velocity.x;
  CorrectedRawImu.angular_velocity.y = msg.angular_velocity.y - RawImuBias.angular_velocity.y;
  CorrectedRawImu.angular_velocity.z = msg.angular_velocity.z - RawImuBias.angular_velocity.z;

  CorrectedRawImu.linear_acceleration.x = msg.linear_acceleration.x - RawImuBias.linear_acceleration.x;
  CorrectedRawImu.linear_acceleration.y = msg.linear_acceleration.y - RawImuBias.linear_acceleration.y;
  CorrectedRawImu.linear_acceleration.z = msg.linear_acceleration.z - RawImuBias.linear_acceleration.z;

  tf::Quaternion q(
    CorrectedRawImu.orientation.x,
    CorrectedRawImu.orientation.y,
    CorrectedRawImu.orientation.z,
    CorrectedRawImu.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);

  //CÁLCULO DE LAS COVARIANZAS
  if(tamColaImu >= maxTamCola){
    //Eliminar el elemento más antiguo de la cola para hacer espacio al nuevo
    cola_orR.erase(cola_orR.begin());
    cola_orP.erase(cola_orP.begin()); 
    cola_orY.erase(cola_orY.begin());  

    cola_avX.erase(cola_avX.begin());
    cola_avY.erase(cola_avY.begin());   
    cola_avZ.erase(cola_avZ.begin());

    cola_laX.erase(cola_laX.begin());
    cola_laY.erase(cola_laY.begin());   
    cola_laZ.erase(cola_laZ.begin());      
    tamColaImu--;                              
  }
    //Añadir nuevo valor leído a la cola
    tamColaImu++;
    cola_orR.push_back(roll);
    cola_orP.push_back(pitch);
    cola_orY.push_back(yaw);

    cola_avX.push_back(CorrectedRawImu.angular_velocity.x);
    cola_avY.push_back(CorrectedRawImu.angular_velocity.y);
    cola_avZ.push_back(CorrectedRawImu.angular_velocity.z);

    cola_laX.push_back(CorrectedRawImu.linear_acceleration.x);
    cola_laY.push_back(CorrectedRawImu.linear_acceleration.y);
    cola_laZ.push_back(CorrectedRawImu.linear_acceleration.z);

   if(tamColaImu >= maxTamCola){
    //Calcular la varianza de las medidas sólo cuando la cola esté llena
    //MÉTODO 1: CÁLCULO DE LA VARIANZA DE LAS 50 MEDIDAS ANTERIOR A ESTA                           
    var_orR = calcularVarianza(cola_orR);
    var_orP = calcularVarianza(cola_orP);
    var_orY = calcularVarianza(cola_orY);

    var_avX = calcularVarianza(cola_avX);
    var_avY = calcularVarianza(cola_avY);
    var_avZ = calcularVarianza(cola_avZ);

    var_laX = calcularVarianza(cola_laX);
    var_laY = calcularVarianza(cola_laY);
    var_laZ = calcularVarianza(cola_laZ);
 
    //RECONSTRUCCIÓN DE LAS COVARIANZAS
    CorrectedRawImu.orientation_covariance[0] = var_orR;
    CorrectedRawImu.orientation_covariance[4] = var_orP;
    CorrectedRawImu.orientation_covariance[8] = var_orY;

    CorrectedRawImu.angular_velocity_covariance[0] = var_avX;
    CorrectedRawImu.angular_velocity_covariance[4] = var_avY;
    CorrectedRawImu.angular_velocity_covariance[8] = var_avZ;

    CorrectedRawImu.linear_acceleration_covariance[0] = var_laX;
    CorrectedRawImu.linear_acceleration_covariance[4] = var_laY;
    CorrectedRawImu.linear_acceleration_covariance[8] = var_laZ;
  }
}

void GPSCallback(const nav_msgs::Odometry& msg){  
  poseGPS.header = msg.header;
  poseGPS.pose.pose.position.x = msg.pose.pose.position.x;
  poseGPS.pose.pose.position.y = msg.pose.pose.position.y;
  poseGPS.pose.covariance = msg.pose.covariance;
}

void EKFCallback(const nav_msgs::Odometry& msg){  
  EKFpose.header = msg.header;
  EKFpose.pose = msg.pose.pose;
}

void imuMagCallback(const sensor_msgs::Imu& msg){  
  CorrectedMagImu = msg;

  //Offset de -90º para que el yaw de la IMU apunte al este geográfico y no al norte (todos nuestros frames tienen como referencia en X el este y no el norte)
  tf::Quaternion q(
    msg.orientation.x,
    msg.orientation.y,
    msg.orientation.z,
    msg.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  yaw -= M_PI/2;  // Conversión de norte a este (offset de -90º)

  tf2::Quaternion qResultado;
  qResultado.setRPY(roll,pitch,yaw);
  qResultado=qResultado.normalize();
  // Tipos de dato incompatibles: se hacen compatibles haciendo que qResultado.x sea qResultado.x()
  CorrectedMagImu.orientation.x = qResultado.x();
  CorrectedMagImu.orientation.y = qResultado.y();
  CorrectedMagImu.orientation.z = qResultado.z();
  CorrectedMagImu.orientation.w = qResultado.w();

  //Publicar lecturas de la IMU con magnetómetro corregidas por el nuevo topic
  //imuMag_pub.publish(CorrectedMagImu); => Ya se hace en main
}

/* MAIN */

int main(int argc, char** argv){
  ros::init(argc, argv, "topicAdapter_publisher");

  //PUBLICACIONES Y SUSCRIPCIONES
  ros::NodeHandle n;
  ros::Subscriber altura_sub       = n.subscribe("/pressure_height", 50, alturaCallback);                        /* Nombre del topic donde vamos a suscribirnos para obtener la altura actual del robot */
  ros::Subscriber magnetometro_sub = n.subscribe("/magnetic", 50, magnetometroCallback);                         /* Nombre del topic donde vamos a suscribirnos para obtener la lectura actual del magnetómetro */
  ros::Subscriber gt_sub           = n.subscribe("/ground_truth/state", 50, gtCallback);                         /* Nombre del topic donde vamos a suscribirnos para obtener la pose (Ground Truth) actual del robot */
  ros::Subscriber imuBias_sub      = n.subscribe("/raw_imu/bias", 50, imuBiasCallback);                          /* Nombre del topic donde vamos a suscribirnos para obtener el offset actual de la IMU */
  ros::Subscriber imu_sub          = n.subscribe("/raw_imu", 50, imuCallback);                                   /* Nombre del topic donde vamos a suscribirnos para obtener la lectura actual de la IMU */
  ros::Subscriber gps_sub          = n.subscribe("/odometry/gps", 50, GPSCallback);                              /* Nombre del topic donde vamos a suscribirnos para obtener la lectura actual del GPS */
  ros::Subscriber ekf_sub          = n.subscribe("/odometry/filtered/global", 50, EKFCallback);                  /* Nombre del topic donde vamos a suscribirnos para obtener la lectura actual del EKF Global */
  ros::Subscriber imuMag_sub       = n.subscribe("/imu/data", 50, imuMagCallback);                               /* Nombre del topic donde vamos a suscribirnos para obtener las lecturas de la IMU + Magnetómetro */


  ros::Publisher altura_pub        = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/pose_height", 50);  /* Nombre del topic donde vamos a publicar */
  ros::Publisher magnetometro_pub  = n.advertise<sensor_msgs::MagneticField>("/magnetic/converted", 50);         /* Nombre del topic donde vamos a publicar */
  ros::Publisher gt_pub            = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/pose_fake", 50);    /* Nombre del topic donde vamos a publicar */
  ros::Publisher imu_pub           = n.advertise<sensor_msgs::Imu>("/raw_imu/corrected", 50);                    /* Nombre del topic donde vamos a publicar */
  ros::Publisher gps_pub           = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/pose/gps", 50);     /* Nombre del topic donde vamos a publicar */
  ros::Publisher ekf_pub           = n.advertise<geometry_msgs::PoseStamped>("/pose/filtered/global", 50);       /* Nombre del topic donde vamos a publicar */
  ros::Publisher imuMag_pub        = n.advertise<sensor_msgs::Imu>("/imu/data/MagCorrected", 50);                /* Nombre del topic donde vamos a publicar */


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
    altura.header.frame_id  = "world";
    GT_xy.header.frame_id   = "world";
    poseGPS.header.frame_id = "world";

    altura_pub.publish(altura);
    magnetometro_pub.publish(lecturaMagnetometro);
    gt_pub.publish(GT_xy);
    imu_pub.publish(CorrectedRawImu);
    imuMag_pub.publish(CorrectedMagImu);
    gps_pub.publish(poseGPS);
    ekf_pub.publish(EKFpose);

    last_time = current_time;
    r.sleep();
  }
}

