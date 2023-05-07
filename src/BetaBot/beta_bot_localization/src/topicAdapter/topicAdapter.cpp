/* NODO QUE:
   - CONVIERTE EL TOPIC /pressure_height, DE TIPO geometry_msgs/PointStamped A geometry_msgs/PoseWithCovarianceStamped (/pose_height)
   - CONVIERTE EL TOPIC /magnetic, DE TIPO /geometry_msgs/Vector3Stamped A sensor_msgs/MagneticField (/magnetic/converted)
   - TOMA EL TOPIC DE SALIDA DEL NODO "imu_filter_node", QUE EN EL LAUNCH DE "robot_localization" FUSIONA MAGNETÓMETRO CON IMU, Y LE 
     QUITA 90º, PUES LA SIMULACIÓN DE ROS TRABAJA CON UN MARCO DE REFERENCIA ENU (X = ESTE; Y = NORTE; Z = UP), Y, POR TANTO, EL YAW
     DEL MAGNETÓMETRO TIENE QUE MEDIR YAW = 0º EN EL ESTE (DONDE ESTÁ LA X) Y NO EN EL NORTE, QUE ES COMO REALMENTE FUNCIONA
     (PARA MÁS INFORMACIÓN, VER NORMA REP 103 DE ROS: https://www.ros.org/reps/rep-0103.html)
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
sensor_msgs::MagneticField lecturaMagnetometro;
sensor_msgs::Imu CorrectedMagImu;

/* variables globales para el cálculo de la covarianza */
int tamColaMag = 0;
int tamColaAltura = 0;
int maxTamCola = 50;
double roll, pitch, yaw;

//Vectores de muestras para el cálculo de covarianzas:
//EL CÁLCULO DE LAS COVARIANZAS DEBE HACERSE EN LAS CALLBACKS, DADO QUE SE METE UN NUEVO ELEMENTO EN LAS COLAS CADA VEZ QUE LLEGA UNA NUEVA MUESTRA (CADA VEZ QUE 
//SALTA UNA VEZ MÁS EL CALLBACK CONCRETO)
std::vector<float> colaX, colaY, colaZ, colaAltura;
float varX, varY, varZ, varAltura;


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
  ros::Subscriber imuMag_sub       = n.subscribe("/imu/data", 50, imuMagCallback);                               /* Nombre del topic donde vamos a suscribirnos para obtener las lecturas de la IMU + Magnetómetro */

  ros::Publisher altura_pub        = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/pose_height", 50);  /* Nombre del topic donde vamos a publicar */
  ros::Publisher magnetometro_pub  = n.advertise<sensor_msgs::MagneticField>("/magnetic/converted", 50);         /* Nombre del topic donde vamos a publicar */
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

    altura_pub.publish(altura);
    magnetometro_pub.publish(lecturaMagnetometro);
    imuMag_pub.publish(CorrectedMagImu);

    last_time = current_time;
    r.sleep();
  }
}

