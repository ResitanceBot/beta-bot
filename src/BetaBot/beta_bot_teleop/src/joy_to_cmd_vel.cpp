#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <cstdio>

#define BREAKOUT_JOYSTICKS 0.4
#define INC_SPEED 0.2
#define DEC_SPEED 0.1
#define MAX_SPEED 3
#define MIN_SPEED 0.1

float SPEED = 1;
float SPEED_ANT = SPEED;

ros::Publisher pub;
ros::Subscriber sub;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    geometry_msgs::Twist twist;

    // AJUSTE SPEED
    SPEED_ANT = SPEED;
    
    if(joy->axes[7] == 1){
      SPEED += INC_SPEED;
      if(SPEED > MAX_SPEED)
        SPEED = MAX_SPEED;
    }
    else if(joy->axes[7] == -1){
      if (SPEED >= 1) SPEED -= INC_SPEED;
      else SPEED -= DEC_SPEED;
      if(SPEED < MIN_SPEED)
        SPEED = MIN_SPEED;
    }

    if(joy->axes[6] == 1){
      SPEED = MIN_SPEED;
    }
    else if(joy->axes[6] == -1){
      SPEED = MAX_SPEED;
    }

    // UP / DOWN (Joystick izq, Eje Vertical == axes[1]) (Acelerador)
    if(joy->axes[1] < -BREAKOUT_JOYSTICKS) twist.linear.z = SPEED * float(-1.0);
    else if(joy->axes[1] > BREAKOUT_JOYSTICKS) twist.linear.z = SPEED * float(1.0);
    // ROTATE LEFT / RIGHT (Joystick izq, Eje Horizontal == axes[0]) (Rotacion)
    if(joy->axes[0] < -BREAKOUT_JOYSTICKS) twist.angular.z = 3*SPEED * float(-1.0);
    else if(joy->axes[0] > BREAKOUT_JOYSTICKS) twist.angular.z = 3*SPEED * float(1.0);
    // FORWARD / BACKWARD (Joystick der, Eje Vertical == axes[4]) (Avance/Retroceso)
    if(joy->axes[4] < -BREAKOUT_JOYSTICKS) twist.linear.x = SPEED * float(-1.0);
    else if(joy->axes[4] > BREAKOUT_JOYSTICKS) twist.linear.x = SPEED * float(1.0);
    // LEFT / RIGHT (Joystick der, Eje Horizontal == axes[3]) (Der/izq)
    if(joy->axes[3] < -BREAKOUT_JOYSTICKS) twist.linear.y = SPEED * float(-1.0);
    else if(joy->axes[3] > BREAKOUT_JOYSTICKS) twist.linear.y = SPEED * float(1.0);


    // INTERFAZ TERMINAL
    printf("------------------------- \n");
    if(joy->axes[1] < -BREAKOUT_JOYSTICKS) printf("DOWN \n");
    else if(joy->axes[1] > BREAKOUT_JOYSTICKS) printf("UP \n");
    // ROTATE LEFT / RIGHT (Joystick izq, Eje Horizontal == axes[0]) (Rotacion)
    if(joy->axes[0] < -BREAKOUT_JOYSTICKS) printf("ROT.RIGHT \n");
    else if(joy->axes[0] > BREAKOUT_JOYSTICKS) printf("ROT.LEFT \n");
    // FORWARD / BACKWARD (Joystick der, Eje Vertical == axes[4]) (Avance/Retroceso)
    if(joy->axes[4] < -BREAKOUT_JOYSTICKS) printf("BACKWARD \n");
    else if(joy->axes[4] > BREAKOUT_JOYSTICKS) printf("FORWARD \n");
    // LEFT / RIGHT (Joystick der, Eje Horizontal == axes[3]) (Der/izq)
    if(joy->axes[3] < -BREAKOUT_JOYSTICKS) printf("MOV.RIGHT \n");
    else if(joy->axes[3] > BREAKOUT_JOYSTICKS) printf("MOV.LEFT \n");
    // SPEED
    if(SPEED != SPEED_ANT) printf("SPEED = %f \n", SPEED);

    pub.publish(twist);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joy_to_cmd_vel");

    ros::NodeHandle nh;
    pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    sub = nh.subscribe("joy", 10, joyCallback);

    while (ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}