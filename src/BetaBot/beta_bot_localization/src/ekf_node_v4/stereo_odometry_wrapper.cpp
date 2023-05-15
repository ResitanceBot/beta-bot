/* WRAPPER TO PROVIDE THE VISUAL ODOMETRY STACK IMPLEMENTED WITH FUNCTIONALITY COMPATIBLE WITH OUR IMPLEMENTED EKF
   
   ARP PROJECT - GIERM
*/

/* HEADERS */
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <rtabmap_msgs/ResetPose.h>
#include <beta_bot_localization/PoseRPYWithCovariance.h>

/* STEREO_ODOMETRY_CLASS */
class StereoOdometryWrapper {
public:
    // Variable to check if 0.3 seconds pass without any readings from VO. In this case, we assume that VO is lost, and we have to reset it
    ros::Time timerVO;

    void TopicManager(ros::NodeHandle &nh) {
        _VO_sub     = nh.subscribe("/stereo_odometry", 10, &StereoOdometryWrapper::callbackVO, this);
        _loc_sub    = nh.subscribe("/estimated_localization/poseRPY", 10, &StereoOdometryWrapper::callbackLoc, this);
        _VO_pub     = nh.advertise<nav_msgs::Odometry>("/stereo_odometry_map", 10);
        resetClient = nh.serviceClient<rtabmap_msgs::ResetPose>("/reset_odom_to_pose");
    }

    void checkVOTimer(){
        std::cout << "[stereo_wrapper]: No new readings from VO. We assume that VO is lost. Resetting to our custom EKF localization..." << std::endl;
        this -> resetVO();
    }

    void resetVO(){
      // Call reset service and establish nav -> base_link = world -> base_link (from IMU MRU Static Model)
      /* This is the service we have to call:
      
      "/reset_odom_to_pose" (rtabmap_msgs/ResetPose)
      Restart odometry to specified transformation. Format: "x y z roll pitch yaw".
      
      */
      rtabmap_msgs::ResetPose srv;
      srv.request.x     = localization.x;
      srv.request.y     = localization.y;
      srv.request.z     = localization.z;
      srv.request.roll  = localization.roll;
      srv.request.pitch = localization.pitch;
      srv.request.yaw   = localization.yaw;

      if (resetClient.call(srv))
      {
        std::cout << "[stereo_wrapper]: Successfully called reset_odom_to_pose service" << std::endl;
      }
      else
      {
        std::cout << "[stereo_wrapper - ERROR]: Failed to call service reset_odom_to_pose" << std::endl;
      }

      timerVO = ros::Time::now();
    }

    /* CALLBACKS */
    // Obtain nav -> front_cam_link transform
    void callbackVO(const nav_msgs::Odometry msg) {
      timerVO = ros::Time::now();
      currentVOmsg = msg;

      // Obtain world -> nav transform
      tf::StampedTransform transform;
      try
      {
        listener.lookupTransform("nav", "world", ros::Time(0), transform);
      }
      catch (tf::TransformException& ex)
      {
        ROS_ERROR("Error al obtener la transformación: %s", ex.what());
      }
      geometry_msgs::PoseWithCovarianceStamped Map_To_Odom;
      Map_To_Odom.header.stamp = transform.stamp_;
      Map_To_Odom.header.frame_id = "world";
      Map_To_Odom.pose.pose.position.x = transform.getOrigin().x();
      Map_To_Odom.pose.pose.position.y = transform.getOrigin().y();
      Map_To_Odom.pose.pose.position.z = transform.getOrigin().z();
      Map_To_Odom.pose.pose.orientation.x = transform.getRotation().x();
      Map_To_Odom.pose.pose.orientation.y = transform.getRotation().y();
      Map_To_Odom.pose.pose.orientation.z = transform.getRotation().z();
      Map_To_Odom.pose.pose.orientation.w = transform.getRotation().w();

      // With base_link -> front_cam_link transform ( <origin xyz="1.50 -0.0 -0.25" rpy="0 0 0"/> ), compute world -> base_link
      geometry_msgs::PoseWithCovarianceStamped FrontCamLink_to_BaseLink;
      FrontCamLink_to_BaseLink.pose.pose.position.x = -1.5;
      FrontCamLink_to_BaseLink.pose.pose.position.y = 0.0;
      FrontCamLink_to_BaseLink.pose.pose.position.z = 0.25;

      nav_msgs::Odometry stereoOdometryMap;
      stereoOdometryMap.header.stamp = ros::Time::now();
      stereoOdometryMap.header.frame_id = "world";
      stereoOdometryMap.child_frame_id  = "base_link";

      stereoOdometryMap.pose.pose.position.x = currentVOmsg.pose.pose.position.x;// + FrontCamLink_to_BaseLink.pose.pose.position.x;
      stereoOdometryMap.pose.pose.position.y = currentVOmsg.pose.pose.position.y;// + FrontCamLink_to_BaseLink.pose.pose.position.y;
      stereoOdometryMap.pose.pose.position.z = currentVOmsg.pose.pose.position.z;// + FrontCamLink_to_BaseLink.pose.pose.position.z;

      stereoOdometryMap.pose.pose.orientation.x = currentVOmsg.pose.pose.orientation.x;
      stereoOdometryMap.pose.pose.orientation.y = currentVOmsg.pose.pose.orientation.y;
      stereoOdometryMap.pose.pose.orientation.z = currentVOmsg.pose.pose.orientation.z;
      stereoOdometryMap.pose.pose.orientation.w = currentVOmsg.pose.pose.orientation.w;

      stereoOdometryMap.twist = currentVOmsg.twist;

      stereoOdometryMap.pose.covariance = currentVOmsg.pose.covariance;

      // Publish world -> base_link transform
      _VO_pub.publish(stereoOdometryMap);
      }

      void callbackLoc(const beta_bot_localization::PoseRPYWithCovariance msg){
        localization = msg;
      }


private:
  ros::Subscriber _VO_sub;
  ros::Subscriber _loc_sub;
  ros::Publisher  _VO_pub;
  ros::ServiceClient resetClient;
  nav_msgs::Odometry currentVOmsg;
  tf::TransformListener listener;
  beta_bot_localization::PoseRPYWithCovariance localization;

};

/* MAIN */
int main(int argc, char **argv) {
  ros::init(argc, argv, "stereo_odometry_wrapper");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  StereoOdometryWrapper stereo_wrapper;
  stereo_wrapper.TopicManager(nh);

  // Debug msg
  ros::Rate rate(50);
  while (ros::ok()) {
    if((ros::Time::now()-stereo_wrapper.timerVO).toSec()>0.3) stereo_wrapper.resetVO();
    rate.sleep();
  }

  ros::waitForShutdown();
}

