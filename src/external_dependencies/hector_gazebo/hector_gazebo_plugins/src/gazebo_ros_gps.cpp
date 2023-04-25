//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <hector_gazebo_plugins/gazebo_ros_gps.h>
#include <gazebo/physics/physics.hh>
#include <random>

// WGS84 constants
static const double equatorial_radius = 6378137.0;
static const double flattening = 1.0/298.257223563;
static const double excentrity2 = 2*flattening - flattening*flattening;

// default reference position
static const double DEFAULT_REFERENCE_LATITUDE  = 49.9;
static const double DEFAULT_REFERENCE_LONGITUDE = 8.9;
static const double DEFAULT_REFERENCE_HEADING   = 0.0;
static const double DEFAULT_REFERENCE_ALTITUDE  = 0.0;

namespace gazebo {

GazeboRosGps::GazeboRosGps()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosGps::~GazeboRosGps()
{
  updateTimer.Disconnect(updateConnection);

  dynamic_reconfigure_server_position_.reset();
  dynamic_reconfigure_server_velocity_.reset();
  dynamic_reconfigure_server_status_.reset();

  node_handle_->shutdown();
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosGps::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  world = _model->GetWorld();

  // load parameters
  if (!_sdf->HasElement("robotNamespace"))
    namespace_.clear();
  else
    namespace_ = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString();

  if (!_sdf->HasElement("bodyName"))
  {
    link = _model->GetLink();
    link_name_ = link->GetName();
  }
  else {
    link_name_ = _sdf->GetElement("bodyName")->GetValue()->GetAsString();
    link = _model->GetLink(link_name_);
  }

  if (!link)
  {
    ROS_FATAL("GazeboRosGps plugin error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }

  // default parameters
  frame_id_ = "/world";
  fix_topic_ = "/odometry/gps";
  velocity_topic_ = "/gps_velocity";

  reference_latitude_  = DEFAULT_REFERENCE_LATITUDE;
  reference_longitude_ = DEFAULT_REFERENCE_LONGITUDE;
  reference_heading_   = DEFAULT_REFERENCE_HEADING * M_PI/180.0;
  reference_altitude_  = DEFAULT_REFERENCE_ALTITUDE;

  if (_sdf->HasElement("frameId"))
    frame_id_ = _sdf->GetElement("frameId")->GetValue()->GetAsString();

  if (_sdf->HasElement("topicName"))
    fix_topic_ = "/odometry/gps";
    //fix_topic_ = _sdf->GetElement("topicName")->GetValue()->GetAsString();

  if (_sdf->HasElement("velocityTopicName"))
    velocity_topic_ = _sdf->GetElement("velocityTopicName")->GetValue()->GetAsString();

  if (_sdf->HasElement("referenceLatitude"))
    _sdf->GetElement("referenceLatitude")->GetValue()->Get(reference_latitude_);

  if (_sdf->HasElement("referenceLongitude"))
    _sdf->GetElement("referenceLongitude")->GetValue()->Get(reference_longitude_);

  if (_sdf->HasElement("referenceHeading"))
    if (_sdf->GetElement("referenceHeading")->GetValue()->Get(reference_heading_))
      reference_heading_ *= M_PI/180.0;

  if (_sdf->HasElement("referenceAltitude"))
    _sdf->GetElement("referenceAltitude")->GetValue()->Get(reference_altitude_);
  velocity_.header.frame_id = frame_id_;

  position_error_model_.Load(_sdf);
  velocity_error_model_.Load(_sdf, "velocity");

  // calculate earth radii
  double temp = 1.0 / (1.0 - excentrity2 * sin(reference_latitude_ * M_PI/180.0) * sin(reference_latitude_ * M_PI/180.0));
  double prime_vertical_radius = equatorial_radius * sqrt(temp);
  radius_north_ = prime_vertical_radius * (1 - excentrity2) * temp;
  radius_east_  = prime_vertical_radius * cos(reference_latitude_ * M_PI/180.0);

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  node_handle_ = new ros::NodeHandle(namespace_);
  gps_pose_publisher_ = node_handle_->advertise<nav_msgs::Odometry>(fix_topic_, 10);
  velocity_publisher_ = node_handle_->advertise<geometry_msgs::Vector3Stamped>(velocity_topic_, 10);

  // setup dynamic_reconfigure servers
  dynamic_reconfigure_server_position_.reset(new dynamic_reconfigure::Server<SensorModelConfig>(ros::NodeHandle(*node_handle_, fix_topic_ + "/position")));
  dynamic_reconfigure_server_velocity_.reset(new dynamic_reconfigure::Server<SensorModelConfig>(ros::NodeHandle(*node_handle_, fix_topic_ + "/velocity")));
  dynamic_reconfigure_server_status_.reset(new dynamic_reconfigure::Server<GNSSConfig>(ros::NodeHandle(*node_handle_, fix_topic_ + "/status")));
  dynamic_reconfigure_server_position_->setCallback(boost::bind(&SensorModel3::dynamicReconfigureCallback, &position_error_model_, _1, _2));
  dynamic_reconfigure_server_velocity_->setCallback(boost::bind(&SensorModel3::dynamicReconfigureCallback, &velocity_error_model_, _1, _2));

  Reset();

  // connect Update function
  updateTimer.setUpdateRate(4.0);
  updateTimer.Load(world, _sdf);
  updateConnection = updateTimer.Connect(boost::bind(&GazeboRosGps::Update, this));
}

void GazeboRosGps::Reset()
{
  updateTimer.Reset();
  position_error_model_.reset();
  velocity_error_model_.reset();
}



////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosGps::Update()
{
  common::Time sim_time = world->SimTime();
  double dt = updateTimer.getTimeSinceLastUpdate().Double();

  ignition::math::Pose3d pose = link->WorldPose();
  std::cout << "Pose del link base_link:" << pose << std::endl;

  //ignition::math::Vector3d velocity = velocity_error_model_(link->WorldLinearVel(), dt);
  //ignition::math::Vector3d position = position_error_model_(pose.Pos()); //,dt


  // An offset error in the velocity is integrated into the position error for the next timestep.
  // Note: Usually GNSS receivers have almost no drift in the velocity signal.
  gps_pose_.header.stamp = ros::Time(sim_time.sec, sim_time.nsec);
  velocity_.header.stamp = gps_pose_.header.stamp;

  //Modificación ARP: ruido gaussiano sobre "pose.pos" del Ground Truth
  // Definir generador de números aleatorios según distribución gaussiana, con un error de entre 0 y 1
  const double mean = 0.0;
  const double stddev = 3;
  auto dist = std::bind(std::normal_distribution<double>{mean, stddev},std::mt19937(std::random_device{}()));
  std::cout << "Valor de la distribución gaussiana = " << dist() << std::endl;
  double poseX, poseY, poseZ;
  poseX = pose.X() + dist();
  poseY = pose.Y() + dist();
  poseZ = pose.Z() + dist();

  //Meter pose.pos en el objeto "gps_pose_"
  gps_pose_.header.frame_id = "world";
  gps_pose_.pose.pose.position.x = poseX;
  gps_pose_.pose.pose.position.y = poseY;
  gps_pose_.pose.pose.position.z = poseZ;
  gps_pose_.pose.covariance[0]  = stddev*stddev;
  gps_pose_.pose.covariance[7]  = stddev*stddev;
  gps_pose_.pose.covariance[14] = stddev*stddev;
  /*
  Covarianza[36]:
  00 01 02 03 04 05 
  06 07 08 09 10 11
  12 13 14 15 16 17
  ...
  */

  gps_pose_publisher_.publish(gps_pose_);
  velocity_publisher_.publish(velocity_);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosGps)

} // namespace gazebo
