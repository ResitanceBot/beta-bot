//
// Created by Vasily Yuryev on 26.06.2021.
//
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <string>
//#include <beacon_gazebo_sim/ReceiverIn.h>
//#include <beacon_gazebo_sim/rssi_noise.h>
#include <vector>

namespace gazebo
{
    class BeaconsSyncWorldPlugin : public WorldPlugin
    {
    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
        {
            if (!ros::isInitialized()) {
                ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                                         << "Load the Gazebo system plugin  in the gazebo_ros package)");
                return;
            }
            ROS_INFO("BeaconsWorldPlugin: Hello Wolrd!");
            this->world = _parent;
            this->gz_node = transport::NodePtr(new transport::Node());
            this->gz_node->Init(this->world->Name());
//            this->node->

            if (_sdf->HasElement("update_rate")) {
                this->update_rate = _sdf->Get<float>("update_rate");
                ROS_INFO("BeaconsSyncWorldPlugin: update_rate: %f", this->update_rate);
            } else {
                ROS_ERROR("Haha update_rate");
                return;
            }
            this->sync_pub = this->gz_node->Advertise<msgs::Empty>("/beacon_sim_gazebo/sync", 1);

            this->l_u_time = common::Time(0.0);
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    boost::bind(&BeaconsSyncWorldPlugin::OnUpdate, this, _1));
        }
        void OnUpdate(const common::UpdateInfo &_info) {
            common::Time simTime = _info.simTime;
            common::Time elapsed = simTime - this->l_u_time;
            if (elapsed >= common::Time(1.0 / this->update_rate)) {
                msgs::Empty msg;
                this->sync_pub->Publish(msg);
                l_u_time = simTime;
            }
        }
    private:
        transport::NodePtr gz_node;
        physics::WorldPtr world;
        transport::PublisherPtr sync_pub;
        event::ConnectionPtr updateConnection;
        common::Time l_u_time;
        float update_rate;
    };


    GZ_REGISTER_WORLD_PLUGIN(BeaconsSyncWorldPlugin)
}