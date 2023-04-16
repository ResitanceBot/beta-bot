//
// Created by Vasily Yuryev on 19.06.2021.
//
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <beacon_gazebo_sim/BeaconSimPose.h>
#include <string>

namespace gazebo {
    class BeaconModelPlugin : public ModelPlugin {
    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override {
            if (!ros::isInitialized()) {
                ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                                         << "Load the Gazebo system plugin  in the gazebo_ros package)");
                return;
            }

            ROS_INFO("BeaconModelPlugin: Hello World!");
            this->model = _parent;

            if (_sdf->HasElement("beacon_link")) {
                std::string beacon_link_name = _sdf->Get<std::string>("beacon_link");
                this->beacon_link = _parent->GetLink(beacon_link_name);

                ROS_INFO("beacon_link_name: %s", beacon_link_name.c_str());
                this->beacon_name = std::string("beacon__") + this->model->GetName();
                if (this->beacon_link == NULL)
                {
                    std::vector<physics::LinkPtr> links = _parent->GetLinks();
                    for (int i = 0; i < links.size(); ++i)
                    {
                        ROS_INFO("Link[%d]: %s", i, links[i]->GetName().c_str());
                    }
                    ROS_ERROR("beacon_link Is NULL");
                    return;
                }
                ROS_INFO("BeaconModelPlugin: name: %s", this->beacon_name.c_str());
            } else {
                ROS_ERROR("Haha beacon_link");
                return;
            }

            ros::NodeHandle n;
            this->beacon_sim_pose_pub = n.advertise<beacon_gazebo_sim::BeaconSimPose>("/beacon_gazebo_sim/beacons", 1);
            this->l_u_time = common::Time(0.0);


            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    boost::bind(&BeaconModelPlugin::OnUpdate, this, _1));
        }

        void OnUpdate(const common::UpdateInfo &_info) {
            common::Time simTime = _info.simTime;
            common::Time elapsed = simTime - this->l_u_time;
            if (elapsed >= common::Time(1.0 / this->update_rate)) {
                this->l_u_time = simTime;

                std::string beacon_name = std::string("beacon__") + model->GetName();
                auto beacon_p = this->beacon_link->WorldPose().Pos();
                beacon_gazebo_sim::BeaconSimPose msg;
                msg.time_stamp = ros::Time::now();
                msg.frame_id = "world";
                msg.id = this->beacon_name;
                msg.position.x = beacon_p.X();
                msg.position.y = beacon_p.Y();
                msg.position.z = beacon_p.Z();
                this->beacon_sim_pose_pub.publish(msg);
            }
        }

    private:
        physics::ModelPtr model;
        event::ConnectionPtr updateConnection;
        physics::LinkPtr beacon_link;
        std::string beacon_name;
        float update_rate = 1;
        common::Time l_u_time;
        ros::Publisher beacon_sim_pose_pub;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(BeaconModelPlugin)
}