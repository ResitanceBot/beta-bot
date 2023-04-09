//
// Created by Vasily Yuryev on 19.06.2021.
//

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <string>
#include <beacon_gazebo_sim/ReceiverIn.h>
#include <beacon_gazebo_sim/ReceiverInSyncPacked.h>
#include <beacon_gazebo_sim/rssi_noise.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>

namespace gazebo {
    class ReceiverModelPlugin : public ModelPlugin {
    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override {

            if (!ros::isInitialized()) {
                ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                                         << "Load the Gazebo system plugin  in the gazebo_ros package)");
                return;
            }

            ROS_INFO("ReceiverModelPlugin: Hello World!");

            this->model = _parent;
            this->world = _parent->GetWorld();
            this->gz_node = transport::NodePtr(new transport::Node());
            this->gz_node->Init(this->world->Name());

            ROS_INFO("ReceiverModelPlugin: model links");

            for (auto &link : this->model->GetLinks()) {
                std::string link_name = link->GetName();
                printf("\tlink:%s\n",link_name.c_str());
            }
            if (_sdf->HasElement("receiver_link")) {
                std::string receiver_link_name = _sdf->Get<std::string>("receiver_link");
                this->receiver_link = _parent->GetLink(receiver_link_name);

                ROS_INFO("receiver_link_name: %s", receiver_link_name.c_str());
                this->receiver_name = std::string("receiver__") + _parent->GetName();
                if (this->receiver_link == NULL)
                {
                    std::vector<physics::LinkPtr> links = _parent->GetLinks();
                    for (int i = 0; i < links.size(); ++i)
                    {
                        ROS_INFO("Link[%d]: %s", i, links[i]->GetName().c_str());
                    }
                    ROS_ERROR("receiver_link Is NULL");
                    return;
                }
            } else {
                ROS_ERROR("Haha receiver_link");
                return;
            }

            if (_sdf->HasElement("send_sim_pose")) {
                this->send_sim_pose = _sdf->Get<bool>("send_sim_pose");
            }

            if (_sdf->HasElement("use_sync")) {
                this->use_sync = _sdf->Get<bool>("use_sync");
            }

            if (_sdf->HasElement("packed_publishing")) {
                this->packed_publishing = _sdf->Get<bool>("packed_publishing");
            }

            if (_sdf->HasElement("update_rate")) {
                this->update_rate = _sdf->Get<float>("update_rate");
                ROS_INFO("ReceiverModelPlugin: update_rate: %f", this->update_rate);
            } else if (!this->use_sync) {
                ROS_ERROR("Haha update_rate");
                return;
            }

            ROS_INFO("ReceiverModelPlugin: receiver_name: %s", this->receiver_name.c_str());
            ROS_INFO("ReceiverModelPlugin: send_sim_pose: %d", this->send_sim_pose);
            ROS_INFO("ReceiverModelPlugin: use_sync: %d", this->use_sync);
            ROS_INFO("ReceiverModelPlugin: packed_publishing: %d", this->packed_publishing);

            ros::NodeHandle n;

            if (this->send_sim_pose)
                this->sim_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/beacon_gazebo_sim/" + this->receiver_name + "/pose", 1);
            if (!this->packed_publishing)
                this->receiver_in_msgs_publisher = n.advertise<beacon_gazebo_sim::ReceiverIn>(
                        "/" + this->model->GetName() + "/" + this->receiver_name + "/receiver_in_msgs", 1);
            else
                this->receiver_in_msgs_packed_publisher = n.advertise<beacon_gazebo_sim::ReceiverInSyncPacked>(
                        "/" + this->model->GetName() + "/" + this->receiver_name + "/receiver_in_msgs_packed", 1);

            this->l_u_time = common::Time(0.0);
            if (!this->use_sync) {
                this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                        boost::bind(&ReceiverModelPlugin::OnUpdateRate, this, _1));
            }
            else {
                this->sync_sub = this->gz_node->Subscribe(std::string("/beacon_sim_gazebo/sync"), &ReceiverModelPlugin::OnSync, this);
            }
        }
        void OnUpdateRate(const common::UpdateInfo &_info) {
            common::Time simTime = _info.simTime;
            common::Time elapsed = simTime - this->l_u_time;
            if (elapsed >= common::Time(1.0/this->update_rate)) {

                this->UpdateFun(simTime);
                this->l_u_time = simTime;
            }
        }
        void OnSync(ConstEmptyPtr &msg) {
            common::Time simTime = this->world->SimTime();
            common::Time elapsed = simTime - this->l_u_time;
            this->UpdateFun(simTime);
            this->l_u_time = simTime;
        }
    private: void sendSimPose() {
            geometry_msgs::PoseStamped msg;
            msg.header.frame_id = "world";
            msg.header.stamp = ros::Time::now();
            auto r_p = this->receiver_link->WorldPose();

            msg.pose.position.x = r_p.X();
            msg.pose.position.y = r_p.Y();
            msg.pose.position.z = r_p.Z();
            sim_pose_pub.publish(msg);
        }
    private: void UpdateFun(const common::Time& simTime){
            if (this->send_sim_pose)
                this->sendSimPose();

            physics::Model_V models = this->world->Models();
            std::vector<beacon_gazebo_sim::ReceiverIn> p_msgs;

            for (auto &model : models) {
                physics::Link_V model_links = model->GetLinks();
                for (auto &link : model_links) {
                    if (link->GetName().find("beacon") == 0) {
                        std::string beacon_name = std::string("beacon__") + model->GetName();

                        if (this->rssi_noise_generators.find(beacon_name) == this->rssi_noise_generators.end()){
                            this->rssi_noise_generators[beacon_name] = beacon_gazebo_sim::RSSINoise(this->m_rssi);
                        }

                        auto beacon_p = link->WorldPose().Pos();
                        auto receiver_p = this->receiver_link->WorldPose().Pos();

                        double d = std::sqrt(
                                std::pow(beacon_p.X() - receiver_p.X(), 2) +
                                std::pow(beacon_p.Y() - receiver_p.Y(), 2) +
                                std::pow(beacon_p.Z() - receiver_p.Z(), 2)
                        );

                        beacon_gazebo_sim::ReceiverIn msg;
                        msg.time_stamp = ros::Time::now();
                        msg.rssi = this->rssi_noise_generators[beacon_name].getRSSI(d, simTime.Double());
                        msg.id = beacon_name;
                        msg.m_rssi = this->m_rssi;
                        if (!this->packed_publishing)
                            this->receiver_in_msgs_publisher.publish(msg);
                        else
                            p_msgs.push_back(msg);
                    }
                }
            }
            if (this->packed_publishing) {
                beacon_gazebo_sim::ReceiverInSyncPacked p_msg_out;
                p_msg_out.time_stamp = ros::Time::now();
                p_msg_out.data = p_msgs;
                this->receiver_in_msgs_packed_publisher.publish(p_msg_out);
            }
        }
    private:
        ignition::transport::Node ign_node;
        transport::NodePtr gz_node;
        physics::ModelPtr model;
        event::ConnectionPtr updateConnection;
        physics::LinkPtr receiver_link;
        std::string receiver_name;
        bool send_sim_pose = false;
        physics::WorldPtr world;
        float update_rate;
        std::map<std::string, beacon_gazebo_sim::RSSINoise> rssi_noise_generators;
        common::Time l_u_time;
        ros::Publisher receiver_in_msgs_publisher;
        bool use_sync = false;
        const double m_rssi = -60.0; // TODO: get m_rssi from config or world
        transport::SubscriberPtr sync_sub;
        ros::Publisher sim_pose_pub;
        bool packed_publishing = false;
        ros::Publisher receiver_in_msgs_packed_publisher;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ReceiverModelPlugin)
}