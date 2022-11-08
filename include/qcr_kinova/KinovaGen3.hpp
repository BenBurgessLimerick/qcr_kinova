#pragma once

#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include <mutex>

// Kinova API Includes
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>

#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>

namespace k_api = Kinova::Api;




#include "KinovaJoint.hpp"

#define N_JOINTS 7

class KinovaGen3 : public hardware_interface::RobotHW {
    public:
        KinovaGen3(ros::NodeHandle);
        void kinova_api_init();
        void register_interfaces();
        void read();
        void write();

    private:
        hardware_interface::JointStateInterface _joint_state_interface;
        hardware_interface::VelocityJointInterface _velocity_joint_interface;
        KinovaJoint _joints[N_JOINTS];

        std::mutex _lock;

        k_api::Base::BaseClient* _api_base;
        k_api::BaseCyclic::BaseCyclicClient* _api_base_cyclic;
        k_api::BaseCyclic::Feedback _api_base_feedback;
        k_api::BaseCyclic::Command  _api_base_command;

};