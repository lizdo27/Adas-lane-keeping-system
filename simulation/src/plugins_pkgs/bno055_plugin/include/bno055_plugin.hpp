/**
 * @file bno055_plugin.hpp
 * @brief BNO055 IMU sensor plugin for Gazebo Sim
 * 
 * This plugin simulates a BNO055 Inertial Measurement Unit (IMU) sensor.
 * It publishes orientation and angular velocity data via Gazebo Transport.
 * The data can be bridged to ROS 2 topics using ros_gz_bridge.
 * 
 * Topic: /automobile/IMU (gz.msgs.Vector3d - simplified IMU data)
 * 
 * @copyright Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
 */

#pragma once
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/vector3d.pb.h>
#include <string>

namespace bno055
{   
    /**
     * @class BNO055
     * @brief Gazebo plugin simulating BNO055 IMU sensor
     * 
     * Publishes orientation data from the model's pose to simulate
     * an IMU sensor. Updates are published at each PostUpdate cycle.
     */
    class BNO055: 
        public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPostUpdate
    {
    private: 
        gz::sim::Entity entity;  ///< Entity this plugin is attached to
        gz::transport::Node node;  ///< Gazebo Transport node for communication
        gz::transport::Node::Publisher imuPub;  ///< Publisher for IMU data
        std::chrono::steady_clock::time_point lastUpdateTime;  ///< Last update timestamp
        bool initialized{false};  ///< Initialization status flag
        std::string topicName{"/automobile/IMU"};  ///< Topic name for IMU data

    public: 
        /**
         * @brief Default constructor
         */
        BNO055();
        
        /**
         * @brief Configure the plugin
         * @param _entity Entity this plugin is attached to
         * @param _sdf SDF element containing plugin configuration
         * @param _ecm Entity Component Manager
         * @param _eventMgr Event manager
         */
        void Configure(const gz::sim::Entity &_entity,
                      const std::shared_ptr<const sdf::Element> &_sdf,
                      gz::sim::EntityComponentManager &_ecm,
                      gz::sim::EventManager &_eventMgr) override;
        
        /**
         * @brief Called after each simulation update to publish IMU data
         * @param _info Update information including simulation time
         * @param _ecm Entity Component Manager for reading entity data
         */
        void PostUpdate(const gz::sim::UpdateInfo &_info,
                       const gz::sim::EntityComponentManager &_ecm) override;        
    };
};

