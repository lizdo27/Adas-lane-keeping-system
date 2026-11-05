/**
 * @file gps_plugin.hpp
 * @brief GPS localization plugin for Gazebo Sim
 * 
 * This plugin simulates a GPS sensor by publishing the model's pose data.
 * It provides position and orientation information via Gazebo Transport
 * at a configurable update rate.
 * 
 * Topic: /automobile/localisation (gz.msgs.Pose)
 * Default update rate: 4 Hz (250ms)
 * 
 * @copyright Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
 */

#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Name.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/pose.pb.h>
#include <memory>
#include <string>

namespace gps
{   
    /**
     * @class GPS
     * @brief Gazebo plugin simulating GPS sensor functionality
     * 
     * Publishes the model's world pose at regular intervals to simulate
     * GPS position updates. The update rate can be configured via SDF.
     */
    class GPS: public gz::sim::System,
               public gz::sim::ISystemConfigure,
               public gz::sim::ISystemPostUpdate
    {
    private: 
        gz::sim::Entity entity;  ///< Entity this plugin is attached to
        gz::sim::Model model;  ///< Model interface for pose access
        gz::transport::Node node;  ///< Gazebo Transport node
        gz::transport::Node::Publisher gpsPub;  ///< Publisher for GPS data
        std::string topicName{"/automobile/localisation_raw"};  ///< Raw GPS topic (bridge to ROS2; sensor_converter adds 1 s delay and republishes as GpsTagPose)
        std::chrono::steady_clock::duration update_period{std::chrono::milliseconds(250)};  ///< Update period (250ms = 4Hz)
        std::chrono::steady_clock::duration last_update_time{0};  ///< Last update time

    public: 
        /**
         * @brief Default constructor
         */
        GPS() = default;
        
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
         * @brief Called after each simulation update to publish GPS data
         * @param _info Update information including simulation time
         * @param _ecm Entity Component Manager for reading pose data
         */
        void PostUpdate(const gz::sim::UpdateInfo &_info,
                       const gz::sim::EntityComponentManager &_ecm) override;
    };
};
