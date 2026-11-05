/**
 * @file carlikerobot_ros_plugin.hpp
 * @brief Gazebo Sim plugin for car-like robot with ROS 2 integration
 * 
 * This plugin integrates the car-like robot kinematics with Gazebo Sim and
 * provides communication via Gazebo Transport. It handles command messages
 * for speed and steering control, and publishes feedback.
 * 
 * @copyright Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
 */

#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <mutex>

#include "carlikerobot.hpp"
#include "rapidjson/document.h"

namespace carlikerobot
{
    /**
     * @class IRobotCommandSetter
     * @brief Interface for setting robot control commands
     * 
     * Defines the interface for classes that can receive and apply
     * speed and steering commands to the robot.
     */
    class IRobotCommandSetter
    {
        public:
            /**
             * @brief Apply the current speed and steering commands
             */
            virtual void setCommand()=0;
            float f_steer;  ///< Current steering angle command (degrees)
            float f_speed;  ///< Current speed command (m/s)
    };       
    
    /// Shared pointer type for robot command setter interface
    typedef std::shared_ptr<IRobotCommandSetter> IRobotCommandSetterPtr;

    /**
     * @class CMessageHandler
     * @brief Handler for incoming command messages from Gazebo Transport
     * 
     * Processes JSON-formatted command messages for speed, steering, and brake
     * controls. Validates messages and sends acknowledgment feedback.
     */
    class CMessageHandler
    {
        public:
            /**
             * @brief Constructor for message handler
             * @param feedbackPub Publisher for feedback messages
             * @param setter Robot command setter interface for applying commands
             */
            CMessageHandler(gz::transport::Node::Publisher& feedbackPub, IRobotCommandSetter* setter);
            ~CMessageHandler();
            
            /**
             * @brief Callback for incoming command messages
             * @param msg String message containing JSON command data
             * 
             * Expected JSON format:
             * - Speed: {"action":"1", "speed":<value>}
             * - Steering: {"action":"2", "steerAngle":<value>}
             * - Brake: {"action":"3", "steerAngle":<value>}
             */
            void OnMsgCommand(const gz::msgs::StringMsg& msg);

        private:
            void unknownMessage();           ///< Handle unknown message types
            void brakeMessage(float msg_val);  ///< Process brake command
            void spedMessage(float msg_val);   ///< Process speed command
            void sterMessage(float msg_val);   ///< Process steering command
            
            IRobotCommandSetter* _robotSetter;  ///< Robot command interface
            gz::transport::Node::Publisher& _feedbackPublisher;  ///< Feedback publisher
    };
    
    /**
     * @class CCarLikeRobotRosPlugin
     * @brief Main Gazebo Sim plugin for car-like robot control
     * 
     * Implements the Gazebo System plugin interface to control a car-like robot
     * with Ackermann steering geometry. Subscribes to command messages via Gazebo
     * Transport and publishes feedback. Uses EntityComponentManager for direct
     * simulation entity manipulation.
     * 
     * Topics:
     * - Subscribes: /automobile/command (JSON commands)
     * - Publishes: /automobile/feedback (acknowledgments)
     */
    class CCarLikeRobotRosPlugin: 
        public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate,
        public IRobotCommandSetter
    {
        public:
            /**
             * @brief Default constructor
             */
            CCarLikeRobotRosPlugin();
            
            /**
             * @brief Configure the plugin
             * @param _entity Entity this plugin is attached to
             * @param _sdf SDF element containing plugin configuration
             * @param _ecm Entity Component Manager for simulation access
             * @param _eventMgr Event manager (unused)
             */
            void Configure(const gz::sim::Entity &_entity,
                          const std::shared_ptr<const sdf::Element> &_sdf,
                          gz::sim::EntityComponentManager &_ecm,
                          gz::sim::EventManager &_eventMgr) override;
            
            /**
             * @brief Called before each simulation update step
             * @param _info Update information including simulation time
             * @param _ecm Entity Component Manager for simulation access
             */
            void PreUpdate(const gz::sim::UpdateInfo &_info,
                          gz::sim::EntityComponentManager &_ecm) override;
            
            /**
             * @brief Apply current speed and steering commands to robot
             */
            void setCommand() override;
            
        private:
            bool LoadParameterJoints(const std::shared_ptr<const sdf::Element> &_sdf,
                                    gz::sim::EntityComponentManager &_ecm);
            
            IWheelsSpeedPtr _rearWheelsSpeedPtr;
            IWheelsSpeedPtr _frontWheelSpeedPtr;
            ISteerWheelsPtr _steerWheelsAnglePtr;
            
            gz::sim::Entity entity;
            gz::sim::EntityComponentManager* ecm{nullptr};
            gz::transport::Node node;
            gz::transport::Node::Publisher feedbackPub;
            std::shared_ptr<CMessageHandler> _messageHandler;
            bool initialized{false};
            
            // Store pending commands to apply in PreUpdate
            std::mutex commandMutex;
            bool hasPendingCommand{false};
            float pendingSpeed{0.0};
            float pendingSteer{0.0};
    };
};
