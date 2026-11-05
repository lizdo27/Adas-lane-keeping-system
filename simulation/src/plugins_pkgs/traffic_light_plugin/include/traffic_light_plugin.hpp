#pragma once

#include <thread>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components/Light.hh>
#include <gz/sim/components/LightCmd.hh>
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/byte.hpp>

namespace trafficLight
{   
    enum TrafficLightColor {RED, YELLOW, GREEN};

    class TrafficLight: 
        public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate
    {
    private: 
        gz::sim::Entity entity;
        gz::sim::Entity greenLensEntity{gz::sim::kNullEntity};
        gz::sim::Entity yellowLensEntity{gz::sim::kNullEntity};
        gz::sim::Entity redLensEntity{gz::sim::kNullEntity};
        rclcpp::Node::SharedPtr m_ros_node;
        rclcpp::Subscription<std_msgs::msg::Byte>::SharedPtr m_ros_subscriber;
        std::string name;
        uint8_t currentLightState{0};
        bool initialized{false};

    public: 
        TrafficLight();
        void Configure(const gz::sim::Entity &_entity,
                      const std::shared_ptr<const sdf::Element> &_sdf,
                      gz::sim::EntityComponentManager &_ecm,
                      gz::sim::EventManager &_eventMgr) override;
        void PreUpdate(const gz::sim::UpdateInfo &_info,
                      gz::sim::EntityComponentManager &_ecm) override;
        void OnRosMsg(std_msgs::msg::Byte);

    private: 
        void SetLightState(gz::sim::EntityComponentManager &_ecm,
                          gz::sim::Entity lightEntity,
                          bool state);
    };
};
