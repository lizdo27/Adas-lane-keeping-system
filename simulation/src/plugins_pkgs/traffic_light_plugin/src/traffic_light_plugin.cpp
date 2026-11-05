#include "traffic_light_plugin.hpp"
#include <gz/sim/components/Link.hh>
#include <gz/sim/Link.hh>

#define DEBUG false

namespace trafficLight
{   
    TrafficLight::TrafficLight() {}

    void TrafficLight::Configure(const gz::sim::Entity &_entity,
                                 const std::shared_ptr<const sdf::Element> &_sdf,
                                 gz::sim::EntityComponentManager &_ecm,
                                 gz::sim::EventManager &/*_eventMgr*/)
    {
        this->entity = _entity;
        this->initialized = false;

        auto nameComp = _ecm.Component<gz::sim::components::Name>(_entity);
        this->name = nameComp ? nameComp->Data() : "unknown";

        if (!rclcpp::ok())
        {
            int argc = 0;
            char** argv = nullptr;
            rclcpp::init(argc, argv);
        }

        rclcpp::NodeOptions node_options;
        node_options.arguments({"--ros-args", "-r", "__node:=traffic_light_plugin_node_" + this->name});
        this->m_ros_node = std::make_shared<rclcpp::Node>("traffic_light_plugin_" + this->name, node_options);

        gz::sim::Model model(_entity);
        this->greenLensEntity = model.LinkByName(_ecm, "green_lens");
        this->yellowLensEntity = model.LinkByName(_ecm, "yellow_lens");
        this->redLensEntity = model.LinkByName(_ecm, "red_lens");

        if (this->greenLensEntity == gz::sim::kNullEntity || 
            this->yellowLensEntity == gz::sim::kNullEntity || 
            this->redLensEntity == gz::sim::kNullEntity)
        {
            std::cerr << "[traffic_light_plugin] ERROR: Could not find lens links for " << this->name << std::endl;
        }

        this->m_ros_subscriber = this->m_ros_node->create_subscription<std_msgs::msg::Byte>(
            "/automobile/trafficlight/" + this->name, 
            1, 
            std::bind(&TrafficLight::OnRosMsg, this, std::placeholders::_1));

        std::cerr << "\n====================================================================\n";
        std::cerr << "[traffic_light_plugin] attached to: " << this->name << "\n";
        std::cerr << "[traffic_light_plugin] listening to: /automobile/trafficlight/" << this->name << "\n";
        std::cerr << "====================================================================\n\n";
    }

    void TrafficLight::SetLightState(gz::sim::EntityComponentManager &_ecm, 
                                     gz::sim::Entity lightEntity, 
                                     bool state)
    {
        if (lightEntity == gz::sim::kNullEntity)
            return;

        auto lightCmd = _ecm.Component<gz::sim::components::LightCmd>(lightEntity);
        gz::msgs::Light lightMsg;
        lightMsg.set_range(state ? 5.0 : 0.0);
        lightMsg.set_attenuation_linear(state ? 0.0 : 1.0);

        if (!lightCmd)
        {
            _ecm.CreateComponent(lightEntity, gz::sim::components::LightCmd(lightMsg));
        }
        else
        {
            *lightCmd = gz::sim::components::LightCmd(lightMsg);
        }
    }

    void TrafficLight::PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm)
    {
        if (_info.paused)
            return;

        rclcpp::spin_some(this->m_ros_node);
        
        switch(this->currentLightState)
        {
            case TrafficLightColor::RED:
                SetLightState(_ecm, redLensEntity, true);
                SetLightState(_ecm, yellowLensEntity, false);
                SetLightState(_ecm, greenLensEntity, false);
                break;
            case TrafficLightColor::YELLOW:
                SetLightState(_ecm, redLensEntity, false);
                SetLightState(_ecm, yellowLensEntity, true);
                SetLightState(_ecm, greenLensEntity, false);
                break;
            case TrafficLightColor::GREEN:
                SetLightState(_ecm, redLensEntity, false);
                SetLightState(_ecm, yellowLensEntity, false);
                SetLightState(_ecm, greenLensEntity, true);
                break;
        }
    }

    void TrafficLight::OnRosMsg(std_msgs::msg::Byte _msg)
    {
        this->currentLightState = _msg.data;
    }
};

GZ_ADD_PLUGIN(trafficLight::TrafficLight,
              gz::sim::System,
              trafficLight::TrafficLight::ISystemConfigure,
              trafficLight::TrafficLight::ISystemPreUpdate)
