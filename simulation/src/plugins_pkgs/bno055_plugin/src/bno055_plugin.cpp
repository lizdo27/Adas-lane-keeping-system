#include "bno055_plugin.hpp"
#include <gz/sim/components/Name.hh>
#include <sstream>
#include <iomanip>

#define DEBUG true

namespace bno055
{   
    BNO055::BNO055() {}

    void BNO055::Configure(const gz::sim::Entity &_entity,
                          const std::shared_ptr<const sdf::Element> &_sdf,
                          gz::sim::EntityComponentManager &_ecm,
                          gz::sim::EventManager &/*_eventMgr*/)
    {
        this->entity = _entity;
        this->initialized = false;
        this->lastUpdateTime = std::chrono::steady_clock::now();

        // Create publisher for IMU data using Gazebo Transport
        this->imuPub = this->node.Advertise<gz::msgs::Vector3d>(this->topicName);

        if(DEBUG)
        {
            auto nameComp = _ecm.Component<gz::sim::components::Name>(_entity);
            std::string modelName = nameComp ? nameComp->Data() : "unknown";
            
            std::cerr << "\n\n";
            std::cerr << "====================================================================" << std::endl;
            std::cerr << "[bno055_plugin] attached to: " << modelName << std::endl;
            std::cerr << "[bno055_plugin] publish to: " << this->topicName << std::endl;
            std::cerr << "[bno055_plugin] Publishing: roll, pitch, yaw (as Vector3d.x, y, z)" << std::endl;
            std::cerr << "====================================================================" << std::endl;
            std::cerr << "[bno055_plugin] Configured successfully with Gazebo Transport" << std::endl;
        }
    }

    void BNO055::PostUpdate(const gz::sim::UpdateInfo &_info,
                           const gz::sim::EntityComponentManager &_ecm)
    {
        if (_info.paused)
            return;

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - this->lastUpdateTime);
        
        // Update at 10 Hz (100ms period)
        if (elapsed.count() < 100)
            return;
        
        this->lastUpdateTime = now;

        auto poseComp = _ecm.Component<gz::sim::components::Pose>(this->entity);
        if (poseComp)
        {
            const auto &pose = poseComp->Data();
            
            // Create IMU message with roll, pitch, yaw
            gz::msgs::Vector3d imuMsg;
            imuMsg.set_x(pose.Rot().Roll());   // roll
            imuMsg.set_y(pose.Rot().Pitch());  // pitch
            imuMsg.set_z(pose.Rot().Yaw());    // yaw
            
            this->imuPub.Publish(imuMsg);
        }
    }
}

GZ_ADD_PLUGIN(bno055::BNO055,
              gz::sim::System,
              bno055::BNO055::ISystemConfigure,
              bno055::BNO055::ISystemPostUpdate)
