#include "carlikerobot_ros_plugin.hpp"
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/Name.hh>

#define DEBUG true

namespace carlikerobot
{
    CMessageHandler::CMessageHandler(gz::transport::Node::Publisher& feedbackPub, IRobotCommandSetter* setter)
        : _feedbackPublisher(feedbackPub)
    {
        this->_robotSetter = setter;

        if (DEBUG)
        {
            std::cerr << "\n\n";
            std::cerr << "====================================================================" << std::endl;
            std::cerr << "[car_plugin] Message handler initialized for Gazebo Transport" << std::endl;
            std::cerr << "[car_plugin] Subscribing to: /automobile/command" << std::endl;
            std::cerr << "[car_plugin] Publishing to: /automobile/feedback" << std::endl;
            std::cerr << "====================================================================" << std::endl;
        }
    }

    CMessageHandler::~CMessageHandler()
    {
    }

    void CMessageHandler::OnMsgCommand(const gz::msgs::StringMsg& msg)
    {           
        rapidjson::Document doc;
        const char* c = msg.data().c_str();
        doc.Parse(c);
        
        if (doc.HasMember("action"))
        {
            std::string command = doc["action"].GetString();
            if(command =="1")
            {
                if (DEBUG){std::cerr << "[car_plugin] Received SPED message" << std::endl;}
                if (doc.HasMember("speed")){ this->spedMessage(doc["speed"].GetFloat());}
                else{std::cerr << "[car_plugin] Invalid message" << std::endl; this->unknownMessage();}
            } 
            else if (command =="2") 
            {
                if (DEBUG){std::cerr << "[car_plugin] Received STER message" << std::endl;}
                if (doc.HasMember("steerAngle")){ this->sterMessage(doc["steerAngle"].GetFloat());}
                else{std::cerr << "[car_plugin] Invalid message" << std::endl; this->unknownMessage();}
            } 
            else if (command =="3") 
            {
                if (DEBUG){std::cerr << "[car_plugin] Received BRAKE message" << std::endl;}
                if (doc.HasMember("steerAngle")){ this->brakeMessage(doc["steerAngle"].GetFloat());}
                else{std::cerr << "[car_plugin] Invalid message" << std::endl; this->unknownMessage();}
            } 
            else 
            {
                std::cerr << "[car_plugin] Received UNKNOWN message" << std::endl;
                this->unknownMessage();
            }
        } 
        else 
        {
            std::cerr << "[car_plugin] Invalid message" << std::endl;
            this->unknownMessage();
        }
    }

    void CMessageHandler::unknownMessage()
    {
        gz::msgs::StringMsg resp;
        resp.set_data("@MESS:err;;");
        this->_feedbackPublisher.Publish(resp);
    }

    void CMessageHandler::brakeMessage(float msg_val)
    {
        _robotSetter->f_speed = 0;
        _robotSetter->f_steer = msg_val;
        _robotSetter->setCommand();
        gz::msgs::StringMsg resp;
        resp.set_data("@3:ack;;");
        this->_feedbackPublisher.Publish(resp);
    }

    void CMessageHandler::spedMessage(float msg_val)
    {
        _robotSetter->f_speed = msg_val;
        _robotSetter->setCommand();
        gz::msgs::StringMsg resp;
        resp.set_data("@1:ack;;");
        this->_feedbackPublisher.Publish(resp);
    }

    void CMessageHandler::sterMessage(float _msg_val)
    {
        _robotSetter->f_steer = _msg_val;
        _robotSetter->setCommand();
        gz::msgs::StringMsg resp;
        resp.set_data("@2:ack;;");
        this->_feedbackPublisher.Publish(resp);
    }

    CCarLikeRobotRosPlugin::CCarLikeRobotRosPlugin()
    {
        this->f_speed = 0.0;
        this->f_steer = 0.0;
    }

    void CCarLikeRobotRosPlugin::Configure(const gz::sim::Entity &_entity,
                                          const std::shared_ptr<const sdf::Element> &_sdf,
                                          gz::sim::EntityComponentManager &_ecm,
                                          gz::sim::EventManager &/*_eventMgr*/)
    {
        this->entity = _entity;
        this->ecm = &_ecm;
        this->initialized = false;

        if (!this->LoadParameterJoints(_sdf, _ecm))
        {
            std::cerr << "[car_plugin] Failed to load joint parameters" << std::endl;
            return;
        }

        // Initialize Gazebo Transport
        this->feedbackPub = this->node.Advertise<gz::msgs::StringMsg>("/automobile/feedback");
        if (!this->feedbackPub)
        {
            std::cerr << "[car_plugin] Failed to advertise /automobile/feedback" << std::endl;
            return;
        }

        // Create message handler
        this->_messageHandler = std::make_shared<CMessageHandler>(this->feedbackPub, this);

        // Subscribe to command topic
        std::function<void(const gz::msgs::StringMsg&)> callback = 
            [this](const gz::msgs::StringMsg& msg) {
                if (this->_messageHandler) {
                    this->_messageHandler->OnMsgCommand(msg);
                }
            };
        
        if (!this->node.Subscribe("/automobile/command", callback))
        {
            std::cerr << "[car_plugin] Failed to subscribe to /automobile/command" << std::endl;
            return;
        }

        std::cerr << "[car_plugin] Configured successfully with Gazebo Transport" << std::endl;
        this->initialized = true;
    }

    void CCarLikeRobotRosPlugin::PreUpdate(const gz::sim::UpdateInfo &_info,
                                          gz::sim::EntityComponentManager &_ecm)
    {
        if (_info.paused)
            return;

        // Apply pending commands in the simulation thread (thread-safe)
        std::lock_guard<std::mutex> lock(this->commandMutex);
        if (this->hasPendingCommand && this->initialized)
        {
            if (this->_rearWheelsSpeedPtr != nullptr)
                this->_rearWheelsSpeedPtr->update(this->pendingSpeed, this->pendingSteer, _ecm);
            
            if (this->_frontWheelSpeedPtr != nullptr)
                this->_frontWheelSpeedPtr->update(this->pendingSpeed, this->pendingSteer, _ecm);
            
            if (this->_steerWheelsAnglePtr != nullptr)
                this->_steerWheelsAnglePtr->update(this->pendingSteer, _ecm);
            
            // this->hasPendingCommand = false;  // Keep applying last command
        }
    }

    void CCarLikeRobotRosPlugin::setCommand()
    {
        if (!this->initialized)
            return;
        
        // Store command for application in PreUpdate (thread-safe)
        std::lock_guard<std::mutex> lock(this->commandMutex);
        this->pendingSpeed = this->f_speed;
        this->pendingSteer = this->f_steer;
        this->hasPendingCommand = true;
    }

    bool CCarLikeRobotRosPlugin::LoadParameterJoints(const std::shared_ptr<const sdf::Element> &_sdf,
                                                     gz::sim::EntityComponentManager &_ecm)
    {
        // Vehicle parameters from the original code
        float axletrack = 0.194;
        float wheelbase = 0.256;
        float wheelradius = 0.032;

        // Create PID controllers with default values
        gz::math::PID pid(1.0, 0.0, 0.0);
        
        // Find joint entities by name
        gz::sim::Entity rearLeftJoint = gz::sim::kNullEntity;
        gz::sim::Entity rearRightJoint = gz::sim::kNullEntity;
        gz::sim::Entity frontLeftJoint = gz::sim::kNullEntity;
        gz::sim::Entity frontRightJoint = gz::sim::kNullEntity;
        gz::sim::Entity steerLeftJoint = gz::sim::kNullEntity;
        gz::sim::Entity steerRightJoint = gz::sim::kNullEntity;

        // Get model entity
        gz::sim::Model model(this->entity);
        
        // Search for joints by name
        rearLeftJoint = model.JointByName(_ecm, "joint_leftrear_rim");
        rearRightJoint = model.JointByName(_ecm, "joint_rightrear_rim");
        frontLeftJoint = model.JointByName(_ecm, "joint_leftfront_rim");
        frontRightJoint = model.JointByName(_ecm, "joint_rightfront_rim");
        steerLeftJoint = model.JointByName(_ecm, "joint_leftfront_steer");
        steerRightJoint = model.JointByName(_ecm, "joint_rightfront_steer");

        // Verify joints were found
        if (rearLeftJoint == gz::sim::kNullEntity || rearRightJoint == gz::sim::kNullEntity)
        {
            std::cerr << "[car_plugin] Failed to find rear wheel joints" << std::endl;
            return false;
        }
        
        if (frontLeftJoint == gz::sim::kNullEntity || frontRightJoint == gz::sim::kNullEntity)
        {
            std::cerr << "[car_plugin] Failed to find front wheel joints" << std::endl;
            return false;
        }
        
        if (steerLeftJoint == gz::sim::kNullEntity || steerRightJoint == gz::sim::kNullEntity)
        {
            std::cerr << "[car_plugin] Failed to find steering joints" << std::endl;
            return false;
        }

        std::cerr << "[car_plugin] Successfully found all joints:" << std::endl;
        std::cerr << "  - Rear wheels: " << rearLeftJoint << ", " << rearRightJoint << std::endl;
        std::cerr << "  - Front wheels: " << frontLeftJoint << ", " << frontRightJoint << std::endl;
        std::cerr << "  - Steering: " << steerLeftJoint << ", " << steerRightJoint << std::endl;

        // Create controller objects with actual joint entities
        this->_rearWheelsSpeedPtr = std::make_shared<CRearWheelsSpeed>(
            axletrack, wheelbase, wheelradius, rearRightJoint, rearLeftJoint, pid, pid);
        
        this->_frontWheelSpeedPtr = std::make_shared<CFrontWheelsSpeed>(
            axletrack, wheelbase, wheelradius, frontRightJoint, frontLeftJoint, pid, pid);
        
        this->_steerWheelsAnglePtr = std::make_shared<CSteerWheelsAngle>(
            axletrack, wheelbase, steerRightJoint, steerLeftJoint, pid, pid);

        return true;
    }
};

GZ_ADD_PLUGIN(carlikerobot::CCarLikeRobotRosPlugin,
              gz::sim::System,
              carlikerobot::CCarLikeRobotRosPlugin::ISystemConfigure,
              carlikerobot::CCarLikeRobotRosPlugin::ISystemPreUpdate)
