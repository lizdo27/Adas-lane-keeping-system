#include "carlikerobot.hpp"
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <cmath>

namespace carlikerobot
{
    CRearWheelsSpeed::CRearWheelsSpeed(float axletrack, float wheelbase, float wheelradius,
                                       gz::sim::Entity jointRight, gz::sim::Entity jointLeft,
                                       gz::math::PID pidRight, gz::math::PID pidLeft)
        : _axletrack(axletrack), _wheelbase(wheelbase), _wheelradius(wheelradius),
          _jointRight(jointRight), _jointLeft(jointLeft), _pidRight(pidRight), _pidLeft(pidLeft)
    {
    }

    void CRearWheelsSpeed::update(float speed, float steer, gz::sim::EntityComponentManager &_ecm)
    {
        // Implement Ackermann steering for rear wheels
        // Rear wheels have same speed, differential is handled by steering geometry
        // Apply 3x multiplier for more responsive movement
        float angularVelocity = (speed * 3.0) / _wheelradius;
        
        if (_jointRight != gz::sim::kNullEntity)
        {
            auto velCmd = _ecm.Component<gz::sim::components::JointVelocityCmd>(_jointRight);
            if (!velCmd)
            {
                _ecm.CreateComponent(_jointRight, 
                    gz::sim::components::JointVelocityCmd({angularVelocity}));
            }
            else
            {
                *velCmd = gz::sim::components::JointVelocityCmd({angularVelocity});
            }
        }
        
        if (_jointLeft != gz::sim::kNullEntity)
        {
            auto velCmd = _ecm.Component<gz::sim::components::JointVelocityCmd>(_jointLeft);
            if (!velCmd)
            {
                _ecm.CreateComponent(_jointLeft, 
                    gz::sim::components::JointVelocityCmd({angularVelocity}));
            }
            else
            {
                *velCmd = gz::sim::components::JointVelocityCmd({angularVelocity});
            }
        }
    }

    CFrontWheelsSpeed::CFrontWheelsSpeed(float axletrack, float wheelbase, float wheelradius,
                                         gz::sim::Entity jointRight, gz::sim::Entity jointLeft,
                                         gz::math::PID pidRight, gz::math::PID pidLeft)
        : _axletrack(axletrack), _wheelbase(wheelbase), _wheelradius(wheelradius),
          _jointRight(jointRight), _jointLeft(jointLeft), _pidRight(pidRight), _pidLeft(pidLeft)
    {
    }

    void CFrontWheelsSpeed::update(float speed, float steer, gz::sim::EntityComponentManager &_ecm)
    {
        // Implement Ackermann steering for front wheels
        // Different speeds for left and right based on steering angle
        // Apply 3x multiplier for more responsive movement
        float adjustedSpeed = speed * 3.0;
        float steerRad = steer * M_PI / 180.0;
        
        float angularVelocityRight = adjustedSpeed / _wheelradius;
        float angularVelocityLeft = adjustedSpeed / _wheelradius;
        
        // Apply differential based on steering (simplified Ackermann)
        if (std::abs(steerRad) > 0.001)
        {
            float turnRadius = _wheelbase / std::tan(std::abs(steerRad));
            float rightRadius = turnRadius + _axletrack / 2.0;
            float leftRadius = turnRadius - _axletrack / 2.0;
            
            if (steerRad > 0) // Turning left
            {
                angularVelocityRight = adjustedSpeed / _wheelradius * rightRadius / turnRadius;
                angularVelocityLeft = adjustedSpeed / _wheelradius * leftRadius / turnRadius;
            }
            else // Turning right
            {
                angularVelocityRight = adjustedSpeed / _wheelradius * leftRadius / turnRadius;
                angularVelocityLeft = adjustedSpeed / _wheelradius * rightRadius / turnRadius;
            }
        }
        
        if (_jointRight != gz::sim::kNullEntity)
        {
            auto velCmd = _ecm.Component<gz::sim::components::JointVelocityCmd>(_jointRight);
            if (!velCmd)
            {
                _ecm.CreateComponent(_jointRight, 
                    gz::sim::components::JointVelocityCmd({angularVelocityRight}));
            }
            else
            {
                *velCmd = gz::sim::components::JointVelocityCmd({angularVelocityRight});
            }
        }
        
        if (_jointLeft != gz::sim::kNullEntity)
        {
            auto velCmd = _ecm.Component<gz::sim::components::JointVelocityCmd>(_jointLeft);
            if (!velCmd)
            {
                _ecm.CreateComponent(_jointLeft, 
                    gz::sim::components::JointVelocityCmd({angularVelocityLeft}));
            }
            else
            {
                *velCmd = gz::sim::components::JointVelocityCmd({angularVelocityLeft});
            }
        }
    }

    CSteerWheelsAngle::CSteerWheelsAngle(float axletrack, float wheelbase,
                                         gz::sim::Entity jointRight, gz::sim::Entity jointLeft,
                                         gz::math::PID pidRight, gz::math::PID pidLeft)
        : _axletrack(axletrack), _wheelbase(wheelbase),
          _jointRight(jointRight), _jointLeft(jointLeft), _pidRight(pidRight), _pidLeft(pidLeft)
    {
    }

    void CSteerWheelsAngle::update(float steer, gz::sim::EntityComponentManager &_ecm)
    {
        // Implement Ackermann steering geometry
        float steerRad = steer * M_PI / 180.0;
        
        // Calculate inner and outer wheel angles using Ackermann geometry
        float innerAngle = steerRad;
        float outerAngle = steerRad;
        
        if (std::abs(steerRad) > 0.001)
        {
            float turnRadius = _wheelbase / std::tan(std::abs(steerRad));
            
            if (steerRad > 0) // Turning left
            {
                innerAngle = std::atan(_wheelbase / (turnRadius - _axletrack / 2.0));
                outerAngle = std::atan(_wheelbase / (turnRadius + _axletrack / 2.0));
            }
            else // Turning right
            {
                innerAngle = -std::atan(_wheelbase / (turnRadius - _axletrack / 2.0));
                outerAngle = -std::atan(_wheelbase / (turnRadius + _axletrack / 2.0));
            }
        }
        
        // Apply steering angles
        if (_jointLeft != gz::sim::kNullEntity)
        {
            auto posReset = _ecm.Component<gz::sim::components::JointPositionReset>(_jointLeft);
            if (!posReset)
            {
                _ecm.CreateComponent(_jointLeft, 
                    gz::sim::components::JointPositionReset({innerAngle}));
            }
            else
            {
                *posReset = gz::sim::components::JointPositionReset({innerAngle});
            }
        }
        
        if (_jointRight != gz::sim::kNullEntity)
        {
            auto posReset = _ecm.Component<gz::sim::components::JointPositionReset>(_jointRight);
            if (!posReset)
            {
                _ecm.CreateComponent(_jointRight, 
                    gz::sim::components::JointPositionReset({outerAngle}));
            }
            else
            {
                *posReset = gz::sim::components::JointPositionReset({outerAngle});
            }
        }
    }
};
