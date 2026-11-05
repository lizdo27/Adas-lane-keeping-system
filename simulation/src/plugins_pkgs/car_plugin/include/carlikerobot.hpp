/**
 * @file carlikerobot.hpp
 * @brief Car-like robot kinematics implementation for Ackermann steering
 * 
 * This file contains the core kinematics classes for simulating a car-like robot
 * with Ackermann steering geometry in Gazebo Sim. It provides wheel speed control
 * for both front and rear wheels, as well as steering angle control.
 * 
 * @copyright Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
 */

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/math/PID.hh>

#ifndef PI
#define PI 3.141592654
#endif

namespace carlikerobot
{
    /**
     * @class IWheelsSpeed
     * @brief Interface for wheel speed control
     * 
     * Abstract interface defining the contract for wheel speed controllers.
     * Implementations handle the specific kinematics for front and rear wheels.
     */
    class IWheelsSpeed
    {
        public:
            /**
             * @brief Update wheel velocities based on speed and steering commands
             * @param speed Linear speed command in m/s
             * @param steer Steering angle in degrees
             * @param _ecm Entity Component Manager for accessing simulation entities
             */
            virtual void update(float,float,gz::sim::EntityComponentManager &_ecm)=0;
    };

    /**
     * @class CRearWheelsSpeed
     * @brief Rear wheel speed controller for Ackermann steering
     * 
     * Controls the velocity of rear wheels. In Ackermann steering, rear wheels
     * typically have the same angular velocity, with differential handled by
     * the front wheel steering geometry.
     */
    class CRearWheelsSpeed: public IWheelsSpeed
    {
        public:
            /**
             * @brief Constructor for rear wheel speed controller
             * @param axletrack Distance between left and right wheels (m)
             * @param wheelbase Distance between front and rear axles (m)
             * @param wheelradius Radius of the wheel (m)
             * @param jointRight Entity ID of the right wheel joint
             * @param jointLeft Entity ID of the left wheel joint
             * @param pidRight PID controller for right wheel
             * @param pidLeft PID controller for left wheel
             */
            CRearWheelsSpeed(float axletrack, float wheelbase, float wheelradius,
                           gz::sim::Entity jointRight, gz::sim::Entity jointLeft,
                           gz::math::PID pidRight, gz::math::PID pidLeft);
            
            /**
             * @brief Update rear wheel velocities
             * @param speed Linear speed command in m/s
             * @param steer Steering angle in degrees (used for advanced control)
             * @param _ecm Entity Component Manager for simulation access
             */
            void update(float,float,gz::sim::EntityComponentManager &_ecm);
        
        private:
            float _axletrack;       ///< Distance between left and right wheels (m)
            float _wheelbase;       ///< Distance between front and rear axles (m)
            float _wheelradius;     ///< Radius of the wheel (m)
            gz::sim::Entity _jointRight;  ///< Right wheel joint entity
            gz::sim::Entity _jointLeft;   ///< Left wheel joint entity
            gz::math::PID _pidRight;      ///< PID controller for right wheel
            gz::math::PID _pidLeft;       ///< PID controller for left wheel
    };

    /**
     * @class CFrontWheelsSpeed
     * @brief Front wheel speed controller with Ackermann steering differential
     * 
     * Controls front wheel velocities with proper Ackermann steering geometry.
     * Calculates differential speeds for inner and outer wheels during turns
     * to prevent tire scrubbing and improve handling.
     */
    class CFrontWheelsSpeed: public IWheelsSpeed
    {
        public:
            /**
             * @brief Constructor for front wheel speed controller
             * @param axletrack Distance between left and right wheels (m)
             * @param wheelbase Distance between front and rear axles (m)
             * @param wheelradius Radius of the wheel (m)
             * @param jointRight Entity ID of the right wheel joint
             * @param jointLeft Entity ID of the left wheel joint
             * @param pidRight PID controller for right wheel
             * @param pidLeft PID controller for left wheel
             */
            CFrontWheelsSpeed(float axletrack, float wheelbase, float wheelradius,
                            gz::sim::Entity jointRight, gz::sim::Entity jointLeft,
                            gz::math::PID pidRight, gz::math::PID pidLeft);
            
            /**
             * @brief Update front wheel velocities with Ackermann differential
             * @param speed Linear speed command in m/s
             * @param steer Steering angle in degrees
             * @param _ecm Entity Component Manager for simulation access
             */
            void update(float,float,gz::sim::EntityComponentManager &_ecm);
        
        private:
            float _axletrack;       ///< Distance between left and right wheels (m)
            float _wheelbase;       ///< Distance between front and rear axles (m)
            float _wheelradius;     ///< Radius of the wheel (m)
            gz::sim::Entity _jointRight;  ///< Right wheel joint entity
            gz::sim::Entity _jointLeft;   ///< Left wheel joint entity
            gz::math::PID _pidRight;      ///< PID controller for right wheel
            gz::math::PID _pidLeft;       ///< PID controller for left wheel
    };
    
    /**
     * @class ISteerWheels
     * @brief Interface for steering wheel control
     * 
     * Abstract interface for steering wheel angle controllers.
     */
    class ISteerWheels
    {
        public:
            /**
             * @brief Update steering angles
             * @param steer Steering angle command in degrees
             * @param _ecm Entity Component Manager for simulation access
             */
            virtual void update(float,gz::sim::EntityComponentManager &_ecm)=0;
    };

    /**
     * @class CSteerWheelsAngle
     * @brief Steering angle controller implementing Ackermann geometry
     * 
     * Controls the steering angles of front wheels using proper Ackermann
     * steering geometry. Calculates different angles for inner and outer
     * wheels based on the vehicle's wheelbase and track width to ensure
     * proper turn radius and minimize tire scrubbing.
     */
    class CSteerWheelsAngle:public ISteerWheels
    {
        public:
            /**
             * @brief Constructor for steering angle controller
             * @param axletrack Distance between left and right wheels (m)
             * @param wheelbase Distance between front and rear axles (m)
             * @param jointRight Entity ID of the right steering joint
             * @param jointLeft Entity ID of the left steering joint
             * @param pidRight PID controller for right steering
             * @param pidLeft PID controller for left steering
             */
            CSteerWheelsAngle(float axletrack, float wheelbase,
                            gz::sim::Entity jointRight, gz::sim::Entity jointLeft,
                            gz::math::PID pidRight, gz::math::PID pidLeft);
            
            /**
             * @brief Update steering angles using Ackermann geometry
             * @param steer Steering angle command in degrees
             * @param _ecm Entity Component Manager for simulation access
             */
            void update(float,gz::sim::EntityComponentManager &_ecm);
        
        private:
            float _axletrack;       ///< Distance between left and right wheels (m)
            float _wheelbase;       ///< Distance between front and rear axles (m)
            gz::sim::Entity _jointRight;  ///< Right steering joint entity
            gz::sim::Entity _jointLeft;   ///< Left steering joint entity
            gz::math::PID _pidRight;      ///< PID controller for right steering
            gz::math::PID _pidLeft;       ///< PID controller for left steering
    };

    /// Shared pointer to wheel speed controller interface
    typedef std::shared_ptr<IWheelsSpeed> IWheelsSpeedPtr;
    /// Shared pointer to steering wheel controller interface
    typedef std::shared_ptr<ISteerWheels> ISteerWheelsPtr;
};
