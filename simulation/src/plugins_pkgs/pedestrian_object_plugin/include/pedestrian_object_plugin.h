#pragma once

#define RC_CAR_NAME "rcCar"

#define L(var) #var 
#define LOG(var) ::std::cerr << L(var) << "= " << var << '\n'
#define CP(var) ::std::cerr << "CHEKCPOINT " << var << "\n";

#include "geometric_point.h"
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/math/Vector3.hh>
#include <chrono>

namespace Cardinal
{
    const double south  = 4.712388;
    const double north  = 1.570796;
    const double east   = 3.141592;
    const double west   = 0;
};

namespace Dist
{
    const double thin_mark          = 0.020;
    const double road_width         = 0.720;
    const double lane_width         = 0.350;
    const double half_lane          = lane_width / 2;
    const double middle_to_middle   = 0.370;
};

namespace pedestrian
{
    enum class States
    {
        wait_for_car,
        car_on_the_first_lane,
        car_on_the_second_lane,
        go_to_the_right_side_of_the_road,
        return_to_the_left_side_of_the_road,
    };
   
    class Pedestrian: 
        public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate
    {
    private: 
        gz::sim::Entity entity;
        gz::sim::Entity rcCarEntity{gz::sim::kNullEntity};
        std::string m_rc_car_name{RC_CAR_NAME};

        States m_state = States::wait_for_car;
        bool m_stopped = true;

        GeometricPoint m_ped_position;
        double m_ped_wz;
        
        GeometricPoint m_rc_car_position;
        double m_rc_car_wz;
        
        GeometricPoint m_near_point;
        GeometricPoint m_far_point;
        GeometricPoint wait_for_car_point;
        GeometricPoint first_lane_point;
        GeometricPoint second_lane_point;
        GeometricPoint right_side_of_the_road_point;

        std::chrono::steady_clock::time_point m_timestamp;
        gz::math::Vector3d m_speed{0,0,0};
        gz::math::Vector3d m_last_speed{0,0,0};

        bool initialized{false};

    public: 
        Pedestrian();
        
        void Configure(const gz::sim::Entity &_entity,
                      const std::shared_ptr<const sdf::Element> &_sdf,
                      gz::sim::EntityComponentManager &_ecm,
                      gz::sim::EventManager &_eventMgr) override;
        
        void PreUpdate(const gz::sim::UpdateInfo &_info,
                      gz::sim::EntityComponentManager &_ecm) override;

    private: 
        GeometricPoint rotate(GeometricPoint, GeometricPoint, double);     
        void updatePedestrianPosition(const gz::sim::EntityComponentManager &_ecm);
        void updateRcCarPosition(const gz::sim::EntityComponentManager &_ecm);
        gz::math::Vector3d calculateVelocity(float velocity, bool isForward);
        bool timeChecker(const std::chrono::steady_clock::time_point &start_time,
                        const std::chrono::steady_clock::time_point &current_time, 
                        const float duration);
        bool hasReached(const GeometricPoint&, const GeometricPoint&, const double&);
        void generatePoints();
    };
};
