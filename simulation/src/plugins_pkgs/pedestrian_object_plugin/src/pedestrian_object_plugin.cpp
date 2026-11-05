#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/math/Vector3.hh>

#define _USE_MATH_DEFINES
#include <cmath>
#include <chrono>
#include <thread>
#include <string>

#include "pedestrian_object_plugin.h"

#define CAR_DIST_METER 0.5

namespace pedestrian
{   
    Pedestrian::Pedestrian() {}

    GeometricPoint Pedestrian::rotate(GeometricPoint origin, GeometricPoint point, double angle)
    {
        GeometricPoint result_point;
        result_point.x = origin.x + (cos(angle) * (point.x - origin.x)) - (sin(angle) * (point.y - origin.y));
        result_point.y = origin.y + (sin(angle) * (point.x - origin.x)) + (cos(angle) * (point.y - origin.y));
        return result_point;
    }

    void Pedestrian::updatePedestrianPosition(const gz::sim::EntityComponentManager &_ecm)
    {
        auto poseComp = _ecm.Component<gz::sim::components::Pose>(this->entity);
        if (poseComp)
        {
            const auto &pose = poseComp->Data();
            m_ped_position.setPoint(pose.Pos().X(), pose.Pos().Y());
            this->m_ped_wz = pose.Rot().Yaw();
        }
    }

    void Pedestrian::updateRcCarPosition(const gz::sim::EntityComponentManager &_ecm)
    {
        if (this->rcCarEntity == gz::sim::kNullEntity)
            return;
            
        auto poseComp = _ecm.Component<gz::sim::components::Pose>(this->rcCarEntity);
        if (poseComp)
        {
            const auto &pose = poseComp->Data();
            this->m_rc_car_position.setPoint(pose.Pos().X(), pose.Pos().Y());
            this->m_rc_car_wz = pose.Rot().Yaw();
        }
    }

    bool Pedestrian::hasReached(const GeometricPoint &first_point, const GeometricPoint &second_point, const double &distance)
    {
        double local_dist = sqrt(pow((first_point.x - second_point.x), 2) + pow((first_point.y - second_point.y), 2));
        if(local_dist < distance)
        {
            this->m_stopped = true;
            return true;
        }
        return false;
    } 

    void Pedestrian::generatePoints()
    {     
        GeometricPoint far_point{this->m_ped_position.x + 0.5, this->m_ped_position.y + Dist::road_width};
        GeometricPoint near_point{this->m_ped_position.x - 0.5, this->m_ped_position.y + Dist::lane_width};

        GeometricPoint result_near = rotate(m_ped_position, near_point, this->m_ped_wz);
        GeometricPoint result_far = rotate(m_ped_position, far_point, this->m_ped_wz);

        this->m_near_point.setPoint(result_near);
        this->m_far_point.setPoint(result_far);

        GeometricPoint first_lane_point{this->m_ped_position.x, this->m_ped_position.y + Dist::lane_width + Dist::thin_mark};
        GeometricPoint second_lane_point{this->m_ped_position.x, this->m_ped_position.y + 2 * Dist::lane_width + 2 * Dist::thin_mark};
        GeometricPoint right_side_of_the_road_point{this->m_ped_position.x, this->m_ped_position.y + Dist::road_width + 2 * Dist::thin_mark + Dist::lane_width};
        
        GeometricPoint result_first_lane_point = rotate(m_ped_position, first_lane_point, this->m_ped_wz);
        GeometricPoint result_second_lane_point = rotate(m_ped_position, second_lane_point, this->m_ped_wz);
        GeometricPoint result_right_side_of_the_road_point = rotate(m_ped_position, right_side_of_the_road_point, this->m_ped_wz);
        
        this->wait_for_car_point.setPoint(this->m_ped_position.x, this->m_ped_position.y);
        this->first_lane_point.setPoint(result_first_lane_point);
        this->second_lane_point.setPoint(result_second_lane_point);
        this->right_side_of_the_road_point.setPoint(result_right_side_of_the_road_point);
    }

    gz::math::Vector3d Pedestrian::calculateVelocity(float velocity, bool isForward)
    {   
        if(velocity != 0)
        {
            this->m_stopped = false;
        }

        int8_t sign = isForward? -1 : 1;
        double x = sign * (velocity * sin(this->m_ped_wz));
        double y = (-1) * sign * (velocity * cos(this->m_ped_wz));
        return gz::math::Vector3d{x, y, 0};
    }

    bool Pedestrian::timeChecker(const std::chrono::steady_clock::time_point &start_time, 
                                 const std::chrono::steady_clock::time_point &current_time, 
                                 const float duration)
    {   
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);
        if(elapsed.count() > duration)
        {    
            return true;
        }
        return false;
    }
    
    void Pedestrian::Configure(const gz::sim::Entity &_entity,
                              const std::shared_ptr<const sdf::Element> &_sdf,
                              gz::sim::EntityComponentManager &_ecm,
                              gz::sim::EventManager &/*_eventMgr*/)
    {
        this->entity = _entity;
        this->initialized = false;
        
        if(_sdf->HasElement("rc_car_name"))
        {
            this->m_rc_car_name = _sdf->Get<std::string>("rc_car_name");
        }
        else
        {
            std::cerr << "\n\n*******************************************************************\n";
            std::cerr << "*******************************************************************\n";
            std::cerr << "WARNING: In [pedestrian_object] sdf file no rc_car_name was found\n";
            std::cerr << "Default rc_car_name: [ " << this->m_rc_car_name << " ]\n";
            std::cerr << "*******************************************************************\n";
            std::cerr << "*******************************************************************\n\n";
        }

        _ecm.Each<gz::sim::components::Name>(
            [&](const gz::sim::Entity &_entity, const gz::sim::components::Name *_name) -> bool
            {
                if (_name->Data() == this->m_rc_car_name)
                {
                    this->rcCarEntity = _entity;
                    return false;
                }
                return true;
            });

        if(this->rcCarEntity == gz::sim::kNullEntity)
        {
            std::cerr << "\n\n*******************************************************************\n";
            std::cerr << "*******************************************************************\n";
            std::cerr << "WARNING: While loading [pedestrian_object_plugin] The rc car model with the name [ " << this->m_rc_car_name << " ] was not found\n";
            std::cerr << "The name can be modified in the sdf file of the pedestrian_object_models, adding the \"<rc_car_name>\" tag\n";
            std::cerr << "*******************************************************************\n";
            std::cerr << "*******************************************************************\n\n";
        }

        updatePedestrianPosition(_ecm);
        generatePoints();
        this->m_timestamp = std::chrono::steady_clock::now();
    }

    void Pedestrian::PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm)
    {
        if (_info.paused)
            return;

        if(this->rcCarEntity == gz::sim::kNullEntity)
        {
            return;
        }

        updatePedestrianPosition(_ecm);
        updateRcCarPosition(_ecm);
        
        auto now = std::chrono::steady_clock::now();
        
        switch (this->m_state)
        {
            case States::wait_for_car:
                if(hasReached(this->m_rc_car_position, this->m_near_point, CAR_DIST_METER))
                {  
                    this->m_speed = calculateVelocity(1, true);
                    this->m_state = States::car_on_the_first_lane;
                }
                else if(hasReached(this->m_rc_car_position, this->m_far_point, CAR_DIST_METER))
                {
                    this->m_speed = calculateVelocity(1, true);
                    this->m_state = States::car_on_the_second_lane;
                }
                break;

            case States::car_on_the_first_lane:
                if (hasReached(this->m_ped_position, this->first_lane_point, 0.05) || this->m_stopped) 
                {   
                    this->m_speed = calculateVelocity(0, true);
                    if(timeChecker(this->m_timestamp, now, 5))
                    {   
                        this->m_speed = calculateVelocity(1, true);
                        this->m_state = States::go_to_the_right_side_of_the_road;
                    }  
                }
                break;

            case States::car_on_the_second_lane:
                if (hasReached(this->m_ped_position, this->second_lane_point, 0.05) || this->m_stopped)
                {   
                    this->m_speed = calculateVelocity(0, true);
                    if(timeChecker(this->m_timestamp, now, 5))
                    {
                        this->m_speed = calculateVelocity(1, true);
                        this->m_state = States::go_to_the_right_side_of_the_road;
                    }
                }
                break;

            case States::go_to_the_right_side_of_the_road:
                if (hasReached(this->m_ped_position, this->right_side_of_the_road_point, 0.05) || this->m_stopped)
                {   
                    this->m_speed = calculateVelocity(0, true);
                    if(timeChecker(this->m_timestamp, now, 5))
                    {
                        this->m_speed = calculateVelocity(1, false);
                        this->m_state = States::return_to_the_left_side_of_the_road;
                    }
                }
                break;

            case States::return_to_the_left_side_of_the_road:
                if (hasReached(this->m_ped_position, this->wait_for_car_point, 0.05) || this->m_stopped)
                {   
                    this->m_speed = calculateVelocity(0, true);
                    if(timeChecker(this->m_timestamp, now, 10))
                    {
                        this->m_state = States::wait_for_car;
                    }
                }
                break;
        }

        if (this->m_speed != this->m_last_speed)
        {
            this->m_timestamp = now;
        }

        auto velCmd = _ecm.Component<gz::sim::components::LinearVelocityCmd>(this->entity);
        if (!velCmd)
        {
            _ecm.CreateComponent(this->entity, gz::sim::components::LinearVelocityCmd(this->m_speed));
        }
        else
        {
            *velCmd = gz::sim::components::LinearVelocityCmd(this->m_speed);
        }
        
        this->m_last_speed = this->m_speed;
    }
}

GZ_ADD_PLUGIN(pedestrian::Pedestrian,
              gz::sim::System,
              pedestrian::Pedestrian::ISystemConfigure,
              pedestrian::Pedestrian::ISystemPreUpdate)
