#ifndef MOBILESYSTEMCONTROL_MOBILESYSTEMCONTROL_MOBILESYSTEMCONTROL_H_
#define MOBILESYSTEMCONTROL_MOBILESYSTEMCONTROL_MOBILESYSTEMCONTROL_H_

#include <ros/ros.h>
#include "carla_msgs/CarlaEgoVehicleControl.h"
#include "carla_msgs/CarlaEgoVehicleStatus.h"
// #include "derived_object_msgs/ObjectArray.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Vector3Stamped.h"


namespace mobile_system_control
{
    class Carla
    {
    public:
        Carla();
        ~Carla();
        void Init(ros::NodeHandle &nh);
        void CarlaEgoCallback(const carla_msgs::CarlaEgoVehicleStatus::ConstPtr& msg);
        void CarlaOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void UserCtrlCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
        void SpinOnce();

    private:
        void setTopic();
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
} // namespace mobile_system_control

#endif // MOBILESYSTEMCONTROL_MOBILESYSTEMCONTROL_MOBILESYSTEMCONTROL_H_
