#ifndef PID_CONTROL_EX_PID_CONTROL_EX_PID_CONTROLLER_H_
#define PID_CONTROL_EX_PID_CONTROL_EX_PID_CONTROLLER_H_

#include <cmath>
#include <memory>
#include <vector>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Vector3Stamped.h>

namespace mobile_system_control
{
    class PIDController
    {
    public:
        PIDController(ros::NodeHandle &nh);
        ~PIDController();
        void CarlaInputCallback(const std_msgs::Float32MultiArray::Ptr &input);
        void ReadPath();
        void RunPID();
        int FindClosestIndex();

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
        struct PointXY;
    };
}
#endif // PID_CONTROL_EX_PID_CONTROL_EX_PID_CONTROLLER_H_