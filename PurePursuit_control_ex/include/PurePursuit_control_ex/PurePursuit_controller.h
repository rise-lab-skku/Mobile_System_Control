#ifndef PURE_PURSUIT_CONTROL_EX_PURE_PURSUIT_CONTROL_EX_PURE_PURSUIT_CONTROLLER_H_
#define PURE_PURSUIT_CONTROL_EX_PURE_PURSUIT_CONTROL_EX_PURE_PURSUIT_CONTROLLER_H_

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
    class PurePursuitController
    {
    public:
        PurePursuitController(ros::NodeHandle &nh);
        ~PurePursuitController();
        void CarlaInputCallback(const std_msgs::Float32MultiArray::Ptr &input);
        void ReadPath();
        void RunPurePursuit();
        int FindClosestIndex();
        struct PointXY;

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}
#endif // PURE_PURSUIT_CONTROL_EX_PURE_PURSUIT_CONTROL_EX_PURE_PURSUIT_CONTROLLER_H_