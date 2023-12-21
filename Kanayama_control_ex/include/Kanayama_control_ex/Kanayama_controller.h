#ifndef KANAYAMA_CONTROL_EX_KANAYAMA_CONTROL_EX_KANAYAMA_CONTROLLER_H_
#define KANAYAMA_CONTROL_EX_KANAYAMA_CONTROL_EX_KANAYAMA_CONTROLLER_H_

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
    class KanayamaController
    {
    public:
        KanayamaController(ros::NodeHandle &nh);
        ~KanayamaController();
        void CarlaInputCallback(const std_msgs::Float32MultiArray::Ptr &input);
        void ReadPath();
        void RunKanayama();
        int FindClosestIndex();
        struct PointXY;

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}
#endif // KANAYAMA_CONTROL_EX_KANAYAMA_CONTROL_EX_KANAYAMA_CONTROLLER_H_