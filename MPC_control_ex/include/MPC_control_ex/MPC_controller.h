#ifndef MPC_CONTROL_EX_KANAYAMA_CONTROL_EX_KANAYAMA_CONTROLLER_H_
#define KANAYAMA_CONTROL_EX_KANAYAMA_CONTROL_EX_KANAYAMA_CONTROLLER_H_

#include <cmath>
#include <memory>
#include <vector>
#include <fstream>
#include <string>

#include <Eigen/Dense>
#include <Eigen/QR>
#include <OsqpEigen/OsqpEigen.h>

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>

namespace mobile_system_control
{
    class MPCControler
    {
    public:
        MPCControler(ros::NodeHandle &nh);
        ~MPCControler();
        void CarlaInputCallback(const std_msgs::Float32MultiArray::Ptr &input);
        void ReadPath();
        void RunMPC();
        int FindClosestIndex();
        struct PointXY;

        void SetPath();
        void BuildMatrices();
        void SetReferences();
        void SetHessianMatrix();
        void SetGradient();
        void SetConstraintMatrix();
        void SetConstrainVectors();
        void KanayamaSteer();

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}
#endif // KANAYAMA_CONTROL_EX_KANAYAMA_CONTROL_EX_KANAYAMA_CONTROLLER_H_