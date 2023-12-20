#include <PID_control_ex/PID_controller.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PID_control_ex_node");
    ros::NodeHandle nh("~");
    mobile_system_control::PIDController pid(nh);
    ros::spin();
}