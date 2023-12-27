#include <MPC_control_ex/MPC_controller.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MPC_control_ex_node");
    ros::NodeHandle nh("~");
    mobile_system_control::MPCControler mpc(nh);
    ros::spin();
}