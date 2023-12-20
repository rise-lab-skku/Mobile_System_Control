#include <PurePursuit_control_ex/PurePursuit_controller.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PurePursuit_control_ex_node");
    ros::NodeHandle nh("~");
    mobile_system_control::PurePursuitController pp(nh);
    ros::spin();
}