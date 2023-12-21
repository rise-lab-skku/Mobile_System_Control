#include <Kanayama_control_ex/Kanayama_controller.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Kanayama_control_ex_node");
    ros::NodeHandle nh("~");
    mobile_system_control::KanayamaController kc(nh);
    ros::spin();
}