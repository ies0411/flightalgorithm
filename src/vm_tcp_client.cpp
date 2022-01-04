
// #include <tello_tcp.h>
#include "vm_tcp_client.h"
int main(int argc, char *argv[]) {
    ros::init(argc, argv, "SendingStatus");
    ROS_INFO("check");
    SendingStatus vm_sending_status;
    ROS_INFO("check2");
    while (vm_sending_status.StationConnect()) {
    }
    ROS_INFO_STREAM("connection!");
    ros::Rate rate(1);
    while (ros::ok()) {
        vm_sending_status.SendPacket(1);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
