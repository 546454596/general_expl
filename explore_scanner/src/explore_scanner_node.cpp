#include <ros/ros.h>
#include "explore_scanner/explore_scanner_fsm.h"

using namespace explore_scanner;

int main(int argc, char** argv) {
    ros::init(argc, argv, "explore_scanner_node");
    ros::NodeHandle nh("~");

    explore_scanner::ExplScanFSM expl_scan_fsm;
    expl_scan_fsm.init(nh);

    ros::Duration(1.0).sleep();
    ros::spin();  

    return 0;
}
