#include <ros/ros.h>
#include "polygon_drawer/polygon_drawer.h"
#include "grid_cover_planner/grid_cover_planner.h"

using namespace explore_scanner;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grid_cover_planner_node");
    ros::NodeHandle nh("~");

    PolygonDrawer drawer(nh);

    ros::Rate rate(10);
    ROS_INFO("Draw polygon in RViz.");

    while (ros::ok() && !drawer.isPolygonReady()) {
        ros::spinOnce();
        rate.sleep();
    }

    auto polygon = drawer.getPolygon();
    ROS_INFO("Polygon with %lu points received.", polygon.polygon.points.size());

    GridCoverPlanner planner;
    planner.initialize(nh);
    planner.run(polygon);

    ros::spin();
    return 0;
}

