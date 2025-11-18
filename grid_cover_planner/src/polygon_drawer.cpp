#include "polygon_drawer/polygon_drawer.h"

PolygonDrawer::PolygonDrawer(ros::NodeHandle& nh)
{
    polygon_sub_ = nh.subscribe("/polygon_draw", 1, &PolygonDrawer::polygonCallback, this);
    polygon_pub_ = nh.advertise<geometry_msgs::PolygonStamped>("/coverage_area_polygon", 1, true);
    ROS_INFO("Polygon drawer initialized. Click in RViz and publish /coverage_area_polygon to complete.");
}

void PolygonDrawer::polygonCallback(const geometry_msgs::PolygonStampedConstPtr& msg)
{
    polygon_msg_ = *msg;
    if (!polygon_ready_) {
        if (polygon_msg_.polygon.points.size() < 3) {
            ROS_WARN("Not enough points for a polygon.");
            return;
        }
        polygon_ready_ = true;
        polygon_msg_.header.stamp = ros::Time::now();
        polygon_msg_.header.frame_id = "world";
        polygon_pub_.publish(polygon_msg_);
        ROS_WARN("Polygon published.");
    }
}

bool PolygonDrawer::isPolygonReady() const
{
    return polygon_ready_;
}

geometry_msgs::PolygonStamped PolygonDrawer::getPolygon() const
{
    return polygon_msg_;
}