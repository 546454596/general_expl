#ifndef POLYGON_DRAWER_H
#define POLYGON_DRAWER_H

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <std_msgs/Bool.h>

class PolygonDrawer
{
public:
    PolygonDrawer(ros::NodeHandle& nh);
    bool isPolygonReady() const;
    geometry_msgs::PolygonStamped getPolygon() const;

private:
    void polygonCallback(const geometry_msgs::PolygonStampedConstPtr& msg);

    ros::Subscriber polygon_sub_;
    ros::Publisher polygon_pub_;

    geometry_msgs::PolygonStamped polygon_msg_;
    bool polygon_ready_ = false;
};

#endif  // POLYGON_DRAWER_H