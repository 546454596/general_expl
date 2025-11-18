#ifndef GRID_COVER_PLANNER_H
#define GRID_COVER_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>

#include <vector>
using namespace std;

namespace explore_scanner {

struct GridCell {
    int row;
    int col;
    double center_x;
    double center_y;
};

struct Block {
    int row_start, row_end, col_start, col_end;
    double center_x;
    double center_y;
    vector<GridCell> cells;
};

class GridCoverPlanner
{
public:
    GridCoverPlanner() {}
    ~GridCoverPlanner() {}

    // 根据给定多边形运行规划流程
    void initialize(ros::NodeHandle& nh);
    void run(const geometry_msgs::PolygonStamped& polygon);
    geometry_msgs::PoseStamped current_waypoint_;
    vector<geometry_msgs::PoseStamped> waypoints_;
    size_t current_wp_id_;
    bool no_waypoint_, has_new_waypoint_;

private:
    ros::Publisher waypoint_pub_, path_pub_, block_pub_;
    ros::Subscriber odom_sub_;
    // ros::Timer exec_timer_;

    double grid_size_;
    double height_;
    double velocity_;

    vector<GridCell> gridCells_;
    vector<Block> blocks_;
    vector<int> visit_order_;
    geometry_msgs::PoseStamped current_pose_;
    double reach_threshold_;
    int block_row_size_, block_col_size_;

    bool pointInPolygon(double x, double y, const std::vector<geometry_msgs::Point32>& polygon_points);
    bool executing_path_;

    vector<GridCell> polygonToGridCells(const geometry_msgs::PolygonStamped& polygon);

    vector<Block> partitionGrid(const std::vector<GridCell>& cells);

    vector<int> computeVisitOrder(const std::vector<Block>& blocks);

    vector<GridCell> planBlockCoveragePath(const Block& block);

    void sendCoverageWaypoints();
    void reorderWaypoints();
    // void updateWaypointExecution(const ros::TimerEvent& e);
    void visualizeBlocks();
    std_msgs::ColorRGBA hsvToRgba(float h, float s, float v, float a);
    void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
    double distanceToWaypoint(const geometry_msgs::PoseStamped& waypoint);
};

}  // namespace explore_scanner

#endif  // GRID_COVER_PLANNER_H

