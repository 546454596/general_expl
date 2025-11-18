#include "grid_cover_planner/grid_cover_planner.h"

#include <algorithm>
#include <limits>
#include <cmath>
#include <ros/duration.h>

namespace explore_scanner {

void GridCoverPlanner::initialize(ros::NodeHandle& nh)
{
    nh.param("grid_size", grid_size_, 1.0);
    nh.param("height", height_, 2.0);
    nh.param("velocity", velocity_, 1.0);
    nh.param("block_row_size", block_row_size_, 1);
    nh.param("block_col_size", block_col_size_, 1);
    nh.param("reach_threshold", reach_threshold_, 0.3);
    odom_sub_ = nh.subscribe("/uav_odom", 1, &GridCoverPlanner::odometryCallback, this);
    // waypoint_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/grid/waypoint", 10);
    path_pub_ = nh.advertise<nav_msgs::Path>("/coverage_path", 1, true);
    block_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/block_visualization", 1);

    // exec_timer_ = nh.createTimer(ros::Duration(0.01), &GridCoverPlanner::updateWaypointExecution, this);

    // no_waypoint_ = true;
    // has_new_waypoint_ = false;
    waypoints_.clear();
    // current_wp_id_ = 0;
}

void GridCoverPlanner::run(const geometry_msgs::PolygonStamped& polygon)
{
    ROS_WARN("before poly");
    gridCells_ = polygonToGridCells(polygon);
    ROS_WARN("after poly");
    blocks_ = partitionGrid(gridCells_);
    ROS_WARN("after blocks");
    visit_order_ = computeVisitOrder(blocks_);
    ROS_WARN("after visit_order_");
    sendCoverageWaypoints();
    ROS_WARN("after sendCoverageWaypoints");
    visualizeBlocks();
    ROS_WARN("After visualization.");
}

bool GridCoverPlanner::pointInPolygon(double x, double y, const std::vector<geometry_msgs::Point32>& polygon_points)
{
    bool inside = false;
    int n = polygon_points.size();
    for (int i = 0, j = n - 1; i < n; j = i++) {
        double xi = polygon_points[i].x, yi = polygon_points[i].y;
        double xj = polygon_points[j].x, yj = polygon_points[j].y;

        bool intersect = ((yi > y) != (yj > y)) &&
                         (x < (xj - xi) * (y - yi) / (yj - yi + 1e-10) + xi);
        if (intersect)
            inside = !inside;
    }
    return inside;
}

std::vector<GridCell> GridCoverPlanner::polygonToGridCells(const geometry_msgs::PolygonStamped& polygon)
{
    auto& pts = polygon.polygon.points;
    double xmin = pts[0].x, xmax = pts[0].x;
    double ymin = pts[0].y, ymax = pts[0].y;
    for (const auto& pt : pts) {
        if (pt.x < xmin)
            xmin = pt.x;
        if (pt.x > xmax)
            xmax = pt.x;
        if (pt.y < ymin)
            ymin = pt.y;
        if (pt.y > ymax)
            ymax = pt.y;
    }
    std::vector<GridCell> cells;
    cout << "before ymin is: " << ymin << endl;
    cout << " grid_size is: " << grid_size_ << endl;
    int row_start = std::floor(ymin / grid_size_);
    cout << "ymin is: " << ymin << " grid_size is: " << grid_size_ << endl;
    int row_end = std::ceil(ymax / grid_size_);
    int col_start = std::floor(xmin / grid_size_);
    int col_end = std::ceil(xmax / grid_size_);
    ROS_WARN("A+"); 
    for (int r = row_start; r <= row_end; ++r) {
        for (int c = col_start; c <= col_end; ++c) {
            double cx = (c + 0.5) * grid_size_;
            double cy = (r + 0.5) * grid_size_;
            ROS_WARN("B+");
            if (pointInPolygon(cx, cy, pts)) {
                cells.push_back({r, c, cx, cy});
            }
        }
    }
    ROS_INFO("Num of grid cells in polygon is: %lu", cells.size());
    return cells;
}

std::vector<Block> GridCoverPlanner::partitionGrid(const std::vector<GridCell>& cells)
{
    int min_row = std::numeric_limits<int>::max();
    int max_row = std::numeric_limits<int>::min();
    int min_col = std::numeric_limits<int>::max();
    int max_col = std::numeric_limits<int>::min();

    for (auto& c : cells) {
        if (c.row < min_row)
            min_row = c.row;
        if (c.row > max_row)
            max_row = c.row;
        if (c.col < min_col)
            min_col = c.col;
        if (c.col > max_col)
            max_col = c.col;
    }

    std::vector<Block> blocks;
    for (int r = min_row; r <= max_row; r += block_row_size_) {
        for (int c = min_col; c <= max_col; c += block_col_size_) {
            Block blk;
            blk.row_start = r;
            blk.row_end = std::min(r + block_row_size_ - 1, max_row);
            blk.col_start = c;
            blk.col_end = std::min(c + block_col_size_ - 1, max_col);
            blk.center_x = 0;
            blk.center_y = 0;

            for (auto& cell : cells) {
                if (cell.row >= blk.row_start && cell.row <= blk.row_end &&
                    cell.col >= blk.col_start && cell.col <= blk.col_end) {
                    blk.cells.push_back(cell);
                    blk.center_x += cell.center_x;
                    blk.center_y += cell.center_y;
                }
            }
            if (!blk.cells.empty()) {
                blk.center_x /= blk.cells.size();
                blk.center_y /= blk.cells.size();
                blocks.push_back(blk);
            }
        }
    }
    ROS_INFO("Partitioned into %lu blocks.", blocks.size());
    return blocks;
}

std::vector<int> GridCoverPlanner::computeVisitOrder(const std::vector<Block>& blocks)
{
    std::vector<bool> visited(blocks.size(), false);
    std::vector<int> order;
    int current = 0;
    visited[0] = true;
    order.push_back(0);

    for (size_t i = 1; i < blocks.size(); ++i) {
        double min_dist = std::numeric_limits<double>::max();
        int next = -1;
        for (size_t j = 0; j < blocks.size(); ++j) {
            if (visited[j])
                continue;
            double dx = blocks[j].center_x - blocks[current].center_x;
            double dy = blocks[j].center_y - blocks[current].center_y;
            double dist = dx * dx + dy * dy;
            if (dist < min_dist) {
                min_dist = dist;
                next = j;
            }
        }
        visited[next] = true;
        order.push_back(next);
        current = next;
    }
    return order;
}

std::vector<GridCell> GridCoverPlanner::planBlockCoveragePath(const Block& block)
{
    std::vector<GridCell> path;
    std::vector<GridCell> sorted_cells = block.cells;

    std::sort(sorted_cells.begin(), sorted_cells.end(), [](const GridCell& a, const GridCell& b) {
        if (a.row != b.row)
            return a.row < b.row;
        return a.col < b.col;
    });

    int current_row = -1;
    bool left_to_right = true;
    std::vector<GridCell> row_cells;

    for (auto& cell : sorted_cells) {
        if (cell.row != current_row) {
            if (!row_cells.empty()) {
                if (!left_to_right)
                    std::reverse(row_cells.begin(), row_cells.end());
                path.insert(path.end(), row_cells.begin(), row_cells.end());
                row_cells.clear();
            }
            current_row = cell.row;
            left_to_right = !left_to_right;
        }
        row_cells.push_back(cell);
    }
    if (!row_cells.empty()) {
        if (!left_to_right)
            std::reverse(row_cells.begin(), row_cells.end());
        path.insert(path.end(), row_cells.begin(), row_cells.end());
    }
    return path;
}

void GridCoverPlanner::sendCoverageWaypoints()
{
    // no_waypoint_ = false;

    for (auto blk_id : visit_order_) {
        Block& blk = blocks_[blk_id];
        auto path_cells = planBlockCoveragePath(blk);

        for (auto& cell : path_cells) {
            geometry_msgs::PoseStamped wp;
            wp.header.stamp = ros::Time::now();
            wp.header.frame_id = "world";
            wp.pose.position.x = cell.center_x;
            wp.pose.position.y = cell.center_y;
            wp.pose.position.z = height_;
            wp.pose.orientation.w = 1.0;

            waypoints_.push_back(wp);
        }
    }

    reorderWaypoints();
}

void GridCoverPlanner::reorderWaypoints() 
{
    if (waypoints_.empty()) return;

    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "world";
    path_msg.header.stamp = ros::Time::now();

    int closest_id = 0;
    double min_dist = numeric_limits<double>::max();

    for (size_t i = 0; i < waypoints_.size(); ++i) {
        double dist = distanceToWaypoint(waypoints_[i]);
        if (dist < min_dist) {
            min_dist = dist;
            closest_id = i;
        }
    }

    vector<geometry_msgs::PoseStamped> new_waypoints;
    for (size_t i = 0; i < waypoints_.size(); ++i) {
        size_t idx = (i + closest_id) % waypoints_.size();
        new_waypoints.push_back(waypoints_[idx]);
        path_msg.poses.push_back(waypoints_[idx]);
    }

    waypoints_ = new_waypoints;
    path_pub_.publish(path_msg);
}

/* void GridCoverPlanner::updateWaypointExecution(const ros::TimerEvent& e)
{
    if (no_waypoint_ || current_wp_id_ >= waypoints_.size()) 
        return;

    if (current_wp_id_ == 0) {
        has_new_waypoint_ = true;
        current_waypoint_ = waypoints_[current_wp_id_];
        waypoint_pub_.publish(waypoints_[current_wp_id_]);
    } 
    
    double dist = distanceToWaypoint(waypoints_[current_wp_id_]);   
    if (dist < reach_threshold_) {
        ++current_wp_id_;
        if (current_wp_id_ < waypoints_.size()) {
            has_new_waypoint_ = true;
            waypoint_pub_.publish(waypoints_[current_wp_id_]);
            current_waypoint_ = waypoints_[current_wp_id_];
            ROS_INFO("Sent waypoint id is: %lu", current_wp_id_);
        } else {
            has_new_waypoint_ = false;
            no_waypoint_ = true;
            ROS_INFO("All waypoints published.");
        }
    }
} */

void GridCoverPlanner::visualizeBlocks() 
{
    visualization_msgs::MarkerArray marker_array;
    int id = 0;
    ROS_WARN("blks size is: %ld", blocks_.size());
    for (size_t blk_idx = 0; blk_idx < blocks_.size(); ++blk_idx)
    {
        const Block& blk = blocks_[blk_idx];
        // 为每个Block分配一种颜色（循环使用一些固定颜色）
        std_msgs::ColorRGBA color;
        float hue = (blk_idx * 0.61803398875);  // golden ratio for distinct hues
        hue = fmod(hue, 1.0);
        color = hsvToRgba(hue, 1.0, 1.0, 0.5);  // alpha=0.8

        for (const auto& cell : blk.cells)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time::now();
            marker.ns = "grid_blocks";
            marker.id = id++;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = cell.center_x;
            marker.pose.position.y = cell.center_y;
            marker.pose.position.z = 0.02;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = grid_size_;
            marker.scale.y = grid_size_;
            marker.scale.z = 0.05;  // flat cube
            marker.color = color;
            marker.lifetime = ros::Duration(0);  // 永久显示
            marker_array.markers.push_back(marker);
        }
    }
    
    while (block_pub_.getNumSubscribers() == 0) {
        ROS_WARN_THROTTLE(1.0, "Waiting for subscribers to /grid_blocks ...");
        ros::Duration(0.1).sleep();
    }
    block_pub_.publish(marker_array);
    ROS_INFO("Published %lu waypoints in blocks.", marker_array.markers.size());
}

void GridCoverPlanner::odometryCallback(const nav_msgs::OdometryConstPtr& msg) 
{
    current_pose_.pose = msg->pose.pose;
}

double GridCoverPlanner::distanceToWaypoint(const geometry_msgs::PoseStamped& waypoint) 
{
    double dx = waypoint.pose.position.x - current_pose_.pose.position.x;
    double dy = waypoint.pose.position.y - current_pose_.pose.position.y;
    double dz = waypoint.pose.position.z - current_pose_.pose.position.z;
    
    return sqrt(dx*dx + dy*dy + dz*dz);   
}

std_msgs::ColorRGBA GridCoverPlanner::hsvToRgba(float h, float s, float v, float a)
{
    std_msgs::ColorRGBA color;
    float r, g, b;

    int i = int(h * 6);
    float f = h * 6 - i;
    float p = v * (1 - s);
    float q = v * (1 - f * s);
    float t = v * (1 - (1 - f) * s);

    switch (i % 6) {
        case 0: r = v, g = t, b = p; break;
        case 1: r = q, g = v, b = p; break;
        case 2: r = p, g = v, b = t; break;
        case 3: r = p, g = q, b = v; break;
        case 4: r = t, g = p, b = v; break;
        case 5: r = v, g = p, b = q; break;
    }

    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}

} // namespace explore_scanner