#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Eigen>
#include <string>

using std::shared_ptr;
using std::string;
using Eigen::Vector3d;
using std::vector;

namespace ego_planner {
class EGOPlannerManager;
class PlanningVisualization;
}

class GridMap;

namespace explore_scanner {
class ExploreScannerManager;
class GridCoverPlanner;
struct FSMParam;
struct FSMData;

enum ExplScanState { INIT, WAIT_TRIGGER, COVER_PLANNING, PUB_TRAJ, COVER_EXECUTION, SCAN_OBJECT, FINISH };

class ExplScanFSM {
public:
    ExplScanFSM() {}
    ~ExplScanFSM() {}

    void init(ros::NodeHandle& nh);
    shared_ptr<GridMap> map;

private:
    ExplScanState current_state_;
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Publisher expl_photo_waypoint_pub_, bspline_pub_;
    /* planning utils */
    shared_ptr<ego_planner::EGOPlannerManager> planner_manager_;
    shared_ptr<GridCoverPlanner> cover_planner_;
    shared_ptr<ExploreScannerManager> expl_scan_manager_;
    shared_ptr<ego_planner::PlanningVisualization> visualization_;

    ros::Timer explScan_timer_, safety_timer_, gridCell_timer_, waypointOcc_timer_;
    shared_ptr<FSMParam> fp_;
    shared_ptr<FSMData> fd_;
    vector<geometry_msgs::PoseStamped> expl_photo_waypoints_;
    Vector3d expl_photo_waypoint_;  
    size_t current_wp_id_;
    bool has_new_waypoint_, waypointOcc_;
    double reach_threshold_, sensing_horizon_;

    void transitState(ExplScanState new_state, string pos_call);

    /* ROS functions */
    void FSMCallback(const ros::TimerEvent& e);
    void safetyCallback(const ros::TimerEvent& e);
    void gridCellCallback(const ros::TimerEvent& e);
    void visualize();
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);
    int callExplScanPlan();
    void wayptOccCallback(const ros::TimerEvent& e);
    double distToWaypt(const Vector3d& waypoint); 
};

} // namespace explore_scanner

