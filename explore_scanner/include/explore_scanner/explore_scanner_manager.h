#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <vector>
#include <geometry_msgs/PoseStamped.h>

using Eigen::Vector3d;
using std::vector;
using std::shared_ptr;

class GridMap; 
class PolygonDrawer;   

namespace ego_planner {
class EGOPlannerManager;
class PlanningVisualization;
}

namespace explore_scanner {

class GridCoverPlanner;

enum EXPLSCAN_PLAN_RESULT { NO_GRIDCELL, FAIL, SUCCEED };

class ExploreScannerManager {
public:
    ExploreScannerManager();
    ~ExploreScannerManager();

    void initialize(ros::NodeHandle& nh);
    void prepareEnvironment();
    void computeCoveragePath();
    bool executeNextCoveragePoint();
    bool detectObject();
    void computeScanPath();
    void executeScan();
    bool checkTrajCollision();
    int planExplScanFromGlobTraj(const int trial_times, const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw);
    int planExplScanFromCurtTraj(const int trial_times, const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw);
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj); // front-end and back-end method
    bool planNextWaypoint(const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& end);
    bool have_new_target_;

    Vector3d end_pt_, end_vel_;  

    shared_ptr<ego_planner::EGOPlannerManager> planner_manager_;
    shared_ptr<ego_planner::PlanningVisualization> visualization_;
    shared_ptr<PolygonDrawer> scan_setter_;
    shared_ptr<GridCoverPlanner> cover_planner_;

private:
    vector<geometry_msgs::PoseStamped> coverage_path_;
    vector<geometry_msgs::PoseStamped> scan_path_;
    size_t coverage_index_;

    /* planning data */
    Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    Vector3d local_target_pt_, local_target_vel_;  
    double planning_horizon_;

    void getLocalTarget();
    
};

} // namespace explore_scanner

