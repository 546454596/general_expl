#include <explore_scanner/explore_scanner_fsm.h>
#include <explore_scanner/explore_scanner_manager.h>
#include <polygon_drawer/polygon_drawer.h>
// #include <grid_cover_planner/grid_cover_planner.h>
#include <plan_manage/planner_manager.h>
#include <plan_env/grid_map.h>
#include <plan_env/obj_predictor.h>
#include <traj_utils/Bspline.h>
#include <traj_utils/planning_visualization.h>

namespace explore_scanner {

ExploreScannerManager::ExploreScannerManager() : coverage_index_(0) {}
ExploreScannerManager::~ExploreScannerManager() {}

void ExploreScannerManager::initialize(ros::NodeHandle& nh) {
    nh.param("fsm/planning_horizon", planning_horizon_, -1.0);

    scan_setter_.reset(new PolygonDrawer(nh));
    // cover_planner_.reset(new GridCoverPlanner(nh));
    visualization_.reset(new ego_planner::PlanningVisualization(nh));
    planner_manager_.reset(new ego_planner::EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);
    planner_manager_->deliverTrajToOptimizer(); // store trajectories
    planner_manager_->setDroneIdtoOpt();

    have_new_target_ = false;
}

void ExploreScannerManager::prepareEnvironment() {
    ROS_INFO("Preparing environment, loading map or objects...");
}

void ExploreScannerManager::computeCoveragePath() {
    ROS_INFO("Computing grid-based coverage path...");
    // 模拟生成航点
    for (int i = 0; i < 10; ++i) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = i;
        pose.pose.position.y = 0;
        pose.pose.position.z = 2.0;
        pose.pose.orientation.w = 1.0;
        coverage_path_.push_back(pose);
    }
}

bool ExploreScannerManager::executeNextCoveragePoint() {
    if (coverage_index_ < coverage_path_.size()) {
        ROS_INFO("Moving to coverage waypoint %lu", coverage_index_);
        // TODO: 发布目标点到飞控系统
        ++coverage_index_;
        return true;
    }
    return false;
}

bool ExploreScannerManager::detectObject() {
    // 简化：模拟某些航点检测到物体
    return coverage_index_ == 3 || coverage_index_ == 7;
}

void ExploreScannerManager::computeScanPath() {
    ROS_INFO("Generating scan path around detected object...");
    // 模拟生成贴面扫描路径
    scan_path_.clear();
    for (int i = 0; i < 5; ++i) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = coverage_path_[coverage_index_ - 1].pose.position.x + 0.5 * sin(i);
        pose.pose.position.y = coverage_path_[coverage_index_ - 1].pose.position.y + 0.5 * cos(i);
        pose.pose.position.z = 2.0;
        pose.pose.orientation.w = 1.0;
        scan_path_.push_back(pose);
    }
}

void ExploreScannerManager::executeScan() {
    ROS_INFO("Executing object surface scan...");
    for (auto& p : scan_path_) {
        // TODO: 发布至控制器轨迹跟踪
        ROS_INFO("Scanning point: %.2f %.2f", p.pose.position.x, p.pose.position.y);
    }
}

bool ExploreScannerManager::checkTrajCollision() {
    ego_planner::LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = (ros::Time::now() - info->start_time_).toSec();
    Eigen::Vector3d p_cur = info->position_traj_.evaluateDeBoorT(t_cur);
    // const double CLEARANCE = 1.0 * planner_manager_->getSwarmClearance();
    double t_cur_global = ros::Time::now().toSec();
    double t_2_3 = info->duration_ * 2 / 3;

    bool occ = false;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      occ |= map->getInflateOccupancy(info->position_traj_.evaluateDeBoorT(t));
    }

    return occ;
}

int ExploreScannerManager::planExplScanFromGlobTraj(const int trial_times, const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw) {
    start_pt_ = pos;
    start_vel_ = vel;
    start_acc_.setZero();

    bool flag_random_poly_init;
    flag_random_poly_init = false;

    for (int i = 0; i < trial_times; i++)
    {
      if (callReboundReplan(true, flag_random_poly_init))
        return SUCCEED;
    }
    return FAIL;
}

int ExploreScannerManager::planExplScanFromCurtTraj(const int trial_times, const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw) {
    start_pt_ = pos;
    start_vel_ = vel;
    start_acc_ = acc;
    bool success = callReboundReplan(false, false);

    if (!success)
    {
        success = callReboundReplan(true, false);
        //changeFSMExecState(EXEC_TRAJ, "FSM");
        if (!success)
        {
        for (int i = 0; i < trial_times; i++)
        {
            success = callReboundReplan(true, true);
            if (success)
            break;
        }
        if (!success)
            return FAIL;
        }
    }

    return SUCCEED;
}

bool ExploreScannerManager::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj) {
    getLocalTarget();
    bool plan_and_refine_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false;

    cout << "refine_success=" << plan_and_refine_success << endl;

    return plan_and_refine_success;
}

bool ExploreScannerManager::planNextWaypoint(const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& end) {
  have_new_target_ = true;
  end_pt_ = end;
  bool success = false;
  success = planner_manager_->planGlobalTraj(pos, vel, acc, end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  if (success)
  {
    /*** display ***/
    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);

    end_vel_.setZero();
    have_new_target_ = true;

    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
  }
  else {
    ROS_ERROR("Unable to generate global trajectory!");
    return false;
  }  

  return true;
}

void ExploreScannerManager::getLocalTarget() {
    double t;

    double t_step = planning_horizon_ / 20 / planner_manager_->pp_.max_vel_;
    double dist_min = 9999, dist_min_t = 0.0;
    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist = (pos_t - start_pt_).norm();

      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizon_)
      {
        // Important cornor case!
        for (; t < planner_manager_->global_data_.global_duration_; t += t_step)
        {
          Eigen::Vector3d pos_t_temp = planner_manager_->global_data_.getPosition(t);
          double dist_temp = (pos_t_temp - start_pt_).norm();
          if (dist_temp < planning_horizon_)
          {
            pos_t = pos_t_temp;
            dist = (pos_t - start_pt_).norm();
            cout << "Escape cornor case \"getLocalTarget\"" << endl;
            break;
          }
        }
      }

      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }

      if (dist >= planning_horizon_)
      {
        local_target_pt_ = pos_t;
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        break;
      }
    }

    if (t > planner_manager_->global_data_.global_duration_) // Last global point
    {
      local_target_pt_ = end_pt_;
      planner_manager_->global_data_.last_progress_time_ = planner_manager_->global_data_.global_duration_;
    }

    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(t);
    }
}

} // namespace explore_scanner

