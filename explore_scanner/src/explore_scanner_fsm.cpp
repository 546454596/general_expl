#include <explore_scanner/explore_scanner_data.h>
#include <explore_scanner/explore_scanner_fsm.h>
#include <explore_scanner/explore_scanner_manager.h>
#include <ros/ros.h>
#include <thread>
#include <polygon_drawer/polygon_drawer.h>
#include <grid_cover_planner/grid_cover_planner.h>
#include <plan_manage/planner_manager.h>
#include <plan_env/grid_map.h>
#include <plan_env/obj_predictor.h>
#include <traj_utils/Bspline.h>
#include <traj_utils/planning_visualization.h>

namespace explore_scanner {

void ExplScanFSM::init(ros::NodeHandle& nh) {
    fp_.reset(new FSMParam);
    fd_.reset(new FSMData);
    
    /* FSM param */
    nh.param("fsm/thresh_replan1", fp_->replan_thresh1_, -1.0);// 0.5
    nh.param("fsm/thresh_replan2", fp_->replan_thresh2_, -1.0);// 0.5
    nh.param("fsm/thresh_replan3", fp_->replan_thresh3_, -1.0);// 1.5
    nh.param("fsm/replan_time", fp_->replan_time_, -1.0);
    nh.param("fsm/reach_threshold", reach_threshold_, 0.3);
    nh.param("fsm/sensing_horizon", sensing_horizon_, 1.0);

    expl_scan_manager_.reset(new ExploreScannerManager);
    expl_scan_manager_->initialize(nh);
    visualization_.reset(new ego_planner::PlanningVisualization(nh));
    planner_manager_ = expl_scan_manager_->planner_manager_;
    cover_planner_.reset(new GridCoverPlanner);
    cover_planner_->initialize(nh);
    expl_photo_waypoints_.clear();
    current_wp_id_ = 0;
    has_new_waypoint_ = false;
    waypointOcc_ = false;

    current_state_ = ExplScanState::INIT;
    fd_->have_odom_ = false;
    fd_->state_str_ = { "INIT", "WAIT_TRIGGER", "COVER_PLANNING", "PUB_TRAJ", "COVER_EXECUTION", "SCAN_OBJECT", "FINISH" };
    fd_->static_state_ = true;
    fd_->trigger_ = false;

    /* subscriber and publisher */
    odom_sub_ = nh.subscribe("/uav_odom", 1, &ExplScanFSM::odomCallback, this);
    bspline_pub_ = nh.advertise<traj_utils::Bspline>("planning/bspline", 10);
    expl_photo_waypoint_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/expl_photo/waypoint", 10);

    /* callback */
    explScan_timer_ = nh_.createTimer(ros::Duration(0.01), &ExplScanFSM::FSMCallback, this);
    safety_timer_ = nh_.createTimer(ros::Duration(0.05), &ExplScanFSM::safetyCallback, this);
    gridCell_timer_ = nh_.createTimer(ros::Duration(0.1), &ExplScanFSM::gridCellCallback, this);
    waypointOcc_timer_ = nh.createTimer(ros::Duration(0.1), &ExplScanFSM::wayptOccCallback, this);
}

void ExplScanFSM::FSMCallback(const ros::TimerEvent& e) {
    switch (current_state_) {
        case INIT: {
            if (!fd_->have_odom_) {
                ROS_WARN_THROTTLE(1.0, "no odom.");
                return;
            }
            // Go to wait trigger when odom is ok
            transitState(WAIT_TRIGGER, "FSM");
            break;
        }    

        case WAIT_TRIGGER: {
            // Do nothing but wait for trigger
            ROS_WARN_THROTTLE(1.0, "wait for trigger.");
            // if(!keyboard_ctrl_intput_)
            //   ROS_INFO("Disable PUB_TRAJ state and wait for keyboard 's'");
            if (expl_scan_manager_->scan_setter_->isPolygonReady())
                transitState(COVER_PLANNING, "triggerCallback");
            break; 
        }

        case COVER_PLANNING: {
            ROS_INFO_THROTTLE(1.0, "begin to cover planning");
            if (fd_->static_state_) {
                // Plan from static state (hover)
                fd_->start_pt_ = fd_->odom_pos_;
                fd_->start_vel_ = fd_->odom_vel_;
                fd_->start_acc_.setZero();

                fd_->start_yaw_(0) = fd_->odom_yaw_;
                fd_->start_yaw_(1) = fd_->start_yaw_(2) = 0.0;
            } else {
                // Replan from non-static state, starting from 'replan_time' seconds later
                ego_planner::LocalTrajData* info = &planner_manager_->local_data_;
                double t_r = (ros::Time::now() - info->start_time_).toSec() + fp_->replan_time_;
        
                fd_->start_pt_ = info->position_traj_.evaluateDeBoorT(t_r);
                fd_->start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
                fd_->start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_r);
                fd_->start_yaw_(0) = fd_->odom_yaw_;
                fd_->start_yaw_(1) = fd_->start_yaw_(2) = 0.0;
            }

            int res;
            if (has_new_waypoint_) {
                bool plan_succeed = expl_scan_manager_->planNextWaypoint(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_, expl_photo_waypoint_);
                
                if (plan_succeed) {
                    res = callExplScanPlan();
                    if (res == EXPLSCAN_PLAN_RESULT::SUCCEED) {
                        transitState(PUB_TRAJ, "FSM");
                    } else if (res == EXPLSCAN_PLAN_RESULT::FAIL) {
                        // Still in COVER_PLANNING state, keep replanning
                        ROS_WARN("Plan fail in first phase.");
                        fd_->static_state_ = true;
                        transitState(COVER_PLANNING, "FSM");
                    }
                } else {
                    ROS_ERROR("Unable to generate global trajectory!");
                    fd_->static_state_ = true;
                    transitState(COVER_PLANNING, "FSM");
                }
            } else if (!has_new_waypoint_) {
                ROS_INFO("Coverage finished.");
                transitState(FINISH, "FSM");;
            } else {
                // Inform traj_server the replanning
                // replan_pub_.publish(std_msgs::Empty());
                res = callExplScanPlan();
                if (res == EXPLSCAN_PLAN_RESULT::SUCCEED) {
                    transitState(PUB_TRAJ, "FSM");
                } else if (res == EXPLSCAN_PLAN_RESULT::FAIL) {
                    // Still in COVER_PLANNING state, keep replanning
                    ROS_WARN("Plan fail in second phase.");
                    fd_->static_state_ = true;
                    transitState(COVER_PLANNING, "FSM");
                }
            }
            break;
        }    

        case PUB_TRAJ: {
            double dt = (ros::Time::now() - fd_->newest_traj_.start_time).toSec();
            if (dt > 0) {
            bspline_pub_.publish(fd_->newest_traj_);
            fd_->static_state_ = false;
            transitState(COVER_EXECUTION, "FSM");
    
            thread vis_thread(&ExplScanFSM::visualize, this);
            vis_thread.detach();
            }
            break;
        }    
                
        case COVER_EXECUTION: {
            ego_planner::LocalTrajData *info = &planner_manager_->local_data_;
            double t_cur = (ros::Time::now() - info->start_time_).toSec();

            // Replan if traj is almost fully executed
            double time_to_end = info->duration_ - t_cur;
            if (time_to_end < fp_->replan_thresh1_) {
                transitState(COVER_PLANNING, "FSM");
                ROS_WARN("Replan: traj fully executed=================================");
                return;
            }
            // Replan after some time
            if (t_cur > fp_->replan_thresh3_) {
                transitState(COVER_PLANNING, "FSM"); 
                ROS_WARN("Replan: periodic call=======================================");
            }
            break;
        }    

        case SCAN_OBJECT: {
            /* ROS_INFO("[State] SCAN_OBJECT");
            expl_scan_manager_->computeScanPath();
            current_state_ = COVER_EXECUTION; // reuse COVER_EXECUTION to执行scan_path_ */
            break;
        }    

        case FINISH: {
            ROS_WARN_THROTTLE(5.0, "[State] DONE. Mission Complete.");
            break;
        }    
    }
}

void ExplScanFSM::safetyCallback(const ros::TimerEvent& e) {
    if (current_state_ == ExplScanState::COVER_EXECUTION) {
        // Check safety and trigger replan if necessary
        // double dist;
        bool unsafe = expl_scan_manager_->checkTrajCollision();
        if (unsafe) {
        ROS_WARN("Replan: collision detected==================================");
        transitState(COVER_PLANNING, "safetyCallback");
        }
    }
}

void ExplScanFSM::gridCellCallback(const ros::TimerEvent& e) {
    static int delay = 0;
    if (++delay < 5) return;

    if (current_state_ == ExplScanState::WAIT_TRIGGER) {
        auto gc = expl_scan_manager_->scan_setter_;
        
        ROS_WARN_THROTTLE(1.0, "Draw polygon in RViz of expl_scan fsm.");
        ros::Rate rate(10);
        while (ros::ok() && !gc->isPolygonReady()) {
            ros::spinOnce();
            rate.sleep();
        }

        auto polygon = gc->getPolygon();
        ROS_INFO_THROTTLE(1.0, "Polygon with %lu points received.", polygon.polygon.points.size());
        cover_planner_->run(polygon);
        expl_photo_waypoints_ = cover_planner_->waypoints_;

        has_new_waypoint_ = true;
        expl_photo_waypoint_ << expl_photo_waypoints_[current_wp_id_].pose.position.x, 
                                expl_photo_waypoints_[current_wp_id_].pose.position.y, 
                                expl_photo_waypoints_[current_wp_id_].pose.position.z;
        expl_photo_waypoint_pub_.publish(expl_photo_waypoints_[current_wp_id_]);
        ROS_INFO("Sent the first waypoint.");
    }

    if (expl_photo_waypoints_.empty()) 
        return;
    else if (current_wp_id_ < expl_photo_waypoints_.size() - 1) {  
        if (distToWaypt(expl_photo_waypoint_) < reach_threshold_) {
            ++current_wp_id_;
            has_new_waypoint_ = true;
            expl_photo_waypoint_ << expl_photo_waypoints_[current_wp_id_].pose.position.x, 
                                    expl_photo_waypoints_[current_wp_id_].pose.position.y, 
                                    expl_photo_waypoints_[current_wp_id_].pose.position.z;
            
            expl_photo_waypoint_pub_.publish(expl_photo_waypoints_[current_wp_id_]);
            ROS_INFO("Sent waypoint id is: %lu", current_wp_id_);
        } else {
            ROS_INFO("Continue flying to current waypoint.");
        }
    } else {
        has_new_waypoint_ = false;
        ROS_WARN_THROTTLE(5.0, "All waypoints published.");
        return;
    }
}

void ExplScanFSM::wayptOccCallback(const ros::TimerEvent& e) {
    map = planner_manager_->grid_map_;
    if (has_new_waypoint_) {
        expl_photo_waypoint_ << expl_photo_waypoints_[current_wp_id_].pose.position.x, 
        expl_photo_waypoints_[current_wp_id_].pose.position.y, 
        expl_photo_waypoints_[current_wp_id_].pose.position.z;
    } else 
        return;
    
    if (distToWaypt(expl_photo_waypoint_) < sensing_horizon_) {
        
        /* ---------- check waypoint ---------- */
        waypointOcc_ |= map->getInflateOccupancy(expl_photo_waypoint_);
    } else {
        waypointOcc_ = false;
        ROS_INFO_THROTTLE(5.0, "Awaiting for checking waypoint's feasibility.");
    }
    if (waypointOcc_) {
        ROS_WARN("Skip waypoint id: %lu.", current_wp_id_);
        ++current_wp_id_;
        waypointOcc_ = false;
    }
}

double ExplScanFSM::distToWaypt(const Vector3d& waypoint) 
{
    double dx = waypoint(0) - fd_->odom_pos_(0);
    double dy = waypoint(1) - fd_->odom_pos_(1);
    double dz = waypoint(2) - fd_->odom_pos_(2);
    
    return sqrt(dx*dx + dy*dy + dz*dz);   
}

int ExplScanFSM::callExplScanPlan() {
    ros::Time time_r = ros::Time::now() + ros::Duration(fp_->replan_time_);
    int res;
    if (has_new_waypoint_) 
        res = expl_scan_manager_->planExplScanFromGlobTraj(10, fd_->start_pt_, fd_->start_vel_, fd_->start_acc_, fd_->start_yaw_);
    else {
        res = expl_scan_manager_->planExplScanFromCurtTraj(1, fd_->start_pt_, fd_->start_vel_, fd_->start_acc_, fd_->start_yaw_);
    }
    
    if (res)
    {
      auto info = &planner_manager_->local_data_;
      info->start_time_ = (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now() : time_r;

      traj_utils::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }
    /*  Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
      for (int i = 0; i < yaw_pts.rows(); ++i) {
        double yaw = yaw_pts(i, 0);
        bspline.yaw_pts.push_back(yaw);
      }
      bspline.yaw_dt = info->yaw_traj_.getKnotSpan(); */
      fd_->newest_traj_ = bspline;
    }  
    return res;
}

void ExplScanFSM::visualize() {
    auto info = &planner_manager_->local_data_;
    // auto ed_ptr = expl_manager_->ed_;
    // Draw trajectory
    visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);
    visualization_->displayGoalPoint(expl_scan_manager_->end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, current_wp_id_);
}

void ExplScanFSM::odomCallback(const nav_msgs::OdometryConstPtr& msg) {
    fd_->odom_pos_(0) = msg->pose.pose.position.x;
    fd_->odom_pos_(1) = msg->pose.pose.position.y;
    fd_->odom_pos_(2) = msg->pose.pose.position.z;

    fd_->odom_vel_(0) = msg->twist.twist.linear.x;
    fd_->odom_vel_(1) = msg->twist.twist.linear.y;
    fd_->odom_vel_(2) = msg->twist.twist.linear.z;

    fd_->odom_orient_.w() = msg->pose.pose.orientation.w;
    fd_->odom_orient_.x() = msg->pose.pose.orientation.x;
    fd_->odom_orient_.y() = msg->pose.pose.orientation.y;
    fd_->odom_orient_.z() = msg->pose.pose.orientation.z;

    Eigen::Vector3d rot_x = fd_->odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
    fd_->odom_yaw_ = atan2(rot_x(1), rot_x(0));
  
    fd_->have_odom_ = true;
}

void ExplScanFSM::transitState(ExplScanState new_state, string pos_call) {
    int pre_s = int(current_state_);
    current_state_ = new_state;
    cout << "[" + pos_call + "]: from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)] << endl;
}

} // namespace explore_scanner
