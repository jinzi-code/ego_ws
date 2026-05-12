
#include <plan_manage/ego_replan_fsm.h>

#include <chrono>

#define PI 3.1415926
#define yaw_error_max 90.0/180*PI

namespace ego_planner
{

constexpr double kAdjustPoseSettleSeconds = 2.0;

namespace
{
  bool &adjustPoseWaitingFlag()
  {
    static bool waiting = false;
    return waiting;
  }

  int64_t &adjustPoseWaitStartNs()
  {
    static int64_t start_ns = 0;
    return start_ns;
  }
}

#define ROS_ERROR(...) RCLCPP_ERROR(node_->get_logger(), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(node_->get_logger(), __VA_ARGS__)

  void EGOReplanFSM::init(const rclcpp::Node::SharedPtr &nh)
  {
    node_ = nh;
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    adjustPoseWaitingFlag() = false;
    adjustPoseWaitStartNs() = 0;

    /*  fsm param  */
    auto param = [&nh](const std::string &name, auto &value, const auto &default_value) {
      nh->declare_parameter<std::decay_t<decltype(default_value)>>(name, default_value);
      nh->get_parameter(name, value);
    };

    param("fsm/flight_type", target_type_, -1);
    param("fsm/thresh_replan", replan_thresh_, -1.0);
    param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);
    param("fsm/planning_horizon", planning_horizen_, -1.0);
    param("fsm/planning_horizen_time", planning_horizen_time_, -1.0);
    param("fsm/emergency_time_", emergency_time_, 1.0);
    param("fsm/w_adjust_", w_adjust, 1.0);
    param("fsm/collision_check_time_horizon", collision_check_time_horizon_, 2.0);
    param("fsm/collision_check_time_step", collision_check_time_step_, 0.05);
    param("fsm/collision_check_max_steps", collision_check_max_steps_, 200);

    param("fsm/waypoint_num", waypoint_num_, -1);
    for (int i = 0; i < waypoint_num_; i++)
    {
      param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);
    dir = POSITIVE;
    goal_last << 0,0,0;

    /* callback */
    exec_timer_ = nh->create_wall_timer(std::chrono::duration<double>(0.01), std::bind(&EGOReplanFSM::execFSMCallback, this));
//    safety_timer_ = nh->create_wall_timer(std::chrono::duration<double>(0.05), std::bind(&EGOReplanFSM::checkCollisionCallback, this));

    odom_sub_ = nh->create_subscription<nav_msgs::msg::Odometry>(
      "/odom_map", 1, std::bind(&EGOReplanFSM::odometryCallback, this, std::placeholders::_1));

    bspline_pub_ = nh->create_publisher<ego_planner::msg::Bspline>("/planning/bspline", 10);
    data_disp_pub_ = nh->create_publisher<ego_planner::msg::DataDisp>("/planning/data_display", 100);
    cmd_pub_ = nh->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel",100);
    adjust_cmd_pub_ = nh->create_publisher<std_msgs::msg::UInt8>("/is_adjust_yaw",100);
    odom_adjust_pub_ = nh->create_publisher<nav_msgs::msg::Odometry>("/odom_adjust",100);
    dir_pub = nh->create_publisher<std_msgs::msg::UInt8>("/direction",100);
    stop_pub = nh->create_publisher<std_msgs::msg::UInt8>("/emergency_stop",100);

    is_target_receive = false;

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      goal_sub_ = nh->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/way_point", rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&EGOReplanFSM::goal_callback, this, std::placeholders::_1));
    }
      //waypoint_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1, &EGOReplanFSM::waypointCallback, this);
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      rclcpp::sleep_for(std::chrono::seconds(1));
      while (rclcpp::ok() && !have_odom_)
        rclcpp::spin_some(node_);
      planGlobalTrajbyGivenWps();
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  void EGOReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps[i](0) = waypoints_[i][0];
      wps[i](1) = waypoints_[i][1];
      wps[i](2) = waypoints_[i][2];

      end_pt_ = wps.back();
    }
    bool success = planner_manager_->planGlobalTrajWaypoints(odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      rclcpp::sleep_for(std::chrono::milliseconds(1));
    }

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      std::vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      // if (exec_state_ == WAIT_TARGET)
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      // else if (exec_state_ == EXEC_TRAJ)
      //   changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      rclcpp::sleep_for(std::chrono::milliseconds(1));
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
      rclcpp::sleep_for(std::chrono::milliseconds(1));
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  void EGOReplanFSM::waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr msg)
  {
    if (msg->poses[0].pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;
    trigger_ = true;
    init_pt_ = odom_pos_;


    bool success = false;
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.0;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

void EGOReplanFSM::goal_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
    end_pt_ << msg->pose.position.x, msg->pose.position.y, odom_pos_(2);

    //cout << "Triggered!" << endl;
    trigger_ = true;

    double error_x = msg->pose.position.x - odom_pos_(0);
    double error_y = msg->pose.position.y - odom_pos_(1);

    init_pt_ = odom_pos_;

    bool success = false;


    goal_last = end_pt_;

//    last_state_ = exec_state_;
//    yaw_start = atan2(error_y,error_x);
//    yaw_error = yaw_start-yaw;
//
//    //first step : calculate the yaw error
//    if(abs(yaw_error)>PI)
//    {
//        yaw_error = yaw_error - yaw_error/abs(yaw_error)*2*PI;
//    }
//    if(abs(yaw_error)>PI/2.0)
//    {
//        if(yaw>0)
//        {
//            yaw -= PI;
//        }else if(yaw<0)
//        {
//            yaw += PI;
//        }
//        changeDirection();
//        //yaw_error = - yaw_error/abs(yaw_error)*(PI-abs(yaw_error));
//        yaw_error = yaw_start - yaw;
//
//    }
//    if(abs(yaw_error)>yaw_error_max)
//    {
//        cmd_vel.twist.linear.x = 0;
//        cmd_vel.twist.angular.z = yaw_error/abs(yaw_error)*w_adjust;
//        bool success = false;
//        end_pt_ << msg->point.x, msg->point.y, msg->point.z;
//        success = planner_manager_->planGlobalTraj(odom_pos_,odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
//        changeFSMExecState(ADJUST_POSE, "TRIG");
//        return;
//    }

    success = planner_manager_->planGlobalTraj(odom_pos_,odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
        /*** display ***/
        constexpr double step_size_t = 0.1;
        int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
        vector<Eigen::Vector3d> gloabl_traj(i_end);
        for (int i = 0; i < i_end; i++)
        {
            gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
        }
        end_vel_.setZero();
        have_target_ = true;
        have_new_target_ = true;

        //goal is too close to current pose

        /*** FSM ***/
        if (exec_state_ == WAIT_TARGET)
        {
            changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
            is_target_receive = true;
        }
        else if (exec_state_ == EXEC_TRAJ)
        {
            changeFSMExecState(REPLAN_TRAJ, "TRIG");
            is_target_receive=true;
        }

        visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);

        // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
        //visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
        ROS_ERROR("Unable to generate global trajectory!");
    }
}


  void EGOReplanFSM::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    tf2::fromMsg(msg->pose.pose.orientation,quat);
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    if(dir==NEGATIVE)
    {
        if(yaw>0)
        {
            yaw -= PI;
        }else if(yaw<0)
        {
            yaw += PI;
        }
    }
      nav_msgs::msg::Odometry odom_adjust;
      tf2::Quaternion q;
      q.setRPY(roll, pitch, yaw);
      geometry_msgs::msg::Quaternion quat = tf2::toMsg(q);
      odom_adjust = *msg;
      odom_adjust.pose.pose.orientation = quat;
      odom_adjust_pub_->publish(odom_adjust);

    have_odom_ = true;
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
    {
      continously_called_times_ = 1;
      adjustPoseWaitingFlag() = false;
      adjustPoseWaitStartNs() = 0;
    }

    static string state_str[7] = {"INIT", "WAIT_TARGET","ADJUST_POSE","GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  void EGOReplanFSM::checkYawError() {
      yaw_error = yaw_start-yaw;
      //first step : find the real yaw error
      if(abs(yaw_error)>yaw_error_max)
      {
          //if yaw error is larger than PI,it means the symbol between current yaw and target yaw is different
          if(abs(yaw_error)>PI)
          {
              //calculate the real yaw error with symbol
              yaw_error = yaw_error - yaw_error/abs(yaw_error)*2*PI;
              //if real yaw error larger than PI/2, change the direction of robot
              if(abs(yaw_error)>PI/2)
              {
                  changeDirection();

              }
          }
          else
          {
              cmd_vel.twist.linear.x = 0;
              cmd_vel.twist.angular.z = yaw_error/abs(yaw_error)*w_adjust;

          }
      }
  }

  double EGOReplanFSM::calculateYawError(double yaw_cur,double yaw_target) {
      double error = yaw_target - yaw_cur;
      if(abs(error)>PI)
      {
          error = error - error/abs(error)*2*PI;
          return error;
      }
      else
      {
          return error;
      }
  }

  void EGOReplanFSM::changeDirection() {
      if(dir == POSITIVE)
      {
          dir = NEGATIVE;
      }else
      {
          dir = POSITIVE;
      }
      std_msgs::msg::UInt8 dir_new;
      dir_new.data = dir;
      dir_pub->publish(dir_new);
  }



  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET","ADJUST_POSE","GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void EGOReplanFSM::execFSMCallback()
  {

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!trigger_)
        cout << "wait for goal." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        return;
      }
      if (!trigger_)
      {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case ADJUST_POSE:
    {
//        if(last_state_!=WAIT_TARGET&&last_state_!=EMERGENCY_STOP)
//        {
//            callEmergencyStop(odom_pos_);
//        }
        yaw_error = yaw_start-yaw;
        //first step : calculate the yaw error
        if(abs(yaw_error)>PI)
        {
            yaw_error = yaw_error - yaw_error/abs(yaw_error)*2*PI;
        }
        static int count=0;
        if(count%100==0)
        {
            string directions[2] = {"POSITIVE","NAGETIVE"};
            cout << "direction : "<<directions[int(dir)]<<endl;
            cout<<"current yaw: "<<yaw<<endl;
            cout<<"yaw error : "<<yaw_error<<endl;
            count=0;
        }
        count+=1;
        if(abs(yaw_error)>yaw_error_max)
        {
            adjustPoseWaitingFlag() = false;
            adjustPoseWaitStartNs() = 0;
            is_adjust_pose.data = 1;
            cmd_vel.twist.linear.x = 0;
            cmd_vel.twist.angular.z = yaw_error/abs(yaw_error)*w_adjust;
            cmd_pub_->publish(cmd_vel);
            adjust_cmd_pub_->publish(is_adjust_pose);
        }else
        {
            const rclcpp::Time now = node_->now();

            cmd_vel.twist.linear.x = 0;
            cmd_vel.twist.angular.z =0;
            cmd_pub_->publish(cmd_vel);

            // Keep the robot settled at zero command for a short time before resuming the trajectory.
            if (!adjustPoseWaitingFlag())
            {
                adjustPoseWaitingFlag() = true;
                adjustPoseWaitStartNs() = now.nanoseconds();
                break;
            }

            if ((now.nanoseconds() - adjustPoseWaitStartNs()) * 1e-9 < kAdjustPoseSettleSeconds)
            {
                break;
            }

            changeFSMExecState(EXEC_TRAJ, "FSM");
            is_adjust_pose.data = 0;
            adjust_cmd_pub_->publish(is_adjust_pose);
            auto info = &planner_manager_->local_data_;
            info->start_time_ = now;
            publishBspline();
        }
        break;
    }

    case GEN_NEW_TRAJ:
    {
      start_pt_ = odom_pos_;
      start_vel_ << 0,0,0;
      start_acc_.setZero();

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      bool flag_random_poly_init;
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      bool success = callReboundReplan(true, flag_random_poly_init);
      if (success)
      {
          Eigen::Vector3d vel_start = planner_manager_->local_data_.velocity_traj_.evaluateDeBoor(0.5);
          yaw_start = atan2(vel_start(1),vel_start(0));
          cout<<"yaw start : "<<yaw_start<<endl;
          yaw_error = yaw_start-yaw;

          //first step : calculate the yaw error
          if(abs(yaw_error)>PI)
          {
              yaw_error = yaw_error - yaw_error/abs(yaw_error)*2*PI;
          }
          cout<<"yaw error : "<<yaw_error<<endl;
          if(abs(yaw_error)>PI/1.0)
          {
              if(yaw>0)
              {
                  yaw -= PI;
              }else if(yaw<0)
              {
                  yaw += PI;
              }
              changeDirection();
              //yaw_error = - yaw_error/abs(yaw_error)*(PI-abs(yaw_error));
              yaw_error = yaw_start - yaw;

          }
          if(abs(yaw_error)>yaw_error_max)
          {
              cmd_vel.twist.linear.x = 0;
              cmd_vel.twist.angular.z = yaw_error/abs(yaw_error)*w_adjust;
              changeFSMExecState(ADJUST_POSE, "TRIG");
              last_state_ = GEN_NEW_TRAJ;
              is_target_receive=false;
              return;
          }
          cout<<"yaw error : "<<yaw_error<<endl;
        auto info = &planner_manager_->local_data_;
        info->start_time_ = node_->now();
        publishBspline();
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromCurrentTraj())
      {
          Eigen::Vector3d vel_start = planner_manager_->local_data_.velocity_traj_.evaluateDeBoor(0.1);
          yaw_start = atan2(vel_start(1),vel_start(0));
          yaw_error = yaw_start-yaw;

          auto info = &planner_manager_->local_data_;
          info->start_time_ = node_->now();
          std_msgs::msg::UInt8 stop_cmd;
          stop_cmd.data = 0;
          stop_pub->publish(stop_cmd);
          publishBspline();
          changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      rclcpp::Time time_now = node_->now();
      double t_cur = (time_now - info->start_time_).seconds();
      t_cur = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2)
      {
        have_target_ = false;

        changeFSMExecState(WAIT_TARGET, "FSM");
        return;
      }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)
      {
        // cout << "near end" << endl;
        return;
      }
      else if ((info->start_pos_ - pos).norm() < replan_thresh_)
      {
        // cout << "near start" << endl;
        return;
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    data_disp_.header.stamp = node_->now();
    data_disp_pub_->publish(data_disp_);
  }

  bool EGOReplanFSM::planFromCurrentTraj()
  {

    LocalTrajData *info = &planner_manager_->local_data_;
    rclcpp::Time time_now = node_->now();
    double t_cur = (time_now - info->start_time_).seconds();

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    yaw_start = atan2((end_pt_-odom_pos_)(1),(end_pt_-odom_pos_)(0));
    yaw_error = yaw_start-yaw;

    if(is_target_receive)
    {
        if(abs(yaw_error)>PI)
        {
            yaw_error = yaw_error - yaw_error/abs(yaw_error)*2*PI;
        }
        if(abs(yaw_error)>PI/1.0)
        {
            if(yaw>0)
            {
                yaw -= PI;
            }else if(yaw<0)
            {
                yaw += PI;
            }
            changeDirection();
            //yaw_error = - yaw_error/abs(yaw_error)*(PI-abs(yaw_error));
            yaw_error = yaw_start - yaw;
            start_vel_ <<-start_vel_(0),-start_vel_(1),0;
            start_acc_ <<-start_acc_(0),-start_acc_(1),0;
            //callEmergencyStop(odom_pos_);
            std_msgs::msg::UInt8 stop_cmd;
            stop_cmd.data = 1;
            stop_pub->publish(stop_cmd);

        }
        is_target_receive = false;
    }
    //first step : calculate the yaw error


    bool success = callReboundReplan(false, false);

    if (!success)
    {
      success = callReboundReplan(true, false);
      //changeFSMExecState(EXEC_TRAJ, "FSM");
      if (!success)
      {
        success = callReboundReplan(true, true);
        if (!success)
        {
          return false;
        }
      }
    }

    return true;
  }

  void EGOReplanFSM::checkCollisionCallback()
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || exec_state_ == ADJUST_POSE || exec_state_ == GEN_NEW_TRAJ || info->start_time_.seconds() < 1e-5)
      return;

    /* ---------- check trajectory ---------- */
    double t_cur = (node_->now() - info->start_time_).seconds();
    double t_2_3 = info->duration_ * 2 / 3;
    double t_end = std::min(info->duration_, t_cur + collision_check_time_horizon_);
    if (t_cur < t_2_3)
      t_end = std::min(t_end, t_2_3);

    int checked_steps = 0;
    for (double t = t_cur; t < t_end && checked_steps < collision_check_max_steps_; t += collision_check_time_step_, ++checked_steps)
    {
      Eigen::Vector3d pos_cur = info->position_traj_.evaluateDeBoorT(t);
      Eigen::Vector2d pos_cur2d;
      pos_cur2d << pos_cur(0),pos_cur(1);
      if (map->getInflateOccupancy2d(pos_cur2d))
      {
        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        else
        {
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            ROS_WARN("Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            //ROS_WARN("current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    getLocalTarget();
    start_pt_(2) = odom_pos_(2);
    start_vel_(2) = 0;
    start_acc_(2) = 0;
    local_target_pt_(2) = odom_pos_(2);
    local_target_vel_(2) = 0;

    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false;

    cout << "final_plan_success=" << plan_success << endl;

    if (plan_success)
    {
      auto info = &planner_manager_->local_data_;
      //publishBspline();
      Eigen::MatrixXd control_points = info->position_traj_.get_control_points();
      for(int i=0;i<control_points.cols();i++) control_points.col(i)(2) = odom_pos_(2);
      visualization_->displayOptimalList(control_points, 0);
    }

    return plan_success;
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    ego_planner::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
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

    bspline_pub_->publish(bspline);

    return true;
  }

  void EGOReplanFSM::getLocalTarget()
  {
    double t;

    double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_;
    double dist_min = 9999, dist_min_t = 0.0;
    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist = (pos_t - start_pt_).norm();

      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizen_)
      {
        // todo
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        return;
      }
      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }
      if (dist >= planning_horizen_)
      {
        local_target_pt_ = pos_t;
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        break;
      }
    }
    if (t > planner_manager_->global_data_.global_duration_) // Last global point
    {
      local_target_pt_ = end_pt_;
    }

    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
      // cout << "A" << endl;
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(t);
      // cout << "AA" << endl;
    }
  }

  void EGOReplanFSM::publishBspline() {

      auto info = &planner_manager_->local_data_;
      info->start_time_ = node_->now();
      /* publish traj */
      ego_planner::msg::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      //cout<<"optimal point : "<<endl<<pos_pts<<endl;
      bspline.pos_pts.reserve(pos_pts.cols());
      Eigen::Vector3d point_temp;
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
          geometry_msgs::msg::Point pt;
          pt.x = pos_pts(0, i);
          pt.y = pos_pts(1, i);
          pt.z = odom_pos_(2);
          bspline.pos_pts.push_back(pt);
          point_temp<<pt.x,pt.x,pt.x;
          //cout<<"point : "<<point_temp<<endl;
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
          bspline.knots.push_back(knots(i));
      }

        bspline_pub_->publish(bspline);
  }

} // namespace ego_planner
