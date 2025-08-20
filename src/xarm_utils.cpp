#include "xarm_utils_cpp/xarm_utils.hpp"
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <thread>

using namespace std::chrono_literals;

// ---- 内部ヘルパ：/move_group のリモートパラメータを設定 ----
template <typename T>
static bool set_remote_param(
  const std::shared_ptr<rclcpp::Node>& node,
  const std::string& remote_node,     // 例: "/move_group"
  const std::string& name,
  const T& value)
{
  auto client = std::make_shared<rclcpp::SyncParametersClient>(node, remote_node);
  while (!client->wait_for_service(1s)) {
    RCLCPP_INFO(node->get_logger(), "Waiting for %s parameter service...", remote_node.c_str());
  }

  auto results = client->set_parameters({ rclcpp::Parameter(name, value) });
  if (results.empty() || !results.front().successful) {
    const auto reason = results.empty() ? "no result" : results.front().reason;
    RCLCPP_ERROR(node->get_logger(), "Failed to set %s on %s: %s",
                 name.c_str(), remote_node.c_str(), reason.c_str());
    return false;
  }

  // ログ（型に応じて整形）
  if constexpr (std::is_same_v<T, bool>) {
    RCLCPP_INFO(node->get_logger(), "Set %s on %s = %s",
                name.c_str(), remote_node.c_str(), value ? "true" : "false");
  } else if constexpr (std::is_same_v<T, std::string>) {
    RCLCPP_INFO(node->get_logger(), "Set %s on %s = \"%s\"",
                name.c_str(), remote_node.c_str(), value.c_str());
  } else {
    RCLCPP_INFO(node->get_logger(), "Set %s on %s = %g",
                name.c_str(), remote_node.c_str(), static_cast<double>(value));
  }
  return true;
}


// =======================  XArmUtils  =======================
XArmUtils::XArmUtils(const std::shared_ptr<rclcpp::Node>& node, const std::string& group_name)
 : node_(node)
{
    setup_xarm_moveit(node_);
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, group_name);
    traj_pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(traj_topic_, 10);
    joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        joint_states_topic_, rclcpp::SensorDataQoS(),
        std::bind(&XArmUtils::joint_state_cb, this, std::placeholders::_1));
}

void XArmUtils::setup_xarm_moveit(const std::shared_ptr<rclcpp::Node>& node)
{
    // get robot description parameters
    auto param_client = std::make_shared<rclcpp::SyncParametersClient>(node, "/move_group");
    while (!param_client->wait_for_service(1s))
        RCLCPP_INFO(node->get_logger(), "Waiting for /move_group parameter service...");

    // get robot description
    std::string urdf, srdf;
    try {
        urdf = param_client->get_parameter<std::string>("robot_description");
        srdf = param_client->get_parameter<std::string>("robot_description_semantic");
    } catch (const std::exception& e) {
        RCLCPP_FATAL(node->get_logger(), "Failed to get robot_description: %s", e.what());
        throw;
    }
    node->declare_parameter("robot_description", urdf);
    node->declare_parameter("robot_description_semantic", srdf);
}

// ---- set move_group parameters ----
bool XArmUtils::set_move_group_parameter(const std::string& name, bool v) {
    return set_remote_param(node_, "/move_group", name, v);
}
bool XArmUtils::set_move_group_parameter(const std::string& name, int v) {
    return set_remote_param(node_, "/move_group", name, v);
}
bool XArmUtils::set_move_group_parameter(const std::string& name, double v) {
    return set_remote_param(node_, "/move_group", name, v);
}
bool XArmUtils::set_move_group_parameter(const std::string& name, const std::string& v) {
    return set_remote_param(node_, "/move_group", name, v);
}

// ---- set planning pipeline ----
void XArmUtils::set_planning_pipeline(const std::string& pipeline_name)
{
    move_group_->setPlanningPipelineId(pipeline_name);
}

// ---- get current joint values ---- TODO: fakeだと動かない
std::vector<double> XArmUtils::get_current_joint_values() {
    return move_group_->getCurrentJointValues();
}

// ---- get current pose ---- TODO: fakeだと動かない
geometry_msgs::msg::Pose XArmUtils::get_current_pose() {
    return move_group_->getCurrentPose().pose;
}

// ---- set joint tolerance ----
void XArmUtils::set_goal_joint_tolerance(double tol) {
    move_group_->setGoalJointTolerance(tol);
}

// ---- set joint value target ----
bool XArmUtils::set_joint_value_target(const std::vector<double>& joint_values) {
    return move_group_->setJointValueTarget(joint_values);
}

std::tuple<bool,
           trajectory_msgs::msg::JointTrajectory,
           double,
           moveit_msgs::msg::MoveItErrorCodes>
XArmUtils::plan()
{
    auto start_time = std::chrono::steady_clock::now();
    auto error_code = move_group_->plan(plan_);
    auto end_time = std::chrono::steady_clock::now();

    double duration_sec = std::chrono::duration<double>(end_time - start_time).count();
    bool success = (error_code == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success) {
        RCLCPP_ERROR(node_->get_logger(), "Planning failed");
        return {false, trajectory_msgs::msg::JointTrajectory(), duration_sec, moveit_msgs::msg::MoveItErrorCodes()};
    }

    // MoveItErrorCodes に変換
    moveit_msgs::msg::MoveItErrorCodes ec_msg;
    ec_msg.val = error_code.val; // enum値をコピー

    return std::make_tuple(
        success,
        plan_.trajectory_.joint_trajectory,
        duration_sec,
        ec_msg
    );

}

// bool XArmUtils::execute(const std::optional<trajectory_msgs::msg::JointTrajectory>& planned_trajectory)
bool XArmUtils::execute()
{
    // if (planned_trajectory) {
    //     // 外部からJointTrajectoryが渡された場合
    //     moveit::planning_interface::MoveGroupInterface::Plan tmp_plan;
    //     tmp_plan.trajectory_.joint_trajectory = *planned_trajectory;

    //     return (move_group_->execute(tmp_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    // } else {
    //     // 従来通り内部plan_を実行
    //     return (move_group_->execute(plan_) == moveit::core::MoveItErrorCode::SUCCESS);
    // }
    return (move_group_->execute(plan_) == moveit::core::MoveItErrorCode::SUCCESS);

}

bool XArmUtils::move_to_initial() {
    // Move to the initial position defined in the MoveGroupInterface
    move_group_->setNamedTarget("home");
    return (move_group_->move() == moveit::core::MoveItErrorCode::SUCCESS);
}

void XArmUtils::sync_start_state_to_current(double wait_sec)
{
   // 現在状態が取り込まれるまで wait_sec 秒待つ（取得できなくても nullptr になるだけ）
   auto current = move_group_->getCurrentState(wait_sec);
   if (!current) {
     RCLCPP_WARN(node_->get_logger(),
                 "[sync_start_state_to_current] current state not available within %.2fs; "
                 "setting start state to whatever is in the monitor.", wait_sec);
   }
 
   // モニタに入っている現在状態を StartState に反映
   move_group_->setStartStateToCurrentState();
}

bool XArmUtils::wait_joint_state_newer_than(const rclcpp::Time& t, double timeout_sec)
{
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_);

  const auto start = node_->get_clock()->now();
  rclcpp::Rate r(100.0);
  while (rclcpp::ok()) {
    exec.spin_some();
    if (last_js_stamp_ > t) return true;  // 1個でも新しいのが来たらOK
    if ((node_->get_clock()->now() - start).seconds() > timeout_sec) return false;
    r.sleep();
  }
  return false;
}


// =======================  Function of Air Cut  =======================
// joint_states コールバック
void XArmUtils::joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // xArm6 の順序で抽出
  static const char* N[6] = {"joint1","joint2","joint3","joint4","joint5","joint6"};
  for (int i = 0; i < 6; ++i) {
    auto it = std::find(msg->name.begin(), msg->name.end(), std::string(N[i]));
    if (it != msg->name.end()) {
      size_t idx = std::distance(msg->name.begin(), it);
      if (idx < msg->position.size()) current_joint_[i] = msg->position[idx];
    }
  }

  last_js_stamp_ = rclcpp::Time(msg->header.stamp.sec,
    msg->header.stamp.nanosec,
    RCL_SYSTEM_TIME);
}

// 関節リミット検証
bool XArmUtils::is_valid_joint_angles(const std::array<double, 6>& q) const
{
  // xArm6の関節角度制限を定義
  const std::array<std::pair<double,double>,6> lim = {
    std::make_pair(-2*M_PI,  2*M_PI),  // j1
    std::make_pair(-2.059,   2.094),   // j2
    std::make_pair(-3.927,   0.192),   // j3
    std::make_pair(-2*M_PI,  2*M_PI),  // j4
    std::make_pair(-1.692,   3.142),   // j5
    std::make_pair(-2*M_PI,  2*M_PI)   // j6
  };
  for (int i=0;i<6;++i) {
    if (q[i] < lim[i].first || q[i] > lim[i].second) return false;
  }
  return true;
}

// 到達判定
bool XArmUtils::has_reached_goal(double tolerance) const
{
  if (!goal_joint_.has_value()) return false;
  for (int i=0;i<6;++i) {
    if (std::fabs(current_joint_[i] - goal_joint_->at(i)) > tolerance) return false;
  }
  return true;
}

// Air Cut 実行関数
bool XArmUtils::xarm6_air_cut(const std::array<double, 6>& goal,
                              double time_sec,
                              double tolerance,
                              double timeout_sec)
{
  // Limitis チェック
  if (!is_valid_joint_angles(goal)) {
    RCLCPP_WARN(node_->get_logger(), "[xarm6_air_cut] invalid joint angles.");
    return false;
  }

  // JointTrajectory を作成して publish
  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = {"joint1","joint2","joint3","joint4","joint5","joint6"};
  trajectory_msgs::msg::JointTrajectoryPoint p;
  p.positions.assign(goal.begin(), goal.end());
  builtin_interfaces::msg::Duration dur_msg;
  if (time_sec < 0.0) time_sec = 0.0;
  dur_msg.sec     = static_cast<int32_t>(std::floor(time_sec));
  dur_msg.nanosec = static_cast<uint32_t>((time_sec - std::floor(time_sec)) * 1e9);
  p.time_from_start = dur_msg;

  traj.points.push_back(p);

  goal_joint_ = goal;
  traj_pub_->publish(traj);
  RCLCPP_INFO(node_->get_logger(), "[xarm6_air_cut] published trajectory (T=%.3fs).", time_sec);

  // 待機ループ
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_);

  const auto start = node_->get_clock()->now();
  rclcpp::Rate rate(50.0);  // 20Hz→50Hz くらいにして反応性を上げてもOK
  while (rclcpp::ok()) {
    exec.spin_some();
    if ((node_->get_clock()->now() - start).seconds() > timeout_sec) {
      RCLCPP_WARN(node_->get_logger(), "[xarm6_air_cut] timeout (%.1fs).", timeout_sec);
      return false;
    }
    if (has_reached_goal(tolerance)) {
      RCLCPP_INFO(node_->get_logger(), "[xarm6_air_cut] reached goal (tol=%.4f rad).", tolerance);

      // 少し待ってからロボットの現在状態を同期
      const auto t_reach = node_->get_clock()->now();
      wait_joint_state_newer_than(t_reach, 0.5);   // 失敗でも続行してOK
      sync_start_state_to_current(0.1);  // 0.1秒待つ
      return true;
    }
    rate.sleep();
  }
  return false;
}
// =====================================================================