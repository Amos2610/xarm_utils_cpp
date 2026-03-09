#include <optional>
#include "xarm_utils_cpp/xarm_utils.hpp"
#include <rclcpp/parameter_client.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/joint_model_group.h>
#include <tf2_eigen/tf2_eigen.hpp>  // tf2::fromMsg for Eigen::Isometry3d

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
 : node_(node), group_name_(group_name)
{
    setup_xarm_moveit(node_, group_name_);
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, group_name_);
}

void XArmUtils::setup_xarm_moveit(const std::shared_ptr<rclcpp::Node>& node, const std::string& group_name)
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
    // if not node.has_parameter("robot_description"):
    // node->declare_parameter("robot_description", urdf);
    // node->declare_parameter("robot_description_semantic", srdf);
    if (!node->has_parameter("robot_description")) {
        node->declare_parameter("robot_description", urdf);
        RCLCPP_INFO(node->get_logger(), "Declared robot_description");
    }
    if (!node->has_parameter("robot_description_semantic")) {
        node->declare_parameter("robot_description_semantic", srdf);
        RCLCPP_INFO(node->get_logger(), "Declared robot_description_semantic");
    }
    // ---- kinematics (dot parameters) ----
    const std::string group = group_name;
    const std::string prefix = "robot_description_kinematics." + group + ".";

    auto copy_string = [&](const std::string& key) {
        const std::string full = prefix + key;
        try {
        auto v = param_client->get_parameter<std::string>(full);
        node->declare_parameter(full, v);
        RCLCPP_INFO(node->get_logger(), "Copied %s", full.c_str());
        } catch (...) {
        RCLCPP_WARN(node->get_logger(), "Missing %s", full.c_str());
        }
    };

    auto copy_double = [&](const std::string& key) {
        const std::string full = prefix + key;
        try {
        auto v = param_client->get_parameter<double>(full);
        node->declare_parameter(full, v);
        RCLCPP_INFO(node->get_logger(), "Copied %s", full.c_str());
        } catch (...) {
        RCLCPP_WARN(node->get_logger(), "Missing %s", full.c_str());
        }
    };

    auto copy_int = [&](const std::string& key) {
        const std::string full = prefix + key;
        try {
        auto v = param_client->get_parameter<int>(full);
        node->declare_parameter(full, v);
        RCLCPP_INFO(node->get_logger(), "Copied %s", full.c_str());
        } catch (...) {
        RCLCPP_WARN(node->get_logger(), "Missing %s", full.c_str());
        }
    };

    auto copy_string_array = [&](const std::string& key) {
        const std::string full = prefix + key;
        try {
        auto v = param_client->get_parameter<std::vector<std::string>>(full);
        node->declare_parameter(full, v);
        RCLCPP_INFO(node->get_logger(), "Copied %s", full.c_str());
        } catch (...) {
        RCLCPP_WARN(node->get_logger(), "Missing %s", full.c_str());
        }
    };

    copy_string("kinematics_solver");
    copy_double("kinematics_solver_search_resolution");
    copy_double("kinematics_solver_timeout");
    copy_int("kinematics_solver_attempts");
    copy_string_array("kinematics_solver_ik_links");  // 存在すればコピー（無ければWARNでOK）
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

// ---- set pose target ----
bool XArmUtils::set_pose_target(const geometry_msgs::msg::Pose& pose) {
    return move_group_->setPoseTarget(pose);
}

// ---- compute IK ----
std::optional<std::vector<double>> XArmUtils::compute_ik(
    const geometry_msgs::msg::Pose& target_pose,
    double timeout_sec,
    int attempts,
    const std::string& ik_link)
{
  RCLCPP_INFO(node_->get_logger(), "Planning frame: %s", move_group_->getPlanningFrame().c_str());
  RCLCPP_INFO(node_->get_logger(), "EndEffectorLink: %s", move_group_->getEndEffectorLink().c_str());
  const std::string link = !ik_link.empty() ? ik_link : move_group_->getEndEffectorLink();
  if (link.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "compute_ik: end effector link is empty. Set ik_link.");
    return std::nullopt;
  }

  moveit::core::RobotStatePtr state = move_group_->getCurrentState(10.0);
  if (!state) {
    RCLCPP_WARN(node_->get_logger(), "compute_ik: getCurrentState failed. Using default RobotState from model.");
    state = std::make_shared<moveit::core::RobotState>(move_group_->getRobotModel());
    state->setToDefaultValues();

    auto model = move_group_->getRobotModel();
    const auto* jmg = model->getJointModelGroup(group_name_);
    for (const auto& n : jmg->getVariableNames()) {
    RCLCPP_INFO(node_->get_logger(), "JMG var: %s", n.c_str());
    }
    if (jmg) {
        std::vector<double> seed = {0.916, 0.724, -1.700, 0.001, 0.977, -0.67};
        state->setJointGroupPositions(jmg, seed);
    }
  }
  const auto* jmg = state->getJointModelGroup(group_name_);
  if (!jmg) {
    RCLCPP_ERROR(node_->get_logger(), "compute_ik: JointModelGroup '%s' not found.", group_name_.c_str());
    return std::nullopt;
  }

  Eigen::Isometry3d eigen_pose;
  tf2::fromMsg(target_pose, eigen_pose);

  bool ok = false;
  for (int i = 0; i < attempts; ++i) {
    // tip(link) を指定するオーバーロード（Humbleで存在）
    ok = state->setFromIK(jmg, eigen_pose, link, timeout_sec);
    if (ok) break;
  }
  if (!ok) {
    RCLCPP_WARN(node_->get_logger(), "compute_ik: IK failed (link=%s).", link.c_str());
    return std::nullopt;
  }

  std::vector<double> solution;
  state->copyJointGroupPositions(jmg, solution);
  return solution;
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

bool XArmUtils::gripper_command(double position, double max_effort, double timeout_sec)
{
  using ActionT = control_msgs::action::GripperCommand;

  // action client 作成（初回のみ）
  if (!gripper_client_) {
    gripper_client_ = rclcpp_action::create_client<ActionT>(node_, "/xarm_gripper/gripper_action");
  }

  // サーバ待ち
  if (!gripper_client_->wait_for_action_server(std::chrono::duration<double>(2.0))) {
    RCLCPP_ERROR(node_->get_logger(), "Gripper action server not available: /xarm_gripper/gripper_action");
    return false;
  }

  // goal 作成
  ActionT::Goal goal;
  goal.command.position = position;
  goal.command.max_effort = max_effort;

  // 送信
  auto send_goal_options = rclcpp_action::Client<ActionT>::SendGoalOptions();

  auto goal_future = gripper_client_->async_send_goal(goal, send_goal_options);
  auto ret1 = rclcpp::spin_until_future_complete(node_, goal_future,
      std::chrono::duration<double>(timeout_sec));

  if (ret1 != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Gripper send_goal timeout/failed");
    return false;
  }

  auto goal_handle = goal_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Gripper goal was rejected (no goal_handle)");
    return false;
  }

  // 結果待ち
  auto result_future = gripper_client_->async_get_result(goal_handle);
  auto ret2 = rclcpp::spin_until_future_complete(node_, result_future,
      std::chrono::duration<double>(timeout_sec));

  if (ret2 != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Gripper get_result timeout/failed");
    return false;
  }

  auto wrapped = result_future.get();
  const auto & res = wrapped.result;

  RCLCPP_INFO(node_->get_logger(),
              "Gripper result: position=%.4f reached=%s effort=%.2f",
              res->position,
              res->reached_goal ? "true" : "false",
              res->effort);

  // reached_goal は false のことがある（ドライバ仕様）ので、ここでは成功扱いにする
  return (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED);
}

bool XArmUtils::gripper_open(double max_effort, double timeout_sec)
{
  // あなたの実測：0.0 が Open
  return gripper_command(0.0, max_effort, timeout_sec);
}

bool XArmUtils::gripper_close(double max_effort, double timeout_sec)
{
  // あなたの実測：0.8 が Close
  return gripper_command(0.8, max_effort, timeout_sec);
}

bool XArmUtils::move_to_initial() {
    // Move to the initial position defined in the MoveGroupInterface
    move_group_->setNamedTarget("home");
    return (move_group_->move() == moveit::core::MoveItErrorCode::SUCCESS);
}
