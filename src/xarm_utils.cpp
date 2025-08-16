#include "xarm_utils_cpp/xarm_utils.hpp"
#include <rclcpp/parameter_client.hpp>

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

bool XArmUtils::execute(const std::optional<trajectory_msgs::msg::JointTrajectory>& planned_trajectory)
{
    if (planned_trajectory) {
        // 外部からJointTrajectoryが渡された場合
        moveit::planning_interface::MoveGroupInterface::Plan tmp_plan;
        tmp_plan.trajectory_.joint_trajectory = *planned_trajectory;

        return (move_group_->execute(tmp_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    } else {
        // 従来通り内部plan_を実行
        return (move_group_->execute(plan_) == moveit::core::MoveItErrorCode::SUCCESS);
    }
}

bool XArmUtils::move_to_initial() {
    // Move to the initial position defined in the MoveGroupInterface
    move_group_->setNamedTarget("home");
    return (move_group_->move() == moveit::core::MoveItErrorCode::SUCCESS);
}
