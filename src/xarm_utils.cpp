#include "xarm_utils_cpp/xarm_utils.hpp"

using namespace std::chrono_literals;

XArmUtils::XArmUtils(const std::shared_ptr<rclcpp::Node>& node, const std::string& group_name) {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, group_name);
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

bool XArmUtils::plan() {
    return (move_group_->plan(plan_) == moveit::core::MoveItErrorCode::SUCCESS);
}

bool XArmUtils::execute() {
    return (move_group_->execute(plan_) == moveit::core::MoveItErrorCode::SUCCESS);
}

bool XArmUtils::move_to_initial() {
    // Move to the initial position defined in the MoveGroupInterface
    move_group_->setNamedTarget("home");
    return (move_group_->move() == moveit::core::MoveItErrorCode::SUCCESS);
}
