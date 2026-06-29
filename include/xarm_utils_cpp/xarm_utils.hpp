#pragma once
#include <memory>
#include <thread>
#include <vector>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/control_msgs/action/gripper_command.hpp>

class XArmUtils {
public:
    XArmUtils(const std::shared_ptr<rclcpp::Node>& node, const std::string& group_name);
    
    // xarmのmoveitをセットアップ
    static void setup_xarm_moveit(const std::shared_ptr<rclcpp::Node>& node, const std::string& group_name);

    // move_groupのパラメータを設定
    bool set_move_group_parameter(const std::string& name, bool v);
    bool set_move_group_parameter(const std::string& name, int v);
    bool set_move_group_parameter(const std::string& name, double v);
    bool set_move_group_parameter(const std::string& name, const std::string& v);

    // パイプライン切替（stomp, ompl）
    void set_planning_pipeline(const std::string& pipeline_name);

    // 現在の関節値を取得
    std::vector<double> get_current_joint_values();
    // 現在のポーズを取得
    geometry_msgs::msg::Pose get_current_pose();

    // グリッパー制御
    bool gripper_command(double position, double max_effort = 50.0, double timeout_sec = 5.0);
    bool gripper_open(double max_effort = 50.0, double timeout_sec = 5.0);
    bool gripper_close(double max_effort = 50.0, double timeout_sec = 5.0);

    // IK計算
    std::optional<std::vector<double>> compute_ik(
      const geometry_msgs::msg::Pose& target_pose,
      double timeout_sec = 0.1,
      int attempts = 10,
      const std::string& ik_link = "");

    // toleranceの設定
    void set_goal_joint_tolerance(double tol);
    
    bool set_joint_value_target(const std::vector<double>& joint_values);
    bool set_pose_target(const geometry_msgs::msg::Pose& pose);
    std::tuple<bool,
               trajectory_msgs::msg::JointTrajectory,
               double,
               moveit_msgs::msg::MoveItErrorCodes>
    plan();
    bool execute();
    bool execute_with_plan(const trajectory_msgs::msg::JointTrajectory& planned_trajectory);
    bool move_to_initial();

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::planning_interface::MoveGroupInterface::Plan plan_;

    std::string group_name_;
    rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_client_;

    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    std::thread executor_thread_;
};
