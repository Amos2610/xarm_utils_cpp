#pragma once
#include <memory>
#include <vector>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

class XArmUtils {
public:
    XArmUtils(const std::shared_ptr<rclcpp::Node>& node, const std::string& group_name);
    
    // xarmのmoveitをセットアップ
    static void setup_xarm_moveit(const std::shared_ptr<rclcpp::Node>& node);

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

    // toleranceの設定
    void set_goal_joint_tolerance(double tol);
    
    bool set_joint_value_target(const std::vector<double>& joint_values);
    std::tuple<bool,
               trajectory_msgs::msg::JointTrajectory,
               double,
               moveit_msgs::msg::MoveItErrorCodes>
    plan();
    bool execute(const std::optional<trajectory_msgs::msg::JointTrajectory>& planned_trajectory = std::nullopt);
    bool move_to_initial();

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::planning_interface::MoveGroupInterface::Plan plan_;
};
