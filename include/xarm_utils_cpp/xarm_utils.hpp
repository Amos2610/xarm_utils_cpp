#pragma once
#include <memory>
#include <vector>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <geometry_msgs/msg/pose.hpp>
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
    // bool execute(const std::optional<trajectory_msgs::msg::JointTrajectory>& planned_trajectory = std::nullopt);
    bool execute();
    bool move_to_initial();
    void sync_start_state_to_current(double wait_sec = 0.5);
    bool wait_joint_state_newer_than(const rclcpp::Time& t, double timeout_sec = 0.5);
    bool xarm6_air_cut(const std::array<double, 6>& goal,
        double time_sec = 2.0,
        double tolerance = 0.005,
        double timeout_sec = 10.0);

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::planning_interface::MoveGroupInterface::Plan plan_;

    rclcpp::Time last_js_stamp_{0, 0, RCL_SYSTEM_TIME};
    void joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg);
    bool is_valid_joint_angles(const std::array<double, 6>& q) const;
    bool has_reached_goal(double tolerance) const;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

    // 現在値・目標値
    std::array<double, 6> current_joint_{0,0,0,0,0,0};
    std::optional<std::array<double, 6>> goal_joint_;

    // トピック名（必要ならパラメータ化）
    std::string traj_topic_ = "/xarm6_traj_controller/joint_trajectory";
    std::string joint_states_topic_ = "/joint_states"; // 必要に応じて "/xarm/joint_states" に変更可
};
