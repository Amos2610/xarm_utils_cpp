#include <rclcpp/rclcpp.hpp>
#include "xarm_utils_cpp/xarm_utils.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("example_xarm_utils");

    // XArmUtils::setup_xarm_moveit(node);

    // XArmUtils xarm("xarm6");
    XArmUtils xarm(node, "xarm6");
    // xarm.set_planning_pipeline("ompl"); // ompl
    xarm.set_planning_pipeline("stomp"); // stomp

    // toleranceの設定
    xarm.set_goal_joint_tolerance(0.01);

    // 適当な目標関節値（例）[0.916, 0.724, -1.700, 0.001, 0.977, -0.67]
    std::vector<double> target = {0.916, 0.724, -1.700, 0.001, 0.977, -0.67};
    // std::vector<double> target = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // 初期位置など適宜変更
    if (xarm.set_joint_value_target(target))
    {
        RCLCPP_INFO(node->get_logger(), "Set target success");
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Set target failed");
    }

    if (xarm.plan())
    {
        RCLCPP_INFO(node->get_logger(), "Plan success");
        // 実機の場合はコメント解除
        if (xarm.execute()) {
            RCLCPP_INFO(node->get_logger(), "Execution success");
            // Move to initial position
            if (xarm.move_to_initial()) {
                RCLCPP_INFO(node->get_logger(), "Moved to initial position successfully");
            } else {
                RCLCPP_ERROR(node->get_logger(), "Failed to move to initial position");
            }
        } else {
          RCLCPP_ERROR(node->get_logger(), "Execution failed");
        }
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Plan failed");
    }

    rclcpp::shutdown();
    return 0;
}
