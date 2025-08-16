#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include "xarm_utils_cpp/xarm_utils.hpp"

namespace py = pybind11;

PYBIND11_MODULE(xarm_utils_py, m) {
    // Pythonからのnodeの初期化はC++でのnodeの初期化と異なる
    // よって，ここでrclcpp::Nodeを作成する必要がある
    py::class_<rclcpp::Node, std::shared_ptr<rclcpp::Node>>(m, "Node")
        .def(py::init([](const std::string & name) {
            int argc = 0;
            char ** argv = nullptr;
            rclcpp::init(argc, argv);
            return std::make_shared<rclcpp::Node>(name);
        }));

    // MoveItErrorCodes の型バインディング
    py::class_<moveit_msgs::msg::MoveItErrorCodes>(m, "MoveItErrorCodes")
        .def(py::init<>())
        .def_readwrite("val", &moveit_msgs::msg::MoveItErrorCodes::val);

    // JointTrajectoryPoint
    py::class_<trajectory_msgs::msg::JointTrajectoryPoint>(m, "JointTrajectoryPoint")
        .def(py::init<>())
        .def_readwrite("positions", &trajectory_msgs::msg::JointTrajectoryPoint::positions)
        .def_readwrite("velocities", &trajectory_msgs::msg::JointTrajectoryPoint::velocities)
        .def_readwrite("accelerations", &trajectory_msgs::msg::JointTrajectoryPoint::accelerations)
        .def_readwrite("effort", &trajectory_msgs::msg::JointTrajectoryPoint::effort)
        .def_readwrite("time_from_start", &trajectory_msgs::msg::JointTrajectoryPoint::time_from_start);

    // JointTrajectory
    py::class_<trajectory_msgs::msg::JointTrajectory>(m, "JointTrajectory")
        .def(py::init<>())
        .def_readwrite("header", &trajectory_msgs::msg::JointTrajectory::header)
        .def_readwrite("joint_names", &trajectory_msgs::msg::JointTrajectory::joint_names)
        .def_readwrite("points", &trajectory_msgs::msg::JointTrajectory::points);
    
    // XArmUtils の型バインディング
    py::class_<XArmUtils, std::shared_ptr<XArmUtils>>(m, "XArmUtils")
        .def(py::init<std::shared_ptr<rclcpp::Node>, const std::string&>())
        .def_static("setup_xarm_moveit", &XArmUtils::setup_xarm_moveit)
        .def("set_move_group_parameter",
            py::overload_cast<const std::string&, bool>(&XArmUtils::set_move_group_parameter))
        .def("set_move_group_parameter",
                py::overload_cast<const std::string&, int>(&XArmUtils::set_move_group_parameter))
        .def("set_move_group_parameter",
                py::overload_cast<const std::string&, double>(&XArmUtils::set_move_group_parameter))
        .def("set_move_group_parameter",
                py::overload_cast<const std::string&, const std::string&>(&XArmUtils::set_move_group_parameter))
        .def("set_planning_pipeline", &XArmUtils::set_planning_pipeline)
        .def("set_joint_value_target", &XArmUtils::set_joint_value_target)
        .def("plan", [](XArmUtils &self) {
            auto [success, traj_cpp, dur, err] = self.plan();

            // Python側の JointTrajectory メッセージ作成
            py::object jt_py = py::module_::import("trajectory_msgs.msg").attr("JointTrajectory")();
            jt_py.attr("joint_names") = traj_cpp.joint_names;

            py::object point_cls = py::module_::import("trajectory_msgs.msg").attr("JointTrajectoryPoint");
            py::list points_py;
            for (auto &p : traj_cpp.points) {
                py::object p_py = point_cls();
                p_py.attr("positions") = p.positions;
                p_py.attr("velocities") = p.velocities;
                p_py.attr("accelerations") = p.accelerations;
                p_py.attr("effort") = p.effort;

                // Duration 型の生成
                py::object duration_cls = py::module_::import("builtin_interfaces.msg").attr("Duration");
                py::object dur_py = duration_cls();
                dur_py.attr("sec") = p.time_from_start.sec;
                dur_py.attr("nanosec") = p.time_from_start.nanosec;
                p_py.attr("time_from_start") = dur_py;

                points_py.append(p_py);
            }
            jt_py.attr("points") = points_py;

            py::object err_py = py::module_::import("moveit_msgs.msg").attr("MoveItErrorCodes")();
            err_py.attr("val") = err.val;

            return py::make_tuple(success, jt_py, dur, err_py);
        })
        .def("execute", &XArmUtils::execute)
        .def("get_current_joint_values", &XArmUtils::get_current_joint_values)
        .def("get_current_pose", &XArmUtils::get_current_pose)
        .def("move_to_initial", &XArmUtils::move_to_initial);
}
