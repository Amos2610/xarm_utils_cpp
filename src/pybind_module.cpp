#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <geometry_msgs/msg/pose.hpp>
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
        }))
        .def("declare_parameter",
        [](rclcpp::Node& self, const std::string& name, py::object value_py) {
            // dict を YAML 文字列として保存するのが一番簡単（ネスト構造を壊さない）
            // ただし MoveIt は通常 “map” を期待するので、この方式は環境によっては効かない可能性あり。
            // まずは通すために文字列で入れる例：
            std::string s = py::str(value_py);  // まず雑に文字列化
            self.declare_parameter(name, s);
        });

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
        // [x,y,z,qx,qy,qz,qw] を受け取る版
        .def("set_pose_target", [](XArmUtils &self, const std::vector<double> &v) {
            if (v.size() != 7) {
                throw std::runtime_error("set_pose_target expects [x,y,z,qx,qy,qz,qw] (len=7)");
            }
            geometry_msgs::msg::Pose p;
            p.position.x = v[0];
            p.position.y = v[1];
            p.position.z = v[2];
            p.orientation.x = v[3];
            p.orientation.y = v[4];
            p.orientation.z = v[5];
            p.orientation.w = v[6];
            return self.set_pose_target(p);
        })
        // Pythonの geometry_msgs.msg.Pose を受け取る版
        .def("set_pose_target", [](XArmUtils &self, py::object pose_py) {
            // geometry_msgs.msg.Pose かどうかを軽くチェック（違っても属性があれば動く）
            geometry_msgs::msg::Pose p;
            auto pos = pose_py.attr("position");
            auto ori = pose_py.attr("orientation");

            p.position.x = pos.attr("x").cast<double>();
            p.position.y = pos.attr("y").cast<double>();
            p.position.z = pos.attr("z").cast<double>();

            p.orientation.x = ori.attr("x").cast<double>();
            p.orientation.y = ori.attr("y").cast<double>();
            p.orientation.z = ori.attr("z").cast<double>();
            p.orientation.w = ori.attr("w").cast<double>();

            return self.set_pose_target(p);
        })
        // .def("compute_ik", &XArmUtils::compute_ik)
        .def("compute_ik",
            [](XArmUtils& self,
                py::object pose_py,
                double timeout_sec,
                int attempts,
                const std::string& ik_link) -> py::object
            {
                geometry_msgs::msg::Pose p;
                auto pos = pose_py.attr("position");
                auto ori = pose_py.attr("orientation");

                p.position.x = pos.attr("x").cast<double>();
                p.position.y = pos.attr("y").cast<double>();
                p.position.z = pos.attr("z").cast<double>();

                p.orientation.x = ori.attr("x").cast<double>();
                p.orientation.y = ori.attr("y").cast<double>();
                p.orientation.z = ori.attr("z").cast<double>();
                p.orientation.w = ori.attr("w").cast<double>();

                auto ret = self.compute_ik(p, timeout_sec, attempts, ik_link);
                if (!ret) return py::none();
                return py::cast(*ret);  // vector<double> -> list[float]
            },
            py::arg("pose"),
            py::arg("timeout_sec") = 0.1,
            py::arg("attempts") = 10,
            py::arg("ik_link") = std::string("")
        )
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
        // .def("get_current_pose", &XArmUtils::get_current_pose)
        .def("get_current_pose", [](XArmUtils &self) {
            geometry_msgs::msg::Pose pose_cpp = self.get_current_pose();

            // Python側の Pose メッセージ作成
            py::object pose_py = py::module_::import("geometry_msgs.msg").attr("Pose")();
            py::object position_py = py::module_::import("geometry_msgs.msg").attr("Point")();
            position_py.attr("x") = pose_cpp.position.x;
            position_py.attr("y") = pose_cpp.position.y;
            position_py.attr("z") = pose_cpp.position.z;
            pose_py.attr("position") = position_py;

            py::object orientation_py = py::module_::import("geometry_msgs.msg").attr("Quaternion")();
            orientation_py.attr("x") = pose_cpp.orientation.x;
            orientation_py.attr("y") = pose_cpp.orientation.y;
            orientation_py.attr("z") = pose_cpp.orientation.z;
            orientation_py.attr("w") = pose_cpp.orientation.w;
            pose_py.attr("orientation") = orientation_py;

            return pose_py;
        })
        .def("gripper_command", &XArmUtils::gripper_command,
             py::arg("position"),
             py::arg("max_effort") = 50.0,
             py::arg("timeout_sec") = 5.0)
        .def("gripper_open", &XArmUtils::gripper_open,
             py::arg("max_effort") = 50.0,
             py::arg("timeout_sec") = 5.0)
        .def("gripper_close", &XArmUtils::gripper_close,
             py::arg("max_effort") = 50.0,
             py::arg("timeout_sec") = 5.0)

        .def("move_to_initial", &XArmUtils::move_to_initial);
}
