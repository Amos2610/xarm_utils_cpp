#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
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
        .def("plan", &XArmUtils::plan)
        .def("execute", &XArmUtils::execute)
        .def("get_current_joint_values", &XArmUtils::get_current_joint_values)
        .def("get_current_pose", &XArmUtils::get_current_pose)
        .def("move_to_initial", &XArmUtils::move_to_initial);
}
