#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include "robot.hpp"
#include "motor.hpp"
#include "serial_struct.hpp"

namespace py = pybind11;
using namespace hightorque_robot;

PYBIND11_MODULE(hightorque_robot_py, m) {
    m.doc() = "Python bindings for hightorque_robot library";

    // Enums
    py::enum_<motor_type>(m, "MotorType")
        .value("null", motor_type::null)
        .value("3536_32", motor_type::m3536_32)
        .value("m4538_19", motor_type::m4538_19)
        .value("m5046_20", motor_type::m5046_20)
        .value("m5047_09", motor_type::m5047_09)
        // .value("m5047_19", motor_type::m5047_19)
        // .value("m5047_20", motor_type::m5047_20)
        // .value("m5047_30", motor_type::m5047_30)
        .value("m5047_36", motor_type::m5047_36)
        // .value("m4438_08", motor_type::m4438_08)
        // .value("m4438_16", motor_type::m4438_16)
        .value("m4438_30", motor_type::m4438_30)
        .value("m4438_32", motor_type::m4438_32)
        // .value("m7136_07", motor_type::m7136_07)
        // .value("m7233_08", motor_type::m7233_08)
        // .value("m6056_08", motor_type::m6056_08)
        .value("m6056_36", motor_type::m6056_36)
        // .value("m3536_32", motor_type::m3536_32)
        .value("m5043_20", motor_type::m5043_20)
        // .value("m5043_35", motor_type::m5043_35)
        .value("m7256_35", motor_type::m7256_35)
        // .value("m6057_36", motor_type::m6057_36)
        .value("60SG_35", motor_type::m60sg_35)
        .value("60BM_35", motor_type::m60bm_35)
        .value("5047_36_2", motor_type::m5047_36_2)
        .value("mGeneral", motor_type::mGeneral);

    py::enum_<pos_vel_convert_type>(m, "PosVelConvertType")
        .value("radian_2pi", pos_vel_convert_type::radian_2pi)
        .value("angle_360", pos_vel_convert_type::angle_360)
        .value("turns", pos_vel_convert_type::turns);

    // Structs
    py::class_<motor_back_t>(m, "MotorBackData")
        .def(py::init<>())
        .def_readwrite("time", &motor_back_t::time)
        .def_readwrite("ID", &motor_back_t::ID)
        .def_readwrite("mode", &motor_back_t::mode)
        .def_readwrite("fault", &motor_back_t::fault)
        .def_readwrite("position", &motor_back_t::position)
        .def_readwrite("velocity", &motor_back_t::velocity)
        .def_readwrite("torque", &motor_back_t::torque);

    py::class_<cdc_rx_motor_version_s>(m, "MotorVersion")
        .def(py::init<>())
        .def_readwrite("id", &cdc_rx_motor_version_s::id)
        .def_readwrite("major", &cdc_rx_motor_version_s::major)
        .def_readwrite("minor", &cdc_rx_motor_version_s::minor)
        .def_readwrite("patch", &cdc_rx_motor_version_s::patch);

    // Motor class
    py::class_<motor>(m, "Motor")
        .def("position", &motor::position, "Set motor position")
        .def("velocity", &motor::velocity, "Set motor velocity")
        .def("torque", &motor::torque, "Set motor torque")
        .def("voltage", &motor::voltage, "Set motor voltage")
        .def("current", &motor::current, "Set motor current")
        .def("pos_vel_MAXtqe", &motor::pos_vel_MAXtqe, "Position-velocity control with max torque limit")
        .def("pos_vel_tqe_kp_kd", &motor::pos_vel_tqe_kp_kd, "Position-velocity-torque control with Kp/Kd(Deprecated, It is recommended to use pos_vel_tqe_kp_kd2)")
        .def("pos_vel_tqe_kp_kd2", &motor::pos_vel_tqe_kp_kd2, "Position-velocity-torque control with Kp/Kd")
        .def("pos_vel_kp_kd", &motor::pos_vel_kp_kd, "Position-velocity control with Kp/Kd")
        .def("pos_vel_acc", &motor::pos_vel_acc, "Position-velocity control with acceleration")
        .def("get_motor_id", &motor::get_motor_id, "Get motor ID")
        .def("get_motor_type", &motor::get_motor_type, "Get motor type")
        .def("get_motor_enum_type", &motor::get_motor_enum_type, "Get motor enum type")
        .def("get_motor_num", &motor::get_motor_num, "Get motor number")
        .def("set_motor_type", &motor::set_motor_type, "Set motor type")
        .def("get_motor_belong_canport", &motor::get_motor_belong_canport, "Get CAN port number")
        .def("get_motor_belong_canboard", &motor::get_motor_belong_canboard, "Get CAN board number")
        .def("get_current_motor_state", &motor::get_current_motor_state, 
             "Get current motor state", py::return_value_policy::reference_internal)
        .def("get_motor_name", &motor::get_motor_name, "Get motor name")
        .def("get_version", &motor::get_version, "Get motor version", py::return_value_policy::reference_internal)
        .def("print_version", &motor::print_version, "Print motor version")
        .def_readwrite("pos_limit_flag", &motor::pos_limit_flag)
        .def_readwrite("tor_limit_flag", &motor::tor_limit_flag);

    // Robot class
    py::class_<robot>(m, "Robot")
        .def(py::init<>())
        .def("lcm_enable", &robot::lcm_enable, "Enable LCM communication")
        .def("motor_send_2", &robot::motor_send_2, "Send motor commands")
        .def("set_stop", &robot::set_stop, "Stop all motors")
        .def("set_reset", &robot::set_reset, "Reset all motors")
        .def("set_reset_zero", py::overload_cast<>(&robot::set_reset_zero), "Reset zero position for all motors")
        .def("set_reset_zero_motors", [](robot& r, const std::vector<int>& motors) {
            // Create a temporary initializer list from vector
            switch(motors.size()) {
                case 0: r.set_reset_zero({}); break;
                case 1: r.set_reset_zero({motors[0]}); break;
                case 2: r.set_reset_zero({motors[0], motors[1]}); break;
                case 3: r.set_reset_zero({motors[0], motors[1], motors[2]}); break;
                case 4: r.set_reset_zero({motors[0], motors[1], motors[2], motors[3]}); break;
                default: r.set_reset_zero({motors[0], motors[1], motors[2], motors[3]}); break; // max 4 motors
            }
        }, "Reset zero position for specific motors")
        .def("set_motor_runzero", &robot::set_motor_runzero, "Run motor zero calibration")
        .def("set_timeout", py::overload_cast<int16_t>(&robot::set_timeout), "Set timeout for all motors")
        .def("set_timeout_port", py::overload_cast<uint8_t, int16_t>(&robot::set_timeout), "Set timeout for specific port")
        .def("motor_version_detection", &robot::motor_version_detection, "Detect motor versions")
        .def("set_data_reset", &robot::set_data_reset, "Reset data")
        .def("canboard_bootloader", &robot::canboard_bootloader, "Enter bootloader mode")
        .def("publishJointStates", &robot::publishJointStates, "Publish joint states via LCM")
        .def("detect_motor_limit", &robot::detect_motor_limit, "Detect motor limits")
        .def("send_get_motor_state_cmd", &robot::send_get_motor_state_cmd, "Send get motor state command")
        .def("send_get_motor_version_cmd", &robot::send_get_motor_version_cmd, "Send get motor version command")
        .def_readonly("Motors", &robot::Motors, "List of motors")
        .def_readwrite("motor_position_limit_flag", &robot::motor_position_limit_flag)
        .def_readwrite("motor_torque_limit_flag", &robot::motor_torque_limit_flag);

    // Utility functions
    m.def("get_motor_type_map", []() { return motor_type2; }, "Get motor type string to enum mapping");
    
    // Constants
    m.attr("MODE_POSITION") = MODE_POSITION;
    m.attr("MODE_VELOCITY") = MODE_VELOCITY;
    m.attr("MODE_TORQUE") = MODE_TORQUE;
    m.attr("MODE_VOLTAGE") = MODE_VOLTAGE;
    m.attr("MODE_CURRENT") = MODE_CURRENT;
    m.attr("MODE_TIME_OUT") = MODE_TIME_OUT;
    m.attr("MODE_POS_VEL_TQE") = MODE_POS_VEL_TQE;
    m.attr("MODE_POS_VEL_TQE_KP_KD") = MODE_POS_VEL_TQE_KP_KD;
    m.attr("MODE_POS_VEL_KP_KD") = MODE_POS_VEL_KP_KD;
    m.attr("MODE_POS_VEL_ACC") = MODE_POS_VEL_ACC;
    m.attr("MODE_POS_VEL_TQE_KP_KD2") = MODE_POS_VEL_TQE_KP_KD2;
}