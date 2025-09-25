#pragma once

#include <string>
#include <chrono>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <array>
#include <cassert>
#include <cmath>
#include <exception>
#include <Eigen/Eigen>
#include <functional> 
#include <future>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/joint/joint-collection.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <controller_interface/controller_interface.hpp>
#include <std_msgs/msg/int32.hpp>
#include "franka_semantic_components/franka_robot_model.hpp"
#include "franka_semantic_components/franka_robot_state.hpp"

#include "math_type_define.h"
#include "suhan_benchmark.h"

namespace ConsoleColor 
{
  inline constexpr const char* RESET = "[0m";
  inline constexpr const char* BLUE  = "[34m"; // Info
  inline constexpr const char* YELLOW= "[33m"; // Warn
  inline constexpr const char* RED   = "[31m"; // Error
}

#define LOGI(node, fmt, ...) RCLCPP_INFO((node)->get_logger(),  (std::string(ConsoleColor::BLUE)   + fmt + ConsoleColor::RESET).c_str(), ##__VA_ARGS__)
#define LOGW(node, fmt, ...) RCLCPP_WARN((node)->get_logger(),  (std::string(ConsoleColor::YELLOW) + fmt + ConsoleColor::RESET).c_str(), ##__VA_ARGS__)
#define LOGE(node, fmt, ...) RCLCPP_ERROR((node)->get_logger(), (std::string(ConsoleColor::RED)    + fmt + ConsoleColor::RESET).c_str(), ##__VA_ARGS__)

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace dyros_fr3_controllers 
{
class TestEffortController : public controller_interface::ControllerInterface 
{
    public:
        // ========================================================================
        // ============================ Core Functions ============================
        // ========================================================================
        [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;
        controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        CallbackReturn on_init() override;
        CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    private:
        // ========================================================================
        // =========================== Franka robot Data ==========================
        // ========================================================================
        // ====== Joint space data ======
        // initial state
        Eigen::Vector7d q_init_;
        Eigen::Vector7d qdot_init_;

        // current state
        Eigen::Vector7d q_;
        Eigen::Vector7d qdot_;
        Eigen::Vector7d torque_;

        // control value
        Eigen::Vector7d q_desired_;
        Eigen::Vector7d qdot_desired_;
        Eigen::Vector7d torque_desired_;

        // Dynamics
        Eigen::Matrix7d M_;
        Eigen::Matrix7d M_inv_;
        Eigen::Vector7d c_;
        Eigen::Vector7d g_;

        // ====== Task space data =======
        // initial state
        Eigen::Affine3d x_init_;
        Eigen::Vector6d xdot_init_;

        // current state
        Eigen::Affine3d x_;
        Eigen::Vector6d xdot_;
        Eigen::Matrix<double, 6, 7> J_;

        //
        Eigen::Affine3d x_desired_;
        Eigen::Vector6d xdot_desired_;

        // Dynamics
        Eigen::Matrix6d M_task_;
        Eigen::Vector6d g_task_;
        Eigen::Matrix<double, 6, 7> J_T_inv_;

        // ========================================================================
        // =========================== Controller data ============================
        // ========================================================================
        const double dt_{0.001};
        Eigen::Vector7d kp_joint_;
        Eigen::Vector7d kv_joint_;
        double play_time_{0.0};
        double control_start_time_{0.0};

        enum class CtrlMode{NONE, HOME};
        CtrlMode control_mode_{CtrlMode::HOME};
        bool is_mode_changed_ {false};

        SuhanBenchmark bench_timer_;

        // ========================================================================
        // ============================== Parameters ==============================
        // ========================================================================
        std::string arm_id_;
        std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
        const int num_joints = 7;
        bool initialization_flag_{true};

        // ========================================================================
        // =========================== ROS Subs & Pubs  ===========================
        // ========================================================================
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr control_mode_sub_;

        // ========================================================================
        // ============================ Mutex & Thread ============================
        // ========================================================================
        std::mutex robot_data_mutex_;
        std::mutex calculation_mutex_;
        std::atomic<bool> compute_inflight_{false};

        // ========================================================================
        // ====================== Main Controller Functions =======================
        // ========================================================================
        void compute();
        void updateJointStates();			
        void updateRobotData();
        void setMode(CtrlMode control_mode);

        // ========================================================================
        // =========================== ROS Subs & Pubs  ===========================
        // ========================================================================
        void controlModeCallback(const std_msgs::msg::Int32& msg);

        // ========================================================================
        // ========================== Utility Functions ===========================
        // ========================================================================
        Eigen::Vector7d JointPDControl(const Eigen::Vector7d target_q, const Eigen::Vector7d target_qdot);


        // ==========================================================================
        // =========================== Pinocchio / Model ============================
        // ==========================================================================
        std::string urdf_path;
        bool use_pinocchio_{false};
        pinocchio::Model model_;
        pinocchio::Data data_;
        pinocchio::Data data_worker_;
        std::string ee_name_{"fr3_hand_tcp"};
        std::vector<std::string> link_names_{"fr3_link0", 
                                             "fr3_link1", 
                                             "fr3_link2",
                                             "fr3_link3",
                                             "fr3_link4",
                                             "fr3_link5",
                                             "fr3_link6",
                                             "fr3_link7"};
};

}  // namespace dyros_fr3_controllers

