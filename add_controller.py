#!/usr/bin/env python3

import argparse
import re
import sys
from pathlib import Path

PACKAGE_DIR = "dyros_fr3_controllers"
SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_PATH = SCRIPT_DIR / PACKAGE_DIR


def to_snake(name: str) -> str:
    s1 = re.sub(r"(.)([A-Z][a-z]+)", r"\1_\2", name)
    return re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", s1).lower()


def read_file(path: str) -> str:
    path = Path(path)
    with path.open("r", encoding="utf-8") as f:
        return f.read()


def write_file(path: str, content: str) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        f.write(content)


def append_file(path: str, content: str) -> None:
    path = Path(path)
    with path.open("a", encoding="utf-8") as f:
        f.write(content)


def update_cmakelists(new_src_rel: str) -> None:
    cmake_path = PACKAGE_PATH / "CMakeLists.txt"
    content = read_file(cmake_path)

    if new_src_rel in content:
        return

    idx = content.find("add_library(")
    if idx == -1:
        write_file(cmake_path, content)
        return
    # Find the closing ')' after the block start
    close_idx = content.find(")\n", idx)
    if close_idx == -1:
        close_idx = len(content)
    new_content = content[:close_idx] + f"\n        {new_src_rel}" + content[close_idx:]
    content = new_content

    write_file(cmake_path, content)


def update_plugin_xml(controller_class: str) -> None:
    xml_path = PACKAGE_PATH / f"{PACKAGE_DIR}_plugin.xml"
    content = read_file(xml_path)

    class_entry = (
        f"  <class name=\"{PACKAGE_DIR}/{controller_class}\"\n"
        f"         type=\"{PACKAGE_DIR}::{controller_class}\" base_class_type=\"controller_interface::ControllerInterface\">\n"
        f"    <description>Auto-generated controller</description>\n"
        f"  </class>\n"
    )

    if class_entry in content:
        return

    if "</library>" not in content:
        print(f"Error: Could not find </library> in {xml_path}")
        sys.exit(1)

    content = content.replace("</library>", class_entry + "</library>")
    write_file(xml_path, content)


def append_to_controllers_yaml(ctrl_name_snake: str, controller_class: str, control_mode: str) -> None:
    yaml_path = PACKAGE_PATH / "config" / "controllers.yaml"

    manager_block = f"""
/**:
  controller_manager:
    ros__parameters:
      {ctrl_name_snake}:
        type: {PACKAGE_DIR}/{controller_class}
"""
    if control_mode == "effort" :
        params_block = f"""
/**:
  {ctrl_name_snake}:
    ros__parameters:
      arm_id: "fr3"
      kp_joint_gains:
        - 600.0
        - 600.0
        - 600.0
        - 600.0
        - 250.0
        - 150.0
        - 50.0
      kv_joint_gains:
        - 30.0
        - 30.0
        - 30.0
        - 30.0
        - 10.0
        - 10.0
        - 5.0
"""
        
    elif control_mode == "velocity":
        params_block = f"""
/**:
  {ctrl_name_snake}:
    ros__parameters:
      arm_id: "fr3"
      kp_joint_gains:
        - 1.0
        - 1.0
        - 1.0
        - 1.0
        - 1.0
        - 1.0
        - 1.0
      kv_joint_gains:
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
"""
    elif control_mode == "position" :
        params_block = f"""
/**:
  {ctrl_name_snake}:
    ros__parameters:
      arm_id: "fr3"
"""
    

    append_file(yaml_path, manager_block)
    append_file(yaml_path, params_block)


def _prepare_template(raw: str) -> str:
    s = raw
    # escape all remaining braces for .format
    s = s.replace("{", "{{").replace("}", "}}")
    s = s.replace("___PKG___", "{0}")
    s = s.replace("___CLASS___", "{1}")
    s = s.replace("___SNAKE___", "{2}")
    s = s.replace("___MODE___", "{3}")
    s = s.replace("___CMD___", "{4}")
    s = s.replace("___DEFAULTCTRL___", "{5}")
    s = s.replace("___UTILITY_H___", "{6}")
    s = s.replace("___UTILITY___", "{7}")

    return s


def generate_from_templates(controller_class: str, control_mode: str) -> None:
    snake = to_snake(controller_class)
    new_h = PACKAGE_PATH / "include" / PACKAGE_DIR / f"{snake}.h"
    new_cpp = PACKAGE_PATH / "src" / f"{snake}.cpp"

    header_tpl_raw = """#pragma once

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
  inline constexpr const char* RESET = "\033[0m";
  inline constexpr const char* BLUE  = "\033[34m"; // Info
  inline constexpr const char* YELLOW= "\033[33m"; // Warn
  inline constexpr const char* RED   = "\033[31m"; // Error
}

#define LOGI(node, fmt, ...) RCLCPP_INFO((node)->get_logger(),  (std::string(ConsoleColor::BLUE)   + fmt + ConsoleColor::RESET).c_str(), ##__VA_ARGS__)
#define LOGW(node, fmt, ...) RCLCPP_WARN((node)->get_logger(),  (std::string(ConsoleColor::YELLOW) + fmt + ConsoleColor::RESET).c_str(), ##__VA_ARGS__)
#define LOGE(node, fmt, ...) RCLCPP_ERROR((node)->get_logger(), (std::string(ConsoleColor::RED)    + fmt + ConsoleColor::RESET).c_str(), ##__VA_ARGS__)

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace ___PKG___ 
{
class ___CLASS___ : public controller_interface::ControllerInterface 
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
        ___UTILITY_H___

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

}  // namespace ___PKG___

"""

    source_tpl_raw = """#include "___PKG___/___SNAKE___.h"
#include <___PKG___/robot_utils.hpp>

namespace ___PKG___ 
{
// ========================================================================
// ============================ Core Functions ============================
// ========================================================================
CallbackReturn ___CLASS___::on_init() 
{
  try 
  {
    auto_declare<std::string>("arm_id", "");
    auto_declare<std::vector<double>>("kp_joint_gains", {});
    auto_declare<std::vector<double>>("kd_joint_gains", {});

    const std::string pkg_share = ament_index_cpp::get_package_share_directory("dyros_fr3_controllers");
    urdf_path = pkg_share + "/urdf/fr3_franka_hand.urdf";
    std::ifstream urdf_file(urdf_path);
    if (!urdf_file.good()) 
    {
      LOGW(get_node(), "URDF not found at path: %s (Pinocchio will be disabled)", urdf_path.c_str());
      use_pinocchio_ = false;
    }

    if(use_pinocchio_)
    {
      pinocchio::urdf::buildModel(urdf_path, model_);
      data_ = pinocchio::Data(model_);
      data_worker_ = pinocchio::Data(model_);
    }

    q_init_.setZero();
    qdot_init_.setZero();

    q_.setZero();
    qdot_.setZero();
    torque_.setZero();

    q_desired_.setZero();
    qdot_desired_.setZero();
    torque_desired_.setZero();

    M_.setIdentity();
    M_inv_.setIdentity();
    c_.setZero();
    g_.setZero();

    x_init_.setIdentity();
    xdot_init_.setZero();

    x_.setIdentity();
    xdot_.setZero();
    J_.setZero();

    M_task_.setIdentity();
    g_task_.setZero();
    J_T_inv_.setZero();

    control_mode_sub_ = get_node()->create_subscription<std_msgs::msg::Int32>(
      "___SNAKE___/control_mode",
      rclcpp::QoS(10),
      std::bind(&___CLASS___::controlModeCallback, this, std::placeholders::_1)
);

  } 
  catch (const std::exception& e) 
  {
    LOGE(get_node(), "Exception during initialization: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ___CLASS___::command_interface_configuration() const 
{
  controller_interface::InterfaceConfiguration command_interface_config;
  command_interface_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) 
  {
    command_interface_config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/___MODE___");
  }
  return command_interface_config;
}

controller_interface::InterfaceConfiguration ___CLASS___::state_interface_configuration() const 
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) 
  {
    state_interfaces_config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
  }
  for (int i = 1; i <= num_joints; ++i) 
  {
    state_interfaces_config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  for (int i = 1; i <= num_joints; ++i) 
  {
    state_interfaces_config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) 
  {
    state_interfaces_config.names.push_back(franka_robot_model_name);
  }
  state_interfaces_config.names.push_back(arm_id_ + "/robot_time");
  return state_interfaces_config;
}

CallbackReturn ___CLASS___::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) 
{
  if (get_node()->get_parameter("arm_id", arm_id_)) 
  {
    arm_id_ = get_node()->get_parameter("arm_id").as_string();
  }
  else
  {
    LOGW(get_node(), "Parameter 'arm_id' not set — using defaults fr3");
    arm_id_ = "fr3";
  }

  const Eigen::Vector7d KP_DEFAULT = (Eigen::Vector7d() << 600,600,600,600,250,150,50).finished();
  const Eigen::Vector7d KV_DEFAULT = (Eigen::Vector7d() <<  30, 30, 30, 30, 10, 10, 5).finished();

  kp_joint_ = KP_DEFAULT;
  kv_joint_ = KV_DEFAULT;

  std::vector<double> kp_in;
  if (get_node()->get_parameter("kp_joint_gains", kp_in)) 
  {
    if (kp_in.size() == static_cast<size_t>(num_joints)) 
    {
      for (int i = 0; i < num_joints; ++i) kp_joint_[i] = kp_in[i];
    } 
    else 
    {
      LOGW(get_node(), "Parameter 'kp_joint_gains' size mismatch (%zu, expected %d) — using defaults", kp_in.size(), num_joints);
    }
  } 
  else 
  {
    LOGW(get_node(), "Parameter 'kp_joint_gains' not set — using defaults: [%.0f %.0f %.0f %.0f %.0f %.0f %.0f]",
         KP_DEFAULT[0], KP_DEFAULT[1], KP_DEFAULT[2], KP_DEFAULT[3], KP_DEFAULT[4], KP_DEFAULT[5], KP_DEFAULT[6]);
  }


  std::vector<double> kv_in;
  if (get_node()->get_parameter("kv_joint_gains", kv_in)) 
  {
    if (kv_in.size() == static_cast<size_t>(num_joints)) 
    {
      for (int i = 0; i < num_joints; ++i) kv_joint_[i] = kv_in[i];
    } 
    else 
    {
      LOGW(get_node(), "Parameter 'kv_joint_gains' size mismatch (%zu, expected %d) — using defaults",
            kv_in.size(), num_joints);
    }
  } 
  else 
  {
    LOGW(get_node(), "Parameter 'kv_joint_gains' not set — using defaults: [%.0f %.0f %.0f %.0f %.0f %.0f %.0f]",
         KV_DEFAULT[0], KV_DEFAULT[1], KV_DEFAULT[2], KV_DEFAULT[3], KV_DEFAULT[4], KV_DEFAULT[5], KV_DEFAULT[6]);
  }

  LOGI(get_node(), "arm_id: %s | kp: [%.0f %.0f %.0f %.0f %.0f %.0f %.0f] | kv: [%.0f %.0f %.0f %.0f %.0f %.0f %.0f]",
       arm_id_.c_str(),
       kp_joint_[0], kp_joint_[1], kp_joint_[2], kp_joint_[3], kp_joint_[4], kp_joint_[5], kp_joint_[6],
       kv_joint_[0], kv_joint_[1], kv_joint_[2], kv_joint_[3], kv_joint_[4], kv_joint_[5], kv_joint_[6]);

  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      franka_semantic_components::FrankaRobotModel(arm_id_ + "/" + "robot_model",
                                                   arm_id_ + "/" + "robot_state"));

  return CallbackReturn::SUCCESS;
}

CallbackReturn ___CLASS___::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) 
{
  initialization_flag_ = true;

  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

  updateJointStates();
  updateRobotData();

  {
    std::lock_guard<std::mutex> lk(robot_data_mutex_);
    q_desired_ = q_;
    qdot_desired_.setZero();
    torque_desired_ = c_;
  }

  LOGI(get_node(), "Controller activated (arm_id: %s, dt: %.3f ms)", arm_id_.c_str(), dt_ * 1000.0);
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ___CLASS___::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) 
{
  franka_robot_model_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type ___CLASS___::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
  SuhanBenchmark bench;

  updateJointStates();
  updateRobotData();

  {
    std::lock_guard<std::mutex> lk(robot_data_mutex_);
    play_time_ = state_interfaces_.back().get_value();
  }

  const double spent_ms = bench.elapsed() * 1000.;
  double budget_ms = (dt_*1000.) - spent_ms - 0.05; // 0.05 ms for command to robot
  if (budget_ms < 0.0) 
  {
    budget_ms = 0.0;
    LOGW(get_node(), "State update exceeded 1.0 ms (%.3f ms)", spent_ms);
  }

  Eigen::Vector7d last_command;
  {
    std::lock_guard<std::mutex> lk(calculation_mutex_);
    last_command = ___CMD___;
  }

  bool used_new_solution = false;

  if (!compute_inflight_.exchange(true, std::memory_order_acq_rel)) 
  {
    std::packaged_task<void()> task([this](){
      try 
      {
        this->compute();
      } 
      catch (const std::exception& e) 
      {
        LOGE(get_node(), "Exception in compute(): %s", e.what());
      } 
      catch (...) 
      {
        LOGE(get_node(), "Unknown exception in compute()");
      }
      compute_inflight_.store(false, std::memory_order_release);
    });
    std::future<void> fut = task.get_future();
    std::thread(std::move(task)).detach();

    const auto wait_dur = std::chrono::duration<double, std::milli>(budget_ms);
    if (fut.wait_for(wait_dur) == std::future_status::ready) 
    {
      used_new_solution = true;
    }
    else
    {
      LOGW(get_node(), "Background compute timed out (waited %.3f ms) — reusing last torque", budget_ms);
    }
  } 

  Eigen::Vector7d command;
  if (used_new_solution) 
  {
    std::lock_guard<std::mutex> lk(calculation_mutex_);
    command = ___CMD___;
  } 
  else 
  {
    command = last_command;
  }

  for (int i = 0; i < num_joints; ++i) 
  {
    command_interfaces_[i].set_value(command[i]);
  }

  return controller_interface::return_type::OK;
}

// ========================================================================
// ====================== Main Controller Functions =======================
// ========================================================================
void ___CLASS___::compute()
{
  std::scoped_lock(robot_data_mutex_, calculation_mutex_);
  if (initialization_flag_ || is_mode_changed_) 
  {            
    control_start_time_ = play_time_;

    q_init_ = q_;
    qdot_init_ = qdot_;
    q_desired_ = q_init_;
    qdot_desired_.setZero();

    x_init_ = x_;
    xdot_init_ = xdot_;
    x_desired_ = x_init_;
    xdot_desired_.setZero();

    initialization_flag_ = false;
    is_mode_changed_ = false;
  }

  if(control_mode_ == CtrlMode::HOME)
  {
    Eigen::Vector7d home_q; 
    home_q << 0, 0, 0, -M_PI/2, 0, M_PI/2, M_PI/4;
    q_desired_ = DyrosMath::cubicVector<7>(play_time_,
                                           control_start_time_,
                                           control_start_time_ + 4.0,
                                           q_init_,
                                           home_q,
                                           qdot_init_,
                                           Eigen::Vector7d::Zero());

    qdot_desired_ = DyrosMath::cubicDotVector<7>(play_time_,
                                                 control_start_time_,
                                                 control_start_time_ + 4.0,
                                                 q_init_,
                                                 home_q,
                                                 qdot_init_,
                                                 Eigen::Vector7d::Zero());
    
    ___DEFAULTCTRL___
  }
  else
  {
    q_desired_ = q_;
    qdot_desired_.setZero();
    torque_desired_ = c_;
  }
}

void ___CLASS___::updateJointStates() 
{
  std::lock_guard<std::mutex> lk(robot_data_mutex_);
  for (int i = 0; i < num_joints; ++i) 
  {
    const auto& position_interface = state_interfaces_.at(i);
    const auto& velocity_interface = state_interfaces_.at(num_joints + i);
    const auto& effort_interface = state_interfaces_.at(2 * num_joints + i);
    q_[i] = position_interface.get_value();
    qdot_[i] = velocity_interface.get_value();
    torque_[i] = effort_interface.get_value();
  }
}

void ___CLASS___::updateRobotData()
{
  std::array<double, 49> mass = franka_robot_model_->getMassMatrix();
  std::array<double, 7> coriolis = franka_robot_model_->getCoriolisForceVector();
  std::array<double, 7> gravity = franka_robot_model_->getGravityForceVector();
  std::array<double, 16> pose = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
  std::array<double, 42> endeffector_jacobian_wrt_base = franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
  {
    std::lock_guard<std::mutex> lock(robot_data_mutex_);
    M_ = Eigen::Map<const Eigen::Matrix<double, 7, 7, Eigen::RowMajor>>(mass.data());
    c_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(coriolis.data());
    g_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(gravity.data());

    x_.matrix() = Eigen::Map<const Eigen::Matrix4d>(pose.data());

    Eigen::Map<const Eigen::Matrix<double,6,7,Eigen::ColMajor>> J_tmp(endeffector_jacobian_wrt_base.data());
    J_ = J_tmp;

    M_inv_ = M_.inverse();
    xdot_ = J_ * qdot_;
    // M_task_ = (J_ * M_inv_ * J_.transpose()).inverse();
    // J_T_inv_ = M_task_ * J_ * M_inv_;
  }
}

void ___CLASS___::setMode(CtrlMode mode)
{
  std::lock_guard<std::mutex> lk(calculation_mutex_);

  if (control_mode_ == mode) 
  {
    LOGI(get_node(), "Mode unchanged: %d", static_cast<int>(mode));
    return;
  }
  control_mode_ = mode;
  is_mode_changed_ = true;

  LOGI(get_node(), "Mode changed: %d", static_cast<int>(control_mode_));
}

// ========================================================================
// =========================== ROS Subs & Pubs  ===========================
// ========================================================================
void ___CLASS___::controlModeCallback(const std_msgs::msg::Int32& msg)
{
  LOGI(get_node(), "Mode input received: %d", msg.data);
  switch (msg.data) 
  {
    case 0: setMode(CtrlMode::NONE); break;
    case 1: setMode(CtrlMode::HOME); break;
    default: LOGW(get_node(), "Unknown mode value: %d (ignored)", msg.data); break;
  }
}


// ========================================================================
// ========================== Utility Functions ===========================
// ========================================================================	
___UTILITY___


}  // namespace ___PKG___
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(___PKG___::___CLASS___,
                       controller_interface::ControllerInterface)

        """
    header_tpl = _prepare_template(header_tpl_raw)
    source_tpl = _prepare_template(source_tpl_raw)

    if control_mode == "position":
        cmd_line = "q_desired_"
        default_control = ""
        utility_h = ""
        utility = ""
    elif control_mode == "velocity":
        cmd_line = "qdot_desired_"
        default_control = "qdot_desired_ = JointPDControl(q_desired_, qdot_desired_);"
        utility_h = "Eigen::Vector7d JointPDControl(const Eigen::Vector7d target_q, const Eigen::Vector7d target_qdot);\n"
        utility = (
            f"Eigen::Vector7d {controller_class}::JointPDControl(const Eigen::Vector7d target_q, "
            "const Eigen::Vector7d target_qdot)\n"
            "{\n"
            "  Eigen::Vector7d q_error = target_q - q_;\n"
            "  Eigen::Vector7d qdot_error = target_qdot - qdot_;\n"
            "  return kp_joint_.asDiagonal() * (q_error) + kv_joint_.asDiagonal() * (qdot_error);\n"
            "}\n"
        )
    else:
        cmd_line = "torque_desired_"
        default_control = "torque_desired_ = JointPDControl(q_desired_, qdot_desired_);"
        utility_h = "Eigen::Vector7d JointPDControl(const Eigen::Vector7d target_q, const Eigen::Vector7d target_qdot);\n"
        utility = (
            f"Eigen::Vector7d {controller_class}::JointPDControl(const Eigen::Vector7d target_q, "
            "const Eigen::Vector7d target_qdot)\n"
            "{\n"
            "  Eigen::Vector7d q_error = target_q - q_;\n"
            "  Eigen::Vector7d qdot_error = target_qdot - qdot_;\n"
            "  return kp_joint_.asDiagonal() * (q_error) + kv_joint_.asDiagonal() * (qdot_error) + c_;\n"
            "}\n"
        )

    h_content = header_tpl.format(PACKAGE_DIR, controller_class, snake, control_mode, cmd_line, default_control, utility_h, utility)
    cpp_content = source_tpl.format(PACKAGE_DIR, controller_class, snake, control_mode, cmd_line, default_control, utility_h, utility)

    write_file(new_h, h_content)
    write_file(new_cpp, cpp_content)

    update_cmakelists(f"src/{snake}.cpp")
    update_plugin_xml(controller_class)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller_name", type=str, 
                                        required=True, 
                                        help="Name of Controller")

    parser.add_argument("--control_mode", type=str, 
                                          required=True, 
                                          help="Control Mode [position, velocity, effort]")

    args = parser.parse_args()

    controller_class = args.controller_name
    control_mode = args.control_mode
    if not re.match(r"^[A-Z][A-Za-z0-9_]*$", controller_class):
        print("Error: controller_name should be a valid C++ class-like identifier starting with uppercase.")
        sys.exit(1)
    if control_mode not in ["position", "velocity", "effort"]:
        print(f"Control Mode must be [position, velocity, effort]! Input is {control_mode}")
        sys.exit(1)

    generate_from_templates(controller_class, control_mode)
    append_to_controllers_yaml(to_snake(controller_class), controller_class, control_mode)

    print(f"Generated controller: {controller_class}")

if __name__ == "__main__":
    main()
