#!/usr/bin/env python3

import argparse
import os
import re
import sys

PACKAGE_DIR = "kyu_franka_controllers"


def to_snake(name: str) -> str:
    s1 = re.sub(r"(.)([A-Z][a-z]+)", r"\1_\2", name)
    return re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", s1).lower()


def read_file(path: str) -> str:
    with open(path, "r", encoding="utf-8") as f:
        return f.read()


def write_file(path: str, content: str) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(content)


def append_file(path: str, content: str) -> None:
    with open(path, "a", encoding="utf-8") as f:
        f.write(content)


def update_cmakelists(new_src_rel: str) -> None:
    cmake_path = os.path.join(PACKAGE_DIR, "CMakeLists.txt")
    content = read_file(cmake_path)

    if new_src_rel in content:
        return

    # Simple replace when the closing parenthesis is on the same line
    replaced = content.replace(
        "src/default_controller.cpp)", f"src/default_controller.cpp\n        {new_src_rel})"
    )
    if replaced != content:
        write_file(cmake_path, replaced)
        return

    # Fallback: try to locate add_library block and inject before its closing parenthesis
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
    xml_path = os.path.join(PACKAGE_DIR, f"{PACKAGE_DIR}_plugin.xml")
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
    yaml_path = os.path.join(PACKAGE_DIR, "config", "controllers.yaml")

    manager_block = f"""
/**:
  controller_manager:
    ros__parameters:
      {ctrl_name_snake}:
        type: {PACKAGE_DIR}/{controller_class}
"""
    if control_mode == "position" or control_mode == "effort" :
        params_block = f"""
/**:
  {ctrl_name_snake}:
    ros__parameters:
      arm_id: "fr3"
      k_gains:
        - 600.0
        - 600.0
        - 600.0
        - 600.0
        - 250.0
        - 150.0
        - 50.0
      d_gains:
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
      k_gains:
        - 1.0
        - 1.0
        - 1.0
        - 1.0
        - 1.0
        - 1.0
        - 1.0
      d_gains:
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
        - 0.0
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
    s = s.replace("___PDCTRL_H___", "{6}")
    s = s.replace("___PDCTRL___", "{7}")

    return s


def generate_from_templates(controller_class: str, control_mode: str) -> None:
    snake = to_snake(controller_class)
    new_h = os.path.join(PACKAGE_DIR, "include", PACKAGE_DIR, f"{snake}.h")
    new_cpp = os.path.join(PACKAGE_DIR, "src", f"{snake}.cpp")

    header_tpl_raw = """// Copyright (c) 2023 Franka Robotics GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <string>
#include <chrono>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <array>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include "franka_semantic_components/franka_robot_model.hpp"
#include "franka_semantic_components/franka_robot_state.hpp"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/joint/joint-collection.hpp>

#include "math_type_define.h"
#include "suhan_benchmark.h"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace ___PKG___ {

class ___CLASS___ : public controller_interface::ControllerInterface {
 public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  // pinocchio data
  std::string urdf_path;
  pinocchio::Model model_;
  pinocchio::Data data_;

  bool assign_parameters();
  std::string arm_id_;
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
  bool is_gazebo_{false};
  std::string robot_description_;
  const int num_joints = 7;
  double initial_robot_time_ = 0.0;
  double robot_time_ = 0.0;
  double trajectory_period_ = 0.001;
  bool initialization_flag_{true};

  // ========================================================================
  // =========================== Franka robot Data ==========================
  // ========================================================================
  std::string ee_name_{"fr3_hand_tcp"};
  // ====== Joint space data ======
  // initial state
  Eigen::Vector7d q_init_;
  Eigen::Vector7d qdot_init_;

  // current state
  Eigen::Vector7d q_;
  Eigen::Vector7d qdot_;
  Eigen::Vector7d qdot_filtered_;
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

  Eigen::Vector7d g_pin_;

  // ====== Task space data =======
  std::vector<std::string> link_names_{"fr3_link0", 
                                       "fr3_link1", 
                                       "fr3_link2",
                                       "fr3_link3",
                                       "fr3_link4",
                                       "fr3_link5",
                                       "fr3_link6",
                                       "fr3_link7"};
  // initial state
  Eigen::Matrix4d x_init_;
  Eigen::Vector6d xdot_init_;
  Eigen::Matrix4d x2_init_;

  // current state
  Eigen::Matrix4d x_;
  Eigen::Matrix4d x2_;
  Eigen::Vector6d xdot_;
  Eigen::Matrix<double, 6, 7> J_;

  // Dynamics
  Eigen::Matrix6d M_task_;
  Eigen::Vector6d g_task_;
  Eigen::Matrix<double, 6, 7> J_T_inv_;

  // ========================================================================
  // ============================ Mutex & Thread ============================
  // ========================================================================
  std::mutex robot_data_mutex_;
  std::mutex calculation_mutex_;
  std::thread async_calculation_thread_;
  std::condition_variable calculation_cv_;

  bool exit_calculation_thread_{false};
  bool work_ready_{false};

  // ========================================================================
  // =========================== Controller data ============================
  // ========================================================================
  const double dt_{0.001};
  Eigen::Vector7d kp_;
  Eigen::Vector7d kv_;
  double start_time_{0.0};
  double play_time_{0.0};
  double control_start_time_{0.0};

  enum CTRL_MODE{NONE, HOME};
  CTRL_MODE control_mode_{NONE};
  bool is_mode_changed_ {false};

  SuhanBenchmark bench_timer_;

  // ========================================================================
  // =========================== Thread Functions ===========================
  // ========================================================================
  void runCalculationWorker();
  void asyncCalculationProc();

  // ========================================================================
  // ========================== Utility Functions ===========================
  // ========================================================================
  void update_joint_states();			
  void updateRobotData();
  Eigen::MatrixXd getEEJac(const Eigen::VectorXd& q);
  Eigen::MatrixXd getGravity(const Eigen::VectorXd& q);
  Eigen::Matrix4d getEEPose(const Eigen::VectorXd& q);
  Eigen::MatrixXd getLinkJac(const Eigen::VectorXd& q, const std::string& link_name);
  Eigen::Matrix4d getLinkPose(const Eigen::VectorXd& q, const std::string& link_name);
  ___PDCTRL_H___
};

}  // namespace ___PKG___
"""

    source_tpl_raw = """#include "___PKG___/___SNAKE___.h"
#include <___PKG___/robot_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>

namespace ___PKG___ {

CallbackReturn ___CLASS___::on_init() 
{
  try {
    auto_declare<std::string>("arm_id", "");
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});

    const std::string pkg_share = ament_index_cpp::get_package_share_directory("kyu_franka_controllers");
    urdf_path = pkg_share + "/urdfs/fr3_franka_hand.urdf";

    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = pinocchio::Data(model_);
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
___CLASS___::command_interface_configuration() const 
{
  controller_interface::InterfaceConfiguration command_interface_config;
  command_interface_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    command_interface_config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/___MODE___");
  }
  return command_interface_config;
}

controller_interface::InterfaceConfiguration
___CLASS___::state_interface_configuration() const 
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    state_interfaces_config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
  }
  for (int i = 1; i <= num_joints; ++i) {
    state_interfaces_config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  for (int i = 1; i <= num_joints; ++i) {
    state_interfaces_config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
    state_interfaces_config.names.push_back(franka_robot_model_name);
  }
  state_interfaces_config.names.push_back(arm_id_ + "/robot_time");
  return state_interfaces_config;
}

CallbackReturn ___CLASS___::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) 
{
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  auto kp = get_node()->get_parameter("k_gains").as_double_array();
  auto kv = get_node()->get_parameter("d_gains").as_double_array();
  if (kp.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (kp.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains should be of size %d but is of size %ld",
                 num_joints, kp.size());
    return CallbackReturn::FAILURE;
  }
  if (kv.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (kv.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains should be of size %d but is of size %ld",
                 num_joints, kv.size());
    return CallbackReturn::FAILURE;
  }
  for (int i = 0; i < num_joints; ++i) {
    kv_(i) = kv.at(i);
    kp_(i) = kp.at(i);
  }
  qdot_filtered_.setZero();

  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "robot_state_publisher");
  parameters_client->wait_for_service();

  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }

  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      franka_semantic_components::FrankaRobotModel(arm_id_ + "/" + "robot_model",
                                                   arm_id_ + "/" + "robot_state"));

  return CallbackReturn::SUCCESS;
}

CallbackReturn ___CLASS___::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) 
{
  initialization_flag_ = true;

  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

  bench_timer_.reset();

  // Start worker thread
  {
    std::lock_guard<std::mutex> lk(calculation_mutex_);
    exit_calculation_thread_ = false;
    work_ready_ = false;
  }
  if (!async_calculation_thread_.joinable()) {
    async_calculation_thread_ = std::thread(&___CLASS___::runCalculationWorker, this);
  }
  update_joint_states();
  updateRobotData();

  q_desired_ = q_;
  torque_desired_ = c_;

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ___CLASS___::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) 
{
  franka_robot_model_->release_interfaces();

  // Stop worker thread
  {
    std::lock_guard<std::mutex> lk(calculation_mutex_);
    exit_calculation_thread_ = true;
    work_ready_ = true;
  }
  calculation_cv_.notify_one();
  if (async_calculation_thread_.joinable()) async_calculation_thread_.join();

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type ___CLASS___::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) 
{
  update_joint_states();
  updateRobotData();

  // Signal worker
  double play_time = state_interfaces_.back().get_value();
  {
    std::lock_guard<std::mutex> lk(calculation_mutex_);
    play_time_ = play_time;           // protect play_time_ handoff
    work_ready_ = true;
  }
  calculation_cv_.notify_one();

  // Copy command under mutex to avoid race with writer thread
  Eigen::Vector7d command;
  {
    std::lock_guard<std::mutex> lk(calculation_mutex_);
    ___CMD___
  }

  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(command[i]);
  }

  return controller_interface::return_type::OK;
}

// ========================================================================
// =========================== Thread Functions ===========================
// ========================================================================

void ___CLASS___::runCalculationWorker()
{
  while (true) {
    {
      std::unique_lock<std::mutex> lk(calculation_mutex_);
      calculation_cv_.wait(lk, [this]{ return work_ready_ || exit_calculation_thread_; });
      if (exit_calculation_thread_) break;
      work_ready_ = false;
    }
    asyncCalculationProc();
  }
}

void ___CLASS___::asyncCalculationProc()
{
  // compute without holding locks
  Eigen::Vector7d q_local, qdot_local;
  Eigen::Matrix4d x_local, x2_local;

  {
    std::lock_guard<std::mutex> rd_lock(robot_data_mutex_);
    q_local = q_;
    qdot_local = qdot_;
    x_local = x_;
    x2_local = x2_;
  }

  static Eigen::Vector7d q_init_local, qdot_init_local;

  if (initialization_flag_ || is_mode_changed_) {
    q_init_local = q_local;
    qdot_init_local = qdot_local;
    x_init_ = x_local;
    x2_init_ = x2_local;
    control_start_time_ = play_time_;
    
    initialization_flag_ = false;
    is_mode_changed_ = false;
  }

  Eigen::Vector7d home_q; home_q << 0, 0, 0, -M_PI/2, 0, M_PI/2, M_PI/4;
  Eigen::Vector7d q_des_local = DyrosMath::cubicVector<7>(play_time_,
                                                          control_start_time_,
                                                          control_start_time_ + 4,
                                                          q_init_local,
                                                          home_q,
                                                          Eigen::Vector7d::Zero(),
                                                          Eigen::Vector7d::Zero());

  ___DEFAULTCTRL___
}

// ================================================================================================
// ====================================== Utility Functions =======================================
// ================================================================================================	

void ___CLASS___::update_joint_states() {
  for (auto i = 0; i < num_joints; ++i) {
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
  // std::cout << "updateRobotData dt: " << bench_timer_.elapsedAndReset() * 1000.0 << " ms" << std::endl;
  std::array<double, 49> mass = franka_robot_model_->getMassMatrix();
  std::array<double, 7> coriolis = franka_robot_model_->getCoriolisForceVector();
  std::array<double, 7> gravity = franka_robot_model_->getGravityForceVector();
  std::array<double, 16> pose = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
  std::array<double, 42> endeffector_jacobian_wrt_base =
      franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
  {
    std::lock_guard<std::mutex> lock(robot_data_mutex_);
    M_ = Eigen::Map<const Eigen::Matrix<double, 7, 7, Eigen::RowMajor>>(mass.data());
    c_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(coriolis.data());
    g_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(gravity.data());
    x_ = getEEPose(q_);
    x2_ = getLinkPose(q_, link_names_[4]);

    // Map Jacobian: libfranka exposes a 6x7 array in column-major (each joint column packed contiguously)
    Eigen::Matrix<double,6,7> J_tmp;
    for (int c = 0; c < 7; ++c) {
      for (int r = 0; r < 6; ++r) {
        J_tmp(r, c) = endeffector_jacobian_wrt_base[c * 6 + r];
      }
    }
    J_ = J_tmp;
  }
}

Eigen::MatrixXd ___CLASS___::getGravity(const Eigen::VectorXd& q)
{
    if(q.size() != model_.nq)
    {
        std::cerr << "getEEJac Error: size of q " << q.size() << " is not equal to model.nq size: " << model_.nq << std::endl;
        return Eigen::MatrixXd::Zero(6, model_.nv);
    }
    pinocchio::FrameIndex ee_index = model_.getFrameId(ee_name_);
    if (ee_index == static_cast<pinocchio::FrameIndex>(-1))  
    {
        std::cerr << "Error: Link name " << ee_name_ << " not found in URDF." << std::endl;
        return Eigen::MatrixXd::Zero(6, model_.nv);
    }

    Eigen::MatrixXd J;
    J.setZero(6, model_.nv);
    pinocchio::Data data_tmp(model_);
    pinocchio::crba(model_, data_tmp, q);
    pinocchio::computeGeneralizedGravity(model_,data_tmp,q);

    return data_tmp.g;
}

Eigen::MatrixXd ___CLASS___::getEEJac(const Eigen::VectorXd& q)
{
    if(q.size() != model_.nq)
    {
        std::cerr << "getEEJac Error: size of q " << q.size() << " is not equal to model.nq size: " << model_.nq << std::endl;
        return Eigen::MatrixXd::Zero(6, model_.nv);
    }
    pinocchio::FrameIndex ee_index = model_.getFrameId(ee_name_);
    if (ee_index == static_cast<pinocchio::FrameIndex>(-1))  
    {
        std::cerr << "Error: Link name " << ee_name_ << " not found in URDF." << std::endl;
        return Eigen::MatrixXd::Zero(6, model_.nv);
    }

    Eigen::MatrixXd J;
    J.setZero(6, model_.nv);
    pinocchio::Data data_tmp(model_);
    pinocchio::computeJointJacobians(model_, data_tmp, q);
    pinocchio::getFrameJacobian(model_, data_tmp, ee_index, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);

    return J;
}

Eigen::Matrix4d ___CLASS___::getEEPose(const Eigen::VectorXd& q)
{
    if(q.size() != model_.nq)
    {
        std::cerr << "getEEPose Error: size of q " << q.size() << " is not equal to model.nq size: " << model_.nq << std::endl;
        return Eigen::Matrix4d::Identity();
    }
    pinocchio::FrameIndex ee_index = model_.getFrameId(ee_name_);
    if (ee_index == static_cast<pinocchio::FrameIndex>(-1))  
    {
        std::cerr << "Error: Link name " << ee_name_ << " not found in URDF." << std::endl;
        return Eigen::Matrix4d::Identity();
    }

    pinocchio::Data data_tmp(model_);
    pinocchio::framesForwardKinematics(model_, data_tmp, q);
    return data_tmp.oMf[ee_index].toHomogeneousMatrix();
}

Eigen::MatrixXd ___CLASS___::getLinkJac(const Eigen::VectorXd& q, const std::string& link_name)
{
    if(q.size() != model_.nq)
    {
        std::cerr << "getEEJac Error: size of q " << q.size() << " is not equal to model.nq size: " << model_.nq << std::endl;
        return Eigen::MatrixXd::Zero(6, model_.nv);
    }
    pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
    if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
    {
        std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
        return Eigen::MatrixXd::Zero(6, model_.nv);
    }

    Eigen::MatrixXd J;
    J.setZero(6, model_.nv);
    pinocchio::Data data_tmp(model_);
    pinocchio::computeJointJacobians(model_, data_tmp, q);
    pinocchio::getFrameJacobian(model_, data_tmp, link_index, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);

    return J;
}

Eigen::Matrix4d ___CLASS___::getLinkPose(const Eigen::VectorXd& q, const std::string& link_name)
{
    if(q.size() != model_.nq)
    {
        std::cerr << "getEEPose Error: size of q " << q.size() << " is not equal to model.nq size: " << model_.nq << std::endl;
        return Eigen::Matrix4d::Identity();
    }
    pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
    if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
    {
        std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
        return Eigen::Matrix4d::Identity();
    }

    pinocchio::Data data_tmp(model_);
    pinocchio::framesForwardKinematics(model_, data_tmp, q);
    return data_tmp.oMf[link_index].toHomogeneousMatrix();
}

___PDCTRL___

}  // namespace ___PKG___
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(___PKG___::___CLASS___,
                       controller_interface::ControllerInterface)
"""
    header_tpl = _prepare_template(header_tpl_raw)
    source_tpl = _prepare_template(source_tpl_raw)

    if control_mode == "position":
        cmd_line = "command = q_desired_;"
        default_ctrl = "q_desired_ = q_des_local;"
        pd_ctrl_h = " "
        pd_ctrl = " "
    elif control_mode == "velocity":
        cmd_line = "command = qdot_desired_;"
        default_ctrl = "JointPDControl(q_des_local);"
        pd_ctrl_h = "void JointPDControl(const Eigen::Vector7d target_q);"
        pd_ctrl = """
void ___CLASS___::JointPDControl(const Eigen::Vector7d target_q)
{
  const double kAlpha = 0.99;
  qdot_filtered_ = (1 - kAlpha) * qdot_filtered_ + kAlpha * qdot_;
  Eigen::Vector7d q_error = target_q - q_;
  qdot_desired_ =  kp_.cwiseProduct(q_error) - kv_.cwiseProduct(qdot_filtered_);
}
        """
    else:
        cmd_line = "command = torque_desired_;"
        default_ctrl = "JointPDControl(q_des_local);"
        pd_ctrl_h = "void JointPDControl(const Eigen::Vector7d target_q);"
        pd_ctrl = """
void ___CLASS___::JointPDControl(const Eigen::Vector7d target_q)
{
  const double kAlpha = 0.99;
  qdot_filtered_ = (1 - kAlpha) * qdot_filtered_ + kAlpha * qdot_;
  Eigen::Vector7d q_error = target_q - q_;
  torque_desired_ =  kp_.cwiseProduct(q_error) - kv_.cwiseProduct(qdot_filtered_) + c_;
}
        """
    pd_ctrl = pd_ctrl.replace("___CLASS___", controller_class)

    h_content = header_tpl.format(PACKAGE_DIR, controller_class, snake, control_mode, cmd_line, default_ctrl, pd_ctrl_h, pd_ctrl)
    cpp_content = source_tpl.format(PACKAGE_DIR, controller_class, snake, control_mode, cmd_line, default_ctrl, pd_ctrl_h, pd_ctrl)

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
        print("Error: --controller_name should be a valid C++ class-like identifier starting with uppercase.")
        sys.exit(1)
    if control_mode not in ["position", "velocity", "effort"]:
        print(f"Control Mode must be [position, velocity, effort]! Input is {control_mode}")
        sys.exit(1)

    generate_from_templates(controller_class, control_mode)
    append_to_controllers_yaml(to_snake(controller_class), controller_class, control_mode)

    print(f"Generated controller: {controller_class}")

if __name__ == "__main__":
    main()
