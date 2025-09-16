// Copyright (c) 2023 Franka Robotics GmbH
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

namespace kyu_franka_controllers {

class DefaultController : public controller_interface::ControllerInterface {
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
  void JointPDControl(const Eigen::Vector7d target_q);

  // ================================================================================================
  // ============================================= HW ===============================================
  // ================================================================================================	
  void HW2_1(const Eigen::Matrix4d x_target, const double duration);
  void HW2_2(const Eigen::Matrix4d x_target, const double duration);
  void HW2_3(const Eigen::Matrix4d x_target, const double duration);
  void HW3_1(const Eigen::Vector3d x1_target, const Eigen::Vector3d x2_target, const double duration);
};

}  // namespace kyu_franka_controllers
