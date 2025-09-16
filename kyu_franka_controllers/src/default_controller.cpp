#include "kyu_franka_controllers/default_controller.h"
#include <kyu_franka_controllers/robot_utils.hpp>

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

namespace kyu_franka_controllers {

CallbackReturn DefaultController::on_init() 
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
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
DefaultController::command_interface_configuration() const 
{
  controller_interface::InterfaceConfiguration command_interface_config;
  command_interface_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    command_interface_config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
    // command_interface_config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
  }
  return command_interface_config;
}

controller_interface::InterfaceConfiguration
DefaultController::state_interface_configuration() const 
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

CallbackReturn DefaultController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) 
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

CallbackReturn DefaultController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) 
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
    async_calculation_thread_ = std::thread(&DefaultController::runCalculationWorker, this);
  }
  update_joint_states();
  updateRobotData();

  q_desired_ = q_;
  torque_desired_ = c_;

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DefaultController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) 
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

controller_interface::return_type DefaultController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) 
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

  // Copy torque under mutex to avoid race with writer thread
  Eigen::Vector7d command;
  {
    std::lock_guard<std::mutex> lk(calculation_mutex_);
    command = torque_desired_;
    // command = q_desired_;
    // command = -g_+g_pin_;
  }

  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(command[i]);
  }

  static int print_count = 0;

  if ((print_count++ % 200) == 0) {
    // std::cout << "J_from_getEEJac (Pinocchio LWA):\n" << J_from_q_desired << std::endl;
    // std::cout << "J_ (ZeroJacobian):\n" << J_ << std::endl;
    std::cout << "x_:\n" << x_ << std::endl;
    std::cout << "x2_:\n" << x2_ << std::endl;
  }

  return controller_interface::return_type::OK;
}

// ========================================================================
// =========================== Thread Functions ===========================
// ========================================================================

void DefaultController::runCalculationWorker()
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

void DefaultController::asyncCalculationProc()
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

    // std::cout << "x_init_:\n" << x_init_ << std::endl;
    
    initialization_flag_ = false;
    is_mode_changed_ = false;
  }

  // Eigen::Vector7d home_q; home_q << 0, 0, 0, -M_PI/2, 0, M_PI/2, M_PI/4;
  // Eigen::Vector7d q_des_local = DyrosMath::cubicVector<7>(play_time_,
  //                                                         control_start_time_,
  //                                                         control_start_time_ + 4,
  //                                                         q_init_local,
  //                                                         home_q,
  //                                                         Eigen::Vector7d::Zero(),
  //                                                         Eigen::Vector7d::Zero());

  // JointPDControl(q_des_local);

  // // ============================================ HW2_1 ============================================
  // Eigen::Matrix4d x_target;
  // x_target.setIdentity();
  // x_target.block(0,0,3,3) << 0, -1,  0,
  //                           -1,  0,  0,
  //                            0,  0, -1;
  // x_target.block(0,3,3,1) << 0.25, 0.28, 0.65;
  // HW2_1(x_target, 10.0);

  // // ============================================ HW2_2 ============================================
  // Eigen::Matrix4d x_target;
  // x_target.setIdentity();
  // x_target.block(0,0,3,3) << 0, -1,  0,
  //                           -1,  0,  0,
  //                            0,  0, -1;
  // x_target.block(0,3,3,1) << 0.25, 0.28, 0.65;
  // HW2_2(x_target, 10.0);

  // // ============================================ HW2_3 ============================================
  // Eigen::Matrix4d x_target;
  // x_target.setIdentity();
  // x_target.block(0,0,3,3) << 0, -1,  0,
  //                           -1,  0,  0,
  //                            0,  0, -1;
  // x_target.block(0,3,3,1) << 0.25, 0.28, 0.65;
  // HW2_3(x_target, 10.0);

  // ============================================ HW3_1 ============================================
  Eigen::Vector3d x1_target, x2_target;
  x1_target << 0.35, 0.28, 0.7;
  x2_target << 0., -0.15, 0.5;
  HW3_1(x1_target, x2_target, 10.0);
}

// ================================================================================================
// ====================================== Utility Functions =======================================
// ================================================================================================	

void DefaultController::update_joint_states() {
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(i);
    const auto& velocity_interface = state_interfaces_.at(num_joints + i);
    const auto& effort_interface = state_interfaces_.at(2 * num_joints + i);
    q_[i] = position_interface.get_value();
    qdot_[i] = velocity_interface.get_value();
    torque_[i] = effort_interface.get_value();
  }
}

void DefaultController::updateRobotData()
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
    // Correctly map robot model outputs to Eigen types
    M_ = Eigen::Map<const Eigen::Matrix<double, 7, 7, Eigen::RowMajor>>(mass.data());
    c_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(coriolis.data());
    g_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(gravity.data());

    // Eigen::Matrix4d pose_mat = Eigen::Map<const Eigen::Matrix4d>(pose.data());
    // x_ = pose_mat;
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
    // J_ = Eigen::Map<const Eigen::Matrix<double, 6, 7, Eigen::RowMajor>>(endeffector_jacobian_wrt_base.data());

    // M_inv_ = M_.inverse();
    // xdot_ = J_ * qdot_;
    // M_task_ = (J_ * M_inv_ * J_.transpose()).inverse();
    // J_T_inv_ = M_task_ * J_ * M_inv_;
    // g_task_ = J_T_inv_ * g_;
    // g_pin_ = getGravity(q_);
  }
  // std::cout << "current_pose :" << x_ << std::endl;
}

Eigen::MatrixXd DefaultController::getGravity(const Eigen::VectorXd& q)
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

Eigen::MatrixXd DefaultController::getEEJac(const Eigen::VectorXd& q)
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

Eigen::Matrix4d DefaultController::getEEPose(const Eigen::VectorXd& q)
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

Eigen::MatrixXd DefaultController::getLinkJac(const Eigen::VectorXd& q, const std::string& link_name)
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

Eigen::Matrix4d DefaultController::getLinkPose(const Eigen::VectorXd& q, const std::string& link_name)
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

void DefaultController::JointPDControl(const Eigen::Vector7d target_q)
{

  // return M_ * (kp * (target_q - q_) + kv * (-qdot_)) + c_;

  const double kAlpha = 0.99;
  qdot_filtered_ = (1 - kAlpha) * qdot_filtered_ + kAlpha * qdot_;
  Eigen::Vector7d q_error = target_q - q_;
  torque_desired_ =  kp_.cwiseProduct(q_error) - kv_.cwiseProduct(qdot_filtered_) + c_;
}

// ================================================================================================
// ============================================= HW ===============================================
// ================================================================================================	

void DefaultController::HW2_1(const Eigen::Matrix4d x_target, const double duration)
{
  Eigen::Vector6d xdot_desired;
  xdot_desired.head(3) = DyrosMath::cubicDotVector<3>(play_time_,
                                                      control_start_time_,
                                                      control_start_time_+duration,
                                                      x_init_.block(0,3,3,1),
                                                      x_target.block(0,3,3,1),
                                                      Eigen::VectorXd::Zero(3),
                                                      Eigen::VectorXd::Zero(3));
  xdot_desired.tail(3).setZero();
  Eigen::MatrixXd J_from_q_desired = getEEJac(q_desired_);
  double lambda = 0.01;
  Eigen::MatrixXd J_pinv = J_from_q_desired.transpose() * (J_from_q_desired * J_from_q_desired.transpose() + lambda * lambda * Eigen::MatrixXd::Identity(6,6)).inverse();
    
  // Eigen::MatrixXd J_pinv = J_.transpose() * (J_ * J_.transpose()).inverse();

  Eigen::VectorXd qdot_desired = J_pinv * xdot_desired;
  q_desired_ += qdot_desired * dt_;
  JointPDControl(q_desired_);
}

void DefaultController::HW2_2(const Eigen::Matrix4d x_target, const double duration)
{
    Eigen::Matrix4d x_desired;
    x_desired.setIdentity();
    x_desired.block(0,3,3,1) = DyrosMath::cubicVector<3>(play_time_,
                                                         control_start_time_,
                                                         control_start_time_+duration,
                                                         x_init_.block(0,3,3,1),
                                                         x_target.block(0,3,3,1),
                                                         Eigen::VectorXd::Zero(3),
                                                         Eigen::VectorXd::Zero(3));
    x_desired.block(0,0,3,3) = x_target.block(0,0,3,3);

    Eigen::Vector6d xdot_desired;
    xdot_desired.head(3) = DyrosMath::cubicDotVector<3>(play_time_,
                                                        control_start_time_,
                                                        control_start_time_+duration,
                                                        x_init_.block(0,3,3,1),
                                                        x_target.block(0,3,3,1),
                                                        Eigen::VectorXd::Zero(3),
                                                        Eigen::VectorXd::Zero(3));
    xdot_desired.tail(3).setZero();

    Eigen::Matrix4d x_from_q_desired = getEEPose(q_desired_);

    Eigen::Vector6d x_error;
    x_error.head(3) = x_desired.block(0,3,3,1) - x_from_q_desired.block(0,3,3,1);
    x_error.tail(3) = -DyrosMath::getPhi(x_from_q_desired.block(0,0,3,3), x_desired.block(0,0,3,3));

    Eigen::Vector6d Kp_diag;
    Kp_diag << 5, 5, 5, 1, 1, 1;

    Eigen::MatrixXd J_from_q_desired = getEEJac(q_desired_);
    double lambda = 0.01;
    Eigen::MatrixXd J_pinv = J_from_q_desired.transpose() * (J_from_q_desired * J_from_q_desired.transpose() + lambda * lambda * Eigen::MatrixXd::Identity(6,6)).inverse();

    Eigen::VectorXd qdot_desired = J_pinv * (xdot_desired + Kp_diag.asDiagonal() * x_error);
    q_desired_ += qdot_desired * dt_;

    JointPDControl(q_desired_);
}

void DefaultController::HW2_3(const Eigen::Matrix4d x_target, const double duration)
{
    Eigen::Matrix4d x_desired;
    x_desired.setIdentity();
    x_desired.block(0,3,3,1) = DyrosMath::cubicVector<3>(play_time_,
                                                         control_start_time_,
                                                         control_start_time_+duration,
                                                         x_init_.block(0,3,3,1),
                                                         x_target.block(0,3,3,1),
                                                         Eigen::VectorXd::Zero(3),
                                                         Eigen::VectorXd::Zero(3));
    x_desired.block(0,0,3,3) = x_target.block(0,0,3,3);

    Eigen::Vector6d xdot_desired;
    xdot_desired.head(3) = DyrosMath::cubicDotVector<3>(play_time_,
                                                        control_start_time_,
                                                        control_start_time_+duration,
                                                        x_init_.block(0,3,3,1),
                                                        x_target.block(0,3,3,1),
                                                        Eigen::VectorXd::Zero(3),
                                                        Eigen::VectorXd::Zero(3));
    xdot_desired.tail(3).setZero();

    Eigen::Matrix4d x_from_q_desired = getEEPose(q_desired_);
    
    Eigen::Vector6d x_error;
    x_error.head(3) = x_desired.block(0,3,3,1) - x_from_q_desired.block(0,3,3,1);
    x_error.tail(3) = -DyrosMath::getPhi(x_from_q_desired.block(0,0,3,3), x_desired.block(0,0,3,3));

    Eigen::Vector6d Kp_diag;
    Kp_diag << 5, 5, 5, 1, 1, 1;

    Eigen::Vector7d Winv_diag;
    Winv_diag << 1, 1, 1, 0.001, 1, 1, 1;

    Eigen::MatrixXd J_from_q_desired = getEEJac(q_desired_);
    double lambda = 0.01;
    Eigen::MatrixXd J_pinv = J_from_q_desired.transpose() * (J_from_q_desired * J_from_q_desired.transpose() + lambda * lambda * Eigen::MatrixXd::Identity(6,6)).inverse();

    Eigen::VectorXd qdot_desired = J_pinv * (xdot_desired + Kp_diag.asDiagonal() * x_error);
    q_desired_ += qdot_desired * dt_;

    JointPDControl(q_desired_); 
}

void DefaultController::HW3_1(const Eigen::Vector3d x1_target, const Eigen::Vector3d x2_target, const double duration)
{
    Eigen::Vector6d x_desired;
    x_desired.head(3) = DyrosMath::cubicVector<3>(play_time_,
                                                  control_start_time_,
                                                  control_start_time_+duration,
                                                  x_init_.block(0,3,3,1),
                                                  x1_target,
                                                  Eigen::VectorXd::Zero(3),
                                                  Eigen::VectorXd::Zero(3));
    x_desired.tail(3) = DyrosMath::cubicVector<3>(play_time_,
                                                  control_start_time_,
                                                  control_start_time_+duration,
                                                  x2_init_.block(0,3,3,1),
                                                  x2_target,
                                                  Eigen::VectorXd::Zero(3),
                                                  Eigen::VectorXd::Zero(3));
    Eigen::Vector6d xdot_desired;
    xdot_desired.head(3) = DyrosMath::cubicDotVector<3>(play_time_,
                                                        control_start_time_,
                                                        control_start_time_+duration,
                                                        x_init_.block(0,3,3,1),
                                                        x1_target,
                                                        Eigen::VectorXd::Zero(3),
                                                        Eigen::VectorXd::Zero(3));
    xdot_desired.tail(3) = DyrosMath::cubicDotVector<3>(play_time_,
                                                        control_start_time_,
                                                        control_start_time_+duration,
                                                        x2_init_.block(0,3,3,1),
                                                        x2_target,
                                                        Eigen::VectorXd::Zero(3),
                                                        Eigen::VectorXd::Zero(3));

    Eigen::Vector6d x_from_q_desired;
    x_from_q_desired << getEEPose(q_desired_).block(0,3,3,1), getLinkPose(q_desired_, link_names_[4]).block(0,3,3,1);

    Eigen::Vector6d x_error = x_desired - x_from_q_desired;

    Eigen::Vector6d Kp_diag;
    Kp_diag << 100, 100, 100, 100, 100, 100;

    Eigen::Matrix<double,6,7> J_from_q_desired;
    J_from_q_desired.block(0,0,3,7) = getEEJac(q_desired_).block(0,0,3,7);
    J_from_q_desired.block(3,0,3,7) = getLinkJac(q_desired_, link_names_[4]).block(0,0,3,7);

    double lambda = 0.01;
    Eigen::MatrixXd J_pinv = J_from_q_desired.transpose() * (J_from_q_desired * J_from_q_desired.transpose() + lambda * lambda * Eigen::MatrixXd::Identity(6,6)).inverse();

    Eigen::VectorXd qdot_desired = J_pinv * (xdot_desired + Kp_diag.asDiagonal() * x_error);
    q_desired_ += qdot_desired * dt_;

    JointPDControl(q_desired_); 
}

}  // namespace kyu_franka_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(kyu_franka_controllers::DefaultController,
                       controller_interface::ControllerInterface)
