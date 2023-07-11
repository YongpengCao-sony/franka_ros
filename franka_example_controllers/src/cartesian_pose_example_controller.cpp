// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/cartesian_pose_example_controller.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <controller_interface/controller_base.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_example_controllers {

bool CartesianPoseExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianPoseExampleController: Could not get Cartesian Pose "
        "interface from hardware");
    return false;
  }

  std::string arm_id;
  sub_ = node_handle.subscribe("/move_group/fake_controller_joint_states", 10, &CartesianPoseExampleController::JointPoseCb, this);

  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianPoseExampleController: Could not get parameter arm_id");
    return false;
  }

  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianPoseExampleController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianPoseExampleController: Could not get state interface from hardware");
    return false;
  }

  try {
    auto state_handle = state_interface->getHandle(arm_id + "_robot");

    std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
            "CartesianPoseExampleController: Robot is not in the expected starting position for "
            "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
            "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianPoseExampleController: Exception getting state handle: " << e.what());
    return false;
  }

  return true;
}

void CartesianPoseExampleController::starting(const ros::Time& /* time */) {
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  elapsed_time_ = ros::Duration(0.0);

  ROS_INFO("Start EE pose x:%f", initial_pose_[12]);
  ROS_INFO("Start EE pose y:%f", initial_pose_[13]);
  ROS_INFO("Start EE pose z:%f", initial_pose_[14]);
}

void CartesianPoseExampleController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  elapsed_time_ += period;

  if (elapsed_time_ <= ros::Duration(4.0)) {
    double disired_dist_x = 0.0;
    double disired_dist_y = 0.0;
    double disired_dist_z = 0.1;

    double angle = (1 - std::cos(M_PI / 2.0 * elapsed_time_.toSec())) / 2;  // M_PI / 4 *
    // double delta_x = radius * std::sin(angle);
    double delta_x = disired_dist_x * angle;
    double delta_y = disired_dist_y * angle;
    double delta_z = disired_dist_z * angle;

    // double delta_z = radius * (std::cos(angle) - 1);
    std::array<double, 16> new_pose = initial_pose_;

    new_pose[12] -= delta_x;
    new_pose[13] -= delta_y;
    new_pose[14] -= delta_z;
    cartesian_pose_handle_->setCommand(new_pose);
  } 
  else if (elapsed_time_ <= ros::Duration(8.0)){
    double disired_dist_x = 0.0;
    double disired_dist_y = 0.1;
    double disired_dist_z = 0.0;

    double angle = (1 - std::cos(M_PI / 2.0 * elapsed_time_.toSec())) / 2;  // M_PI / 4 *
    // double delta_x = radius * std::sin(angle);
    double delta_x = disired_dist_x * angle;
    double delta_y = disired_dist_y * angle;
    double delta_z = disired_dist_z * angle;

    // double delta_z = radius * (std::cos(angle) - 1);
    std::array<double, 16> new_pose = initial_pose_;

    new_pose[12] -= delta_x;
    new_pose[13] -= delta_y;
    new_pose[14] -= delta_z;
    cartesian_pose_handle_->setCommand(new_pose);    
  }
  else {
    ROS_INFO("Current EE pose x:%f", (cartesian_pose_handle_->getRobotState().O_T_EE_d)[12]);
    ROS_INFO("Current EE pose y:%f", (cartesian_pose_handle_->getRobotState().O_T_EE_d)[13]);
    ROS_INFO("Current EE pose z:%f", (cartesian_pose_handle_->getRobotState().O_T_EE_d)[14]);
    stopping(ros::Time::now());
    }
}

void CartesianPoseExampleController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

void CartesianPoseExampleController::JointPoseCb(const sensor_msgs::JointState& msg)
{
  // if (!msg->header.frame_id.empty() && msg->header.frame_id != this->root_frame_)
  // {
  //   ROS_WARN_STREAM("Reference poses need to be in the root frame '" << this->root_frame_ << "'. Ignoring.");
  //   return;
  // }
  std::vector<double> positions= msg.position;
  ROS_INFO("Current fake position is:%f", (positions[1]));
  // const Eigen::Quaterniond last_orientation_d_target(this->orientation_d_);
  // Eigen::Quaterniond orientation_d;
  // orientation_d.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
  //     msg->pose.orientation.w;
  // if (last_orientation_d_target.coeffs().dot(this->orientation_d_.coeffs()) < 0.0)
  // {
  //   this->orientation_d_.coeffs() << -this->orientation_d_.coeffs();
  // }
  // this->setReferencePose(position_d, orientation_d);
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianPoseExampleController,
                       controller_interface::ControllerBase)
