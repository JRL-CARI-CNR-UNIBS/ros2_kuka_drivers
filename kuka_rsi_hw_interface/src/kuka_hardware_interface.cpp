/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014 Norwegian University of Science and Technology
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Norwegian University of Science and
*     Technology, nor the names of its contributors may be used to
*     endorse or promote products derived from this software without
*     specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/*
 * Author: Lars Tingelstad
 * Author: Svastits Aron
 */

#include <stdexcept>
#include <string>
#include <memory>
#include <vector>

#include "kuka_rsi_hw_interface/kuka_hardware_interface.hpp"

namespace kuka_rsi_hw_interface
{
CallbackReturn KukaRSIHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("KukaRSIHardwareInterface"), "A001");
    return CallbackReturn::ERROR;
  }

  hw_states_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "KukaRSIHardwareInterface"), "expecting exactly 1 command interface");
      RCLCPP_INFO_STREAM(rclcpp::get_logger("KukaRSIHardwareInterface"), "A002");
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "KukaRSIHardwareInterface"), "expecting only POSITION command interface");
      RCLCPP_INFO_STREAM(rclcpp::get_logger("KukaRSIHardwareInterface"), "A003");
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1) {
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "KukaRSIHardwareInterface"), "expecting exactly 1 state interface");
      RCLCPP_INFO_STREAM(rclcpp::get_logger("KukaRSIHardwareInterface"), "A004");
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "KukaRSIHardwareInterface"), "expecting only POSITION state interface");
      RCLCPP_INFO_STREAM(rclcpp::get_logger("KukaRSIHardwareInterface"), "A005");
      return CallbackReturn::ERROR;
    }
  }

  // RSI
  in_buffer_.resize(1024);  // udp_server.h --> #define BUFSIZE 1024
  out_buffer_.resize(1024);

  initial_joint_pos_.resize(info_.joints.size(), 0.0);
  joint_pos_correction_deg_.resize(info_.joints.size(), 0.0);
  ipoc_ = 0;

  rsi_ip_address_ = info_.hardware_parameters["client_ip"];
  rsi_port_ = std::stoi(info_.hardware_parameters["client_port"]);

  RCLCPP_INFO(
    rclcpp::get_logger("KukaRSIHardwareInterface"),
    "IP of client machine: %s:%d", rsi_ip_address_.c_str(), rsi_port_);

  RCLCPP_INFO_STREAM(rclcpp::get_logger("KukaRSIHardwareInterface"), "A006");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> KukaRSIHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_states_[i]);
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger("KukaRSIHardwareInterface"), "A007");
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> KukaRSIHardwareInterface::
export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_commands_[i]);
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger("KukaRSIHardwareInterface"), "A008");
  return command_interfaces;
}

CallbackReturn KukaRSIHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  stop_flag_ = false;
  // Wait for connection from robot
  server_.reset(new UDPServer(rsi_ip_address_, rsi_port_));
  //server_->set_timeout(50000);  // Set receive timeout to 10 seconds for activation


  RCLCPP_INFO(rclcpp::get_logger("KukaRSIHardwareInterface"), "Connecting to robot . . .");

  int bytes = server_->recv(in_buffer_);
  if (bytes == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("KukaRSIHardwareInterface"), "Connection timeout");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("KukaRSIHardwareInterface"), "A009");
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("KukaRSIHardwareInterface"), "Got data from robot");

  // Drop empty <rob> frame with RSI <= 2.3
  if (bytes < 100) {
    bytes = server_->recv(in_buffer_);
  }

  rsi_state_ = RSIState(in_buffer_);

  RCLCPP_INFO_STREAM(rclcpp::get_logger("KukaRSIHardwareInterface"), in_buffer_);
  RCLCPP_WARN_STREAM(rclcpp::get_logger("KukaRSIHardwareInterface"),
                     "rsi_state_ position: [" <<
                      rsi_state_.positions[0] << " " <<
                      rsi_state_.positions[1] << " " <<
                      rsi_state_.positions[2] << " " <<
                      rsi_state_.positions[3] << " " <<
                      rsi_state_.positions[4] << " " <<
                      rsi_state_.positions[5] << "]");

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    hw_states_[i] = rsi_state_.positions[i] * KukaRSIHardwareInterface::D2R;
    hw_commands_[i] = hw_states_[i];
    initial_joint_pos_[i] = rsi_state_.initial_positions[i] * KukaRSIHardwareInterface::D2R;
  }
  ipoc_ = rsi_state_.ipoc;

  out_buffer_ = RSICommand(joint_pos_correction_deg_, ipoc_, stop_flag_).xml_doc;

  RCLCPP_ERROR_STREAM(rclcpp::get_logger("KukaRSIHardwareInterface"), out_buffer_);

  server_->send(out_buffer_);
  server_->set_timeout(1000);  // Set receive timeout to 1 second


  for (size_t im=0; im< 250; im++)
  {
      bytes = server_->recv(in_buffer_);
      rsi_state_ = RSIState(in_buffer_);

      RCLCPP_INFO_STREAM(rclcpp::get_logger("KukaRSIHardwareInterface"), in_buffer_);
      ipoc_ = rsi_state_.ipoc;
      out_buffer_.resize(1024);
      joint_pos_correction_deg_[5]*=-1.0;
      out_buffer_ = RSICommand(joint_pos_correction_deg_, ipoc_, stop_flag_).xml_doc;
      server_->send(out_buffer_);
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("KukaRSIHardwareInterface"), out_buffer_);

      std::this_thread::sleep_for(std::chrono::milliseconds(4));
  }

  RCLCPP_INFO(rclcpp::get_logger("KukaRSIHardwareInterface"), "System Successfully started!");
  is_active_ = true;

  RCLCPP_INFO_STREAM(rclcpp::get_logger("KukaRSIHardwareInterface"), "A010");
  return CallbackReturn::SUCCESS;
}

CallbackReturn KukaRSIHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  stop_flag_ = true;
  RCLCPP_INFO(rclcpp::get_logger("KukaRSIHardwareInterface"), "Stop flag was set!");
  return CallbackReturn::SUCCESS;
}

return_type KukaRSIHardwareInterface::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{

  in_buffer_.resize(1024);
  if (!is_active_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    RCLCPP_INFO_STREAM(rclcpp::get_logger("KukaRSIHardwareInterface"), "A011");
    return return_type::OK;
  }

  if (server_->recv(in_buffer_) == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("KukaRSIHardwareInterface"), "No data received from robot");
    this->on_deactivate(this->get_state());
    RCLCPP_INFO_STREAM(rclcpp::get_logger("KukaRSIHardwareInterface"), "A012");
    return return_type::ERROR;
  }
  rsi_state_ = RSIState(in_buffer_);

  RCLCPP_WARN_STREAM(rclcpp::get_logger("KukaRSIHardwareInterface"), in_buffer_);

  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    hw_states_[i] = rsi_state_.positions[i] * KukaRSIHardwareInterface::D2R;
  }
  ipoc_ = rsi_state_.ipoc;
  RCLCPP_INFO_STREAM(rclcpp::get_logger("KukaRSIHardwareInterface"), "A013");
  return return_type::OK;
}

return_type KukaRSIHardwareInterface::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{

  out_buffer_.resize(1024);
  // It is possible, that write is called immediately after activation
  // In this case write in that tick should be skipped to be able to read state at first
  // First cycle (with 0 ipoc) is handled in the on_activate method, so 0 ipoc means
  //  read was not called yet
  if (!is_active_ || ipoc_ == 0) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("KukaRSIHardwareInterface"), "A014");
    return return_type::OK;
  }

  if (stop_flag_) {is_active_ = false;}

  RCLCPP_WARN_STREAM(rclcpp::get_logger("KukaRSIHardwareInterface"),
                     "hw_commands_: [" <<
                      hw_commands_[0] << " " <<
                      hw_commands_[1] << " " <<
                      hw_commands_[2] << " " <<
                      hw_commands_[3] << " " <<
                      hw_commands_[4] << " " <<
                      hw_commands_[5] << "]");

  for (size_t i = 0; i < info_.joints.size(); i++) {
    joint_pos_correction_deg_[i] = (hw_commands_[i] - initial_joint_pos_[i]) *
      KukaRSIHardwareInterface::R2D;
  }

  out_buffer_ = RSICommand(joint_pos_correction_deg_, ipoc_, stop_flag_).xml_doc;
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("KukaRSIHardwareInterface"), out_buffer_);
  server_->send(out_buffer_);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("KukaRSIHardwareInterface"), "A015");
  return return_type::OK;
}
}  // namespace namespace kuka_rsi_hw_interface

PLUGINLIB_EXPORT_CLASS(
  kuka_rsi_hw_interface::KukaRSIHardwareInterface,
  hardware_interface::SystemInterface
)
