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
      return CallbackReturn::ERROR;
    }

//    hw_states_.resize(info_.joints.size(), 0.0);
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

//        new version
      if (joint.state_interfaces.size() == 0)
      {
        RCLCPP_FATAL(
              rclcpp::get_logger(
                "KukaRSIHardwareInterface"), "expecting at least 1 state interface");
        return CallbackReturn::ERROR;
      }
      else
      {
        int   hw_pos_num = 0,
            hw_vel_num = 0,
            hw_acc_num = 0,
              hw_eff_num = 0,
              hw_cur_num= 0;

        for (std::size_t i = 0; i < joint.state_interfaces.size(); i++)
        {
          if (joint.state_interfaces[i].name == hardware_interface::HW_IF_POSITION)
          {
            hw_pos_num++;
            hw_state_interfaces_[joint.name]["position"] = 0;
          }
          if (joint.state_interfaces[i].name == hardware_interface::HW_IF_VELOCITY)
          {
            hw_vel_num++;
            hw_state_interfaces_[joint.name]["velocity"] = 0;
          }
          if (joint.state_interfaces[i].name == hardware_interface::HW_IF_ACCELERATION)
          {
            hw_acc_num++;
            hw_state_interfaces_[joint.name]["acceleration"] = 0;
          }
          if (joint.state_interfaces[i].name == hardware_interface::HW_IF_EFFORT)
          {
            hw_eff_num++;
            hw_state_interfaces_[joint.name]["effort"] = 0;
          }
          if (joint.state_interfaces[i].name == "current")
          {
            hw_cur_num++;
            hw_state_interfaces_[joint.name]["current"] = 0;
          }
        }
        if (hw_pos_num == 0)
        {
          RCLCPP_FATAL(
                rclcpp::get_logger(
                  "KukaRSIHardwareInterface"), "expecting POSITION state interface");
          return CallbackReturn::ERROR;
        }
        else if (hw_pos_num > 1)
        {
          RCLCPP_FATAL(
                rclcpp::get_logger(
                  "KukaRSIHardwareInterface"), "expecting only one POSITION state interface");
          return CallbackReturn::ERROR;
        }
        if (hw_vel_num > 1)
        {
          RCLCPP_FATAL(
                rclcpp::get_logger(
                  "KukaRSIHardwareInterface"), "expecting only one VELOCITY state interface");
          return CallbackReturn::ERROR;
        }
        if (hw_acc_num > 1)
        {
          RCLCPP_FATAL(
                rclcpp::get_logger(
                  "KukaRSIHardwareInterface"), "expecting only one ACCELERATION state interface");
          return CallbackReturn::ERROR;
        }
        if (hw_eff_num > 1)
        {
          RCLCPP_FATAL(
                rclcpp::get_logger(
                  "KukaRSIHardwareInterface"), "expecting only one EFFORT state interface");
          return CallbackReturn::ERROR;
        }
        if (hw_cur_num > 1)
        {
          RCLCPP_FATAL(
                rclcpp::get_logger(
                  "KukaRSIHardwareInterface"), "expecting only one CURRENT state interface");
          return CallbackReturn::ERROR;
        }
      }

//      end new version


//      if (joint.state_interfaces.size() != 1) {
//        RCLCPP_FATAL(
//              rclcpp::get_logger(
//                "KukaRSIHardwareInterface"), "expecting exactly 1 state interface");
//        return CallbackReturn::ERROR;
//      }

//      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
//        RCLCPP_FATAL(
//              rclcpp::get_logger(
//                "KukaRSIHardwareInterface"), "expecting only POSITION state interface");
//        RCLCPP_INFO_STREAM(rclcpp::get_logger("KukaRSIHardwareInterface"), "A005");
//        return CallbackReturn::ERROR;
//      }
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

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> KukaRSIHardwareInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("KukaRSIHardwareInterface"), "Export " << info_.joints[i].name);
      state_interfaces.emplace_back(
            info_.joints[i].name,
            "position",
            &hw_state_interfaces_[info_.joints[i].name]["position"]);
      RCLCPP_INFO_STREAM(rclcpp::get_logger("KukaRSIHardwareInterface"), "       " << "position");
      if (hw_state_interfaces_[info_.joints[i].name].find("velocity") != hw_state_interfaces_[info_.joints[i].name].end())
      {
        state_interfaces.emplace_back(
              info_.joints[i].name,
              "velocity",
              &hw_state_interfaces_[info_.joints[i].name]["velocity"]);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("KukaRSIHardwareInterface"), "       " << "velocity");
      }
      if (hw_state_interfaces_[info_.joints[i].name].find("acceleration") != hw_state_interfaces_[info_.joints[i].name].end())
      {
        state_interfaces.emplace_back(
              info_.joints[i].name,
              "acceleration",
              &hw_state_interfaces_[info_.joints[i].name]["acceleration"]);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("KukaRSIHardwareInterface"), "       " << "acceleration");
      }
      if (hw_state_interfaces_[info_.joints[i].name].find("effort") != hw_state_interfaces_[info_.joints[i].name].end())
      {
        state_interfaces.emplace_back(
              info_.joints[i].name,
              "effort",
              &hw_state_interfaces_[info_.joints[i].name]["effort"]);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("KukaRSIHardwareInterface"), "       " << "effort");
      }
      if (hw_state_interfaces_[info_.joints[i].name].find("current") != hw_state_interfaces_[info_.joints[i].name].end())
      {
        state_interfaces.emplace_back(
              info_.joints[i].name,
              "current",
              &hw_state_interfaces_[info_.joints[i].name]["current"]);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("KukaRSIHardwareInterface"), "       " << "current");
      }

//      state_interfaces.emplace_back(
//            info_.joints[i].name,
//            hardware_interface::HW_IF_POSITION,
//            &hw_states_[i]);
    }
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
      return CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(rclcpp::get_logger("KukaRSIHardwareInterface"), "Got data from robot");

    // Drop empty <rob> frame with RSI <= 2.3
    if (bytes < 100) {
      bytes = server_->recv(in_buffer_);
    }

    rsi_state_ = RSIState(in_buffer_);

    for (size_t i = 0; i < info_.joints.size(); ++i) {
//      hw_states_[i] = rsi_state_.positions[i] * KukaRSIHardwareInterface::D2R;
//      hw_commands_[i] = hw_states_[i];
      initial_joint_pos_[i] = rsi_state_.initial_positions[i] * KukaRSIHardwareInterface::D2R;

      hw_state_interfaces_[info_.joints[i].name]["position"] = rsi_state_.positions[i] * KukaRSIHardwareInterface::D2R;
      hw_commands_[i] = hw_state_interfaces_[info_.joints[i].name]["position"];

      if (hw_state_interfaces_[info_.joints[i].name].find("velocity") != hw_state_interfaces_[info_.joints[i].name].end())
      {
        hw_state_interfaces_[info_.joints[i].name]["velocity"] = rsi_state_.velocities[i] * KukaRSIHardwareInterface::D2R;
      }
      if (hw_state_interfaces_[info_.joints[i].name].find("acceleration") != hw_state_interfaces_[info_.joints[i].name].end())
      {
        hw_state_interfaces_[info_.joints[i].name]["acceleration"] = rsi_state_.accelerations[i] * KukaRSIHardwareInterface::D2R;
      }
      if (hw_state_interfaces_[info_.joints[i].name].find("effort") != hw_state_interfaces_[info_.joints[i].name].end())
      {
        hw_state_interfaces_[info_.joints[i].name]["effort"] = rsi_state_.torques[i];
      }
      if (hw_state_interfaces_[info_.joints[i].name].find("current") != hw_state_interfaces_[info_.joints[i].name].end())
      {
        hw_state_interfaces_[info_.joints[i].name]["current"] = rsi_state_.currents[i];
      }
    }

    ipoc_ = rsi_state_.ipoc;

    out_buffer_ = RSICommand(joint_pos_correction_deg_, ipoc_, stop_flag_).xml_doc;

    server_->send(out_buffer_);
    server_->set_timeout(1000);  // Set receive timeout to 1 second

    RCLCPP_INFO(rclcpp::get_logger("KukaRSIHardwareInterface"), "System Successfully started!");
    is_active_ = true;

    rsi_state_interfaces_ = hw_state_interfaces_;
//    rsi_states_ = hw_states_;
    rsi_commands_ = hw_commands_;
    rsi_communication_thread_ = std::make_shared<std::thread>([this]{rsi_communication();});

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn KukaRSIHardwareInterface::on_deactivate(
      const rclcpp_lifecycle::State &)
  {
    stop_flag_ = true;

    RCLCPP_ERROR(rclcpp::get_logger("KukaRSIHardwareInterface"), "In deactivate");

    if (rsi_communication_thread_->joinable())
      rsi_communication_thread_->join();

    RCLCPP_INFO(rclcpp::get_logger("KukaRSIHardwareInterface"), "Stop flag was set!");
    return CallbackReturn::SUCCESS;
  }

  return_type KukaRSIHardwareInterface::read(
      const rclcpp::Time &,
      const rclcpp::Duration &)
  {
    std::lock_guard<std::mutex> read_guard(mutex_);
//    if (hw_states_.size() == rsi_states_.size())
//    {
//      hw_states_ = rsi_states_;
//    }
    if (hw_state_interfaces_.size() == rsi_state_interfaces_.size())
    {
      for (std::size_t i = 0; i < info_.joints.size(); i++)
      {
        hw_state_interfaces_[info_.joints[i].name]["position"] = rsi_state_interfaces_[info_.joints[i].name]["position"];

        if (hw_state_interfaces_[info_.joints[i].name].find("velocity") != hw_state_interfaces_[info_.joints[i].name].end())
        {
          hw_state_interfaces_[info_.joints[i].name]["velocity"] = rsi_state_interfaces_[info_.joints[i].name]["velocity"];
        }
        if (hw_state_interfaces_[info_.joints[i].name].find("acceleration") != hw_state_interfaces_[info_.joints[i].name].end())
        {
          hw_state_interfaces_[info_.joints[i].name]["acceleration"] = rsi_state_interfaces_[info_.joints[i].name]["acceleration"];
        }
        if (hw_state_interfaces_[info_.joints[i].name].find("effort") != hw_state_interfaces_[info_.joints[i].name].end())
        {
          hw_state_interfaces_[info_.joints[i].name]["effort"] = rsi_state_interfaces_[info_.joints[i].name]["effort"];
        }
        if (hw_state_interfaces_[info_.joints[i].name].find("current") != hw_state_interfaces_[info_.joints[i].name].end())
        {
          hw_state_interfaces_[info_.joints[i].name]["current"] = rsi_state_interfaces_[info_.joints[i].name]["current"];
        }
      }
//      hw_state_interfaces_ = rsi_state_interfaces_;

//      for (std::size_t i = 0; i < hw_state_interfaces_.size(); i++)
//      {
//        RCLCPP_ERROR_STREAM(rclcpp::get_logger("KukaRSIHardwareInterface"), "Hw name       : " << info_.joints[i].name);
//        RCLCPP_ERROR_STREAM(rclcpp::get_logger("KukaRSIHardwareInterface"), "Hw value      : " << hw_state_interfaces_[info_.joints[i].name]["position"]);
//        RCLCPP_ERROR_STREAM(rclcpp::get_logger("KukaRSIHardwareInterface"), "Hw index      : " << &hw_state_interfaces_[info_.joints[i].name]["position"]);
//        RCLCPP_ERROR_STREAM(rclcpp::get_logger("KukaRSIHardwareInterface"), " ");
//      }
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("KukaRSIHardwareInterface"), "hw_states and rsi_states have different size!");
    }
    return communication_state_;
  }

  return_type KukaRSIHardwareInterface::write(
      const rclcpp::Time &,
      const rclcpp::Duration &)
  {
    std::lock_guard<std::mutex> write_guard(mutex_);
    if (hw_commands_.size() == rsi_commands_.size())
    {
      rsi_commands_ = hw_commands_;
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("KukaRSIHardwareInterface"), "hw_commands and rsi_commands have different size!");
    }
    return communication_state_;
  }

  void KukaRSIHardwareInterface::rsi_communication()
  {
    while (rclcpp::ok()){
      if (stop_flag_)
        break;
      //  rsi read
      in_buffer_.resize(1024);
      if (!is_active_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        communication_state_ = return_type::OK;
      }

      if (server_->recv(in_buffer_) == 0) {
        RCLCPP_ERROR(rclcpp::get_logger("KukaRSIHardwareInterface"), "No data received from robot");
        this->on_deactivate(this->get_state());
        communication_state_ =  return_type::ERROR;
        return;
      }
//      std::cout << in_buffer_ << std::endl;
      rsi_state_ = RSIState(in_buffer_);

      if (rsi_state_.delay != 0)
      {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("KukaRSIHardwareInterface"), "Delay value: " << rsi_state_.delay);
      }

      {
        std::lock_guard<std::mutex> read_guard(mutex_);
        for (std::size_t i = 0; i < info_.joints.size(); ++i) {
//          rsi_states_[i] = rsi_state_.positions[i] * KukaRSIHardwareInterface::D2R;
          rsi_state_interfaces_[info_.joints[i].name]["position"] = rsi_state_.positions[i] * KukaRSIHardwareInterface::D2R;

//          RCLCPP_ERROR_STREAM(rclcpp::get_logger("KukaRSIHardwareInterface"), "Read rsi        " << rsi_state_.positions[i]);
//          RCLCPP_ERROR_STREAM(rclcpp::get_logger("KukaRSIHardwareInterface"), "Read rsi rad    " << rsi_state_.positions[i] * KukaRSIHardwareInterface::D2R);
//          RCLCPP_ERROR_STREAM(rclcpp::get_logger("KukaRSIHardwareInterface"), "Internal value: " << rsi_state_interfaces_[info_.joints[i].name]["position"]);

          if (rsi_state_interfaces_[info_.joints[i].name].find("velocity") != rsi_state_interfaces_[info_.joints[i].name].end())
          {
            rsi_state_interfaces_[info_.joints[i].name]["velocity"] = rsi_state_.velocities[i] * KukaRSIHardwareInterface::D2R;
          }
          if (rsi_state_interfaces_[info_.joints[i].name].find("acceleration") != rsi_state_interfaces_[info_.joints[i].name].end())
          {
            rsi_state_interfaces_[info_.joints[i].name]["acceleration"] = rsi_state_.accelerations[i] * KukaRSIHardwareInterface::D2R;
          }
          if (rsi_state_interfaces_[info_.joints[i].name].find("effort") != rsi_state_interfaces_[info_.joints[i].name].end())
          {
            rsi_state_interfaces_[info_.joints[i].name]["effort"] = rsi_state_.torques[i];
          }
          if (rsi_state_interfaces_[info_.joints[i].name].find("current") != rsi_state_interfaces_[info_.joints[i].name].end())
          {
            rsi_state_interfaces_[info_.joints[i].name]["current"] = rsi_state_.currents[i];
          }
        }
      }
      ipoc_ = rsi_state_.ipoc;
      communication_state_ =  return_type::OK;

      //  rsi write
      out_buffer_.resize(1024);
      // It is possible, that write is called immediately after activation
      // In this case write in that tick should be skipped to be able to read state at first
      // First cycle (with 0 ipoc) is handled in the on_activate method, so 0 ipoc means
      //  read was not called yet
      if (!is_active_ || ipoc_ == 0) {
        communication_state_ = return_type::OK;
      }

      if (stop_flag_) {is_active_ = false;}

      {
        std::lock_guard<std::mutex> write_guard(mutex_);
        for (size_t i = 0; i < info_.joints.size(); i++) {
          joint_pos_correction_deg_[i] = (rsi_commands_[i] - initial_joint_pos_[i]) *
                                         KukaRSIHardwareInterface::R2D;
        }
      }

      out_buffer_ = RSICommand(joint_pos_correction_deg_, ipoc_, stop_flag_).xml_doc;
      server_->send(out_buffer_);
      communication_state_ = return_type::OK;
    }
  }
}  // namespace namespace kuka_rsi_hw_interface

PLUGINLIB_EXPORT_CLASS(
    kuka_rsi_hw_interface::KukaRSIHardwareInterface,
    hardware_interface::SystemInterface
    )
