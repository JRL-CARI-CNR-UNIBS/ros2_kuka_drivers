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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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
 * Author: Lars Tingelstad <lars.tingelstad@ntnu.no>
*/

#ifndef KUKA_RSI_HW_INTERFACE__RSI_STATE_H_
#define KUKA_RSI_HW_INTERFACE__RSI_STATE_H_

#include <tinyxml.h>
#include <string>
#include <vector>

namespace kuka_rsi_hw_interface
{
class RSIState
{
private:
  std::string xml_doc_;

public:
  RSIState()
  {
    xml_doc_.resize(1024);
  }

  explicit RSIState(std::string xml_doc)
  : xml_doc_(xml_doc)
  {
    // Parse message from robot
    TiXmlDocument bufferdoc;
    bufferdoc.Parse(xml_doc_.c_str());
    // Get the Rob node:
    TiXmlElement * rob = bufferdoc.FirstChildElement("Rob");
    // Extract axis specific actual position
    TiXmlElement * AIPos_el = rob->FirstChildElement("AIPos");
    AIPos_el->Attribute("A1", &positions[0]);
    AIPos_el->Attribute("A2", &positions[1]);
    AIPos_el->Attribute("A3", &positions[2]);
    AIPos_el->Attribute("A4", &positions[3]);
    AIPos_el->Attribute("A5", &positions[4]);
    AIPos_el->Attribute("A6", &positions[5]);
    // Extract axis specific setpoint position
    TiXmlElement * ASPos_el = rob->FirstChildElement("ASPos");
    ASPos_el->Attribute("A1", &initial_positions[0]);
    ASPos_el->Attribute("A2", &initial_positions[1]);
    ASPos_el->Attribute("A3", &initial_positions[2]);
    ASPos_el->Attribute("A4", &initial_positions[3]);
    ASPos_el->Attribute("A5", &initial_positions[4]);
    ASPos_el->Attribute("A6", &initial_positions[5]);
    // Extract cartesian actual position
    TiXmlElement * RIst_el = rob->FirstChildElement("RIst");
    RIst_el->Attribute("X", &cart_position[0]);
    RIst_el->Attribute("Y", &cart_position[1]);
    RIst_el->Attribute("Z", &cart_position[2]);
    RIst_el->Attribute("A", &cart_position[3]);
    RIst_el->Attribute("B", &cart_position[4]);
    RIst_el->Attribute("C", &cart_position[5]);
    // Extract cartesian actual position
    TiXmlElement * RSol_el = rob->FirstChildElement("RSol");
    RSol_el->Attribute("X", &initial_cart_position[0]);
    RSol_el->Attribute("Y", &initial_cart_position[1]);
    RSol_el->Attribute("Z", &initial_cart_position[2]);
    RSol_el->Attribute("A", &initial_cart_position[3]);
    RSol_el->Attribute("B", &initial_cart_position[4]);
    RSol_el->Attribute("C", &initial_cart_position[5]);
    // Extract joint actual velocity
    TiXmlElement * Velocity_el = rob->FirstChildElement("Velocity");
    if (Velocity_el != nullptr)
    {
      Velocity_el->Attribute("A1", &velocities[0]);
      Velocity_el->Attribute("A2", &velocities[1]);
      Velocity_el->Attribute("A3", &velocities[2]);
      Velocity_el->Attribute("A4", &velocities[3]);
      Velocity_el->Attribute("A5", &velocities[4]);
      Velocity_el->Attribute("A6", &velocities[5]);
    }
    // Extract joint actual acceleration
    TiXmlElement * Acceleration_el = rob->FirstChildElement("Acceleration");
    if (Acceleration_el != nullptr)
    {
      Acceleration_el->Attribute("A1", &accelerations[0]);
      Acceleration_el->Attribute("A2", &accelerations[1]);
      Acceleration_el->Attribute("A3", &accelerations[2]);
      Acceleration_el->Attribute("A4", &accelerations[3]);
      Acceleration_el->Attribute("A5", &accelerations[4]);
      Acceleration_el->Attribute("A6", &accelerations[5]);
    }
    // Extract joint actual torque
    TiXmlElement * Torque_el = rob->FirstChildElement("Torque");
    if (Torque_el != nullptr)
    {
      Torque_el->Attribute("A1", &torques[0]);
      Torque_el->Attribute("A2", &torques[1]);
      Torque_el->Attribute("A3", &torques[2]);
      Torque_el->Attribute("A4", &torques[3]);
      Torque_el->Attribute("A5", &torques[4]);
      Torque_el->Attribute("A6", &torques[5]);
    }
    // Extract joint actual current
    TiXmlElement * Current_el = rob->FirstChildElement("Current");
    if (Current_el != nullptr)
    {
      Current_el->Attribute("A1", &currents[0]);
      Current_el->Attribute("A2", &currents[1]);
      Current_el->Attribute("A3", &currents[2]);
      Current_el->Attribute("A4", &currents[3]);
      Current_el->Attribute("A5", &currents[4]);
      Current_el->Attribute("A6", &currents[5]);
    }
    // Get the Delay
    TiXmlElement * delay_el = rob->FirstChildElement("Delay");
    delay_el->Attribute("D", &delay);
    // Get the IPOC timestamp
    TiXmlElement * ipoc_el = rob->FirstChildElement("IPOC");
    ipoc = std::stoull(ipoc_el->FirstChild()->Value());
  }

  std::vector<double> positions = std::vector<double>(6, 0.0);
  std::vector<double> velocities = std::vector<double>(6, 0.0);
  std::vector<double> accelerations = std::vector<double>(6, 0.0);
  std::vector<double> torques = std::vector<double>(6, 0.0);
  std::vector<double> currents = std::vector<double>(6, 0.0);
  std::vector<double> initial_positions = std::vector<double>(6, 0.0);
  std::vector<double> cart_position = std::vector<double>(6, 0.0);
  std::vector<double> initial_cart_position = std::vector<double>(6, 0.0);
  int delay = 0;
  uint64_t ipoc = 0;
};
}  // namespace kuka_rsi_hw_interface

#endif  // KUKA_RSI_HW_INTERFACE__RSI_STATE_H_
