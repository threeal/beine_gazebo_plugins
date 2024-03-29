// Copyright (c) 2021 Alfi Maulana
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef BEINE_GAZEBO_PLUGINS__LEGS_PLUGIN_HPP_
#define BEINE_GAZEBO_PLUGINS__LEGS_PLUGIN_HPP_

#include <beine_cpp/beine_cpp.hpp>
#include <gazebo/common/Plugin.hh>
#include <rclcpp/rclcpp.hpp>

#include <map>
#include <memory>
#include <string>

namespace beine_gazebo_plugins
{

class LegsPlugin : public gazebo::ModelPlugin
{
public:
  enum Joint
  {
    LEFT_HIP_PITCH,
    LEFT_KNEE_PITCH,
    LEFT_ANKLE_PITCH,
    RIGHT_HIP_PITCH,
    RIGHT_KNEE_PITCH,
    RIGHT_ANKLE_PITCH
  };

  LegsPlugin();

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);

private:
  void Update();

  void MovePosition(const beine_cpp::Position & target_position);
  void MoveOrientation(const beine_cpp::Orientation & target_orientation);

  void MoveJointsPosition(const std::map<Joint, double> & target_joints_position);

  rclcpp::Node::SharedPtr node;

  std::shared_ptr<beine_cpp::LegsConsumer> legs_consumer;

  gazebo::physics::ModelPtr model;

  std::map<Joint, gazebo::physics::JointPtr> joints;

  std::map<Joint, double> standing_joints_position;
  std::map<Joint, double> sitting_joints_position;

  double translation_speed;
  double rotation_speed;

  double joint_force_strength;
  double joint_force_smoothness;

  gazebo::event::ConnectionPtr update_connection;
};

}  // namespace beine_gazebo_plugins

#endif  // BEINE_GAZEBO_PLUGINS__LEGS_PLUGIN_HPP_
