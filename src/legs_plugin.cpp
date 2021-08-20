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

#include <beine_gazebo_plugins/legs_plugin.hpp>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <keisan/keisan.hpp>

#include <algorithm>
#include <map>
#include <memory>
#include <string>

namespace ksn = keisan;

namespace beine_gazebo_plugins
{

LegsPlugin::LegsPlugin()
: standing_joints_position({
    {LEFT_HIP_PITCH, 0.0},
    {LEFT_KNEE_PITCH, 0.0},
    {LEFT_ANKLE_PITCH, 0.0},
    {RIGHT_HIP_PITCH, 0.0},
    {RIGHT_KNEE_PITCH, 0.0},
    {RIGHT_ANKLE_PITCH, 0.0}
  }),
  sitting_joints_position({
    {LEFT_HIP_PITCH, 45.0},
    {LEFT_KNEE_PITCH, -90.0},
    {LEFT_ANKLE_PITCH, 45.0},
    {RIGHT_HIP_PITCH, 45.0},
    {RIGHT_KNEE_PITCH, -90.0},
    {RIGHT_ANKLE_PITCH, 45.0}
  }),
  translation_speed(2.0),
  rotation_speed(2.0),
  joint_force_strength(1000.0),
  joint_force_smoothness(0.5)
{
}

void LegsPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Initialize the node
  node = gazebo_ros::Node::Get(sdf);

  // Initialize the legs consumer
  {
    beine_cpp::LegsConsumer::Options options;
    options.legs_prefix = sdf->Get<std::string>("legs_prefix", options.legs_prefix).first;

    legs_consumer = std::make_shared<beine_cpp::LegsConsumer>(node, options);
  }

  // Initialize the model
  this->model = model;

  // Initialize the joints
  {
    // Load joints name from the SDF
    std::map<std::string, Joint> joints_name;
    {
      // Map of joint enum and SDF parameter
      std::map<Joint, std::string> sdfs_name = {
        {LEFT_HIP_PITCH, "left_hip_pitch_joint"},
        {LEFT_KNEE_PITCH, "left_knee_pitch_joint"},
        {LEFT_ANKLE_PITCH, "left_ankle_pitch_joint"},
        {RIGHT_HIP_PITCH, "right_hip_pitch_joint"},
        {RIGHT_KNEE_PITCH, "right_knee_pitch_joint"},
        {RIGHT_ANKLE_PITCH, "right_ankle_pitch_joint"}
      };

      // Add each SDF parameter's value to joints name map
      for (const auto & sdf_name_pair : sdfs_name) {
        auto joint_name = sdf->Get<std::string>(sdf_name_pair.second, sdf_name_pair.second).first;
        joints_name.emplace(joint_name, sdf_name_pair.first);
      }
    }

    // Map the simulation joints according to the joint_name
    for (const auto & joint : model->GetJoints()) {
      // Skip if joint is fixed (does not have an axis)
      if ((joint->GetType() & gazebo::physics::Joint::FIXED_JOINT) != 0) {
        continue;
      }

      // Find joint index by name
      const auto & joint_name_pair = joints_name.find(joint->GetName());
      if (joint_name_pair != joints_name.end()) {
        joints.emplace(joint_name_pair->second, joint);
      }
    }

    // Log joints result
    if (joints.size() > 0) {
      if (joints.size() < 6) {
        RCLCPP_WARN(node->get_logger(), "Some joint not found!");
      }

      std::stringstream ss;

      ss << "\nFound joints:";
      for (const auto & joint : joints) {
        ss << "\n- " << joint.second->GetName();
      }

      RCLCPP_INFO(node->get_logger(), ss.str());
    } else {
      RCLCPP_WARN(node->get_logger(), "No joints found!");
    }
  }

  // Load parameters from the SDF
  {
    translation_speed = sdf->Get<double>("translation_speed", translation_speed).first;
    rotation_speed = sdf->Get<double>("rotation_speed", rotation_speed).first;
    joint_force_strength = sdf->Get<double>("joint_force_strength", joint_force_strength).first;

    joint_force_smoothness = sdf->Get<double>(
      "joint_force_smoothness", joint_force_smoothness).first;

    RCLCPP_INFO_STREAM(
      node->get_logger(), "\nUsing the following parameters:" <<
        "\n- translation_speed\t: " << translation_speed <<
        "\n- rotation_speed\t\t: " << rotation_speed <<
        "\n- joint_force_strength\t: " << joint_force_strength <<
        "\n- joint_force_smoothness\t: " << joint_force_smoothness);
  }

  // Initialize the update connection
  {
    update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&LegsPlugin::Update, this)
    );

    RCLCPP_INFO(node->get_logger(), "Connected to the world update!");
  }
}

void LegsPlugin::Update()
{
  // Move Position and Orientation
  MovePosition(legs_consumer->get_position());
  MoveOrientation(legs_consumer->get_orientation());

  // Move joints according to current stance
  if (legs_consumer->get_stance().is_sitting()) {
    MoveJointsPosition(sitting_joints_position);
  } else {
    MoveJointsPosition(standing_joints_position);
  }
}

void LegsPlugin::MovePosition(const beine_cpp::Position & target_position)
{
  // Get current simulation position
  const auto & pos = model->WorldPose().Pos();
  auto current = ksn::Point2(pos.X(), pos.Y());

  // Get target position
  auto target = ksn::Point2(target_position.x, target_position.y);

  // Calculate velocity based on current and target position
  auto velocity = target - current;
  if (velocity.magnitude() > 1.0) {
    velocity = velocity.normalize();
  }

  velocity *= translation_speed;

  // Keep the gravity
  auto gravity = std::min(model->WorldLinearVel().Z(), 0.0);

  // Modify the simulation velocity
  model->SetLinearVel({velocity.x, velocity.y, gravity});
}

void LegsPlugin::MoveOrientation(const beine_cpp::Orientation & target_orientation)
{
  // Get current simulation yaw orientation
  auto rot = model->WorldPose().Rot();
  auto current = ksn::make_radian(rot.Yaw());

  // Get target yaw orientation
  auto target = ksn::make_degree(target_orientation.z);

  // Calculate velocity based on current and target yaw orientation
  auto velocity = current.difference_to(target).normalize();

  // Modify the simulation velocity
  model->SetAngularVel({0.0, 0.0, velocity.radian() * rotation_speed});

  // Lock pitch and roll rotations
  {
    auto pose = model->RelativePose();
    auto rot = pose.Rot();

    // Only keep the yaw rotation
    rot.Euler(0.0, 0.0, rot.Yaw());
    pose.Set(pose.Pos(), rot);

    model->SetRelativePose(pose);
  }
}

void LegsPlugin::MoveJointsPosition(const std::map<Joint, double> & target_joints_position)
{
  // For each simulation joint
  for (const auto & joint_pair : joints) {
    const auto & target_pair = target_joints_position.find(joint_pair.first);
    if (target_pair != target_joints_position.end()) {
      // Get current simulation joint position
      auto current = ksn::make_radian(joint_pair.second->Position());

      // Get Target joint position
      auto target = ksn::make_degree(target_pair->second);

      // Calculate force based on current and target joint position
      auto delta = current.difference_to(target).normalize();

      // This equation cause the graph to increase slowly the larger the x is
      // see the graph of x ^ 1/2.
      auto force = ksn::sign(delta) *
        std::pow(std::abs(delta.radian()), joint_force_smoothness) * joint_force_strength;

      // Modify the simulation joint force on axis 0
      joint_pair.second->SetForce(0, force);
    }
  }
}

GZ_REGISTER_MODEL_PLUGIN(LegsPlugin)

}  // namespace beine_gazebo_plugins
