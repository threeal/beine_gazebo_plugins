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

namespace beine_gazebo_plugins
{

LegsPlugin::LegsPlugin()
: joints_name({
    {"left_upper_leg_joint", LEFT_HIP_PITCH},
    {"left_lower_leg_joint", LEFT_KNEE_PITCH},
    {"left_foot_joint", LEFT_ANKLE_PITCH},
    {"right_upper_leg_joint", RIGHT_HIP_PITCH},
    {"right_lower_leg_joint", RIGHT_KNEE_PITCH},
    {"right_foot_joint", RIGHT_ANKLE_PITCH}
  }),
  standing_joints_position({
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
  })
{
}

void LegsPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Initialize the node
  node = gazebo_ros::Node::Get(sdf);

  // Initialize the legs consumer
  legs_consumer = std::make_shared<beine_cpp::LegsConsumer>(node);

  // Initialize the model
  this->model = model;

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
      RCLCPP_INFO_STREAM(node->get_logger(), "Found joint " << joint->GetName() << "!");
    }
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
  auto current = keisan::Point2(pos.X(), pos.Y());

  // Get target position
  auto target = keisan::Point2(target_position.x, target_position.y);

  // Calculate velocity based on current and target position
  auto velocity = target - current;
  if (velocity.magnitude() > 1.0) {
    velocity = velocity.normalize();
  }

  velocity *= 2.0;

  // Keep the gravity
  auto gravity = std::min(model->WorldLinearVel().Z(), 0.0);

  // Modify the simulation velocity
  model->SetLinearVel({velocity.x, velocity.y, gravity});
}

void LegsPlugin::MoveOrientation(const beine_cpp::Orientation & target_orientation)
{
  // Get current simulation yaw orientation
  auto rot = model->WorldPose().Rot();
  auto current = rot.Yaw();

  // Get target yaw orientation
  auto target = keisan::deg_to_rad(target_orientation.z);

  // Calculate velocity based on current and target yaw orientation
  auto velocity = keisan::delta_rad(current, target) * 2.0;

  // Modify the simulation velocity
  model->SetAngularVel({0.0, 0.0, velocity});

  // Lock pitch and roll rotations
  {
    auto pose = model->RelativePose();
    auto rot = pose.Rot();

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
      auto current = joint_pair.second->Position();

      // Get Target joint position
      auto target = keisan::deg_to_rad(target_pair->second);

      // Calculate force based on current and target joint position
      auto delta = keisan::delta_rad(current, target);

      // This equation cause the graph to increase slowly the larger the x is
      // see the graph of x ^ 1/2.
      auto force = std::pow(std::abs(delta), 0.5) * 1000.0;
      if (delta < 0) {
        force = -force;
      }

      // Modify the simulation joint force on axis 0
      joint_pair.second->SetForce(0, force);
    }
  }
}

GZ_REGISTER_MODEL_PLUGIN(LegsPlugin)

}  // namespace beine_gazebo_plugins
