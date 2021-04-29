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

#include <memory>

namespace beine_gazebo_plugins
{

LegsPlugin::LegsPlugin()
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
  // Update position
  {
    auto pos = model->WorldPose().Pos();
    auto current = keisan::Point3(pos.X(), pos.Y(), pos.Z());

    auto position = legs_consumer->get_position();
    auto target = keisan::Point3(position.x, position.y, position.z);

    auto velocity = target - current;
    if (velocity.magnitude() > 1.0) {
      velocity = velocity.normalize();
    }

    velocity *= 5.0;

    model->SetLinearVel({velocity.x, velocity.y, velocity.z});
  }

  // Update orientation
  {
    auto rot = model->WorldPose().Rot();
    auto current = rot.Yaw();

    auto orientation = legs_consumer->get_orientation();
    auto target = keisan::deg_to_rad(orientation.z);

    auto velocity = keisan::delta_rad(current, target) * 5.0;

    model->SetAngularVel({0.0, 0.0, velocity});
  }
}

GZ_REGISTER_MODEL_PLUGIN(LegsPlugin)

}  // namespace beine_gazebo_plugins
