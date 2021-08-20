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

#include <beine_gazebo_plugins/status_plugin.hpp>

namespace beine_gazebo_plugins
{

StatusPlugin::StatusPlugin()
{
  this->setStyleSheet(
    "QFrame {"
    "  background-color : rgba(255, 255, 255, 255);"
    "  color : black;"
    "  font-size: 24px;"
    "}");

  auto main_layout = new QHBoxLayout;
  auto main_frame = new QFrame();
  main_layout->setContentsMargins(0, 0, 0, 0);

  auto frame_layout = new QVBoxLayout();
  frame_layout->setContentsMargins(4, 4, 4, 4);

  auto test_label = new QLabel(tr("Test"));
  frame_layout->addWidget(test_label);
  frame_layout->setAlignment(test_label, Qt::AlignCenter);

  main_frame->setLayout(frame_layout);
  main_layout->addWidget(main_frame);

  this->setLayout(main_layout);
}

StatusPlugin::~StatusPlugin()
{
}

void StatusPlugin::Load(sdf::ElementPtr /*sdf*/)
{
}

GZ_REGISTER_GUI_PLUGIN(StatusPlugin)

}  // namespace beine_gazebo_plugins
