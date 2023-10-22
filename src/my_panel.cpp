//
//  Copyright 2020 TIER IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

#include "my_panel.hpp"

#include <QGridLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QString>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>

#include <memory>
#include <string>

namespace rviz_plugins
{

MyPanel::MyPanel(QWidget *parent) : rviz_common::Panel(parent)
{
    auto *button = new QPushButton("button");
    auto *label = new QLabel("a label");
    auto *v_layout = new QVBoxLayout;
    v_layout->addWidget(button);
    v_layout->addWidget(label);

    connect(button, SIGNAL(clicked()), this, SLOT(onClick()));

    setLayout(v_layout);
}

void MyPanel::onInitialize()
{
    using std::placeholders::_1;

    raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

    sub_initialpose_ = raw_node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1,
                                                                                                        std::bind(&MyPanel::onInitialPose, this, _1)); 
}

void MyPanel::onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose)
{
    RCLCPP_INFO(raw_node_->get_logger(), "got initial pose, frame id: ");
    RCLCPP_INFO(raw_node_->get_logger(), pose->header.frame_id.c_str());
}

void MyPanel::onClick()
{
    RCLCPP_INFO(raw_node_->get_logger(), "clicked");
}

} // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::MyPanel, rviz_common::Panel)
