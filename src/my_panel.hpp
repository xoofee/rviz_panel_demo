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

#ifndef MY_PANEL_HPP_
#define MY_PANEL_HPP_

#include <QGroupBox>
#include <QLabel>
#include <QLayout>
#include <QPushButton>
#include <QSpinBox>
#include <QTableWidget>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <memory>

namespace rviz_plugins
{
class MyPanel : public rviz_common::Panel
{
    Q_OBJECT

public:
    explicit MyPanel(QWidget *parent = nullptr);
    void onInitialize() override;

public Q_SLOTS: // NOLINT for Qt
    void onClick();

protected:
    void onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose);
    rclcpp::Node::SharedPtr raw_node_;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initialpose_;
};

} // namespace rviz_plugins

#endif // MY_PANEL_HPP_
