// Copyright 2022 The Autoware Foundation.
//
// Licensed under the Apache License, Version 2.0 (the "LicenseW");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef EMBOTECH_PRODRIVER_CONNECTOR__EMBOTECH_PRODRIVER_CONNECTOR_HPP_
#define EMBOTECH_PRODRIVER_CONNECTOR__EMBOTECH_PRODRIVER_CONNECTOR_HPP_

// ROS
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

// Autoware
#include "lanelet2_extension/projection/mgrs_projector.hpp"

#include "autoware_auto_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"

// Pro-DRIVER
#include "embo_time.h"
#include "lanelet2_core/primitives/GPSPoint.h"
#include "ptcl/messages/ptcl_perception_frame.h"
#include "ptcl/ports/ptcl_port_udp.h"
#include "ptcl/ptcl.h"
#include "ptcl/utils/si_ptcl_unit_conversion.h"

#include <memory>
#include <string>
#include <vector>

namespace embotech_prodriver_connector
{

using autoware_auto_perception_msgs::msg::ObjectClassification;
using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_perception_msgs::msg::PredictedObjectKinematics;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_perception_msgs::msg::PredictedPath;
using autoware_auto_perception_msgs::msg::Shape;
using autoware_auto_vehicle_msgs::msg::SteeringReport;

using geometry_msgs::msg::AccelWithCovarianceStamped;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Odometry;

// Autoware coordinates
using MGRSPoint = lanelet::BasicPoint3d;

// intermediate coordinate
using UTMPoint = lanelet::BasicPoint3d;

class EmbotechProDriverConnector : public rclcpp::Node
{
public:
  explicit EmbotechProDriverConnector(const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<Odometry>::SharedPtr sub_kinematic_state_;
  rclcpp::Subscription<AccelWithCovarianceStamped>::SharedPtr sub_acceleration_;
  rclcpp::Subscription<SteeringReport>::SharedPtr sub_steering_;
  rclcpp::Subscription<PredictedObjects>::SharedPtr sub_dynamic_object_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_goal_;

  // port setup
  PTCL_Context ptcl_context_;
  PTCL_UdpPort ptcl_udp_port_;

  // coordinates conversion
  lanelet::projection::MGRSProjector mgrs_projector_;
  lanelet::GPSPoint origin_prodriver_latlon_;
  UTMPoint origin_prodriver_utm_;

  std::string mgrs_code_ = "54SUE";  // mgrs_code for odaiba and virtual_map
  // std::string mgrs_code_ = "53SQU" // mgrs_code for ryuyo_ci1,2

  Odometry::ConstSharedPtr current_kinematics_;
  SteeringReport::ConstSharedPtr current_steer_;
  AccelWithCovarianceStamped::ConstSharedPtr current_acceleration_;
  PredictedObjects::ConstSharedPtr current_objects_;
  PoseStamped::ConstSharedPtr current_goal_;

  // autoware callbacks
  void on_kinematic_state(const Odometry::ConstSharedPtr msg);
  void on_steering(const SteeringReport::ConstSharedPtr msg);
  void on_acceleration(const AccelWithCovarianceStamped::ConstSharedPtr msg);
  void on_dynamic_object(const PredictedObjects::ConstSharedPtr msg);
  void on_goal(const PoseStamped::ConstSharedPtr msg);

  // setup port
  void setup_port(
    const unsigned int & id_address_map_size, const unsigned int & source_id,
    const uint32_t ip_local_host, const uint16_t & source_port, PTCL_Context & context,
    PTCL_UdpPort & udp_port);
  void setup_PTCL();

  // conversion: ego
  PTCL_CarState calc_PTCL_car_state();
  void send_to_PTCL(const PTCL_CarState & car_state);

  // conversion: perception
  PTCL_PerceptionFrame to_PTCL_perception_object(const PredictedObjects & object);
  void send_to_PTCL(const PTCL_PerceptionFrame & perception_frame);

  // conversion: goal
  PTCL_Route to_PTCL_route(const PoseStamped & goal);
  void send_to_PTCL(const PTCL_Route & route);

  // common
  PTCL_Position convert_to_PTCL_Point(const MGRSPoint & mgrs_point);
  PTCL_Polytope to_PTCL_polytope(const Shape & shape, const Pose & pose);
};
}  // namespace embotech_prodriver_connector

#endif  // EMBOTECH_PRODRIVER_CONNECTOR__EMBOTECH_PRODRIVER_CONNECTOR_HPP_
