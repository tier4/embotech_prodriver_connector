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
#include "ptcl/ports/ptcl_port_udp_config.h"
#include "rclcpp/rclcpp.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"

#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <component_interface_specs/planning.hpp>
#include <component_interface_utils/rclcpp.hpp>

// Autoware
#include "lanelet2_extension/projection/mgrs_projector.hpp"
#include "signal_processing/lowpass_filter_1d.hpp"

#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "autoware_auto_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_auto_planning_msgs/msg/had_map_route.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"

// Pro-DRIVER
#include "lanelet2_core/primitives/GPSPoint.h"
#include "lanelet2_projection/UTM.h"
#include "ptcl/messages/ptcl_perception_frame.h"
#include "ptcl/ports/ptcl_port_udp.h"
#include "ptcl/ptcl.h"
#include "ptcl/utils/ptcl_si_unit_conversion.h"
#include "ptcl/utils/si_ptcl_unit_conversion.h"

#include <memory>
#include <string>
#include <vector>

namespace embotech_prodriver_connector
{
using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_perception_msgs::msg::ObjectClassification;
using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_perception_msgs::msg::PredictedObjectKinematics;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_perception_msgs::msg::PredictedPath;
using autoware_auto_perception_msgs::msg::Shape;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using autoware_auto_vehicle_msgs::msg::HazardLightsCommand;
using autoware_auto_vehicle_msgs::msg::SteeringReport;
using autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand;

using planning_interface::Route;
using planning_interface::RouteState;
using SetRoutePoints = planning_interface::SetRoutePoints;
using SetRoute = planning_interface::SetRoute;
using ClearRoute = planning_interface::ClearRoute;

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
  rclcpp::Publisher<Trajectory>::SharedPtr pub_trajectory_;
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_control_;
  rclcpp::Publisher<TurnIndicatorsCommand>::SharedPtr pub_turn_signal_;
  rclcpp::Publisher<HazardLightsCommand>::SharedPtr pub_hazard_signal_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_kinematic_state_;
  rclcpp::Subscription<AccelWithCovarianceStamped>::SharedPtr sub_acceleration_;
  rclcpp::Subscription<SteeringReport>::SharedPtr sub_steering_;
  rclcpp::Subscription<PredictedObjects>::SharedPtr sub_dynamic_object_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_goal_;
  // API services
  component_interface_utils::Service<ClearRoute>::SharedPtr srv_clear_route_;
  component_interface_utils::Service<SetRoute>::SharedPtr srv_set_route_;
  component_interface_utils::Service<SetRoutePoints>::SharedPtr srv_set_route_points_;


  RouteState::Message state_;
  component_interface_utils::Publisher<RouteState>::SharedPtr pub_state_;
  component_interface_utils::Publisher<Route>::SharedPtr pub_route_;

  rclcpp::TimerBase::SharedPtr on_timer_;  //!< @brief timer for trajectory CB

  // parameters
  bool calculating_accel_;
  std::string map_frame_;
  rclcpp::Duration control_delay_time_ = rclcpp::Duration(0, 0);

  // PTCL
  PTCL_Context ptcl_context_;
  PTCL_UdpPort ptcl_udp_port_;
  PTCL_Context ptcl_context_receiver_;
  PTCL_UdpPort ptcl_udp_port_receiver_;
  PTCL_CarTrajectoryReceiverContext trajectory_receiver_context_;
  std::vector<PTCL_UdpIdAddressPair> sender_id_address_pairs_;
  std::vector<PTCL_UdpIdAddressPair> receiver_id_address_pairs_;
  std::vector<PTCL_Id> destinations_car_state_;
  std::vector<PTCL_Id> destinations_route_;
  std::vector<PTCL_Id> destinations_perception_frame_;

  // coordinates conversion
  lanelet::projection::MGRSProjector mgrs_projector_;
  lanelet::projection::UtmProjector utm_projector_{lanelet::Origin::defaultOrigin()};

  // cached data
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  LowpassFilter1d accel_filter_{0.2};
  Odometry::ConstSharedPtr current_kinematics_;
  Odometry::ConstSharedPtr prev_kinematics_;
  SteeringReport::ConstSharedPtr current_steer_;
  AccelWithCovarianceStamped::ConstSharedPtr current_acceleration_;
  PredictedObjects::ConstSharedPtr current_objects_;

  // autoware callbacks
  void on_kinematic_state(const Odometry::ConstSharedPtr msg);
  void on_steering(const SteeringReport::ConstSharedPtr msg);
  void on_acceleration(const AccelWithCovarianceStamped::ConstSharedPtr msg);
  void on_dynamic_object(const PredictedObjects::ConstSharedPtr msg);
  void on_goal(const PoseStamped::ConstSharedPtr msg);

  // service callbacks
  void on_clear_route(
    const ClearRoute::Service::Request::SharedPtr req,
    const ClearRoute::Service::Response::SharedPtr res);
  void on_set_route(
    const SetRoute::Service::Request::SharedPtr req,
    const SetRoute::Service::Response::SharedPtr res);
  void on_set_route_points(
    const SetRoutePoints::Service::Request::SharedPtr req,
    const SetRoutePoints::Service::Response::SharedPtr res);

  // setup port
  void setup_port(
    const unsigned int source_id, const uint32_t ip_local_host, const uint16_t source_port,
    const std::vector<PTCL_UdpIdAddressPair> & id_address_pairs, PTCL_Context & context,
    PTCL_UdpPort & udp_port);
  void setup_PTCL();
  void on_timer();

  // conversion: trajectory
  Trajectory to_autoware_trajectory(const PTCL_CarTrajectory & car_trajectory);
  // conversion: hazard light
  HazardLightsCommand to_autoware_hazard_light_command(const PTCL_CarTrajectory & car_trajectory);
  // conversion: turn signal
  TurnIndicatorsCommand to_autoware_turn_indicator(const PTCL_CarTrajectory & car_trajectory);

  // conversion: ego state
  PTCL_CarState to_PTCL_car_state(
    const Odometry & odometry, const SteeringReport & steering,
    const AccelWithCovarianceStamped & acceleration);
  void send_to_PTCL(const PTCL_CarState & car_state);

  // conversion: perception
  PTCL_PerceptionFrame to_PTCL_perception_object(const PredictedObjects & object);
  void send_to_PTCL(const PTCL_PerceptionFrame & perception_frame);

  // conversion: goal
  boost::optional<geometry_msgs::msg::PoseStamped> transform_pose(
    const geometry_msgs::msg::PoseStamped & input_pose, const std::string & target_frame);
  PTCL_Route to_PTCL_route(const PoseStamped & goal);
  void send_to_PTCL(const PTCL_Route & route);

  // common
  PTCL_Position convert_to_PTCL_Point(const MGRSPoint & mgrs_point);
  MGRSPoint convert_to_MGRS_Point(const PTCL_Position & ptcl_point);
  PTCL_Polytope to_PTCL_polytope(const Shape & shape, const Pose & pose);
};
}  // namespace embotech_prodriver_connector

#endif  // EMBOTECH_PRODRIVER_CONNECTOR__EMBOTECH_PRODRIVER_CONNECTOR_HPP_
