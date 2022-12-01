// Copyright 2022 The Autoware Foundation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
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

#include "embotech_prodriver_connector/embotech_prodriver_connector.hpp"

#include "embotech_prodriver_connector/embotech_prodriver_connector_utils.hpp"
#include "lanelet2_io/Projection.h"
#include "ptcl/ports/ptcl_port_udp_config.h"
#include "ptcl/subtypes/ptcl_indicators.h"
#include "ptcl/subtypes/ptcl_perception_object.h"
#include "ptcl/subtypes/ptcl_scalar_types.h"
#include "ptcl/utils/ptcl_car_trajectory_utils.h"
#include "ptcl/utils/ptcl_si_unit_conversion.h"
#include "ptcl/utils/ptcl_time_utils.h"

#include <lanelet2_projection/Mercator.h>
#include <lanelet2_projection/UTM.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

// PTCL msg
typedef struct CarTrajectoryData
{
  PTCL_CarTrajectory car_trajectory;
  volatile bool msg_received;
} CarTrajectoryData;
CarTrajectoryData car_trajectory_data_;

// Callback function. Called when a new message is received on a port it was
// installed on. Copies message to userData and sets flag for main process.
static void car_trajectory_CB(
  const PTCL_CarTrajectory * msg, const PTCL_MsgInfoHandle msgInfoHandle, void * userData)
{
  // car trajectory is sent by PRODRIVER with 10Hz
  (void)msgInfoHandle;
  CarTrajectoryData * car_trajectory_data_ = reinterpret_cast<CarTrajectoryData *>(userData);
  // Copy received message msg to location provided by the user in *userData
  memcpy(&(car_trajectory_data_->car_trajectory), msg, sizeof(PTCL_CarTrajectory));
  // Set flag
  car_trajectory_data_->msg_received = true;
}

namespace embotech_prodriver_connector
{

EmbotechProDriverConnector::EmbotechProDriverConnector(const rclcpp::NodeOptions & options)
: Node("embotech_prodriver_connector", options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
{
  map_frame_ = declare_parameter("map_frame", "map");

  using rclcpp::QoS;
  using std::placeholders::_1;

  setup_PTCL();

  pub_trajectory_ = create_publisher<Trajectory>("~/output/trajectory", 1);
  pub_control_ = create_publisher<AckermannControlCommand>("~/output/control", 1);
  pub_route_ = create_publisher<HADMapRoute>("~/output/route", QoS{1}.transient_local());
  pub_turn_signal_ = create_publisher<TurnIndicatorsCommand>("~/output/turn_indicators_cmd", 1);
  pub_hazard_signal_ = create_publisher<HazardLightsCommand>("~/output/hazard_lights_cmd", 1);

  sub_kinematic_state_ = create_subscription<Odometry>(
    "~/input/kinematic_state", 1,
    std::bind(&EmbotechProDriverConnector::on_kinematic_state, this, _1));
  sub_steering_ = create_subscription<SteeringReport>(
    "~/input/steering_status", 1, std::bind(&EmbotechProDriverConnector::on_steering, this, _1));
  sub_acceleration_ = create_subscription<AccelWithCovarianceStamped>(
    "~/input/acceleration", 1, std::bind(&EmbotechProDriverConnector::on_acceleration, this, _1));
  sub_dynamic_object_ = create_subscription<PredictedObjects>(
    "~/input/objects", 1, std::bind(&EmbotechProDriverConnector::on_dynamic_object, this, _1));
  sub_goal_ = create_subscription<PoseStamped>(
    "~/input/goal", 1, std::bind(&EmbotechProDriverConnector::on_goal, this, _1));

  constexpr auto timer_sampling_time_ms = static_cast<uint32_t>(25);
  on_timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::milliseconds(timer_sampling_time_ms),
    std::bind(&EmbotechProDriverConnector::on_timer, this));

  calculating_accel_ = declare_parameter<bool>("calculate_acceleration_from_twist");
  const auto control_delay_time_ms = declare_parameter<int>("control_time_delay");
  control_delay_time_ = std::chrono::milliseconds(control_delay_time_ms);

  // origin of lat/lon coordinates in PTCL are map
  lanelet::GPSPoint origin_prodriver_latlon;
  const auto mgrs_code = declare_parameter<std::string>("map_origin.mgrs_code");
  origin_prodriver_latlon.lat = declare_parameter<double>("map_origin.latitude");
  origin_prodriver_latlon.lon = declare_parameter<double>("map_origin.longitude");
  const auto offset_x = declare_parameter<double>("map_origin.offset_x");
  const auto offset_y = declare_parameter<double>("map_origin.offset_y");
  mgrs_projector_.setMGRSCode(mgrs_code);
  utm_projector_ = lanelet::projection::UtmProjector(lanelet::Origin(origin_prodriver_latlon));
  auto intermediate_xy = utm_projector_.forward(origin_prodriver_latlon);
  intermediate_xy.x() += offset_x;
  intermediate_xy.y() += offset_y;
  const auto corrected_latlon = utm_projector_.reverse(intermediate_xy);
  utm_projector_ = lanelet::projection::UtmProjector(lanelet::Origin(corrected_latlon));
}

void EmbotechProDriverConnector::on_timer()
{
  if (!car_trajectory_data_.msg_received) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000 /*ms*/, "embotech trajectory not received.");
    return;
  }
  const auto current_trajectory = to_autoware_trajectory(car_trajectory_data_.car_trajectory);
  if (current_trajectory.points.empty()) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000 /*ms*/, "embotech trajectory is empty.");
    return;
  }

  const auto turn_indicator_msg = to_autoware_turn_indicator(car_trajectory_data_.car_trajectory);
  const auto hazard_light_msg =
    to_autoware_hazard_light_command(car_trajectory_data_.car_trajectory);

  pub_trajectory_->publish(current_trajectory);
  pub_turn_signal_->publish(turn_indicator_msg);
  pub_hazard_signal_->publish(hazard_light_msg);

  PTCL_CarTrajectoryElement current_element;
  const auto current_time = get_clock()->now();
  const auto delayed_time = current_time + control_delay_time_;
  const auto delayed_time_ptcl = fromRosTime(delayed_time);
  if (PTCL_CarTrajectory_getInterpolatedElement(
        &(car_trajectory_data_.car_trajectory), delayed_time_ptcl, &current_element)) {
    AckermannControlCommand cmd;
    cmd.longitudinal.stamp = current_time;
    cmd.lateral.stamp = current_time;

    cmd.longitudinal.speed = PTCL_toSpeed(current_element.velLon);
    cmd.longitudinal.acceleration = PTCL_toAccel(current_element.accelLon);

    cmd.lateral.steering_tire_angle = PTCL_toAngleWrapped(current_element.angleSteeredWheels);
    cmd.lateral.steering_tire_rotation_rate = PTCL_toAngleRate(current_element.angleRateYaw);
    pub_control_->publish(cmd);
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to find control command in the PRODRIVER trajectory");
  }
}

void EmbotechProDriverConnector::setup_PTCL()
{
  const auto ip_prodriver_raw = declare_parameter<std::vector<int64_t>>("network.prodriver_ip");
  const std::vector<uint8_t> ip_prodriver_vec(ip_prodriver_raw.begin(), ip_prodriver_raw.end());
  const uint32_t ip_prodriver = PTCL_UdpPort_getIpFromArray(ip_prodriver_vec.data());

  const auto ip_localhost_raw = declare_parameter<std::vector<int64_t>>("network.localhost_ip");
  const std::vector<uint8_t> ip_localhost_vec(ip_localhost_raw.begin(), ip_localhost_raw.end());
  const uint32_t ip_localhost = PTCL_UdpPort_getIpFromArray(ip_localhost_vec.data());

  const PTCL_Id navigator_id = declare_parameter<int>("network.navigator_id");
  const uint16_t navigator_port = declare_parameter<int>("network.navigator_port");
  const PTCL_Id motion_planner_id = declare_parameter<int>("network.motion_planner_id");
  const uint16_t motion_planner_port = declare_parameter<int>("network.motion_planner_port");
  const PTCL_Id protect_id = declare_parameter<int>("network.protect_id");
  const uint16_t protect_port = declare_parameter<int>("network.protect_port");
  const PTCL_Id developer_ui_id = declare_parameter<int>("network.developer_ui_id");
  const uint16_t developer_ui_port = declare_parameter<int>("network.developer_ui_port");
  const PTCL_Id autoware_id = declare_parameter<int>("network.autoware_id");
  const uint16_t autoware_port = declare_parameter<int>("network.autoware_port");
  const PTCL_Id trajectory_receiver_id = declare_parameter<int>("network.trajectory_receiver_id");
  const uint16_t trajectory_receiver_port =
    declare_parameter<int>("network.trajectory_receiver_port");

  sender_id_address_pairs_ = {
    {navigator_id, ip_prodriver, navigator_port},
    {motion_planner_id, ip_prodriver, motion_planner_port},
    {protect_id, ip_prodriver, protect_port},
    {developer_ui_id, ip_prodriver, developer_ui_port},
    {autoware_id, ip_prodriver, autoware_port}};

  receiver_id_address_pairs_ = {{trajectory_receiver_id, ip_localhost, trajectory_receiver_port}};

  destinations_car_state_ = {navigator_id, motion_planner_id, protect_id, developer_ui_id};
  destinations_route_ = {navigator_id};
  destinations_perception_frame_ = {motion_planner_id, protect_id, developer_ui_id};

  PTCL_setLogPrefix("EX");
  PTCL_setLogThreshold(PTCL_getLogThresholdFromEnv());
  setup_port(
    autoware_id, ip_prodriver, autoware_port, sender_id_address_pairs_, ptcl_context_,
    ptcl_udp_port_);
  setup_port(
    trajectory_receiver_id, ip_localhost, trajectory_receiver_port, receiver_id_address_pairs_,
    ptcl_context_receiver_, ptcl_udp_port_receiver_);
  const bool installSuccess = PTCL_CarTrajectory_installCallback(
    &ptcl_context_receiver_, car_trajectory_CB, &car_trajectory_data_, protect_id,
    &trajectory_receiver_context_);

  if (installSuccess) {
    PTCL_UdpPort_startReceiving(&ptcl_udp_port_receiver_);
    printf(
      "Initialized callback on receiver port for message from"
      "sender port with PTCL_Id %u.\n",
      protect_id);
  } else {
    printf("Start receiving message from protect failed.\n");
  }
}

void EmbotechProDriverConnector::on_kinematic_state(const Odometry::ConstSharedPtr msg)
{
  if (calculating_accel_) {
    prev_kinematics_ = current_kinematics_;
    current_kinematics_ = msg;
    if (prev_kinematics_ && current_kinematics_) {
      const auto dv =
        current_kinematics_->twist.twist.linear.x - prev_kinematics_->twist.twist.linear.x;
      const auto dt = std::max(
        (rclcpp::Time(current_kinematics_->header.stamp) -
         rclcpp::Time(prev_kinematics_->header.stamp))
          .seconds(),
        1e-03);
      const auto accel = dv / dt;
      const auto current_acc = accel_filter_.filter(accel);
      auto * accel_msg = new AccelWithCovarianceStamped();
      accel_msg->accel.accel.linear.x = current_acc;
      current_acceleration_.reset(accel_msg);
    }
  }

  const auto missing_accel = !calculating_accel_ && !current_acceleration_;
  if (!current_steer_)
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000 /*ms*/, "waiting for steering message");
  if (!current_acceleration_)
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000 /*ms*/, "waiting for acceleration message");
  if (!current_steer_ || !current_acceleration_) return;

  const auto PTCL_car_state = to_PTCL_car_state(*msg, *current_steer_, *current_acceleration_);
  send_to_PTCL(PTCL_car_state);
}

void EmbotechProDriverConnector::on_steering(const SteeringReport::ConstSharedPtr msg)
{
  current_steer_ = msg;
}

void EmbotechProDriverConnector::on_acceleration(
  const AccelWithCovarianceStamped::ConstSharedPtr msg)
{
  current_acceleration_ = msg;
}

void EmbotechProDriverConnector::on_dynamic_object(const PredictedObjects::ConstSharedPtr msg)
{
  const auto ptcl_object = to_PTCL_perception_object(*msg);
  send_to_PTCL(ptcl_object);
}

void EmbotechProDriverConnector::on_goal(const PoseStamped::ConstSharedPtr msg)
{
  // transform goal pose
  const auto opt_transformed_goal_pose = transform_pose(*msg, map_frame_);
  if (!opt_transformed_goal_pose) {
    RCLCPP_ERROR(
      get_logger(), "Failed to get goal pose in map frame. Aborting frame transformation");
    return;
  }
  const auto transformed_goal_pose = opt_transformed_goal_pose.get();

  const auto PTCL_route = to_PTCL_route(transformed_goal_pose);
  send_to_PTCL(PTCL_route);

  // publish an empty route with the received goal to Autoware
  // needed by the 'ad_service_state_monitor'
  autoware_auto_planning_msgs::msg::HADMapRoute route;
  route.header = msg->header;
  route.goal_pose = msg->pose;
  pub_route_->publish(route);
}

PTCL_CarState EmbotechProDriverConnector::to_PTCL_car_state(
  const Odometry & odometry, const SteeringReport & steering,
  const AccelWithCovarianceStamped & acceleration)
{
  const auto mgrs_pos = odometry.pose.pose.position;
  const auto ptcl_pos = convert_to_PTCL_Point({mgrs_pos.x, mgrs_pos.y, mgrs_pos.z});

  const auto current_time_ptcl = fromRosTime(get_clock()->now());
  PTCL_CarState car_state;
  car_state.header.measurementTime = current_time_ptcl;
  car_state.header.timeReference = current_time_ptcl;
  car_state.header.vehicleId = 1;
  car_state.pose.position.x = ptcl_pos.x;
  car_state.pose.position.y = ptcl_pos.y;
  const double yaw_pose = tf2::getYaw(odometry.pose.pose.orientation);
  car_state.pose.heading = PTCL_toPTCLAngleWrapped(yaw_pose);
  car_state.angleRateYaw = PTCL_toPTCLAngleRate(odometry.twist.twist.angular.z);
  car_state.velLon = PTCL_toPTCLSpeed(odometry.twist.twist.linear.x);
  car_state.velLat = PTCL_toPTCLSpeed(0.0);  // TODO(Sugahara) think later. Autoware does not
                                             // support lateral velocity.
  car_state.accelLon = PTCL_toPTCLAccel(acceleration.accel.accel.linear.x);
  car_state.gear = PTCL_GEAR_SELECTOR_D;
  car_state.controllerMode.latControlActive = true;
  car_state.controllerMode.lonControlActive = true;
  car_state.angleSteeredWheels = PTCL_toPTCLAngleWrapped(steering.steering_tire_angle);
  car_state.numSemiTrailers = static_cast<uint8_t>(0);

  return car_state;
}

PTCL_PerceptionFrame EmbotechProDriverConnector::to_PTCL_perception_object(
  const PredictedObjects & object)
{
  PTCL_PerceptionFrame ptcl_frame;
  ptcl_frame.header.measurementTime = fromRosTime(get_clock()->now());
  ptcl_frame.header.oemId = 0;       // TODO(Sugahara): think later
  ptcl_frame.header.mapId = 0;       // TODO(Sugahara): think later
  ptcl_frame.header.mapCrc = 0;      // TODO(Sugahara): think later
  ptcl_frame.header.vehicleId = 1U;  // TODO(Sugahara): think later

  if (object.objects.size() > PTCL_PERCEPTION_FRAME_NUM_OBJECTS_MAX) {
    RCLCPP_WARN(
      get_logger(), "Too many detected objects (%lu). Only using %d.", object.objects.size(),
      PTCL_PERCEPTION_FRAME_NUM_OBJECTS_MAX);
  }
  ptcl_frame.numObjects =
    std::min(object.objects.size(), static_cast<size_t>(PTCL_PERCEPTION_FRAME_NUM_OBJECTS_MAX));
  // for each predicted object
  for (size_t i = 0; i < ptcl_frame.numObjects; ++i) {
    const auto & obj = object.objects.at(i);
    auto & ptcl_object = ptcl_frame.objects[i];
    ptcl_object.id = i;  // TODO(Sugahara): object ID used in the Autoware has 128-bit uuid.
    const auto classification = getHighestProbClass(obj.classification);
    ptcl_object.type = to_PTCL_object_type(classification);
    ptcl_object.confidenceType = static_cast<uint8_t>(255 * (1.0f - classification.probability));
    ptcl_object.properties = 0;  // immobile(1), yielding(2), or ego-vehicle(4).
    const auto predicted_path = get_highest_prob_path(obj.kinematics);
    if (predicted_path.path.size() > PTCL_PERCEPTION_OBJECT_NUM_INSTANCES_MAX) {
      RCLCPP_WARN(
        get_logger(), "Object %lu: too many predicted points. Reducing from %lu to %d.", i,
        predicted_path.path.size(), PTCL_PERCEPTION_OBJECT_NUM_INSTANCES_MAX);
    }
    ptcl_object.numInstances = std::min(
      static_cast<int8_t>(predicted_path.path.size()),
      static_cast<int8_t>(PTCL_PERCEPTION_OBJECT_NUM_INSTANCES_MAX));
    // for each predicted path point
    int16_t time_offset = 0;
    const auto dt_ms =
      static_cast<int16_t>(rclcpp::Duration(predicted_path.time_step).nanoseconds() / 1e6);
    for (size_t j = 0; j < predicted_path.path.size(); ++j) {
      ptcl_object.instances[j].timeOffset = time_offset;
      ptcl_object.instances[j].shape = to_PTCL_polytope(obj.shape, predicted_path.path.at(j));
      time_offset += dt_ms;  // int16_t [ms] (-32s ~ 32s)
    }

    ptcl_object.hasState = false;
  }

  // TODO(Sugahara): what should be done for FieldOfRegard? not used now...?
  ptcl_frame.numFieldOfRegardElementsPositive = 0;
  ptcl_frame.numFieldOfRegardElementsNegative = 0;

  return ptcl_frame;
}

Trajectory EmbotechProDriverConnector::to_autoware_trajectory(
  const PTCL_CarTrajectory & ptcl_car_trajectory)
{
  Trajectory trajectory;
  trajectory.header.stamp = toRosTime(ptcl_car_trajectory.header.timeReference);
  trajectory.header.frame_id = "map";

  constexpr size_t start_idx = 1;  // to ignore front point since it has 0 velocity

  for (size_t i = start_idx; i < unsigned(ptcl_car_trajectory.numElements); ++i) {
    const auto & car_trajectory_element = ptcl_car_trajectory.elements[i];
    // const auto & element_position =
    // ptcl_car_trajectory.elements[i].pose.position;
    TrajectoryPoint trajectory_point;
    const auto element_pos = convert_to_MGRS_Point(car_trajectory_element.pose.position);
    trajectory_point.time_from_start = rclcpp::Duration::from_nanoseconds(
      PTCL_toTimeOffset(car_trajectory_element.timeOffset) * 1e6);
    trajectory_point.pose.position.x = element_pos.x();
    trajectory_point.pose.position.y = element_pos.y();
    trajectory_point.pose.position.z = element_pos.z();
    trajectory_point.pose.orientation = tier4_autoware_utils::createQuaternionFromRPY(
      0.0, 0.0, PTCL_toAngleWrapped(car_trajectory_element.pose.heading));
    trajectory_point.longitudinal_velocity_mps = PTCL_toSpeed(car_trajectory_element.velLon);
    trajectory_point.lateral_velocity_mps = PTCL_toSpeed(car_trajectory_element.velLat);
    trajectory_point.acceleration_mps2 = PTCL_toAccel(car_trajectory_element.accelLon);
    trajectory_point.front_wheel_angle_rad =
      PTCL_toPTCLAngleWrapped(car_trajectory_element.angleSteeredWheels);
    trajectory_point.rear_wheel_angle_rad = 0;  // think later

    // to avoid Autoware error.
    if (!trajectory.points.empty()) {
      const auto & p = trajectory_point.pose.position;
      const auto & p_pre = trajectory.points.back().pose.position;
      if (p.x == p_pre.x && p.y == p_pre.y) {
        continue;
      }
    }
    trajectory.points.push_back(trajectory_point);
  }

  return trajectory;
}

HazardLightsCommand EmbotechProDriverConnector::to_autoware_hazard_light_command(
  const PTCL_CarTrajectory & ptcl_car_trajectory)
{
  HazardLightsCommand cmd;
  cmd.stamp = toRosTime(ptcl_car_trajectory.header.timeReference);
  cmd.command = ptcl_car_trajectory.indicators == PTCL_INDICATORS_HAZARD
                  ? HazardLightsCommand::ENABLE
                  : HazardLightsCommand::NO_COMMAND;
  return cmd;
}

TurnIndicatorsCommand EmbotechProDriverConnector::to_autoware_turn_indicator(
  const PTCL_CarTrajectory & ptcl_car_trajectory)
{
  TurnIndicatorsCommand cmd;
  cmd.stamp = toRosTime(ptcl_car_trajectory.header.timeReference);
  switch (ptcl_car_trajectory.indicators) {
    case PTCL_INDICATORS_LEFT:
      cmd.command = TurnIndicatorsCommand::ENABLE_LEFT;
      break;
    case PTCL_INDICATORS_RIGHT:
      cmd.command = TurnIndicatorsCommand::ENABLE_RIGHT;
      break;
    default:
      cmd.command = TurnIndicatorsCommand::NO_COMMAND;
  }
  return cmd;
}

PTCL_Route EmbotechProDriverConnector::to_PTCL_route(const PoseStamped & goal)
{
  PTCL_Route ptcl_route;
  const auto mgrs_pos = goal.pose.position;
  const auto ptcl_pos = convert_to_PTCL_Point({mgrs_pos.x, mgrs_pos.y, mgrs_pos.z});
  const double yaw_pose = tf2::getYaw(goal.pose.orientation);

  ptcl_route.vehicleId = 1U;
  ptcl_route.numElements = 1U;
  ptcl_route.numGoalStates = 1U;
  ptcl_route.elements[0].mapId = 0;
  ptcl_route.elements[0].type = PTCL_ROUTE_ELEMENT_TYPE_GOAL_STATE;
  ptcl_route.elements[0].typeId = 1U;
  ptcl_route.elements[0].enableNominalSpeed = false;
  // ptcl_route_elements[0].properties; // should I define?
  ptcl_route.goalStates[0].id = 1U;
  ptcl_route.goalStates[0].pose.position.x = ptcl_pos.x;
  ptcl_route.goalStates[0].pose.position.y = ptcl_pos.y;
  ptcl_route.goalStates[0].pose.heading = PTCL_toPTCLAngleWrapped(yaw_pose);
  // desired steered wheels angle amd velocity at the goal state is basically 0
  ptcl_route.goalStates[0].angleSteeredWheels = 0;
  ptcl_route.goalStates[0].velLon = 0;
  ptcl_route.goalStates[0].duration = 0;  // rest time after reaching goal

  return ptcl_route;
}

void EmbotechProDriverConnector::send_to_PTCL(const PTCL_CarState & car_state)
{
  for (const auto & destination : destinations_car_state_) {
    const bool sent_state = PTCL_CarState_send(&ptcl_context_, &car_state, destination);
    if (!sent_state)
      RCLCPP_ERROR(get_logger(), "Failed to send car_state message to PTCL ID %u.", destination);
  }
}

void EmbotechProDriverConnector::send_to_PTCL(const PTCL_PerceptionFrame & perception_frame)
{
  for (const auto & destination : destinations_perception_frame_) {
    const bool sent_perception_frame =
      PTCL_PerceptionFrame_send(&ptcl_context_, &perception_frame, destination);
    if (!sent_perception_frame)
      RCLCPP_ERROR(
        get_logger(), "Failed to send perception_frame message to PTCL ID %u.", destination);
  }
}

void EmbotechProDriverConnector::send_to_PTCL(const PTCL_Route & route)
{
  for (const auto & destination : destinations_route_) {
    const bool sent_route = PTCL_Route_send(&ptcl_context_, &route, destination);
    if (!sent_route)
      RCLCPP_ERROR(get_logger(), "Failed to send route message to PTCL ID %u.", destination);
  }
}

PTCL_Position EmbotechProDriverConnector::convert_to_PTCL_Point(const MGRSPoint & mgrs_point)
{
  const auto gps_point = mgrs_projector_.reverse(mgrs_point);

  const auto lat = round(gps_point.lat * 1e8) / 1e8;
  const auto lon = round(gps_point.lon * 1e8) / 1e8;

  const auto utm_point = utm_projector_.forward({lat, lon});

  // converted to local coordinate (int32)
  PTCL_Position ptcl_pos;
  ptcl_pos.x = PTCL_toPTCLCoordinate(utm_point.x());
  ptcl_pos.y = PTCL_toPTCLCoordinate(utm_point.y());
  return ptcl_pos;
}

MGRSPoint EmbotechProDriverConnector::convert_to_MGRS_Point(const PTCL_Position & ptcl_pos)
{
  // convert to global coordinate (double)
  UTMPoint utm_point;
  utm_point.x() = PTCL_toCoordinate(ptcl_pos.x);
  utm_point.y() = PTCL_toCoordinate(ptcl_pos.y);
  utm_point.z() = 0;

  const auto gps_point = utm_projector_.reverse({utm_point.x(), utm_point.y(), utm_point.z()});

  const MGRSPoint mgrs_point = mgrs_projector_.forward(gps_point);
  return mgrs_point;
}

PTCL_Polytope EmbotechProDriverConnector::to_PTCL_polytope(const Shape & shape, const Pose & pose)
{
  Polygon2d polygon;
  calcObjectPolygon(shape, pose, &polygon);

  PTCL_Polytope ptcl_polygon;
  const auto size = std::min(
    polygon.outer().size(),
    static_cast<size_t>(PTCL_PERCEPTION_FRAME_NUM_FIELD_OF_REGARD_ELEMENTS_MAX));
  ptcl_polygon.numVertices = size;
  for (size_t i = 0; i + 1 < size; ++i) {
    const auto & p = polygon.outer()[i];
    const auto p_ptcl = convert_to_PTCL_Point({p.x(), p.y(), 0.0});
    ptcl_polygon.vertices[i].x = p_ptcl.x;
    ptcl_polygon.vertices[i].y = p_ptcl.y;
  }
  ptcl_polygon.vertices[size - 1] = ptcl_polygon.vertices[0];
  return ptcl_polygon;
}

void EmbotechProDriverConnector::setup_port(
  const unsigned int source_id, const uint32_t ip, const uint16_t source_port,
  const std::vector<PTCL_UdpIdAddressPair> & id_address_pairs, PTCL_Context & context,
  PTCL_UdpPort & udp_port)
{
  constexpr int32_t ptcl_timeout_ms = 1000;
  const PTCL_PortInterface * port_interface = PTCL_UdpPort_init(
    ip, source_port, id_address_pairs.data(), id_address_pairs.size(), ptcl_timeout_ms, source_id,
    &context, &udp_port);

  bool setup_success = (port_interface != NULL);
  if (setup_success) {
    RCLCPP_INFO(this->get_logger(), "initialized UDP port with source_id %u", source_id);
  } else {
    PTCL_UdpPort_destroy(&udp_port);
    RCLCPP_ERROR(this->get_logger(), "Init of context failed.\n");
  }
}

boost::optional<geometry_msgs::msg::PoseStamped> EmbotechProDriverConnector::transform_pose(
  const geometry_msgs::msg::PoseStamped & input_pose, const std::string & target_frame)
{
  try {
    geometry_msgs::msg::PoseStamped output_pose;
    const auto transform =
      tf_buffer_.lookupTransform(target_frame, input_pose.header.frame_id, tf2::TimePointZero);
    tf2::doTransform(input_pose, output_pose, transform);
    return output_pose;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "%s", ex.what());
  }

  return {};
}

}  // namespace embotech_prodriver_connector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(embotech_prodriver_connector::EmbotechProDriverConnector)
