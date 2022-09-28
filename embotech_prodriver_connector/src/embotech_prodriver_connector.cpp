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

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

using namespace std::literals::chrono_literals;

#define EXIT_VALUE_OK (0)
#define EXIT_VALUE_ERR (1)

// PTCL UDP Port configuration data
const uint8_t ip_local_host_array[] = {127U, 0U, 0U, 1U};
const uint32_t ip_local_host = PTCL_UdpPort_getIpFromArray(ip_local_host_array);


const PTCL_Id navigator_id = 3U;
const uint16_t navigator_port = 4983U;
const PTCL_Id motion_planner_id = 4U;
const uint16_t motion_planner_port = 4984U;
const PTCL_Id protect_id = 5U;
const uint16_t protect_port = 4985U;
const PTCL_Id developer_ui_id = 9U;
const uint16_t developer_ui_port = 4989U;
const PTCL_Id autoware_id = 10U;
const uint16_t autoware_port = 4990U;
const uint32_t num_ip_address_pairs = 6U;
const PTCL_UdpIdAddressPair id_address_pairs[] = {
  {navigator_id, ip_local_host, navigator_port},
  {motion_planner_id, ip_local_host, motion_planner_port},
  {protect_id, ip_local_host, protect_port},
  {developer_ui_id, ip_local_host, developer_ui_port},
  {autoware_id, ip_local_host, autoware_port}};

const uint8_t num_destinations_state = 4U;
const PTCL_Id destinations_car_state[] = {
  navigator_id, motion_planner_id, protect_id, developer_ui_id};
const uint8_t num_destinations_route = 1U;
const PTCL_Id destinations_route[] = {navigator_id};
const uint8_t num_destinations_perception_frame = 3U;
const PTCL_Id destinations_perception_frame[] = {motion_planner_id, protect_id, developer_ui_id};

const int32_t ptcl_timeout = 1000;  // Timeout of blocking UDP receiver call in milliseconds

namespace embotech_prodriver_connector
{

EmbotechProDriverConnector::EmbotechProDriverConnector(const rclcpp::NodeOptions & options)
: Node("embotech_prodriver_connector", options)
{
  using rclcpp::QoS;
  using std::placeholders::_1;

  sub_kinematic_state_ = create_subscription<Odometry>(
    "/localization/kinematic_state", QoS{1},
    std::bind(&EmbotechProDriverConnector::on_kinematic_state, this, _1));

  sub_steering_ = create_subscription<SteeringReport>(
    "/vehicle/status/steering_status", QoS{1},
    std::bind(&EmbotechProDriverConnector::on_steering, this, _1));

  sub_acceleration_ = create_subscription<AccelWithCovarianceStamped>(
    "/localization/acceleration", QoS{1},
    std::bind(&EmbotechProDriverConnector::on_acceleration, this, _1));

  sub_dynamic_object_ = create_subscription<PredictedObjects>(
    "/perception/object_recognition/objects", QoS{1},
    std::bind(&EmbotechProDriverConnector::on_dynamic_object, this, _1));

  sub_goal_ = create_subscription<PoseStamped>(
    "/planning/mission_planning/goal", QoS{1},
    std::bind(&EmbotechProDriverConnector::on_goal, this, _1));

  // origin of lat/lon coordinates in PTCL are map
  //  odaiba
  //  prodriver_origin_latlon_.lat = 35.61458614188;
  //  prodriver_origin_latlon_.lon = 139.76947350053;

  // virtual_map
  origin_prodriver_latlon_.lat = 35.68386482855;
  origin_prodriver_latlon_.lon = 139.68506426425;

  // ryuyo_ci1
  // prodriver_origin_latlon_.lat = 34.66441053284202;
  // prodriver_origin_latlon_.lon = 137.83339405223919;

  // ryuyo_ci2
  // prodriver_origin_latlon_.lat = 34.66444508923468;
  // prodriver_origin_latlon_.lon = 137.83333262993906;

  // calculate GPSPoint
  mgrs_projector_.setMGRSCode(mgrs_code_);
  origin_prodriver_utm_ = convert_LatLon_to_UTM_coordinate(origin_prodriver_latlon_);

  setup_PTCL();
}

void EmbotechProDriverConnector::setup_PTCL()
{
  // Initialize PTCL logging
  //  - Log threshold of console determined by environment variable
  PTCL_setLogPrefix("EX");
  PTCL_setLogThreshold(PTCL_getLogThresholdFromEnv());

  setup_port(
    num_ip_address_pairs, autoware_id, ip_local_host, autoware_port, ptcl_context_, ptcl_udp_port_);
}

void EmbotechProDriverConnector::on_kinematic_state(const Odometry::ConstSharedPtr msg)
{
  current_kinematics_ = msg;

  if (!current_kinematics_ || !current_steer_ || !current_acceleration_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000 /*ms*/, "waiting data...");
    return;
  }

  const auto PTCL_cat_state = calc_PTCL_car_state();
  send_to_PTCL(PTCL_cat_state);
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
  current_objects_ = msg;

  const auto ptcl_object = to_PTCL_perception_object(*current_objects_);
  send_to_PTCL(ptcl_object);
}

void EmbotechProDriverConnector::on_goal(const PoseStamped::ConstSharedPtr msg)
{
  current_goal_ = msg;

  const auto PTCL_route = to_PTCL_route(*current_goal_);
  send_to_PTCL(PTCL_route);
}

PTCL_CarState EmbotechProDriverConnector::calc_PTCL_car_state()
{
  const auto mgrs_pos = current_kinematics_->pose.pose.position;
  const auto ptcl_pos = convert_to_PTCL_Point({mgrs_pos.x, mgrs_pos.y, mgrs_pos.z});

  // TODO: How to calculate yaw? Should we use it directly, or calculate as below?
  // const PTCL_Coordinate position_y_PTCL = std::atan2(position_y_PTCL, position_x_PTCL);

  PTCL_CarState cat_state;
  const Embo_Time current_time = Embo_Time_getCurrentTime();
  cat_state.header.measurementTime = current_time;
  cat_state.header.timeReference = current_time;
  cat_state.header.vehicleId = 1;
  cat_state.pose.position.x = ptcl_pos.x;
  cat_state.pose.position.y = ptcl_pos.y;
  const double yaw_pose = tf2::getYaw(current_kinematics_->pose.pose.orientation);
  cat_state.pose.heading = PTCL_toPTCLAngleWrapped(yaw_pose);
  cat_state.angleRateYaw = PTCL_toPTCLAngleRate(current_kinematics_->twist.twist.angular.z);
  cat_state.velLon = PTCL_toPTCLSpeed(current_kinematics_->twist.twist.linear.x);
  cat_state.velLat =
    PTCL_toPTCLSpeed(0.0);  // TODO think later. Autoware does not support lateral velocity.
  cat_state.accelLon = PTCL_toPTCLAccel(current_acceleration_->accel.accel.linear.x);
  cat_state.gear = PTCL_GEAR_SELECTOR_D;
  cat_state.controllerMode.latControlActive = true;
  cat_state.controllerMode.lonControlActive = true;
  cat_state.angleSteeredWheels = PTCL_toPTCLAngleWrapped(current_steer_->steering_tire_angle);
  cat_state.numSemiTrailers = static_cast<uint8_t>(0);

  return cat_state;
}

PTCL_PerceptionFrame EmbotechProDriverConnector::to_PTCL_perception_object(
  const PredictedObjects & object)
{
  PTCL_PerceptionFrame ptcl_frame;

  const Embo_Time current_time = Embo_Time_getCurrentTime();
  ptcl_frame.header.measurementTime = current_time;
  ptcl_frame.header.oemId = 0;       // TODO: think later
  ptcl_frame.header.mapId = 0;       // TODO: think later
  ptcl_frame.header.mapCrc = 0;      // TODO: think later
  ptcl_frame.header.vehicleId = 1U;  // TODO: think later

  if (object.objects.size() >= PTCL_PERCEPTION_FRAME_NUM_OBJECTS_MAX) {
    throw std::runtime_error(
      "perception object num is greater than maximum threshold.");  // TODO!!!
  }
  ptcl_frame.numObjects = object.objects.size();

  // for each predicted object
  for (size_t i = 0; i < object.objects.size(); ++i) {
    const auto & obj = object.objects.at(i);
    auto & ptcl_object = ptcl_frame.objects[i];
    ptcl_object.id = i;  // TODO: object ID used in the Autoware has 128-bit uuid.
    const auto classification = getHighestProbClass(obj.classification);
    ptcl_object.type = to_PTCL_object_type(classification);
    ptcl_object.confidenceType = static_cast<uint8_t>(255 * (1.0f - classification.probability));
    ptcl_object.properties = 0;  // immobile(1), yielding(2), or ego-vehicle(4).
    ptcl_object.numInstances = obj.kinematics.predicted_paths.size();
    if (obj.kinematics.predicted_paths.size() >= PTCL_PERCEPTION_OBJECT_NUM_INSTANCES_MAX) {
      throw std::runtime_error("predicted_paths size is greater than maximum threshold.");  // TODO
    }

    const auto predicted_path = get_highest_prob_path(obj.kinematics);

    // for each predicted path point
    int16_t time_offset = 0;
    const auto dt_ms =
      static_cast<int16_t>(rclcpp::Duration(predicted_path.time_step).nanoseconds() * 10e-6);
    for (size_t j = 0; j < predicted_path.path.size(); ++j) {
      ptcl_object.instances[j].timeOffset = time_offset;
      ptcl_object.instances[j].shape = to_PTCL_polytope(obj.shape, predicted_path.path.at(i));
      time_offset += dt_ms;  // int16_t [ms] (-32s ~ 32s)
    }

    ptcl_object.hasState = false;
  }

  // TODO: what should be done for FieldOfRegard? not used now...?
  ptcl_frame.numFieldOfRegardElementsPositive = 0;
  ptcl_frame.numFieldOfRegardElementsNegative = 0;

  return ptcl_frame;
}

PTCL_Route EmbotechProDriverConnector::to_PTCL_route(const PoseStamped & goal)
{
  PTCL_Route ptcl_route;
  const auto mgrs_pos = goal.pose.position;
  const auto ptcl_pos = convert_to_PTCL_Point({mgrs_pos.x, mgrs_pos.y, mgrs_pos.z});

  ptcl_route.vehicleId = 1U;
  ptcl_route.numGoalStates = 1U;
  ptcl_route.goalStates[1U].id += 1U;  // when new goal is shown up, unique id + 1;
  ptcl_route.goalStates[1U].pose.position.x = ptcl_pos.x;
  ptcl_route.goalStates[1U].pose.position.y = ptcl_pos.y;
  // desired steered wheels angle amd velocity at the goal state is basically 0
  ptcl_route.goalStates[1U].angleSteeredWheels = 0;
  ptcl_route.goalStates[1U].velLon = 0;
  ptcl_route.goalStates[1U].duration = 0;  // rest time after reaching goal

  return ptcl_route;
}

void EmbotechProDriverConnector::send_to_PTCL(const PTCL_CarState & car_state)
{
  // Send PTCL car_state.
  for (uint8_t dstIdx = 0U; dstIdx < num_destinations_state; ++dstIdx) {
    bool sent_state =
      PTCL_CarState_send(&ptcl_context_, &car_state, destinations_car_state[dstIdx]);
    if (!sent_state) {
      RCLCPP_ERROR(
        get_logger(), "Failed to send car_state message to PTCL ID %u.",
        destinations_car_state[dstIdx]);
    } else {
      RCLCPP_INFO(
        get_logger(), "car_state message sent to PTCL ID %u.", destinations_car_state[dstIdx]);
    }
  }
}

void EmbotechProDriverConnector::send_to_PTCL(const PTCL_PerceptionFrame & perception_frame)
{
  // Send PTCL perception_frame.
  for (uint8_t dstIdx = 0U; dstIdx < num_destinations_perception_frame; ++dstIdx) {
    bool sent_perception_frame = PTCL_PerceptionFrame_send(
      &ptcl_context_, &perception_frame, destinations_perception_frame[dstIdx]);
    if (!sent_perception_frame) {
      RCLCPP_ERROR(
        get_logger(), "Failed to send perception_frame message to PTCL ID %u.",
        destinations_perception_frame[dstIdx]);
    } else {
      RCLCPP_INFO(
        get_logger(), "perception_frame message sent to PTCL ID %u.",
        destinations_perception_frame[dstIdx]);
    }
  }
}

void EmbotechProDriverConnector::send_to_PTCL(const PTCL_Route & route)
{
  // Send PTCL car_state.
  for (uint8_t dstIdx = 0U; dstIdx < num_destinations_route; ++dstIdx) {
    bool sent_route = PTCL_Route_send(&ptcl_context_, &route, destinations_route[dstIdx]);
    if (!sent_route) {
      RCLCPP_ERROR(
        get_logger(), "Failed to send route message to PTCL ID %u.", destinations_route[dstIdx]);
    } else {
      RCLCPP_INFO(get_logger(), "route message sent to PTCL ID %u.", destinations_route[dstIdx]);
    }
  }
}

PTCL_Position EmbotechProDriverConnector::convert_to_PTCL_Point(const MGRSPoint & mgrs_point)
{
  // TODO(K.Sugahara): return zone number
  const auto gps_point = mgrs_projector_.reverse(mgrs_point);

  const auto lat = round(gps_point.lat * 1e8) / 1e8;
  const auto lon = round(gps_point.lon * 1e8) / 1e8;

  const auto utm_point = convert_LatLon_to_UTM_coordinate({lat, lon});

  // converted to local coordinate (int32)
  PTCL_Position ptcl_pos;
  ptcl_pos.x = PTCL_toPTCLCoordinate(utm_point.x() - origin_prodriver_utm_.x());
  ptcl_pos.y = PTCL_toPTCLCoordinate(utm_point.y() - origin_prodriver_utm_.y());

  return ptcl_pos;
}

PTCL_Polytope EmbotechProDriverConnector::to_PTCL_polytope(const Shape & shape, const Pose & pose)
{
  Polygon2d polygon;
  calcObjectPolygon(shape, pose, &polygon);

  PTCL_Polytope ptcl_polygon;
  ptcl_polygon.numVertices = polygon.outer().size();
  for (size_t i = 0; i < polygon.outer().size(); ++i) {
    const auto & p = polygon.outer().at(i);
    const auto p_ptcl = convert_to_PTCL_Point({p.x(), p.y(), 0.0});
    ptcl_polygon.vertices[i].x = p_ptcl.x;
    ptcl_polygon.vertices[i].y = p_ptcl.y;
  }
  return ptcl_polygon;
}

void EmbotechProDriverConnector::setup_port(
  const unsigned int & id_address_map_size, const unsigned int & source_id,
  const uint32_t ip_local_host, const uint16_t & source_port, PTCL_Context & context,
  PTCL_UdpPort & udp_port)
{
  // Setup PTCL context
  PTCL_PortInterface * port_interface = PTCL_UdpPort_init(
    ip_local_host, source_port, id_address_pairs, id_address_map_size, ptcl_timeout, source_id,
    &context, &udp_port);

  bool setup_success = (port_interface != NULL);
  if (setup_success) {
    RCLCPP_INFO(this->get_logger(), "initialized sender UDP port with source_id %u", source_id);
  } else {
    PTCL_UdpPort_destroy(&udp_port);
    RCLCPP_ERROR(this->get_logger(), "Init of context failed.\n");
    // printf("Init of context failed.\n");
  }
}

}  // namespace embotech_prodriver_connector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(embotech_prodriver_connector::EmbotechProDriverConnector)
