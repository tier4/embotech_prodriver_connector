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
#include "autoware_auto_vehicle_msgs/msg/detail/hazard_lights_command__struct.hpp"
#include "autoware_auto_vehicle_msgs/msg/detail/turn_indicators_command__struct.hpp"
#include "embotech_prodriver_connector/embotech_prodriver_connector_utils.hpp"
#include "lanelet2_io/Projection.h"
#include "ptcl/ports/ptcl_port_udp_config.h"
#include "ptcl/subtypes/ptcl_indicators.h"

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
typedef struct CarTrajectoryData {
  PTCL_CarTrajectory car_trajectory;
  volatile bool msg_received;
} CarTrajectoryData;
CarTrajectoryData car_trajectory_data_;

// Callback function. Called when a new message is received on a port it was
// installed on. Copies message to userData and sets flag for main process.
static void car_trajectory_CB(const PTCL_CarTrajectory *msg,
                              const PTCL_MsgInfoHandle msgInfoHandle,
                              void *userData) {
  // car trajectory is sent by PRODRIVER with 10Hz
  (void)msgInfoHandle;
  CarTrajectoryData *car_trajectory_data_ = (CarTrajectoryData *)userData;
  // Copy received message msg to location provided by the user in *userData
  memcpy(&(car_trajectory_data_->car_trajectory), msg,
         sizeof(PTCL_CarTrajectory));
  // Set flag
  car_trajectory_data_->msg_received = true;
}

namespace embotech_prodriver_connector {

EmbotechProDriverConnector::EmbotechProDriverConnector(
    const rclcpp::NodeOptions &options)
    : Node("embotech_prodriver_connector", options) {
  using rclcpp::QoS;
  using std::placeholders::_1;

  setup_PTCL();

  pub_trajectory_ = create_publisher<Trajectory>("~/output/trajectory", 1);
  pub_route_ = create_publisher<HADMapRoute>("~/output/route", QoS{1}.transient_local());
  pub_turn_signal_ = create_publisher<TurnIndicatorsCommand>("~/output/turn_indicators_cmd", 1);
  pub_hazard_signal_ = create_publisher<HazardLightsCommand>("~/output/hazard_lights_cmd", 1);

  sub_kinematic_state_ = create_subscription<Odometry>(
      "~/input/kinematic_state", 1,
      std::bind(&EmbotechProDriverConnector::on_kinematic_state, this, _1));
  sub_steering_ = create_subscription<SteeringReport>(
      "~/input/steering_status", 1,
      std::bind(&EmbotechProDriverConnector::on_steering, this, _1));
  sub_acceleration_ = create_subscription<AccelWithCovarianceStamped>(
      "~/input/acceleration", 1,
      std::bind(&EmbotechProDriverConnector::on_acceleration, this, _1));
  sub_dynamic_object_ = create_subscription<PredictedObjects>(
      "~/input/objects", 1,
      std::bind(&EmbotechProDriverConnector::on_dynamic_object, this, _1));
  sub_goal_ = create_subscription<PoseStamped>(
      "~/input/goal", 1,
      std::bind(&EmbotechProDriverConnector::on_goal, this, _1));

  constexpr auto timer_sampling_time_ms = static_cast<uint32_t>(25);
  on_timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::milliseconds(timer_sampling_time_ms),
      std::bind(&EmbotechProDriverConnector::on_timer, this));

  // origin of lat/lon coordinates in PTCL are map
  lanelet::GPSPoint origin_prodriver_latlon;
  //  odaiba
  // constexpr auto offset_x = 0.0;
  // constexpr auto offset_y = 0.0;
  // origin_prodriver_latlon.lat = 35.61450386813798;
  // origin_prodriver_latlon.lon = 139.76964575666943;

  // virtual_map
  constexpr auto offset_x = 0.0;
  constexpr auto offset_y = 0.0;
  origin_prodriver_latlon.lat = 35.68386376304963;
  origin_prodriver_latlon.lon = 139.68506451866963;

  // ryuyo_ci1
  // constexpr auto offset_x = 143693.7799;
  // constexpr auto offset_y = 108023.170;
  // origin_prodriver_latlon.lat = 34.66441053284202;
  // origin_prodriver_latlon.lon = 137.83339405223919;

  // ryuyo_ci2
  // constexpr auto offset_x = 143693.65344;
  // constexpr auto offset_y = 108022.3417698;
  // origin_prodriver_latlon.lat = 34.66444508923468;
  // origin_prodriver_latlon.lon = 137.83333262993906;

  // calculate GPSPoint
  constexpr auto mgrs_code = "54SUE";  // mgrs_code for odaiba and virtual_map
  // constexpr auto mgrs_code = "53SQU" // mgrs_code for ryuyo_ci1,2
  mgrs_projector_.setMGRSCode(mgrs_code);
  utm_projector_ = lanelet::projection::UtmProjector(lanelet::Origin(origin_prodriver_latlon));
  auto intermediate_xy = utm_projector_.forward(origin_prodriver_latlon);
  intermediate_xy.x() += offset_x;
  intermediate_xy.y() += offset_y;
  const auto corrected_latlon = utm_projector_.reverse(intermediate_xy);
  utm_projector_ = lanelet::projection::UtmProjector(lanelet::Origin(corrected_latlon));
}

void EmbotechProDriverConnector::on_timer() {
  if (!car_trajectory_data_.msg_received) {
    return;
  }
  const auto current_trajectory =
      to_autoware_trajectory(car_trajectory_data_.car_trajectory);
  const auto turn_indicator_msg = to_autoware_turn_indicator(car_trajectory_data_.car_trajectory);
  const auto hazard_light_msg = to_autoware_hazard_light_command(car_trajectory_data_.car_trajectory);

  if (!current_trajectory.points.empty()) {
    pub_trajectory_->publish(current_trajectory);
    pub_turn_signal_->publish(turn_indicator_msg);
    pub_hazard_signal_->publish(hazard_light_msg);
  } else {
    RCLCPP_ERROR(get_logger(), "embotech trajectory is empty. not published.");
  }
}

void EmbotechProDriverConnector::setup_PTCL() {
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
  const PTCL_Id trajectory_receiver_id = 11U;
  const uint16_t trajectory_receiver_port = 4991U;

  id_address_pairs_ = {
      {navigator_id, ip_local_host, navigator_port},
      {motion_planner_id, ip_local_host, motion_planner_port},
      {protect_id, ip_local_host, protect_port},
      {developer_ui_id, ip_local_host, developer_ui_port},
      {autoware_id, ip_local_host, autoware_port},
      {trajectory_receiver_id, ip_local_host, trajectory_receiver_port}};

  destinations_car_state_ = {navigator_id, motion_planner_id, protect_id, developer_ui_id};
  destinations_route_ = {navigator_id};
  destinations_perception_frame_ = {motion_planner_id, protect_id, developer_ui_id};

  PTCL_setLogPrefix("EX");
  PTCL_setLogThreshold(PTCL_getLogThresholdFromEnv());
  setup_port(autoware_id, ip_local_host, autoware_port,
             ptcl_context_, ptcl_udp_port_);
  setup_port(trajectory_receiver_id, ip_local_host,
             trajectory_receiver_port, ptcl_context_receiver_,
             ptcl_udp_port_receiver_);
  const bool installSuccess = PTCL_CarTrajectory_installCallback(
      &ptcl_context_receiver_, car_trajectory_CB, &car_trajectory_data_,
      protect_id, &trajectory_receiver_context_);

  if (installSuccess) {
    PTCL_UdpPort_startReceiving(&ptcl_udp_port_receiver_);
    printf("Initialized callback on receiver port for message from"
           "sender port with PTCL_Id %u.\n",
           protect_id);
  } else {
    printf("Start receiving message from protect failed.\n");
  }
}

void EmbotechProDriverConnector::on_kinematic_state(
    const Odometry::ConstSharedPtr msg) {
  current_kinematics_ = msg;

  if (!current_kinematics_ || !current_steer_ || !current_acceleration_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000 /*ms*/,
                         "waiting data...");
    return;
  }

  const auto PTCL_cat_state = to_PTCL_car_state();
  send_to_PTCL(PTCL_cat_state);
}

void EmbotechProDriverConnector::on_steering(
    const SteeringReport::ConstSharedPtr msg) {
  current_steer_ = msg;
}

void EmbotechProDriverConnector::on_acceleration(
    const AccelWithCovarianceStamped::ConstSharedPtr msg) {
  current_acceleration_ = msg;
}

void EmbotechProDriverConnector::on_dynamic_object(
    const PredictedObjects::ConstSharedPtr msg) {
  current_objects_ = msg;

  const auto ptcl_object = to_PTCL_perception_object(*current_objects_);
  send_to_PTCL(ptcl_object);
}

void EmbotechProDriverConnector::on_goal(
    const PoseStamped::ConstSharedPtr msg) {
  current_goal_ = msg;

  const auto PTCL_route = to_PTCL_route(*current_goal_);
  send_to_PTCL(PTCL_route);

  // publish an empty route with the received goal to Autoware
  // needed by the 'ad_service_state_monitor'
  autoware_auto_planning_msgs::msg::HADMapRoute route;
  route.header = msg->header;
  route.goal_pose = msg->pose;
  pub_route_->publish(route);
}

PTCL_CarState EmbotechProDriverConnector::to_PTCL_car_state() {
  const auto mgrs_pos = current_kinematics_->pose.pose.position;

  const auto ptcl_pos =
      convert_to_PTCL_Point({mgrs_pos.x, mgrs_pos.y, mgrs_pos.z});

  const auto current_time_ms = get_clock()->now().nanoseconds() / 1000000U;
  PTCL_CarState cat_state;
  cat_state.header.measurementTime = current_time_ms;
  cat_state.header.timeReference = current_time_ms;
  cat_state.header.vehicleId = 1;
  cat_state.pose.position.x = ptcl_pos.x;
  cat_state.pose.position.y = ptcl_pos.y;
  const double yaw_pose =
      tf2::getYaw(current_kinematics_->pose.pose.orientation);
  cat_state.pose.heading = PTCL_toPTCLAngleWrapped(yaw_pose);
  cat_state.angleRateYaw =
      PTCL_toPTCLAngleRate(current_kinematics_->twist.twist.angular.z);
  cat_state.velLon =
      PTCL_toPTCLSpeed(current_kinematics_->twist.twist.linear.x);
  cat_state.velLat = PTCL_toPTCLSpeed(
      0.0); // TODO think later. Autoware does not support lateral velocity.
  cat_state.accelLon =
      PTCL_toPTCLAccel(current_acceleration_->accel.accel.linear.x);
  cat_state.gear = PTCL_GEAR_SELECTOR_D;
  cat_state.controllerMode.latControlActive = true;
  cat_state.controllerMode.lonControlActive = true;
  cat_state.angleSteeredWheels =
      PTCL_toPTCLAngleWrapped(current_steer_->steering_tire_angle);
  cat_state.numSemiTrailers = static_cast<uint8_t>(0);

  return cat_state;
}

PTCL_PerceptionFrame EmbotechProDriverConnector::to_PTCL_perception_object(
    const PredictedObjects &object) {
  PTCL_PerceptionFrame ptcl_frame;
  const auto current_time_ms = get_clock()->now().nanoseconds() / 1000000U;
  ptcl_frame.header.measurementTime = current_time_ms;
  ptcl_frame.header.oemId = 0;                       // TODO: think later
  ptcl_frame.header.mapId = 0;                       // TODO: think later
  ptcl_frame.header.mapCrc = 0;                      // TODO: think later
  ptcl_frame.header.vehicleId = 1U;                  // TODO: think later

  if (object.objects.size() >= PTCL_PERCEPTION_FRAME_NUM_OBJECTS_MAX) {
    throw std::runtime_error(
        "perception object num is greater than maximum threshold."); // TODO!!!
  }
  ptcl_frame.numObjects = object.objects.size();
  // for each predicted object
  for (size_t i = 0; i < object.objects.size(); ++i) {
    const auto &obj = object.objects.at(i);
    auto &ptcl_object = ptcl_frame.objects[i];
    ptcl_object.id =
        i; // TODO: object ID used in the Autoware has 128-bit uuid.
    const auto classification = getHighestProbClass(obj.classification);
    ptcl_object.type = to_PTCL_object_type(classification);
    ptcl_object.confidenceType =
        static_cast<uint8_t>(255 * (1.0f - classification.probability));
    ptcl_object.properties = 0; // immobile(1), yielding(2), or ego-vehicle(4).
    const auto predicted_path = get_highest_prob_path(obj.kinematics);
    ptcl_object.numInstances = static_cast<int8_t>(predicted_path.path.size());
    if (ptcl_object.numInstances >= PTCL_PERCEPTION_OBJECT_NUM_INSTANCES_MAX) {
      throw std::runtime_error(
          "predicted_paths size is greater than maximum threshold."); // TODO
    }
    // for each predicted path point
    int16_t time_offset = 0;
    const auto dt_ms = static_cast<int16_t>(
        rclcpp::Duration(predicted_path.time_step).nanoseconds() * 10e-7);
    for (size_t j = 0; j < predicted_path.path.size(); ++j) {
      ptcl_object.instances[j].timeOffset = time_offset;
      ptcl_object.instances[j].shape =
          to_PTCL_polytope(obj.shape, predicted_path.path.at(j));
      time_offset += dt_ms; // int16_t [ms] (-32s ~ 32s)
    }

    ptcl_object.hasState = false;
  }

  // TODO: what should be done for FieldOfRegard? not used now...?
  ptcl_frame.numFieldOfRegardElementsPositive = 0;
  ptcl_frame.numFieldOfRegardElementsNegative = 0;

  return ptcl_frame;
}

Trajectory EmbotechProDriverConnector::to_autoware_trajectory(
    const PTCL_CarTrajectory &ptcl_car_trajectory) {
  Trajectory trajectory;
  const float64_t time_reference_sec = PTCL_toTime(
      ptcl_car_trajectory.header.timeReference); // uint64_t -> double
  trajectory.header.stamp = rclcpp::Time(time_reference_sec);
  trajectory.header.frame_id = "map";

  constexpr size_t start_idx =
      1; // to ignore front point since it has 0 velocity

  for (size_t i = start_idx; i < unsigned(ptcl_car_trajectory.numElements);
       ++i) {
    const auto &car_trajectory_element = ptcl_car_trajectory.elements[i];
    // const auto & element_position =
    // ptcl_car_trajectory.elements[i].pose.position;
    TrajectoryPoint trajectory_point;
    const auto element_pos =
        convert_to_MGRS_Point(car_trajectory_element.pose.position);
    trajectory_point.time_from_start = rclcpp::Duration::from_nanoseconds(
        PTCL_toTimeOffset(car_trajectory_element.timeOffset) * 10e6);
    trajectory_point.pose.position.x = element_pos.x();
    trajectory_point.pose.position.y = element_pos.y();
    trajectory_point.pose.position.z = element_pos.z();
    trajectory_point.pose.orientation =
        tier4_autoware_utils::createQuaternionFromRPY(
            0.0, 0.0, PTCL_toAngleWrapped(car_trajectory_element.pose.heading));
    trajectory_point.longitudinal_velocity_mps =
        PTCL_toSpeed(car_trajectory_element.velLon);
    trajectory_point.lateral_velocity_mps =
        PTCL_toSpeed(car_trajectory_element.velLat);
    trajectory_point.acceleration_mps2 =
        PTCL_toAccel(car_trajectory_element.accelLon);
    trajectory_point.front_wheel_angle_rad =
        PTCL_toPTCLAngleWrapped(car_trajectory_element.angleSteeredWheels);
    trajectory_point.rear_wheel_angle_rad = 0; // think later

    // to avoid Autoware error.
    if (!trajectory.points.empty()) {
      const auto &p = trajectory_point.pose.position;
      const auto &p_pre = trajectory.points.back().pose.position;
      if (p.x == p_pre.x && p.y == p_pre.y) {
        continue;
      }
    }
    trajectory.points.push_back(trajectory_point);
  }

  return trajectory;
}

HazardLightsCommand EmbotechProDriverConnector::to_autoware_hazard_light_command(const PTCL_CarTrajectory & ptcl_car_trajectory) {
  HazardLightsCommand cmd;
  float64_t time_reference_sec = PTCL_toTime(ptcl_car_trajectory.header.timeReference);
  cmd.stamp = rclcpp::Time(time_reference_sec);
  cmd.command = ptcl_car_trajectory.indicators == PTCL_INDICATORS_HAZARD ? HazardLightsCommand::ENABLE : HazardLightsCommand::NO_COMMAND;
  return cmd;
}

TurnIndicatorsCommand EmbotechProDriverConnector::to_autoware_turn_indicator(const PTCL_CarTrajectory & ptcl_car_trajectory) {
  TurnIndicatorsCommand cmd;
  float64_t time_reference_sec = PTCL_toTime(ptcl_car_trajectory.header.timeReference);
  cmd.stamp = rclcpp::Time(time_reference_sec);
  switch(ptcl_car_trajectory.indicators) {
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

PTCL_Route EmbotechProDriverConnector::to_PTCL_route(const PoseStamped &goal) {
  PTCL_Route ptcl_route;
  const auto mgrs_pos = goal.pose.position;
  const auto ptcl_pos =
      convert_to_PTCL_Point({mgrs_pos.x, mgrs_pos.y, mgrs_pos.z});
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
  ptcl_route.goalStates[0].duration = 0; // rest time after reaching goal

  return ptcl_route;
}

void EmbotechProDriverConnector::send_to_PTCL(const PTCL_CarState &car_state) {
  for(const auto & destination : destinations_car_state_) {
    const bool sent_state = PTCL_CarState_send(&ptcl_context_, &car_state, destination);
    if (!sent_state)
      RCLCPP_ERROR(get_logger(), "Failed to send car_state message to PTCL ID %u.", destination);
  }
}

void EmbotechProDriverConnector::send_to_PTCL(const PTCL_PerceptionFrame &perception_frame) {
  for (const auto & destination:  destinations_perception_frame_) {
    const bool sent_perception_frame = PTCL_PerceptionFrame_send(&ptcl_context_, &perception_frame, destination);
    if (!sent_perception_frame)
      RCLCPP_ERROR(get_logger(), "Failed to send perception_frame message to PTCL ID %u.", destination);
  }
}

void EmbotechProDriverConnector::send_to_PTCL(const PTCL_Route &route) {
  for (const auto & destination : destinations_route_) {
    const bool sent_route = PTCL_Route_send(&ptcl_context_, &route, destination);
    if (!sent_route)
      RCLCPP_ERROR(get_logger(), "Failed to send route message to PTCL ID %u.", destination);
  }
}

PTCL_Position
EmbotechProDriverConnector::convert_to_PTCL_Point(const MGRSPoint &mgrs_point) {
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

MGRSPoint EmbotechProDriverConnector::convert_to_MGRS_Point(
    const PTCL_Position &ptcl_pos) {
  // convert to global coordinate (double)
  UTMPoint utm_point;
  utm_point.x() = PTCL_toCoordinate(ptcl_pos.x);
  utm_point.y() = PTCL_toCoordinate(ptcl_pos.y);
  utm_point.z() = 0;

  const auto gps_point = utm_projector_.reverse(
      {utm_point.x(), utm_point.y(), utm_point.z()});

  const MGRSPoint mgrs_point = mgrs_projector_.forward(gps_point);
  return mgrs_point;
}

PTCL_Polytope EmbotechProDriverConnector::to_PTCL_polytope(const Shape &shape,
                                                           const Pose &pose) {
  Polygon2d polygon;
  calcObjectPolygon(shape, pose, &polygon);

  PTCL_Polytope ptcl_polygon;
  ptcl_polygon.numVertices = polygon.outer().size();
  for (size_t i = 0; i < polygon.outer().size(); ++i) {
    const auto &p = polygon.outer().at(i);
    const auto p_ptcl = convert_to_PTCL_Point({p.x(), p.y(), 0.0});
    ptcl_polygon.vertices[i].x = p_ptcl.x;
    ptcl_polygon.vertices[i].y = p_ptcl.y;
  }
  return ptcl_polygon;
}

void EmbotechProDriverConnector::setup_port(
    const unsigned int source_id,
    const uint32_t ip_local_host, const uint16_t source_port,
    PTCL_Context &context, PTCL_UdpPort &udp_port) {
  const int32_t ptcl_timeout_ms = 1000;
  const PTCL_PortInterface *port_interface = PTCL_UdpPort_init(
      ip_local_host, source_port, id_address_pairs_.data(), id_address_pairs_.size(),
      ptcl_timeout_ms, source_id, &context, &udp_port);

  bool setup_success = (port_interface != NULL);
  if (setup_success) {
    RCLCPP_INFO(this->get_logger(), "initialized UDP port with source_id %u",
                source_id);
  } else {
    PTCL_UdpPort_destroy(&udp_port);
    RCLCPP_ERROR(this->get_logger(), "Init of context failed.\n");
  }
}

} // namespace embotech_prodriver_connector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
    embotech_prodriver_connector::EmbotechProDriverConnector)