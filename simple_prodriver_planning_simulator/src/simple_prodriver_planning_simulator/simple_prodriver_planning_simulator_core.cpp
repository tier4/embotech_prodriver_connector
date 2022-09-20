// Copyright 2021 The Autoware Foundation.
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

#include "simple_prodriver_planning_simulator/simple_prodriver_planning_simulator_core.hpp"

#include "autoware_auto_tf2/tf2_autoware_auto_msgs.hpp"
#include "common/types.hpp"
#include "motion_common/motion_common.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "simple_prodriver_planning_simulator/vehicle_model/sim_model.hpp"
#include "tier4_autoware_utils/ros/update_param.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"
#include "car_state.h"

#include <lanelet2_extension/projection/mgrs_projector.hpp>

#include <tf2/LinearMath/Quaternion.h>
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
#define ID_ADDRESS_MAP_SIZE (2U)
#define AUTOWARE_ID (10U)
#define AUTOWARE_PORT (4990U)
#define PRODRIVER_ID (9U)
#define PRODRIVER_PORT (4989U)

static const uint8_t autoware_ip[4] = {127U, 0U, 0U, 1U};
static const uint8_t prodriver_ip[4] = {127U, 0U, 0U, 1U};

const uint8_t num_destinations_state = 1U;
const PTCL_Id destinations_car_state[] = {PRODRIVER_ID};

/// Timeout of blocking UDP receiver call in milliseconds
#define UDP_PORT_TIMEOUT_MS (1000)

/// Duration of sleep call in while loop in microseconds
#define WHILE_LOOP_SLEEP_DURATION_US (100)

namespace
{

autoware_auto_vehicle_msgs::msg::VelocityReport to_velocity_report(
  const std::shared_ptr<SimModelInterface> vehicle_model_ptr)
{
  autoware_auto_vehicle_msgs::msg::VelocityReport velocity;
  velocity.longitudinal_velocity = static_cast<float32_t>(vehicle_model_ptr->getVx());
  velocity.lateral_velocity = 0.0F;
  velocity.heading_rate = static_cast<float32_t>(vehicle_model_ptr->getWz());
  return velocity;
}

nav_msgs::msg::Odometry to_odometry(const std::shared_ptr<SimModelInterface> vehicle_model_ptr)
{
  nav_msgs::msg::Odometry odometry;
  odometry.pose.pose.position.x = vehicle_model_ptr->getX();
  odometry.pose.pose.position.y = vehicle_model_ptr->getY();
  odometry.pose.pose.orientation = motion::motion_common::from_angle(vehicle_model_ptr->getYaw());
  odometry.twist.twist.linear.x = vehicle_model_ptr->getVx();
  odometry.twist.twist.angular.z = vehicle_model_ptr->getWz();

  return odometry;
}

autoware_auto_vehicle_msgs::msg::SteeringReport to_steering_report(
  const std::shared_ptr<SimModelInterface> vehicle_model_ptr)
{
  autoware_auto_vehicle_msgs::msg::SteeringReport steer;
  steer.steering_tire_angle = static_cast<float32_t>(vehicle_model_ptr->getSteer());
  return steer;
}

}  // namespace

namespace simulation
{
namespace simple_prodriver_planning_simulator
{

SimpleProdriverPlanningSimulator::SimpleProdriverPlanningSimulator(const rclcpp::NodeOptions & options)
: Node("simple_prodriver_planning_simulator", options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
{
  simulated_frame_id_ = declare_parameter("simulated_frame_id", "base_link");
  origin_frame_id_ = declare_parameter("origin_frame_id", "odom");
  add_measurement_noise_ = declare_parameter("add_measurement_noise", false);
  simulate_motion_ = declare_parameter<bool>("initial_engage_state");

  using rclcpp::QoS;
  using std::placeholders::_1;
  using std::placeholders::_2;

  sub_init_pose_ = create_subscription<PoseWithCovarianceStamped>(
    "/initialpose", QoS{1}, std::bind(&SimpleProdriverPlanningSimulator::on_initialpose, this, _1));
  sub_ackermann_cmd_ = create_subscription<AckermannControlCommand>(
    "input/ackermann_control_command", QoS{1},
    [this](const AckermannControlCommand::SharedPtr msg) { current_ackermann_cmd_ = *msg; });
  sub_manual_ackermann_cmd_ = create_subscription<AckermannControlCommand>(
    "input/manual_ackermann_control_command", QoS{1},
    [this](const AckermannControlCommand::SharedPtr msg) { current_manual_ackermann_cmd_ = *msg; });
  sub_gear_cmd_ = create_subscription<GearCommand>(
    "input/gear_command", QoS{1},
    [this](const GearCommand::SharedPtr msg) { current_gear_cmd_ = *msg; });
  sub_manual_gear_cmd_ = create_subscription<GearCommand>(
    "input/manual_gear_command", QoS{1},
    [this](const GearCommand::SharedPtr msg) { current_manual_gear_cmd_ = *msg; });
  sub_turn_indicators_cmd_ = create_subscription<TurnIndicatorsCommand>(
    "input/turn_indicators_command", QoS{1},
    std::bind(&SimpleProdriverPlanningSimulator::on_turn_indicators_cmd, this, _1));
  sub_hazard_lights_cmd_ = create_subscription<HazardLightsCommand>(
    "input/hazard_lights_command", QoS{1},
    std::bind(&SimpleProdriverPlanningSimulator::on_hazard_lights_cmd, this, _1));
  sub_trajectory_ = create_subscription<Trajectory>(
    "input/trajectory", QoS{1}, std::bind(&SimpleProdriverPlanningSimulator::on_trajectory, this, _1));

  srv_mode_req_ = create_service<tier4_vehicle_msgs::srv::ControlModeRequest>(
    "input/control_mode_request",
    std::bind(&SimpleProdriverPlanningSimulator::on_control_mode_request, this, _1, _2));

  // TODO(Horibe): should be replaced by mode_request. Keep for the backward compatibility.
  sub_engage_ = create_subscription<Engage>(
    "input/engage", rclcpp::QoS{1}, std::bind(&SimpleProdriverPlanningSimulator::on_engage, this, _1));

  pub_control_mode_report_ =
    create_publisher<ControlModeReport>("output/control_mode_report", QoS{1});
  pub_gear_report_ = create_publisher<GearReport>("output/gear_report", QoS{1});
  pub_turn_indicators_report_ =
    create_publisher<TurnIndicatorsReport>("output/turn_indicators_report", QoS{1});
  pub_hazard_lights_report_ =
    create_publisher<HazardLightsReport>("output/hazard_lights_report", QoS{1});
  pub_current_pose_ = create_publisher<PoseStamped>("/current_pose", QoS{1});
  pub_velocity_ = create_publisher<VelocityReport>("output/twist", QoS{1});
  pub_odom_ = create_publisher<Odometry>("output/odometry", QoS{1});
  pub_steer_ = create_publisher<SteeringReport>("output/steering", QoS{1});
  pub_acc_ = create_publisher<AccelWithCovarianceStamped>("output/acceleration", QoS{1});
  pub_tf_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf", QoS{1});

  /* set param callback */
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&SimpleProdriverPlanningSimulator::on_parameter, this, _1));

  timer_sampling_time_ms_ = static_cast<uint32_t>(declare_parameter("timer_sampling_time_ms", 25));
  on_timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::milliseconds(timer_sampling_time_ms_),
    std::bind(&SimpleProdriverPlanningSimulator::on_timer, this));

  tier4_api_utils::ServiceProxyNodeInterface proxy(this);
  group_api_service_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_set_pose_ = proxy.create_service<tier4_external_api_msgs::srv::InitializePose>(
    "/api/simulator/set/pose", std::bind(&SimpleProdriverPlanningSimulator::on_set_pose, this, _1, _2),
    rmw_qos_profile_services_default, group_api_service_);

  // set vehicle model type
  initialize_vehicle_model();

  // set initialize source
  const auto initialize_source = declare_parameter("initialize_source", "INITIAL_POSE_TOPIC");
  RCLCPP_INFO(this->get_logger(), "initialize_source : %s", initialize_source.c_str());
  if (initialize_source == "ORIGIN") {
    Pose p;
    p.orientation.w = 1.0;          // yaw = 0
    set_initial_state(p, Twist{});  // initialize with 0 for all variables
  } else if (initialize_source == "INITIAL_POSE_TOPIC") {
    // initialpose sub already exists. Do nothing.
  }

  // measurement noise
  {
    std::random_device seed;
    auto & m = measurement_noise_;
    m.rand_engine_ = std::make_shared<std::mt19937>(seed());
    float64_t pos_noise_stddev = declare_parameter("pos_noise_stddev", 1e-2);
    float64_t vel_noise_stddev = declare_parameter("vel_noise_stddev", 1e-2);
    float64_t rpy_noise_stddev = declare_parameter("rpy_noise_stddev", 1e-4);
    float64_t steer_noise_stddev = declare_parameter("steer_noise_stddev", 1e-4);
    m.pos_dist_ = std::make_shared<std::normal_distribution<>>(0.0, pos_noise_stddev);
    m.vel_dist_ = std::make_shared<std::normal_distribution<>>(0.0, vel_noise_stddev);
    m.rpy_dist_ = std::make_shared<std::normal_distribution<>>(0.0, rpy_noise_stddev);
    m.steer_dist_ = std::make_shared<std::normal_distribution<>>(0.0, steer_noise_stddev);

    x_stddev_ = declare_parameter("x_stddev", 0.0001);
    y_stddev_ = declare_parameter("y_stddev", 0.0001);
  }

  // Initialize PTCL logging
  //  - Log threshold of console determined by environment variable
  PTCL_setLogPrefix("EX");
  PTCL_setLogThreshold(PTCL_getLogThresholdFromEnv());

  PTCL_UdpIdAddressPair id_address_map_car_state[ID_ADDRESS_MAP_SIZE] = {
    {AUTOWARE_ID, PTCL_UdpPort_getIpFromArray(autoware_ip), AUTOWARE_PORT},
    {PRODRIVER_ID, PTCL_UdpPort_getIpFromArray(prodriver_ip), PRODRIVER_PORT}};

  // Setup PTCL context for car state
  port_interface_car_state_ = PTCL_UdpPort_init(
    PTCL_UdpPort_getIpFromArray(autoware_ip), AUTOWARE_PORT, id_address_map_car_state,
    ID_ADDRESS_MAP_SIZE, UDP_PORT_TIMEOUT_MS, AUTOWARE_ID, &context_car_state_,
    &udp_port_car_state_);

  bool setup_success = (port_interface_car_state_ != NULL);
  if (setup_success) {
    RCLCPP_ERROR(
      this->get_logger(), "initialized sender UDP port with AUTOWARE_ID %u.\n", AUTOWARE_ID);
  } else {
    PTCL_UdpPort_destroy(&udp_port_car_state_);
    printf("Init of sender context failed.\n");
  }

  // control mode
  current_control_mode_.data = ControlMode::AUTO;
  current_manual_gear_cmd_.command = GearCommand::DRIVE;
}

void SimpleProdriverPlanningSimulator::initialize_vehicle_model()
{
  const auto vehicle_model_type_str = declare_parameter("vehicle_model_type", "IDEAL_STEER_VEL");

  RCLCPP_INFO(this->get_logger(), "vehicle_model_type = %s", vehicle_model_type_str.c_str());

  const float64_t vel_lim = declare_parameter("vel_lim", 50.0);
  const float64_t vel_rate_lim = declare_parameter("vel_rate_lim", 7.0);
  const float64_t steer_lim = declare_parameter("steer_lim", 1.0);
  const float64_t steer_rate_lim = declare_parameter("steer_rate_lim", 5.0);
  const float64_t acc_time_delay = declare_parameter("acc_time_delay", 0.1);
  const float64_t acc_time_constant = declare_parameter("acc_time_constant", 0.1);
  const float64_t vel_time_delay = declare_parameter("vel_time_delay", 0.25);
  const float64_t vel_time_constant = declare_parameter("vel_time_constant", 0.5);
  const float64_t steer_time_delay = declare_parameter("steer_time_delay", 0.24);
  const float64_t steer_time_constant = declare_parameter("steer_time_constant", 0.27);
  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
  const float64_t wheelbase = vehicle_info.wheel_base_m;

  if (vehicle_model_type_str == "IDEAL_STEER_VEL") {
    vehicle_model_type_ = VehicleModelType::IDEAL_STEER_VEL;
    vehicle_model_ptr_ = std::make_shared<SimModelIdealSteerVel>(wheelbase);
  } else if (vehicle_model_type_str == "IDEAL_STEER_ACC") {
    vehicle_model_type_ = VehicleModelType::IDEAL_STEER_ACC;
    vehicle_model_ptr_ = std::make_shared<SimModelIdealSteerAcc>(wheelbase);
  } else if (vehicle_model_type_str == "IDEAL_STEER_ACC_GEARED") {
    vehicle_model_type_ = VehicleModelType::IDEAL_STEER_ACC_GEARED;
    vehicle_model_ptr_ = std::make_shared<SimModelIdealSteerAccGeared>(wheelbase);
  } else if (vehicle_model_type_str == "DELAY_STEER_VEL") {
    vehicle_model_type_ = VehicleModelType::DELAY_STEER_VEL;
    vehicle_model_ptr_ = std::make_shared<SimModelDelaySteerVel>(
      vel_lim, steer_lim, vel_rate_lim, steer_rate_lim, wheelbase, timer_sampling_time_ms_ / 1000.0,
      vel_time_delay, vel_time_constant, steer_time_delay, steer_time_constant);
  } else if (vehicle_model_type_str == "DELAY_STEER_ACC") {
    vehicle_model_type_ = VehicleModelType::DELAY_STEER_ACC;
    vehicle_model_ptr_ = std::make_shared<SimModelDelaySteerAcc>(
      vel_lim, steer_lim, vel_rate_lim, steer_rate_lim, wheelbase, timer_sampling_time_ms_ / 1000.0,
      acc_time_delay, acc_time_constant, steer_time_delay, steer_time_constant);
  } else if (vehicle_model_type_str == "DELAY_STEER_ACC_GEARED") {
    vehicle_model_type_ = VehicleModelType::DELAY_STEER_ACC_GEARED;
    vehicle_model_ptr_ = std::make_shared<SimModelDelaySteerAccGeared>(
      vel_lim, steer_lim, vel_rate_lim, steer_rate_lim, wheelbase, timer_sampling_time_ms_ / 1000.0,
      acc_time_delay, acc_time_constant, steer_time_delay, steer_time_constant);
  } else {
    throw std::invalid_argument("Invalid vehicle_model_type: " + vehicle_model_type_str);
  }
}

rcl_interfaces::msg::SetParametersResult SimpleProdriverPlanningSimulator::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  try {
    tier4_autoware_utils::updateParam(parameters, "x_stddev", x_stddev_);
    tier4_autoware_utils::updateParam(parameters, "y_stddev", y_stddev_);
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
  }

  return result;
}

void SimpleProdriverPlanningSimulator::on_timer()
{
  // RCLCPP_ERROR(this->get_logger(), "is_initialized_ in timer function: %d", is_initialized_);  
  if (!is_initialized_) {
    printf("Not yet init.\n");
    // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "waiting initialization...");
    return;
  }

  // update vehicle dynamics
  {
    const float64_t dt = delta_time_.get_dt(get_clock()->now());

    if (current_control_mode_.data == ControlMode::AUTO) {
      vehicle_model_ptr_->setGear(current_gear_cmd_.command);
      set_input(current_ackermann_cmd_);
    } else {
      vehicle_model_ptr_->setGear(current_manual_gear_cmd_.command);
      set_input(current_manual_ackermann_cmd_);
    }

    if (simulate_motion_) {
      vehicle_model_ptr_->update(dt);
    }
  }

  // set current state
  current_odometry_ = to_odometry(vehicle_model_ptr_);
  current_odometry_.pose.pose.position.z = get_z_pose_from_trajectory(
    current_odometry_.pose.pose.position.x, current_odometry_.pose.pose.position.y);

  current_velocity_ = to_velocity_report(vehicle_model_ptr_);
  current_steer_ = to_steering_report(vehicle_model_ptr_);

  if (add_measurement_noise_) {
    add_measurement_noise(current_odometry_, current_velocity_, current_steer_);
  }

  // add estimate covariance
  {
    current_odometry_.pose.covariance[0 * 6 + 0] = x_stddev_;
    current_odometry_.pose.covariance[1 * 6 + 1] = y_stddev_;
  }

  // publish vehicle state
  publish_odometry(current_odometry_);
  publish_velocity(current_velocity_);
  publish_steering(current_steer_);
  publish_acceleration();
  // send car state to PRODRIVER
  // send_car_state(current_odometry_, current_velocity_, current_steer_, context_car_state);

  const float64_t currentTime = get_clock()->now().seconds();

  // Fill PTCL_CarState example data.
  static PTCL_CarState car_state;
  memset(&car_state, 0, sizeof(PTCL_CarState));
  car_state.header.measurementTime = currentTime;
  car_state.header.timeReference = currentTime;
  car_state.header.vehicleId = 1U;
  car_state.pose.position.x = PTCL_toPTCLCoordinate(100.0);
  car_state.pose.position.y = PTCL_toPTCLCoordinate(100.0);
  car_state.pose.heading = PTCL_toPTCLAngleWrapped(3.0);
  car_state.forceTractionTotal = PTCL_toPTCLForce(3.0);
  car_state.angleSteeredWheels = PTCL_toPTCLAngleWrapped(3.0);
  car_state.velLon = PTCL_toPTCLSpeed(30.0);
  car_state.accelLon = PTCL_toPTCLAccel(3.0);
  car_state.gear = PTCL_GEAR_SELECTOR_D;
  car_state.controllerMode.latControlActive = true;
  car_state.controllerMode.lonControlActive = true;

  // Send PTCL car_state frame.
  for (uint8_t dstIdx = 0U; dstIdx < num_destinations_state; ++dstIdx) {
    bool sent_state = PTCL_CarState_send(
      &context_car_state_, &car_state, destinations_car_state[dstIdx]);
    if (!sent_state) {
      printf(
        "Failed to send car_stateFrame message to PTCL ID %u.\n",
        destinations_car_state[dstIdx]);
    } else {
      printf(
        "car_stateFrame message sent to PTCL ID %u.\n",
        destinations_car_state[dstIdx]);
    }
  }

  publish_control_mode_report();
  publish_gear_report();
  publish_turn_indicators_report();
  publish_hazard_lights_report();
  publish_tf(current_odometry_);
}

void SimpleProdriverPlanningSimulator::on_initialpose(const PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  // save initial pose
  Twist initial_twist;
  PoseStamped initial_pose;
  initial_pose.header = msg->header;
  initial_pose.pose = msg->pose.pose;
  set_initial_state_with_transform(initial_pose, initial_twist);
}

void SimpleProdriverPlanningSimulator::on_set_pose(
  const InitializePose::Request::ConstSharedPtr request,
  const InitializePose::Response::SharedPtr response)
{
  // save initial pose
  Twist initial_twist;
  PoseStamped initial_pose;
  initial_pose.header = request->pose.header;
  initial_pose.pose = request->pose.pose.pose;
  set_initial_state_with_transform(initial_pose, initial_twist);
  response->status = tier4_api_utils::response_success();
}

void SimpleProdriverPlanningSimulator::set_input(const AckermannControlCommand & cmd)
{
  const auto steer = cmd.lateral.steering_tire_angle;
  const auto vel = cmd.longitudinal.speed;
  const auto accel = cmd.longitudinal.acceleration;

  using autoware_auto_vehicle_msgs::msg::GearCommand;
  Eigen::VectorXd input(vehicle_model_ptr_->getDimU());
  const auto gear = vehicle_model_ptr_->getGear();

  // TODO(Watanabe): The definition of the sign of acceleration in REVERSE mode is different
  // between .auto and proposal.iv, and will be discussed later.
  float acc = accel;
  if (gear == GearCommand::NONE) {
    acc = 0.0;
  } else if (gear == GearCommand::REVERSE || gear == GearCommand::REVERSE_2) {
    acc = -accel;
  }

  if (
    vehicle_model_type_ == VehicleModelType::IDEAL_STEER_VEL ||
    vehicle_model_type_ == VehicleModelType::DELAY_STEER_VEL) {
    input << vel, steer;
  } else if (  // NOLINT
    vehicle_model_type_ == VehicleModelType::IDEAL_STEER_ACC ||
    vehicle_model_type_ == VehicleModelType::DELAY_STEER_ACC) {
    input << acc, steer;
  } else if (  // NOLINT
    vehicle_model_type_ == VehicleModelType::IDEAL_STEER_ACC_GEARED ||
    vehicle_model_type_ == VehicleModelType::DELAY_STEER_ACC_GEARED) {
    input << acc, steer;
  }
  vehicle_model_ptr_->setInput(input);
}

void SimpleProdriverPlanningSimulator::on_turn_indicators_cmd(
  const TurnIndicatorsCommand::ConstSharedPtr msg)
{
  current_turn_indicators_cmd_ptr_ = msg;
}

void SimpleProdriverPlanningSimulator::on_hazard_lights_cmd(const HazardLightsCommand::ConstSharedPtr msg)
{
  current_hazard_lights_cmd_ptr_ = msg;
}

void SimpleProdriverPlanningSimulator::on_trajectory(const Trajectory::ConstSharedPtr msg)
{
  current_trajectory_ptr_ = msg;
}

void SimpleProdriverPlanningSimulator::on_engage(const Engage::ConstSharedPtr msg)
{
  simulate_motion_ = msg->engage;
}

void SimpleProdriverPlanningSimulator::on_control_mode_request(
  const ControlModeRequest::Request::SharedPtr request,
  const ControlModeRequest::Response::SharedPtr response)
{
  current_control_mode_ = request->mode;
  response->success = true;
  return;
}

void SimpleProdriverPlanningSimulator::add_measurement_noise(
  Odometry & odom, VelocityReport & vel, SteeringReport & steer) const
{
  auto & n = measurement_noise_;
  odom.pose.pose.position.x += (*n.pos_dist_)(*n.rand_engine_);
  odom.pose.pose.position.y += (*n.pos_dist_)(*n.rand_engine_);
  const auto velocity_noise = (*n.vel_dist_)(*n.rand_engine_);
  odom.twist.twist.linear.x += velocity_noise;
  float32_t yaw = motion::motion_common::to_angle(odom.pose.pose.orientation);
  yaw += static_cast<float>((*n.rpy_dist_)(*n.rand_engine_));
  odom.pose.pose.orientation = motion::motion_common::from_angle(yaw);

  vel.longitudinal_velocity += static_cast<float32_t>(velocity_noise);

  steer.steering_tire_angle += static_cast<float32_t>((*n.steer_dist_)(*n.rand_engine_));
}

void SimpleProdriverPlanningSimulator::set_initial_state_with_transform(
  const PoseStamped & pose_stamped, const Twist & twist)
{
  auto transform = get_transform_msg(origin_frame_id_, pose_stamped.header.frame_id);
  Pose pose;
  pose.position.x = pose_stamped.pose.position.x + transform.transform.translation.x;
  pose.position.y = pose_stamped.pose.position.y + transform.transform.translation.y;
  pose.position.z = pose_stamped.pose.position.z + transform.transform.translation.z;
  pose.orientation = pose_stamped.pose.orientation;
  set_initial_state(pose, twist);
}

void SimpleProdriverPlanningSimulator::set_initial_state(const Pose & pose, const Twist & twist)
{
  const float64_t x = pose.position.x;
  const float64_t y = pose.position.y;
  const float64_t yaw = ::motion::motion_common::to_angle(pose.orientation);
  const float64_t vx = twist.linear.x;
  const float64_t steer = 0.0;
  const float64_t accx = 0.0;

  Eigen::VectorXd state(vehicle_model_ptr_->getDimX());

  if (vehicle_model_type_ == VehicleModelType::IDEAL_STEER_VEL) {
    state << x, y, yaw;
  } else if (  // NOLINT
    vehicle_model_type_ == VehicleModelType::IDEAL_STEER_ACC ||
    vehicle_model_type_ == VehicleModelType::IDEAL_STEER_ACC_GEARED) {
    state << x, y, yaw, vx;
  } else if (  // NOLINT
    vehicle_model_type_ == VehicleModelType::DELAY_STEER_VEL) {
    state << x, y, yaw, vx, steer;
  } else if (  // NOLINT
    vehicle_model_type_ == VehicleModelType::DELAY_STEER_ACC ||
    vehicle_model_type_ == VehicleModelType::DELAY_STEER_ACC_GEARED) {
    state << x, y, yaw, vx, steer, accx;
  }
  vehicle_model_ptr_->setState(state);

  is_initialized_ = true;
  // RCLCPP_ERROR(this->get_logger(), "is_initialized_ updated: %d", is_initialized_);  
}

double SimpleProdriverPlanningSimulator::get_z_pose_from_trajectory(const double x, const double y)
{
  // calculate closest point on trajectory
  if (!current_trajectory_ptr_) {
    return 0.0;
  }

  const double max_sqrt_dist = std::numeric_limits<double>::max();
  double min_sqrt_dist = max_sqrt_dist;
  size_t index;
  bool found = false;
  for (size_t i = 0; i < current_trajectory_ptr_->points.size(); ++i) {
    const double dist_x = (current_trajectory_ptr_->points.at(i).pose.position.x - x);
    const double dist_y = (current_trajectory_ptr_->points.at(i).pose.position.y - y);
    double sqrt_dist = dist_x * dist_x + dist_y * dist_y;
    if (sqrt_dist < min_sqrt_dist) {
      min_sqrt_dist = sqrt_dist;
      index = i;
      found = true;
    }
  }
  if (found) {
    return current_trajectory_ptr_->points.at(index).pose.position.z;
  }

  return 0.0;
}

TransformStamped SimpleProdriverPlanningSimulator::get_transform_msg(
  const std::string parent_frame, const std::string child_frame)
{
  TransformStamped transform;
  while (true) {
    try {
      const auto time_point = tf2::TimePoint(std::chrono::milliseconds(0));
      transform = tf_buffer_.lookupTransform(
        parent_frame, child_frame, time_point, tf2::durationFromSec(0.0));
      break;
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
      rclcpp::sleep_for(std::chrono::milliseconds(500));
    }
  }
  return transform;
}

void SimpleProdriverPlanningSimulator::publish_velocity(const VelocityReport & velocity)
{
  VelocityReport msg = velocity;
  msg.header.stamp = get_clock()->now();
  msg.header.frame_id = simulated_frame_id_;
  pub_velocity_->publish(msg);
}

void SimpleProdriverPlanningSimulator::publish_odometry(const Odometry & odometry)
{
  Odometry msg = odometry;
  msg.header.frame_id = origin_frame_id_;
  msg.header.stamp = get_clock()->now();
  msg.child_frame_id = simulated_frame_id_;
  pub_odom_->publish(msg);
}

void SimpleProdriverPlanningSimulator::publish_steering(const SteeringReport & steer)
{
  SteeringReport msg = steer;
  msg.stamp = get_clock()->now();
  pub_steer_->publish(msg);
}

void SimpleProdriverPlanningSimulator::publish_acceleration()
{
  AccelWithCovarianceStamped msg;
  msg.header.frame_id = "/base_link";
  msg.header.stamp = get_clock()->now();
  msg.accel.accel.linear.x = vehicle_model_ptr_->getAx();

  constexpr auto COV = 0.001;
  msg.accel.covariance.at(6 * 0 + 0) = COV;  // linear x
  msg.accel.covariance.at(6 * 1 + 1) = COV;  // linear y
  msg.accel.covariance.at(6 * 2 + 2) = COV;  // linear z
  msg.accel.covariance.at(6 * 3 + 3) = COV;  // angular x
  msg.accel.covariance.at(6 * 4 + 4) = COV;  // angular y
  msg.accel.covariance.at(6 * 5 + 5) = COV;  // angular z
  pub_acc_->publish(msg);
}

void SimpleProdriverPlanningSimulator::send_car_state(
  const Odometry & odometry, const VelocityReport & velocity, const SteeringReport & steer,
  PTCL_Context & context_car_state)
{
  // calculate GPSPoint
  lanelet::projection::MGRSProjector projector;
  lanelet::BasicPoint3d mgrs_point;
  mgrs_point.x() = odometry.pose.pose.position.x;
  mgrs_point.y() = odometry.pose.pose.position.y;
  mgrs_point.z() = odometry.pose.pose.position.z;
  projector.setMGRSCode("54SUE");
  lanelet::GPSPoint gps_point = projector.reverse(mgrs_point);
  double car_lat = round(gps_point.lat * 1e8) / 1e8;
  double car_lon = round(gps_point.lon * 1e8) / 1e8;
  // double car_lat = gps_point.lat;
  // double car_lon = gps_point.lon;
  // RCLCPP_ERROR(this->get_logger(),"pose_mgrs_x %lf.\n", mgrs_point.x());
  // RCLCPP_ERROR(this->get_logger(), "pose_lat %lf.\n", car_lat);
  // RCLCPP_ERROR(this->get_logger(), "pose_lon %lf.\n", car_lon);
  // RCLCPP_ERROR(this->get_logger(),"pose_mgrs_looped_x %lf.\n", looped_mgrs_point.x());

  lanelet::GPSPoint original_position_UTM{origin_lat_, origin_lon_};
  lanelet::GPSPoint vehicle_position_UTM{car_lat, car_lon};

  const auto [target_UTM_x, target_UTM_y] = convert_pose_to_UTM_coordinate(vehicle_position_UTM);
  const auto [origin_UTM_x, origin_UTM_y] = convert_pose_to_UTM_coordinate(original_position_UTM);

  // debug print for latlon -> UTM conversion
  // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "latlon (%.12f, %.12f) -> UTM (%.12f,
  // %.12f)", car_lat, car_lon, target_UTM_x, target_UTM_y);

  const PTCL_Coordinate position_x_PTCL =
    PTCL_toPTCLCoordinate(target_UTM_x - origin_UTM_x);  // converted to int
  const PTCL_Coordinate position_y_PTCL =
    PTCL_toPTCLCoordinate(target_UTM_y - origin_UTM_y);  // converted to int

  // TODO: How to calculate yaw? Should we use it directly, or calculate as below?
  // const PTCL_Coordinate position_y_PTCL = std::atan2(position_y_PTCL, position_x_PTCL);

  // Compute some "measurement" using model.
  float64_t forceTractionTotal = vehicle_model_ptr_->getAx() * PTCL_toMass(2000);
  // float64_t wheelbase = PTCL_toDistance(2.7);
  // float64_t angleSteeredWheels = atan(wheelbase * 0.0);

  static PTCL_CarState car_state;
  memset(&car_state, 0, sizeof(PTCL_CarState));
  const float64_t current_time = get_clock()->now().seconds();
  // RCLCPP_ERROR(this->get_logger(),"current_time %lf.\n", current_time);
  car_state.header.measurementTime = PTCL_toPTCLTime(current_time);
  car_state.header.timeReference = PTCL_toPTCLTime(current_time);
  car_state.header.vehicleId = 1;
  car_state.pose.position.x = position_x_PTCL;
  car_state.pose.position.y = position_y_PTCL;
  car_state.pose.heading = PTCL_toPTCLAngleWrapped(tf2::getYaw(odometry.pose.pose.orientation));
  car_state.forceTractionTotal = PTCL_toPTCLForce(forceTractionTotal);
  car_state.angleSteeredWheels = PTCL_toPTCLAngleWrapped(steer.steering_tire_angle);
  car_state.velLon = PTCL_toPTCLSpeed(velocity.longitudinal_velocity);
  car_state.velLat = PTCL_toPTCLSpeed(velocity.lateral_velocity);
  car_state.accelLon = PTCL_toPTCLAccel(vehicle_model_ptr_->getAx());
  car_state.angleRateYaw = PTCL_toPTCLAngleRate(odometry.twist.twist.angular.z);

  car_state.gear = PTCL_GEAR_SELECTOR_D;
  car_state.controllerMode.latControlActive = true;
  car_state.controllerMode.lonControlActive = true;

  // Send PTCL car_state.
  for (uint8_t dstIdx = 0U; dstIdx < num_destinations_state; ++dstIdx) {
    bool sent_state = PTCL_CarState_send(
      &context_car_state, &car_state, destinations_car_state[dstIdx]);
    if (!sent_state) {
      printf(
        "Failed to send car_stateFrame message to PTCL ID %u.\n",
        destinations_car_state[dstIdx]);
    } else {
      printf(
        "car_stateFrame message sent to PTCL ID %u.\n",
        destinations_car_state[dstIdx]);
    }
  }

  return;
}

std::pair<double, double> SimpleProdriverPlanningSimulator::convert_pose_to_UTM_coordinate(
  const lanelet::GPSPoint & p_target)
{
  // TODO(K.Sugahara): this is from the Turbo87/UTM repository. Take right way to manage this code.
  // https://github.com/Turbo87/utm/blob/master/utm/conversion.py#L190-L286

  // parameters
  constexpr auto K0 = 0.9996;

  constexpr auto E = 0.00669438;
  constexpr auto E2 = E * E;
  constexpr auto E3 = E2 * E;
  constexpr auto E_P2 = E / (1 - E);

  constexpr auto M1 = (1 - E / 4 - 3 * E2 / 64 - 5 * E3 / 256);
  constexpr auto M2 = (3 * E / 8 + 3 * E2 / 32 + 45 * E3 / 1024);
  constexpr auto M3 = (15 * E2 / 256 + 45 * E3 / 1024);
  constexpr auto M4 = (35 * E3 / 3072);

  constexpr auto R = 6378137;

  constexpr double LATLON_TO_RAD = M_PI / 180.0;

  const auto zone_number_to_central_longitude = [](const auto zone_number) {
    return (zone_number - 1) * 6 - 180 + 3;
  };

  const auto mod_angle = [](const auto value) { return std::fmod(value + M_PI, 2 * M_PI) - M_PI; };

  const auto latlon_to_zone_number = [](const auto latitude, const auto longitude) {
    if ((56 <= latitude && latitude < 64) && (3 <= longitude && longitude < 12)) return 32;
    if ((72 <= latitude && latitude <= 84) && (longitude >= 0)) {
      if (longitude < 9) {
        return 31;
      } else if (longitude < 21) {
        return 33;
      } else if (longitude < 33) {
        return 35;
      } else if (longitude < 42) {
        return 37;
      }
    }
    return static_cast<int>((longitude + 180) / 6) + 1;
  };

  const auto lat_rad = p_target.lat * LATLON_TO_RAD;
  const auto lat_sin = std::sin(lat_rad);
  const auto lat_cos = std::cos(lat_rad);

  const auto lat_tan = lat_sin / lat_cos;
  const auto lat_tan2 = lat_tan * lat_tan;
  const auto lat_tan4 = lat_tan2 * lat_tan2;

  const auto zone_number = latlon_to_zone_number(p_target.lat, p_target.lon);

  if (zone_number != 54) {
    throw std::runtime_error("TEMP: zone_number must be 54 in this test");
  }

  // const auto zone_letter = 'S'; // unused

  const auto lon_rad = p_target.lon * LATLON_TO_RAD;
  const auto central_lon = zone_number_to_central_longitude(zone_number);

  const auto central_lon_rad = central_lon * LATLON_TO_RAD;

  const auto n = R / std::sqrt(1 - E * lat_sin * lat_sin);
  const auto c = E_P2 * lat_cos * lat_cos;

  const auto a = lat_cos * mod_angle(lon_rad - central_lon_rad);
  const auto a2 = a * a;
  const auto a3 = a2 * a;
  const auto a4 = a3 * a;
  const auto a5 = a4 * a;
  const auto a6 = a5 * a;

  const auto m = R * (M1 * lat_rad - M2 * std::sin(2 * lat_rad) + M3 * std::sin(4 * lat_rad) -
                      M4 * std::sin(6 * lat_rad));

  const auto easting = K0 * n *
                         (a + a3 / 6 * (1 - lat_tan2 + c) +
                          a5 / 120 * (5 - 18 * lat_tan2 + lat_tan4 + 72 * c - 58 * E_P2)) +
                       500000;

  auto northing = K0 * (m + n * lat_tan *
                              (a2 / 2 + a4 / 24 * (5 - lat_tan2 + 9 * c + 4 * c * c) +
                               a6 / 720 * (61 - 58 * lat_tan2 + lat_tan4 + 600 * c - 330 * E_P2)));

  if (p_target.lat < 0) {
    northing += 10000000;
  }

  return {easting, northing};
}

// bool start_car_state_loop(const uint32_t interval, CarState *car_state){
//     bool success = false;
//     if (car_state != NULL)
//     {
//         car_state->interval = interval;
//         success = Embo_PeriodicTask_start(&(car_state->periodicTask), interval, 0U);
//         if (!success)
//         {
//             printf("Failed to start car_state loop.\n");
//         }
//     }
//     return success;
// }

// bool Embo_PeriodicTask_start(Embo_PeriodicTask *periodicTask,
//                              const Embo_Duration period,
//                              const Embo_Duration initialDelay)
// {
//     bool result = false;

//     if (NULL != periodicTask)
//     {
//         result = Embo_Time_startPeriodicTimer(&(periodicTask->timer), period, initialDelay);
//     }

//     return result;
// }

void SimpleProdriverPlanningSimulator::publish_control_mode_report()
{
  ControlModeReport msg;
  msg.stamp = get_clock()->now();
  if (current_control_mode_.data == ControlMode::AUTO) {
    msg.mode = ControlModeReport::AUTONOMOUS;
  } else {
    msg.mode = ControlModeReport::MANUAL;
  }
  pub_control_mode_report_->publish(msg);
}

void SimpleProdriverPlanningSimulator::publish_gear_report()
{
  GearReport msg;
  msg.stamp = get_clock()->now();
  msg.report = vehicle_model_ptr_->getGear();
  pub_gear_report_->publish(msg);
}

void SimpleProdriverPlanningSimulator::publish_turn_indicators_report()
{
  if (!current_turn_indicators_cmd_ptr_) {
    return;
  }
  TurnIndicatorsReport msg;
  msg.stamp = get_clock()->now();
  msg.report = current_turn_indicators_cmd_ptr_->command;
  pub_turn_indicators_report_->publish(msg);
}

void SimpleProdriverPlanningSimulator::publish_hazard_lights_report()
{
  if (!current_hazard_lights_cmd_ptr_) {
    return;
  }
  HazardLightsReport msg;
  msg.stamp = get_clock()->now();
  msg.report = current_hazard_lights_cmd_ptr_->command;
  pub_hazard_lights_report_->publish(msg);
}

void SimpleProdriverPlanningSimulator::publish_tf(const Odometry & odometry)
{
  TransformStamped tf;
  tf.header.stamp = get_clock()->now();
  tf.header.frame_id = origin_frame_id_;
  tf.child_frame_id = simulated_frame_id_;
  tf.transform.translation.x = odometry.pose.pose.position.x;
  tf.transform.translation.y = odometry.pose.pose.position.y;
  tf.transform.translation.z = odometry.pose.pose.position.z;
  tf.transform.rotation = odometry.pose.pose.orientation;

  tf2_msgs::msg::TFMessage tf_msg{};
  tf_msg.transforms.emplace_back(std::move(tf));
  pub_tf_->publish(tf_msg);
}
}  // namespace simple_prodriver_planning_simulator
}  // namespace simulation

RCLCPP_COMPONENTS_REGISTER_NODE(simulation::simple_prodriver_planning_simulator::SimpleProdriverPlanningSimulator)
