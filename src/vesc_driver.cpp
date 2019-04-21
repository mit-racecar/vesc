// -*- mode:c++; fill-column: 100; -*-

#include "vesc_driver/vesc_driver.h"

#include <atomic>
#include <cassert>
#include <cmath>
#include <sstream>

#include <boost/bind.hpp>
#include <vesc/VescStateStamped.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

static const bool kDebug = false;

namespace {

template <typename T>
inline bool getRequiredParam(const ros::NodeHandle& nh,
                             std::string name,
                             T& value) {
  if (nh.getParam(name, value)) return true;
  ROS_FATAL("AckermannToVesc: Parameter %s is required.", name.c_str());
  return false;
}

}  // namespace

namespace vesc_driver
{

VescDriver::VescDriver(ros::NodeHandle nh,
                       ros::NodeHandle private_nh) :
    vesc_(std::string(),
          boost::bind(&VescDriver::vescPacketCallback, this, _1),
          boost::bind(&VescDriver::vescErrorCallback, this, _1)),
    duty_cycle_limit_(private_nh, "duty_cycle", -1.0, 1.0),
    current_limit_(private_nh, "current"),
    brake_limit_(private_nh, "brake"),
    speed_limit_(private_nh, "speed"),
    position_limit_(private_nh, "position"),
    servo_limit_(private_nh, "servo", 0.0, 1.0),
    driver_mode_(MODE_INITIALIZING),
    fw_version_major_(-1),
    fw_version_minor_(-1),
    allow_low_level_commands_(false),
    t_last_command_(0),
    last_speed_command_(0) {
  // get vesc serial port address
  std::string port;
  if (!private_nh.getParam("port", port)) {
    ROS_FATAL("VESC communication port parameter required.");
    ros::shutdown();
    return;
  }

  // This is an optional parameter.
  if (!private_nh.getParam("allow_low_level_commands_",
                           allow_low_level_commands_)) {
    allow_low_level_commands_ = false;
  }

  // Load conversion parameters.
  if (!getRequiredParam(private_nh,
                        "speed_to_erpm_gain",
                        speed_to_erpm_gain_) ||
      !getRequiredParam(private_nh,
                        "speed_to_erpm_offset",
                        speed_to_erpm_offset_) ||
      !getRequiredParam(private_nh,
                        "steering_angle_to_servo_gain",
                        steering_to_servo_gain_) ||
      !getRequiredParam(private_nh,
                        "steering_angle_to_servo_offset",
                        steering_to_servo_offset_)) {
    ros::shutdown();
    return;
  }

  // attempt to connect to the serial port
  try {
    if (kDebug) printf("CONNECT\n");
    vesc_.connect(port);
  } catch (SerialException e) {
    ROS_FATAL("Failed to connect to the VESC, %s.", e.what());
    ros::shutdown();
    return;
  }
  if (kDebug) printf("CONNECTED\n");
  // create vesc state (telemetry) publisher
  state_pub_ = nh.advertise<vesc::VescStateStamped>("sensors/core", 10);

  // since vesc state does not include the servo position, publish the commanded
  // servo position as a "sensor"
  servo_sensor_pub_ =
      nh.advertise<std_msgs::Float64>("sensors/servo_position_command", 10);

  // create ackermann subscriber.
  ackermann_sub_ = nh.subscribe(
      "commands/ackermann", 10, &VescDriver::ackermannCmdCallback, this);

  if (allow_low_level_commands_) {
    ROS_INFO("Low-level commands enabled.");
    duty_cycle_sub_ = nh.subscribe(
        "commands/motor/duty_cycle", 10,&VescDriver::dutyCycleCallback, this);
    current_sub_ = nh.subscribe(
        "commands/motor/current", 10, &VescDriver::currentCallback, this);
    brake_sub_ = nh.subscribe(
        "commands/motor/brake", 10, &VescDriver::brakeCallback, this);
    position_sub_ = nh.subscribe(
        "commands/motor/position", 10, &VescDriver::positionCallback, this);
    speed_sub_ = nh.subscribe(
        "commands/motor/speed", 10, &VescDriver::speedCallback, this);
    servo_sub_ = nh.subscribe(
        "commands/servo/position", 10, &VescDriver::servoCallback, this);
  }

  if (kDebug) printf("TIMER START\n");
  // create a 50Hz timer, used for state machine & polling VESC telemetry
  timer_ = nh.createSteadyTimer(ros::WallDuration(0.02),
                                &VescDriver::timerCallback,
                                this);
  if (kDebug) printf("DONE INIT\n");
}

void VescDriver::checkCommandTimeout() {
  static const double kTimeout = 0.5;
  static const double kDecelRate = 1000;
  const double t_now = ros::WallTime::now().toSec();
  if (t_now > t_last_command_ + kTimeout) {
    double speed = last_speed_command_;
    if (speed != 0) {
      printf("Safety engaged.\n");
      if (fabs(speed > kDecelRate)) {
        if (speed > 0.0) {
          speed = speed - kDecelRate;
        } else {
          speed = speed + kDecelRate;
        }
      } else {
        speed = 0;
      }
      vesc_.setSpeed(speed);
      last_speed_command_ = speed;
    }
  }
}

  /* TODO or TO-THINKABOUT LIST
    - what should we do on startup? send brake or zero command?
    - what to do if the vesc interface gives an error?
    - check version number against know compatable?
    - should we wait until we receive telemetry before sending commands?
    - should we track the last motor command
    - what to do if no motor command received recently?
    - what to do if no servo command received recently?
    - what is the motor safe off state (0 current?)
    - what to do if a command parameter is out of range, ignore?
    - try to predict vesc bounds (from vesc config) and command detect bounds errors
  */

void VescDriver::timerCallback(const ros::SteadyTimerEvent& event) {
  static const double kMaxInitPeriod = 2.0;
  static const double kTStart = ros::WallTime::now().toSec();

  if (kDebug) printf("TIMER CALLBACK\n");
  checkCommandTimeout();
  // VESC interface should not unexpectedly disconnect, but test for it anyway
  if (!vesc_.isConnected()) {
    ROS_FATAL("Unexpectedly disconnected from serial port.");
    timer_.stop();
    ros::shutdown();
    return;
  }

  /*
   * Driver state machine, modes:
   *  INITIALIZING - request and wait for vesc version
   *  OPERATING - receiving commands from subscriber topics
   */
  if (driver_mode_ == MODE_INITIALIZING) {
    if (ros::WallTime::now().toSec() > kTStart + kMaxInitPeriod) {
      ROS_FATAL("FAIL: Timed out while trying to initialize VESC.");
      ros::shutdown();
      return;
    }
    if (kDebug) printf("INITIALIZING\n");
    // request version number, return packet will update the internal version numbers
    vesc_.requestFWVersion();
    if (fw_version_major_ >= 0 && fw_version_minor_ >= 0) {
      ROS_INFO("Connected to VESC with firmware version %d.%d",
               fw_version_major_, fw_version_minor_);
      driver_mode_ = MODE_OPERATING;
    }
  }
  else if (driver_mode_ == MODE_OPERATING) {
    if (kDebug) printf("OPERATING\n");
    // poll for vesc state (telemetry)
    vesc_.requestState();
  }
  else {
    if (kDebug) printf("FAIL: UNKNOWN STATE!\n");
    // unknown mode, how did that happen?
    assert(false && "unknown driver mode");
  }
}

void VescDriver::vescPacketCallback(const boost::shared_ptr<VescPacket const>& packet)
{
  if (packet->name() == "Values") {
    boost::shared_ptr<VescPacketValues const> values =
      boost::dynamic_pointer_cast<VescPacketValues const>(packet);

    vesc::VescStateStamped::Ptr state_msg(new vesc::VescStateStamped);
    state_msg->header.stamp = ros::Time::now();
    state_msg->state.voltage_input = values->v_in();
    state_msg->state.temperature_pcb = values->temp_pcb();
    state_msg->state.current_motor = values->current_motor();
    state_msg->state.current_input = values->current_in();
    state_msg->state.speed = values->rpm();
    state_msg->state.duty_cycle = values->duty_now();
    state_msg->state.charge_drawn = values->amp_hours();
    state_msg->state.charge_regen = values->amp_hours_charged();
    state_msg->state.energy_drawn = values->watt_hours();
    state_msg->state.energy_regen = values->watt_hours_charged();
    state_msg->state.displacement = values->tachometer();
    state_msg->state.distance_traveled = values->tachometer_abs();
    state_msg->state.fault_code = values->fault_code();

    state_pub_.publish(state_msg);
  }
  else if (packet->name() == "FWVersion") {
    boost::shared_ptr<VescPacketFWVersion const> fw_version =
      boost::dynamic_pointer_cast<VescPacketFWVersion const>(packet);
    // todo: might need lock here
    fw_version_major_ = fw_version->fwMajor();
    fw_version_minor_ = fw_version->fwMinor();
  }
}

void VescDriver::vescErrorCallback(const std::string& error)
{
  ROS_ERROR("%s", error.c_str());
}

/**
 * @param duty_cycle Commanded VESC duty cycle. Valid range for this driver is -1 to +1. However,
 *                   note that the VESC may impose a more restrictive bounds on the range depending
 *                   on its configuration, e.g. absolute value is between 0.05 and 0.95.
 */
void VescDriver::dutyCycleCallback(const std_msgs::Float64::ConstPtr& duty_cycle)
{
  if (driver_mode_ == MODE_OPERATING) {
    vesc_.setDutyCycle(duty_cycle_limit_.clip(duty_cycle->data));
  }
}

/**
 * @param current Commanded VESC current in Amps. Any value is accepted by this driver. However,
 *                note that the VESC may impose a more restrictive bounds on the range depending on
 *                its configuration.
 */
void VescDriver::currentCallback(const std_msgs::Float64::ConstPtr& current)
{
  if (driver_mode_ == MODE_OPERATING) {
    vesc_.setCurrent(current_limit_.clip(current->data));
  }
}

/**
 * @param brake Commanded VESC braking current in Amps. Any value is accepted by this driver.
 *              However, note that the VESC may impose a more restrictive bounds on the range
 *              depending on its configuration.
 */
void VescDriver::brakeCallback(const std_msgs::Float64::ConstPtr& brake)
{
  if (driver_mode_ == MODE_OPERATING) {
    vesc_.setBrake(brake_limit_.clip(brake->data));
  }
}

/**
 * @param speed Commanded VESC speed in electrical RPM. Electrical RPM is the mechanical RPM
 *              multiplied by the number of motor poles. Any value is accepted by this
 *              driver. However, note that the VESC may impose a more restrictive bounds on the
 *              range depending on its configuration.
 */
void VescDriver::speedCallback(const std_msgs::Float64::ConstPtr& speed)
{
  if (kDebug) printf("Desired speed: %f\n", speed->data);
  if (driver_mode_ == MODE_OPERATING) {
    vesc_.setSpeed(speed_limit_.clip(speed->data));
    t_last_command_ = ros::WallTime::now().toSec();
    last_speed_command_ = speed->data;
  }
}

/**
 * @param position Commanded VESC motor position in radians. Any value is accepted by this driver.
 *                 Note that the VESC must be in encoder mode for this command to have an effect.
 */
void VescDriver::positionCallback(const std_msgs::Float64::ConstPtr& position)
{
  if (driver_mode_ == MODE_OPERATING) {
    // ROS uses radians but VESC seems to use degrees. Convert to degrees.
    double position_deg = position_limit_.clip(position->data) * 180.0 / M_PI;
    vesc_.setPosition(position_deg);
  }
}

/**
 * @param servo Commanded VESC servo output position. Valid range is 0 to 1.
 */
void VescDriver::servoCallback(const std_msgs::Float64::ConstPtr& servo)
{
  if (driver_mode_ == MODE_OPERATING) {
    double servo_clipped(servo_limit_.clip(servo->data));
    vesc_.setServo(servo_clipped);
    // publish clipped servo value as a "sensor"
    std_msgs::Float64::Ptr servo_sensor_msg(new std_msgs::Float64);
    servo_sensor_msg->data = servo_clipped;
    servo_sensor_pub_.publish(servo_sensor_msg);
  }
}

/**
 * @param cmd Ackermann steering command.
 */
void VescDriver::ackermannCmdCallback(
  const ackermann_msgs::AckermannDriveStamped::ConstPtr& cmd) {
  // calc vesc electric RPM (speed)
  const double erpm =
      speed_to_erpm_gain_ * cmd->drive.speed + speed_to_erpm_offset_;

  // calc steering angle (servo)
  const double servo = steering_to_servo_gain_ * cmd->drive.steering_angle +
    steering_to_servo_offset_;

  if (driver_mode_ == MODE_OPERATING) {
    // Set speed command.
    vesc_.setSpeed(speed_limit_.clip(erpm));
    t_last_command_ = ros::WallTime::now().toSec();
    last_speed_command_ = erpm;

    // Set servo position command.
    const double servo_clipped(servo_limit_.clip(servo));
    vesc_.setServo(servo_clipped);
    // publish clipped servo value as a "sensor"
    std_msgs::Float64::Ptr servo_sensor_msg(new std_msgs::Float64);
    servo_sensor_msg->data = servo_clipped;
    servo_sensor_pub_.publish(servo_sensor_msg);
  }
}

VescDriver::CommandLimit::CommandLimit(const ros::NodeHandle& nh, const std::string& str,
                                       const boost::optional<double>& min_lower,
                                       const boost::optional<double>& max_upper) :
  name(str)
{
  // check if user's minimum value is outside of the range min_lower to max_upper
  double param_min;
  if (nh.getParam(name + "_min", param_min)) {
    if (min_lower && param_min < *min_lower) {
      lower = *min_lower;
      ROS_WARN_STREAM("Parameter " << name << "_min (" << param_min <<
                      ") is less than the feasible minimum (" << *min_lower << ").");
    }
    else if (max_upper && param_min > *max_upper) {
      lower = *max_upper;
      ROS_WARN_STREAM("Parameter " << name << "_min (" << param_min <<
                      ") is greater than the feasible maximum (" << *max_upper << ").");
    }
    else {
      lower = param_min;
    }
  }
  else if (min_lower) {
    lower = *min_lower;
  }

  // check if the uers' maximum value is outside of the range min_lower to max_upper
  double param_max;
  if (nh.getParam(name + "_max", param_max)) {
    if (min_lower && param_max < *min_lower) {
      upper = *min_lower;
      ROS_WARN_STREAM("Parameter " << name << "_max (" << param_max <<
                      ") is less than the feasible minimum (" << *min_lower << ").");
    }
    else if (max_upper && param_max > *max_upper) {
      upper = *max_upper;
      ROS_WARN_STREAM("Parameter " << name << "_max (" << param_max <<
                      ") is greater than the feasible maximum (" << *max_upper << ").");
    }
    else {
      upper = param_max;
    }
  }
  else if (max_upper) {
    upper = *max_upper;
  }

  // check for min > max
  if (upper && lower && *lower > *upper) {
    ROS_WARN_STREAM("Parameter " << name << "_max (" << *upper
                    << ") is less than parameter " << name << "_min (" << *lower << ").");
    double temp(*lower);
    lower = *upper;
    upper = temp;
  }

  std::ostringstream oss;
  oss << "  " << name << " limit: ";
  if (lower) oss << *lower << " "; else oss << "(none) ";
  if (upper) oss << *upper; else oss << "(none)";
  ROS_DEBUG_STREAM(oss.str());
}

double VescDriver::CommandLimit::clip(double value)
{
  if (lower && value < lower) {
    ROS_INFO_THROTTLE(10, "%s command value (%f) below minimum limit (%f), clipping.",
                      name.c_str(), value, *lower);
    return *lower;
  }
  if (upper && value > upper) {
    ROS_INFO_THROTTLE(10, "%s command value (%f) above maximum limit (%f), clipping.",
                      name.c_str(), value, *upper);
    return *upper;
  }
  return value;
}


} // namespace vesc_driver
