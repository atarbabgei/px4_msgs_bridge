#include "px4_bridge.hpp"
#include "frame_conversions.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <limits>
#include <rclcpp_components/register_node_macro.hpp>

namespace px4_bridge {

static rclcpp::QoS px4Qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(5))
    .best_effort()
    .transient_local();
}

static rclcpp::QoS rosQos()
{
  return rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)).keep_last(5);
}

// PX4 uXRCE-DDS (UXRCE_DDS_SYNCT=1) converts all timestamps to companion
// wall clock before serialization. We just convert microseconds -> nanoseconds.
rclcpp::Time Px4Bridge::toRosTime(uint64_t px4_timestamp_us)
{
  return rclcpp::Time(static_cast<int64_t>(px4_timestamp_us) * 1000, RCL_SYSTEM_TIME);
}

Px4Bridge::Px4Bridge(const rclcpp::NodeOptions& options)
  : Node("px4_bridge", options)
{
  const std::string px4_ns = declare_parameter("px4_namespace", "");
  namespace_ = declare_parameter("namespace", "vehicle");
  const double odom_rate = declare_parameter("odom_rate", 50.0);
  const bool enable_tf = declare_parameter("enable_tf", true);
  const bool enable_external_odom = declare_parameter("enable_external_odom", false);
  const std::string ext_odom_topic =
    declare_parameter("external_odom_topic", "/odom/sample");

  // PX4 topic prefix: "" -> "/fmu", "drone_0" -> "/drone_0/fmu"
  const std::string fmu = px4_ns.empty() ? "/fmu" : "/" + px4_ns + "/fmu";

  odom_frame_ = namespace_ + "/odom";
  base_link_frame_ = namespace_ + "/base_link";

  odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
    fmu + "/out/vehicle_odometry", px4Qos(),
    std::bind(&Px4Bridge::odomCallback, this, std::placeholders::_1));

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
    "/" + namespace_ + "/odom", rosQos());

  if (enable_tf) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  const auto period = std::chrono::duration<double>(1.0 / odom_rate);
  odom_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&Px4Bridge::odomTimerCallback, this));

  imu_sub_ = create_subscription<px4_msgs::msg::SensorCombined>(
    fmu + "/out/sensor_combined", px4Qos(),
    std::bind(&Px4Bridge::imuCallback, this, std::placeholders::_1));

  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
    "/" + namespace_ + "/imu", rosQos());

  gps_sub_ = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
    fmu + "/out/vehicle_global_position", px4Qos(),
    std::bind(&Px4Bridge::gpsCallback, this, std::placeholders::_1));

  trigger_sub_ = create_subscription<px4_msgs::msg::CameraTrigger>(
    fmu + "/out/camera_trigger", px4Qos(),
    std::bind(&Px4Bridge::triggerCallback, this, std::placeholders::_1));

  trigger_pub_ = create_publisher<sensor_msgs::msg::TimeReference>(
    "/" + namespace_ + "/camera_trigger", rosQos());

  if (enable_external_odom) {
    ext_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      ext_odom_topic, rclcpp::QoS(5).reliable(),
      std::bind(&Px4Bridge::externalOdomCallback, this, std::placeholders::_1));

    vio_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
      fmu + "/in/vehicle_visual_odometry", px4Qos());
  }

  RCLCPP_INFO(get_logger(), "PX4 Bridge started (px4=%s, ns=%s)",
    px4_ns.empty() ? "(default)" : px4_ns.c_str(), namespace_.c_str());
  RCLCPP_INFO(get_logger(), "  %s/out/vehicle_odometry -> /%s/odom (%.0fHz)", fmu.c_str(), namespace_.c_str(), odom_rate);
  RCLCPP_INFO(get_logger(), "  %s/out/sensor_combined  -> /%s/imu", fmu.c_str(), namespace_.c_str());
  RCLCPP_INFO(get_logger(), "  %s/out/vehicle_global_position -> /%s/gps (on fix)", fmu.c_str(), namespace_.c_str());
  RCLCPP_INFO(get_logger(), "  %s/out/camera_trigger -> /%s/camera_trigger", fmu.c_str(), namespace_.c_str());
  if (tf_broadcaster_)
    RCLCPP_INFO(get_logger(), "  TF: %s -> %s", odom_frame_.c_str(), base_link_frame_.c_str());
  if (enable_external_odom)
    RCLCPP_INFO(get_logger(), "  %s -> %s/in/vehicle_visual_odometry", ext_odom_topic.c_str(), fmu.c_str());
}

void Px4Bridge::odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
  latest_odom_ = msg;
}

void Px4Bridge::odomTimerCallback()
{
  if (!latest_odom_) {
    return;
  }

  const auto& src = *latest_odom_;
  const auto stamp = toRosTime(src.timestamp_sample);

  const Eigen::Vector3d pos_enu = positionNedToEnu(
    Eigen::Vector3d(src.position[0], src.position[1], src.position[2]));

  const Eigen::Quaterniond q_ned(src.q[0], src.q[1], src.q[2], src.q[3]);
  const Eigen::Quaterniond q_enu = attitudeNedToEnu(q_ned);

  // PX4 sends velocity in NED world frame; convert to body-frame FLU
  // for nav_msgs/Odometry (twist must be in child_frame per REP-105).
  const Eigen::Vector3d vel_ned(src.velocity[0], src.velocity[1], src.velocity[2]);
  const Eigen::Vector3d vel_enu = positionNedToEnu(vel_ned);
  const Eigen::Vector3d vel_body = q_enu.inverse() * vel_enu;

  const Eigen::Vector3d angvel_flu = frdToFlu(
    Eigen::Vector3d(src.angular_velocity[0], src.angular_velocity[1], src.angular_velocity[2]));

  const Eigen::Vector3d pos_var_enu = varianceNedToEnu(
    Eigen::Vector3d(src.position_variance[0], src.position_variance[1], src.position_variance[2]));
  const Eigen::Vector3d ori_var_enu = varianceNedToEnu(
    Eigen::Vector3d(src.orientation_variance[0], src.orientation_variance[1], src.orientation_variance[2]));
  const Eigen::Vector3d vel_var_enu = varianceNedToEnu(
    Eigen::Vector3d(src.velocity_variance[0], src.velocity_variance[1], src.velocity_variance[2]));

  auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
  odom_msg->header.stamp = stamp;
  odom_msg->header.frame_id = odom_frame_;
  odom_msg->child_frame_id = base_link_frame_;

  odom_msg->pose.pose.position.x = pos_enu.x();
  odom_msg->pose.pose.position.y = pos_enu.y();
  odom_msg->pose.pose.position.z = pos_enu.z();
  odom_msg->pose.pose.orientation.w = q_enu.w();
  odom_msg->pose.pose.orientation.x = q_enu.x();
  odom_msg->pose.pose.orientation.y = q_enu.y();
  odom_msg->pose.pose.orientation.z = q_enu.z();

  odom_msg->pose.covariance[0] = pos_var_enu.x();
  odom_msg->pose.covariance[7] = pos_var_enu.y();
  odom_msg->pose.covariance[14] = pos_var_enu.z();
  odom_msg->pose.covariance[21] = ori_var_enu.x();
  odom_msg->pose.covariance[28] = ori_var_enu.y();
  odom_msg->pose.covariance[35] = ori_var_enu.z();

  odom_msg->twist.twist.linear.x = vel_body.x();
  odom_msg->twist.twist.linear.y = vel_body.y();
  odom_msg->twist.twist.linear.z = vel_body.z();
  odom_msg->twist.twist.angular.x = angvel_flu.x();
  odom_msg->twist.twist.angular.y = angvel_flu.y();
  odom_msg->twist.twist.angular.z = angvel_flu.z();

  odom_msg->twist.covariance[0] = vel_var_enu.x();
  odom_msg->twist.covariance[7] = vel_var_enu.y();
  odom_msg->twist.covariance[14] = vel_var_enu.z();

  if (tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header = odom_msg->header;
    tf.child_frame_id = base_link_frame_;
    tf.transform.translation.x = odom_msg->pose.pose.position.x;
    tf.transform.translation.y = odom_msg->pose.pose.position.y;
    tf.transform.translation.z = odom_msg->pose.pose.position.z;
    tf.transform.rotation = odom_msg->pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
  }

  odom_pub_->publish(std::move(odom_msg));
}

void Px4Bridge::imuCallback(const px4_msgs::msg::SensorCombined::SharedPtr msg)
{
  const auto stamp = toRosTime(msg->timestamp);

  const Eigen::Vector3d gyro_flu = frdToFlu(
    Eigen::Vector3d(msg->gyro_rad[0], msg->gyro_rad[1], msg->gyro_rad[2]));
  const Eigen::Vector3d accel_flu = frdToFlu(
    Eigen::Vector3d(msg->accelerometer_m_s2[0], msg->accelerometer_m_s2[1], msg->accelerometer_m_s2[2]));

  auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
  imu_msg->header.stamp = stamp;
  imu_msg->header.frame_id = base_link_frame_;

  imu_msg->orientation.w = 1.0;
  imu_msg->orientation_covariance[0] = -1.0;

  imu_msg->angular_velocity.x = gyro_flu.x();
  imu_msg->angular_velocity.y = gyro_flu.y();
  imu_msg->angular_velocity.z = gyro_flu.z();

  imu_msg->linear_acceleration.x = accel_flu.x();
  imu_msg->linear_acceleration.y = accel_flu.y();
  imu_msg->linear_acceleration.z = accel_flu.z();

  imu_pub_->publish(std::move(imu_msg));
}

void Px4Bridge::gpsCallback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
{
  if (!msg->lat_lon_valid || !msg->alt_valid) {
    return;
  }

  if (!gps_pub_) {
    gps_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(
      "/" + namespace_ + "/gps", rosQos());
    RCLCPP_INFO(get_logger(), "GPS fix acquired, publishing on /%s/gps", namespace_.c_str());
  }

  auto gps_msg = std::make_unique<sensor_msgs::msg::NavSatFix>();
  gps_msg->header.stamp = toRosTime(msg->timestamp);
  gps_msg->header.frame_id = base_link_frame_;

  gps_msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  gps_msg->status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

  gps_msg->latitude = msg->lat;
  gps_msg->longitude = msg->lon;
  gps_msg->altitude = msg->alt;

  gps_msg->position_covariance_type =
    sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  const double eph2 = static_cast<double>(msg->eph) * msg->eph;
  const double epv2 = static_cast<double>(msg->epv) * msg->epv;
  gps_msg->position_covariance[0] = eph2;
  gps_msg->position_covariance[4] = eph2;
  gps_msg->position_covariance[8] = epv2;

  gps_pub_->publish(std::move(gps_msg));
}

void Px4Bridge::triggerCallback(const px4_msgs::msg::CameraTrigger::SharedPtr msg)
{
  auto ref_msg = std::make_unique<sensor_msgs::msg::TimeReference>();
  ref_msg->header.stamp = toRosTime(msg->timestamp);
  ref_msg->header.frame_id = base_link_frame_;

  // timestamp_utc is the trigger time in microseconds UTC
  ref_msg->time_ref = rclcpp::Time(static_cast<int64_t>(msg->timestamp_utc) * 1000, RCL_SYSTEM_TIME);
  ref_msg->source = std::to_string(msg->seq);

  trigger_pub_->publish(std::move(ref_msg));
}

void Px4Bridge::externalOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  auto px4_msg = std::make_unique<px4_msgs::msg::VehicleOdometry>();

  px4_msg->timestamp = static_cast<uint64_t>(
    msg->header.stamp.sec * 1000000ULL + msg->header.stamp.nanosec / 1000ULL);
  px4_msg->timestamp_sample = px4_msg->timestamp;

  // FAST-LIO (and similar LIO/VIO) outputs in body-init FLU frame.
  // Convert FLU (x,y,z) -> FRD (x,-y,-z) for PX4.
  px4_msg->position[0] = static_cast<float>(msg->pose.pose.position.x);
  px4_msg->position[1] = static_cast<float>(-msg->pose.pose.position.y);
  px4_msg->position[2] = static_cast<float>(-msg->pose.pose.position.z);

  // Quaternion FLU -> FRD: keep w,x, negate y,z
  px4_msg->q[0] = static_cast<float>(msg->pose.pose.orientation.w);
  px4_msg->q[1] = static_cast<float>(msg->pose.pose.orientation.x);
  px4_msg->q[2] = static_cast<float>(-msg->pose.pose.orientation.y);
  px4_msg->q[3] = static_cast<float>(-msg->pose.pose.orientation.z);

  px4_msg->pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD;
  px4_msg->velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_FRD;

  // Set velocities to NaN — position-only fusion
  px4_msg->velocity[0] = std::numeric_limits<float>::quiet_NaN();
  px4_msg->velocity[1] = std::numeric_limits<float>::quiet_NaN();
  px4_msg->velocity[2] = std::numeric_limits<float>::quiet_NaN();
  px4_msg->angular_velocity[0] = std::numeric_limits<float>::quiet_NaN();
  px4_msg->angular_velocity[1] = std::numeric_limits<float>::quiet_NaN();
  px4_msg->angular_velocity[2] = std::numeric_limits<float>::quiet_NaN();

  // Variance: no axis swap needed for FLU->FRD (variance is always positive)
  const auto& pc = msg->pose.covariance;
  px4_msg->position_variance[0] = static_cast<float>(pc[0]);
  px4_msg->position_variance[1] = static_cast<float>(pc[7]);
  px4_msg->position_variance[2] = static_cast<float>(pc[14]);

  px4_msg->orientation_variance[0] = static_cast<float>(pc[21]);
  px4_msg->orientation_variance[1] = static_cast<float>(pc[28]);
  px4_msg->orientation_variance[2] = static_cast<float>(pc[35]);

  px4_msg->velocity_variance[0] = 0.0f;
  px4_msg->velocity_variance[1] = 0.0f;
  px4_msg->velocity_variance[2] = 0.0f;

  px4_msg->quality = 1;

  vio_pub_->publish(std::move(px4_msg));
}

}  // namespace px4_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(px4_bridge::Px4Bridge)
