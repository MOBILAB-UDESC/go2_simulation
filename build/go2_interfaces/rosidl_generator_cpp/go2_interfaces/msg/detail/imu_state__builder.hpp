// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from go2_interfaces:msg/IMUState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "go2_interfaces/msg/imu_state.hpp"


#ifndef GO2_INTERFACES__MSG__DETAIL__IMU_STATE__BUILDER_HPP_
#define GO2_INTERFACES__MSG__DETAIL__IMU_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "go2_interfaces/msg/detail/imu_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace go2_interfaces
{

namespace msg
{

namespace builder
{

class Init_IMUState_temperature
{
public:
  explicit Init_IMUState_temperature(::go2_interfaces::msg::IMUState & msg)
  : msg_(msg)
  {}
  ::go2_interfaces::msg::IMUState temperature(::go2_interfaces::msg::IMUState::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return std::move(msg_);
  }

private:
  ::go2_interfaces::msg::IMUState msg_;
};

class Init_IMUState_rpy
{
public:
  explicit Init_IMUState_rpy(::go2_interfaces::msg::IMUState & msg)
  : msg_(msg)
  {}
  Init_IMUState_temperature rpy(::go2_interfaces::msg::IMUState::_rpy_type arg)
  {
    msg_.rpy = std::move(arg);
    return Init_IMUState_temperature(msg_);
  }

private:
  ::go2_interfaces::msg::IMUState msg_;
};

class Init_IMUState_accelerometer
{
public:
  explicit Init_IMUState_accelerometer(::go2_interfaces::msg::IMUState & msg)
  : msg_(msg)
  {}
  Init_IMUState_rpy accelerometer(::go2_interfaces::msg::IMUState::_accelerometer_type arg)
  {
    msg_.accelerometer = std::move(arg);
    return Init_IMUState_rpy(msg_);
  }

private:
  ::go2_interfaces::msg::IMUState msg_;
};

class Init_IMUState_gyroscope
{
public:
  explicit Init_IMUState_gyroscope(::go2_interfaces::msg::IMUState & msg)
  : msg_(msg)
  {}
  Init_IMUState_accelerometer gyroscope(::go2_interfaces::msg::IMUState::_gyroscope_type arg)
  {
    msg_.gyroscope = std::move(arg);
    return Init_IMUState_accelerometer(msg_);
  }

private:
  ::go2_interfaces::msg::IMUState msg_;
};

class Init_IMUState_quaternion
{
public:
  Init_IMUState_quaternion()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_IMUState_gyroscope quaternion(::go2_interfaces::msg::IMUState::_quaternion_type arg)
  {
    msg_.quaternion = std::move(arg);
    return Init_IMUState_gyroscope(msg_);
  }

private:
  ::go2_interfaces::msg::IMUState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::go2_interfaces::msg::IMUState>()
{
  return go2_interfaces::msg::builder::Init_IMUState_quaternion();
}

}  // namespace go2_interfaces

#endif  // GO2_INTERFACES__MSG__DETAIL__IMU_STATE__BUILDER_HPP_
