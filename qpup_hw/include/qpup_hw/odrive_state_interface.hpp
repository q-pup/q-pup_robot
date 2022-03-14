#pragma once
#include <hardware_interface/internal/hardware_resource_manager.h>

#include <string>
namespace qpup_hw {
class OdriveStateHandle {
 public:
  OdriveStateHandle() = default;

  OdriveStateHandle(const std::string& name, const uint32_t* axis_error, const uint8_t* axis_state,
                    const uint8_t* motor_flags, const uint8_t* encoder_flags, const uint8_t* controller_flags,
                    const uint32_t* motor_error, const uint32_t* encoder_error, const uint32_t* sensorless_error,
                    const uint32_t* shadow_count, const uint32_t* count_in_cpr, const float* iq_setpoint,
                    const float* iq_measured, const float* vbus_voltage, std::atomic_flag* do_not_clear_errors_flag)
      : name_(name),
        axis_error_(axis_error),
        axis_state_(axis_state),
        motor_flags_(motor_flags),
        encoder_flags_(encoder_flags),
        controller_flags_(controller_flags),
        motor_error_(motor_error),
        encoder_error_(encoder_error),
        sensorless_error_(sensorless_error),
        shadow_count_(shadow_count),
        count_in_cpr_(count_in_cpr),
        iq_setpoint_(iq_setpoint),
        iq_measured_(iq_measured),
        vbus_voltage_(vbus_voltage),
        do_not_clear_errors_flag_(do_not_clear_errors_flag) {}

  std::string getName() const {
    return name_;
  }

  uint32_t getAxisError() const {
    return *axis_error_;
  }
  uint8_t getAxisState() const {
    return *axis_state_;
  }
  uint8_t getMotorFlags() const {
    return *motor_flags_;
  }
  uint8_t getEncoderFlags() const {
    return *encoder_flags_;
  }
  uint8_t getControllerFlags() const {
    return *controller_flags_;
  }
  uint32_t getMotorError() const {
    return *motor_error_;
  }
  uint32_t getEncoderError() const {
    return *encoder_error_;
  }
  uint32_t getSensorlessError() const {
    return *sensorless_error_;
  }
  uint32_t getShadowCount() const {
    return *shadow_count_;
  }
  uint32_t getCountInCPR() const {
    return *count_in_cpr_;
  }
  float getIQSetpoint() const {
    return *iq_setpoint_;
  }
  float getIQMeasured() const {
    return *iq_measured_;
  }
  float getVbusVoltage() const {
    return *vbus_voltage_;
  }

  void triggerClearErrors() {
    do_not_clear_errors_flag_->clear();
  }

 private:
  std::string name_;

  const uint32_t* axis_error_ = {nullptr};
  const uint8_t* axis_state_ = {nullptr};
  const uint8_t* motor_flags_ = {nullptr};
  const uint8_t* encoder_flags_ = {nullptr};
  const uint8_t* controller_flags_ = {nullptr};

  const uint32_t* motor_error_ = {nullptr};
  const uint32_t* encoder_error_ = {nullptr};
  const uint32_t* sensorless_error_ = {nullptr};

  const uint32_t* shadow_count_ = {nullptr};
  const uint32_t* count_in_cpr_ = {nullptr};

  const float* iq_setpoint_ = {nullptr};
  const float* iq_measured_ = {nullptr};

  const float* vbus_voltage_ = {nullptr};

  std::atomic_flag* do_not_clear_errors_flag_ = {nullptr};
};

class OdriveStateInterface : public hardware_interface::HardwareResourceManager<OdriveStateHandle> {};
}  // namespace qpup_hw
