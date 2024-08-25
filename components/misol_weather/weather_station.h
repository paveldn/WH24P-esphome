#pragma once

#include <chrono>
#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif

namespace esphome {
namespace misol_weather {

enum class PacketType {
  WRONG_PACKET = -1,
  BASIC_PACKET = 0,
  BASIC_WITH_PRESSURE,
};

class WeatherStation : public Component, public uart::UARTDevice {
#ifdef USE_SENSOR
  SUB_SENSOR(temperature)
  SUB_SENSOR(humidity)
  SUB_SENSOR(pressure)
  SUB_SENSOR(wind_speed)
  SUB_SENSOR(wind_gust)
  SUB_SENSOR(wind_direction_degrees)
  SUB_SENSOR(accumulated_precipitation)
  SUB_SENSOR(uv_intensity)
  SUB_SENSOR(uv_index)
  SUB_SENSOR(light)
  SUB_SENSOR(precipitation_intensity)
#endif
#ifdef USE_BINARY_SENSOR
  SUB_BINARY_SENSOR(battery_level)
  SUB_BINARY_SENSOR(night)
  void set_upper_night_threshold(float upper_night_threshold) { this->upper_night_threshold_ = upper_night_threshold; };
  void set_lower_night_threshold(float lower_night_threshold) { this->lower_night_threshold_ = lower_night_threshold; };
#endif
#if defined(USE_SENSOR)
  void set_precipitation_intensity_interval(unsigned int precipitation_intensity_interval) {
    this->precipitation_intensity_interval_ = std::chrono::minutes(precipitation_intensity_interval);
  }
#endif  // USE_SENSOR
 public:
  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void loop() override;

 protected:
  PacketType check_packet_(const uint8_t *data, size_t len);
  void process_packet_(const uint8_t *data, size_t len, bool has_pressure,
                       const std::chrono::steady_clock::time_point &now);
  void reset_sub_entities_();
  bool first_data_received_{false};
  std::chrono::steady_clock::time_point last_packet_time_;
#if defined(USE_SENSOR)
  std::chrono::milliseconds precipitation_intensity_interval_{std::chrono::minutes(5)};
  std::chrono::steady_clock::time_point previous_precipitation_timestamp_;
  esphome::optional<uint16_t> previous_precipitation_{};
#endif  // USE_SENSOR
#ifdef USE_TEXT_SENSOR
  int north_correction_{0};
  bool secondary_intercardinal_direction_{false};
#endif
#ifdef USE_BINARY_SENSOR
  float upper_night_threshold_{5.5};
  float lower_night_threshold_{4.5};
#endif // USE_BINARY_SENSOR
};

}  // namespace misol_weather
}  // namespace esphome
