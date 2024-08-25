#include "esphome/components/uart/uart.h"
#include "esphome/core/helpers.h"
#include "weather_station.h"
#include <memory>
#include <string>

namespace {

bool detect_night(float uv_intensity, float lower_threshold, float upper_threshold) {
  static bool previous_result = false;
  static bool first_run = true;
  bool result = false;
  if (first_run) {
    result = uv_intensity < ((lower_threshold + upper_threshold) / 2.0f);
    first_run = false;
  } else {
    result = previous_result ? (uv_intensity < upper_threshold) : (uv_intensity < lower_threshold);
  }
  previous_result = result;
  return result;
}

}  // namespace

namespace esphome {
namespace misol_weather {

static const char *const TAG = "misol_weather";
constexpr std::chrono::milliseconds COMMUNICATION_TIMOUT = std::chrono::minutes(2);
constexpr std::chrono::milliseconds PRECIPITATION_INTENSITY_INTERVAL = std::chrono::minutes(3);

void WeatherStation::loop() {
  // Checking timeout
  std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
  if (this->first_data_received_ && (now - this->last_packet_time_ > COMMUNICATION_TIMOUT)) {
    ESP_LOGW(TAG, "Communication timeout");
    this->first_data_received_ = false;
    this->reset_sub_entities_();
  }
  auto size = this->available();
  if (size > 0) {
    std::unique_ptr<uint8_t[]> buffer(new uint8_t[size]);
    for (int i = 0; i < size; i++) {
      buffer[i] = this->read();
    }
    ESP_LOGD(TAG, "%s received: %s", this->first_data_received_ ? "Packet" : "First packet",
             format_hex_pretty(buffer.get(), size).c_str());
    this->first_data_received_ = true;
    this->last_packet_time_ = now;
    PacketType packet_type = check_packet_(buffer.get(), size);
    if (packet_type != PacketType::WRONG_PACKET) {
      this->process_packet_(buffer.get(), size, packet_type == PacketType::BASIC_WITH_PRESSURE, now);
    } else {
      ESP_LOGW(TAG, "Unknown packet received: %s", format_hex_pretty(buffer.get(), size).c_str());
    }
  }
}

void WeatherStation::reset_sub_entities_() {
#ifdef USE_SENSOR
  if (this->temperature_sensor_ != nullptr)
    this->temperature_sensor_->publish_state(NAN);
  if (this->humidity_sensor_ != nullptr)
    this->humidity_sensor_->publish_state(NAN);
  if (this->pressure_sensor_ != nullptr)
    this->pressure_sensor_->publish_state(NAN);
  if (this->wind_speed_sensor_ != nullptr)
    this->wind_speed_sensor_->publish_state(NAN);
  if (this->wind_gust_sensor_ != nullptr)
    this->wind_gust_sensor_->publish_state(NAN);
  if (this->wind_direction_degrees_sensor_ != nullptr)
    this->wind_direction_degrees_sensor_->publish_state(NAN);
  if (this->accumulated_precipitation_sensor_ != nullptr)
    this->accumulated_precipitation_sensor_->publish_state(NAN);
  if (this->uv_intensity_sensor_ != nullptr)
    this->uv_intensity_sensor_->publish_state(NAN);
  if (this->light_sensor_ != nullptr)
    this->light_sensor_->publish_state(NAN);
  if (this->uv_index_sensor_ != nullptr)
    this->uv_index_sensor_->publish_state(NAN);
  if (this->precipitation_intensity_sensor_ != nullptr) {
    this->precipitation_intensity_sensor_->publish_state(NAN);
    this->previous_precipitation_.reset();
  }
#endif  // USE_SENSOR
}

PacketType WeatherStation::check_packet_(const uint8_t *data, size_t len) {
  // Checking basic packet
  if (len < 17) {
    return PacketType::WRONG_PACKET;
  }
  if (data[0] != 0x24) {
    return PacketType::WRONG_PACKET;
  }
  uint8_t checksum = 0;
  for (int i = 0; i < 16; i++) {
    checksum += data[i];
  }
  if (checksum != data[16]) {
    return PacketType::WRONG_PACKET;
  }
  // Checking barometry pressure packet
  if (len > 17) {
    if (len < 21) {
      return PacketType::BASIC_PACKET;
    }
    checksum = 0;
    for (int i = 17; i < 20; i++) {
      checksum += data[i];
    }
    if (checksum != data[20]) {
      return PacketType::BASIC_PACKET;
    }
    return PacketType::BASIC_WITH_PRESSURE;
  }
  return PacketType::BASIC_PACKET;
}

void WeatherStation::process_packet_(const uint8_t *data, size_t len, bool has_pressure,
                                     const std::chrono::steady_clock::time_point &now) {
#ifdef USE_SENSOR
  if (this->pressure_sensor_ != nullptr) {
    if (has_pressure) {
      float pressure = ((((uint32_t) data[17]) << 16) + (((uint32_t) data[18]) << 8) + data[19]) / 100.0f;
      this->pressure_sensor_->publish_state(pressure);
    } else {
      this->pressure_sensor_->publish_state(NAN);
    }
  }
#endif  // USE_SENSOR
#ifdef USE_SENSOR
  if (this->wind_direction_degrees_sensor_ != nullptr) {
    uint16_t wind_direction = data[2] + (((uint16_t) (data[3] & 0x80)) << 1);
    if (wind_direction != 0x1FF) {
      this->wind_direction_degrees_sensor_->publish_state(wind_direction);
    } else {
      this->wind_direction_degrees_sensor_->publish_state(NAN);
    }
  }
#endif  // USE_SENSOR
#ifdef USE_BINARY_SENSOR
  bool low_battery = (data[3] & 0x08) != 0;
  if (this->battery_level_binary_sensor_ != nullptr) {
    this->battery_level_binary_sensor_->publish_state(low_battery);
  }
#endif  // USE_BINARY_SENSOR
  uint16_t tmp_val = data[4] + (((uint16_t) (data[3] & 0x07)) << 8);
  float temperature = (tmp_val != 0x7FF) ?  (tmp_val - 400) / 10.0 : NAN;
#ifdef USE_SENSOR
  if (this->temperature_sensor_ != nullptr) {
    this->temperature_sensor_->publish_state(temperature);
  }
#endif  // USE_SENSOR
  uint8_t humidity = data[5];
#ifdef USE_SENSOR
  if (this->humidity_sensor_ != nullptr) {
    this->humidity_sensor_->publish_state(humidity);
  }
#endif  // USE_SENSOR
  uint16_t wind_speed_val = data[6] + (((uint16_t) (data[3] & 0x10)) << 4);
  float wind_speed = (wind_speed_val != 0x1FF) ? wind_speed_val / 8.0 * 1.12 : NAN;
#ifdef USE_SENSOR
  if (this->wind_speed_sensor_ != nullptr) {
    this->wind_speed_sensor_->publish_state(wind_speed);
  }
#endif  // USE_SENSOR
#ifdef USE_SENSOR
  if (this->wind_gust_sensor_ != nullptr) {
    uint8_t wind_gust = data[7];
    if (wind_gust != 0xFF) {
      this->wind_gust_sensor_->publish_state(wind_gust * 1.12);
    } else {
      this->wind_gust_sensor_->publish_state(NAN);
    }
  }
#endif  // USE_SENSOR
#if defined(USE_SENSOR)
  bool precipitation_intensity_updated = false;
  uint16_t accumulated_precipitation = data[9] + (((uint16_t) data[8]) << 8);
  float precipitation_intensity = NAN;
  if (this->previous_precipitation_.has_value()) {
    std::chrono::seconds interval =
        std::chrono::duration_cast<std::chrono::seconds>(now - this->previous_precipitation_timestamp_);
    if (interval > this->precipitation_intensity_interval_) {
      precipitation_intensity = ((float) (accumulated_precipitation - this->previous_precipitation_.value())) * 0.3f /
                                (interval.count() / 3600.0f);
      this->previous_precipitation_ = accumulated_precipitation;
      this->previous_precipitation_timestamp_ = now;
      precipitation_intensity_updated = true;
    }
  } else {
    this->previous_precipitation_ = accumulated_precipitation;
    this->previous_precipitation_timestamp_ = now;
  }
#endif  // USE_SENSOR
#ifdef USE_SENSOR
  if (this->accumulated_precipitation_sensor_ != nullptr) {
    this->accumulated_precipitation_sensor_->publish_state(accumulated_precipitation * 0.3);
  }
  if ((this->precipitation_intensity_sensor_ != nullptr) && (precipitation_intensity_updated)) {
    this->precipitation_intensity_sensor_->publish_state(precipitation_intensity);
  }
#endif  // USE_SENSOR
  unsigned int uv_intensity_val = data[11] + (((uint16_t) data[10]) << 8);
  float uv_intensity = (uv_intensity_val != 0xFFFF) ? uv_intensity_val / 10.0 : NAN;
#ifdef USE_SENSOR
  if (this->uv_intensity_sensor_ != nullptr) {
    this->uv_intensity_sensor_->publish_state(uv_intensity);
  }
  if (this->uv_index_sensor_ != nullptr) {
    if (!std::isnan(uv_intensity)) {
      uint8_t uv_index = uv_intensity_val / 400;
      this->uv_index_sensor_->publish_state(uv_index);
    } else {
      this->uv_index_sensor_->publish_state(NAN);
    }
  }
#endif  // USE_SENSOR
#ifdef USE_BINARY_SENSOR
  if ((this->night_binary_sensor_ != nullptr) && !std::isnan(uv_intensity)) {
    bool night = detect_night(uv_intensity, this->lower_night_threshold_, this->upper_night_threshold_);
    this->night_binary_sensor_->publish_state(night);
  }
#endif  // USE_BINARY_SENSOR
  uint32_t light_val = (data[14] + (data[13] << 8) + (data[12] << 16));
  float light = (light_val != 0xFFFFFF) ? light_val / 10.0 : NAN;
#ifdef USE_SENSOR
  if (this->light_sensor_ != nullptr) {
    this->light_sensor_->publish_state(light);
  }
#endif  // USE_SENSOR
}

}  // namespace misol_weather
}  // namespace esphome
