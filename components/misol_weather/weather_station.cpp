#include "esphome/components/uart/uart.h"
#include "esphome/core/helpers.h"
#include "weather_station.h"
#include <memory>
#include <string>

namespace {

std::string precipitation_to_description(float mm_per_hour) {
    if (std::isnan(mm_per_hour)) {
        return "Unknown";
    } else if (mm_per_hour <= 0) {
        return "No precipitation";
    } else if (mm_per_hour > 0 && mm_per_hour <= 2.5) {
        return "Light rain";
    } else if (mm_per_hour > 2.5 && mm_per_hour <= 7.5) {
        return "Moderate rain";
    } else if (mm_per_hour > 7.5 && mm_per_hour <= 50) {
        return "Heavy rain";
    } else if (mm_per_hour > 50) {
        return "Violent rain";
    } else {
        return "Extreme precipitation";
    }
}

std::string light_level_to_description(float lux) {
    if (lux < 2) {
        return "Overcast night";
    } else if (lux < 3) {
        return "Clear night sky";
    } else if (lux < 50) {
        return "Rural night sky";
    } else if (lux < 400) {
        return "Dark overcast sky";
    } else if (lux < 4500) {
        return "Overcast day";
    } else if (lux < 28500) {
        return "Full daylight";
    } else if (lux < 120000) {
        return "Direct sunlight";
    } else {
        return "Bright direct sunlight";
    }
}

std::string wind_speed_to_description(float speed) {
    if (speed < 0.3) {
        return "Calm";
    } else if (speed >= 0.3 && speed <= 1.5) {
        return "Light air";
    } else if (speed > 1.5 && speed <= 3.3) {
        return "Light breeze";
    } else if (speed > 3.3 && speed <= 5.5) {
        return "Gentle breeze";
    } else if (speed > 5.5 && speed <= 7.9) {
        return "Moderate breeze";
    } else if (speed > 7.9 && speed <= 10.7) {
        return "Fresh breeze";
    } else if (speed > 10.7 && speed <= 13.8) {
        return "Strong breeze";
    } else if (speed > 13.8 && speed <= 17.1) {
        return "High wind";
    } else if (speed > 17.1 && speed <= 20.7) {
        return "Gale";
    } else if (speed > 20.7 && speed <= 24.4) {
        return "Severe gale";
    } else if (speed > 24.4 && speed <= 28.4) {
        return "Storm";
    } else if (speed > 28.4 && speed <= 32.6) {
        return "Violent storm";
    } else {
        return "Hurricane force";
    }
}

std::string angle_to_direction(float angle) {
    const char* directions[] = {
        "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
        "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW", "N"
    };
    int index = static_cast<int>((angle + 11.25) / 22.5);
    return directions[index % 16];
}

} // namespace

namespace esphome {
namespace misol {

static const char *TAG = "misol.weather_station"; 
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
    ESP_LOGD(TAG, "%s received: %s", this->first_data_received_ ? "Packet" : "First packet", format_hex_pretty(buffer.get(), size).c_str());
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
    this->previos_precipitation_ = 0xFFFF;
  }    
#endif // USE_SENSOR
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

void WeatherStation::process_packet_(const uint8_t *data, size_t len, bool has_pressure, const std::chrono::steady_clock::time_point &now) {
#ifdef USE_SENSOR
  if (this->pressure_sensor_ != nullptr) {
    if (has_pressure) {
      uint32_t pressure = (((uint32_t)data[17]) << 16) + (((uint32_t)data[18]) << 8) + data[19];
      this->pressure_sensor_->publish_state(pressure / 100.0f);
    } else {
      this->pressure_sensor_->publish_state(NAN);
    }
  }
#endif // USE_SENSOR
#ifdef USE_SENSOR
  if (this->wind_direction_degrees_sensor_ != nullptr) {
    uint16_t wind_direction = data[2] + (((uint16_t)(data[3] & 0x80)) << 1);
    if (wind_direction != 0x1FF) {
      this->wind_direction_degrees_sensor_->publish_state(wind_direction);
    } else {
      this->wind_direction_degrees_sensor_->publish_state(NAN);
    }
  }
#endif // USE_SENSOR
#ifdef USE_TEXT_SENSOR
  if (this->wind_direction_text_sensor_ != nullptr) {
    uint16_t wind_direction = data[2] + (((uint16_t)(data[3] & 0x80)) << 1);
    if (wind_direction != 0x1FF) {
      this->wind_direction_text_sensor_->publish_state(angle_to_direction(wind_direction));
    } else {
      this->wind_direction_text_sensor_->publish_state("Unknown");
    }
  }
#endif // USE_TEXT_SENSOR
#ifdef USE_BINARY_SENSOR
  bool low_battery = (data[3] & 0x08) != 0;
  this->battery_level_binary_sensor_->publish_state(low_battery);
#endif // USE_BINARY_SENSOR
#ifdef USE_SENSOR
  if (this->temperature_sensor_ != nullptr) {
    uint16_t temperature = data[4] + (((uint16_t)(data[3] & 0x07)) << 8);
    if (temperature != 0x7FF) {
      this->temperature_sensor_->publish_state((temperature - 400) / 10.0);
    } else {
      this->temperature_sensor_->publish_state(NAN);
    }
  }
#endif // USE_SENSOR
#ifdef USE_SENSOR
  if (this->humidity_sensor_ != nullptr) {
    uint8_t humidity = data[5];
    if (humidity != 0xFF) {
      this->humidity_sensor_->publish_state(humidity);
    } else {
      this->humidity_sensor_->publish_state(NAN);
    }
  }
#endif // USE_SENSOR
#ifdef USE_SENSOR
  if (this->wind_speed_sensor_ != nullptr) {
    uint16_t wind_speed = data[6] + (((uint16_t)(data[3] & 0x10)) << 4);
    if (wind_speed != 0x1FF) {
      this->wind_speed_sensor_->publish_state(wind_speed / 8.0 * 1.12);
    } else {
      this->wind_speed_sensor_->publish_state(NAN);
    }
  }
#endif // USE_SENSOR
#ifdef USE_TEXT_SENSOR
  if (this->wind_speed_text_sensor_ != nullptr) {
    uint16_t wind_speed = data[6] + (((uint16_t)(data[3] & 0x10)) << 4);
    if (wind_speed != 0x1FF) {
      this->wind_speed_text_sensor_->publish_state(wind_speed_to_description(wind_speed / 8.0 * 1.12));
    } else {
      this->wind_speed_text_sensor_->publish_state("Unknown");
    }
  }
#endif // USE_TEXT_SENSOR
#ifdef USE_SENSOR
  if (this->wind_gust_sensor_ != nullptr) {
    uint8_t wind_gust = data[7];
    if (wind_gust != 0xFF) {
      this->wind_gust_sensor_->publish_state(wind_gust * 1.12);
    } else {
      this->wind_gust_sensor_->publish_state(NAN);
    }
  }
#endif // USE_SENSOR
#if defined(USE_SENSOR) || defined(USE_TEXT_SENSOR)
  bool precipitation_intensity_updated = false;
  uint16_t accumulated_precipitation = data[9] + (((uint16_t)data[8]) << 8);
  float precipitation_intensity = NAN;
  if (accumulated_precipitation != 0xFFFF) {
      if (this->previos_precipitation_ != 0xFFFF) {
        std::chrono::seconds interval = std::chrono::duration_cast<std::chrono::seconds>(now - this->previos_precipitation_timestamp_);
        if (interval > PRECIPITATION_INTENSITY_INTERVAL) {
          this->previos_precipitation_ = accumulated_precipitation;
          this->previos_precipitation_timestamp_ = now;
          precipitation_intensity = (accumulated_precipitation - this->previos_precipitation_) * 0.3 / (interval.count() / 3600.0);
          precipitation_intensity_updated = true;
        }
      } else {
        this->previos_precipitation_ = accumulated_precipitation;
        this->previos_precipitation_timestamp_ = std::chrono::steady_clock::now();
      }
  } else {
    this->previos_precipitation_ = 0xFFFF;
    precipitation_intensity_updated = true;
  }
#endif // USE_SENSOR || USE_TEXT_SENSOR
#ifdef USE_SENSOR
  if (this->accumulated_precipitation_sensor_ != nullptr) {
    this->accumulated_precipitation_sensor_->publish_state(accumulated_precipitation * 0.3);
  }
  if ((this->precipitation_intensity_sensor_ != nullptr) && (precipitation_intensity_updated)) {
    this->precipitation_intensity_sensor_->publish_state(precipitation_intensity);
  }
#endif // USE_SENSOR
#ifdef USE_TEXT_SENSOR
  if ((this->precipitation_intensity_text_sensor_ != nullptr) && (precipitation_intensity_updated)) {
    this->precipitation_intensity_text_sensor_->publish_state(precipitation_to_description(precipitation_intensity));
  }
#endif

#ifdef USE_SENSOR
  unsigned int uv_intensity = data[11] + (((uint16_t)data[10]) << 8);
  if (this->uv_intensity_sensor_ != nullptr) {
    if (uv_intensity != 0xFFFF) {
      this->uv_intensity_sensor_->publish_state(uv_intensity / 10.0f);
    } else {
      this->uv_intensity_sensor_->publish_state(NAN);
    }
  }
  if (this->uv_index_sensor_ != nullptr) {
    if (uv_intensity != 0xFFFF) {
      uint8_t uv_index = uv_intensity / 400;
      this->uv_index_sensor_->publish_state(uv_index);
    } else {
      this->uv_index_sensor_->publish_state(NAN);
    }
  }
#endif // USE_SENSOR
#ifdef USE_SENSOR
  if (this->light_sensor_ != nullptr) {
    uint32_t light = (data[14] + (data[13] << 8) + (data[12] << 16));
    if (light != 0xFFFFFF) {
      this->light_sensor_->publish_state(light / 10.0);
    } else {
      this->light_sensor_->publish_state(NAN);
    }
  }
#endif // USE_SENSOR
#ifdef USE_TEXT_SENSOR
  if (this->light_text_sensor_ != nullptr) {
    uint32_t light = (data[14] + (data[13] << 8) + (data[12] << 16));
    if (light != 0xFFFFFF) {
      this->light_text_sensor_->publish_state(light_level_to_description(light / 10.0));
    } else {
      this->light_text_sensor_->publish_state("Unknown");
    }
  }
  #endif // USE_TEXT_SENSOR
}

} // misol
} // esphome


