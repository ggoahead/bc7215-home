#pragma once

#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/core/helpers.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/sensor/sensor.h"

#include <SoftwareSerial.h>
// Vendored via lib_deps, see README. Pinned to a bitcode-tech/bc7215ac release tag.
#include <bc7215.h>
#include <bc7215ac.h>

namespace esphome {
namespace bc7215_ac {

// Persisted across reboots via ESPHome's preference store (flash, wear-leveled).
// Layout mirrors bitcode-tech's own EEPROM examples (format + data + unit).
struct BC7215ACSavedConfig {
  bool is_celsius;
  int8_t match_cnt;
  bc7215FormatPkt_t ir_format;
  bc7215DataMaxPkt_t ir_data;
} __attribute__((packed));

enum class OpState : uint8_t {
  STARTUP = 0,
  PAIRING,
  WORKING,
  SENDING,
  NOT_CONNECTED,
};

class BC7215ACClimate : public climate::Climate, public Component {
 public:
  void set_mod_pin(int pin) { this->mod_pin_ = pin; }
  void set_busy_pin(int pin) { this->busy_pin_ = pin; }
  void set_rx_pin(int pin) { this->rx_pin_ = pin; }
  void set_tx_pin(int pin) { this->tx_pin_ = pin; }
  void set_led_pin(int pin) { this->led_pin_ = pin; }
  void set_pair_pin(int pin) { this->pair_pin_ = pin; }
  void set_temperature_sensor(sensor::Sensor *sens) { this->temperature_sensor_ = sens; }

  void setup() override;
  void loop() override;
  void dump_config() override;

  climate::ClimateTraits traits() override;
  void control(const climate::ClimateCall &call) override;

 protected:
  // state machine
  void enter_pairing_();
  void enter_working_();
  void enter_sending_();
  void handle_startup_();
  void handle_pairing_();
  void handle_working_();
  void handle_sending_();
  void handle_not_connected_();
  void update_pair_button_();

  // persistence
  void save_config_();
  bool load_config_();

  // LED — normal WORKING state stays dark by design. Only pairing / fault /
  // brief TX activity blink. See project LED policy in the device YAML comments.
  void led_on_() { digitalWrite(this->led_pin_, LOW); }   // on-board LED is active-low
  void led_off_() { digitalWrite(this->led_pin_, HIGH); }
  void led_toggle_() { digitalWrite(this->led_pin_, !digitalRead(this->led_pin_)); }

  // library <-> ESPHome enum mapping
  static int mode_to_lib_(climate::ClimateMode mode);
  static climate::ClimateMode lib_to_mode_(int m);
  static int fan_to_lib_(climate::ClimateFanMode fan);
  static climate::ClimateFanMode lib_to_fan_(int f);
  void update_action_();

  int mod_pin_{-1};
  int busy_pin_{-1};
  int rx_pin_{-1};
  int tx_pin_{-1};
  int led_pin_{-1};
  int pair_pin_{-1};

  sensor::Sensor *temperature_sensor_{nullptr};

  SoftwareSerial bc7215_serial_;
  BC7215 *bc7215_{nullptr};
  BC7215AC *ac_{nullptr};
  ESPPreferenceObject pref_;

  OpState state_{OpState::STARTUP};
  uint8_t startup_step_{0};
  uint32_t state_time_{0};
  uint32_t led_time_{0};

  bool btn_last_{false};
  uint32_t btn_pressed_time_{0};
  bool long_press_handled_{false};

  int8_t match_cnt_{0};
};

}  // namespace bc7215_ac
}  // namespace esphome
