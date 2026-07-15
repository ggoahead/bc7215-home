#include "bc7215_ac_climate.h"
#include "esphome/core/log.h"
#include <cstring>
#include <cmath>

namespace esphome {
namespace bc7215_ac {

static const char *const TAG = "bc7215_ac";

// ---------------------------------------------------------------------------
// enum mapping: library uses 0=Auto 1=Cool 2=Heat 3=Dry 4=Fan (5=Keep, 6=n/a)
// ---------------------------------------------------------------------------
int BC7215ACClimate::mode_to_lib_(climate::ClimateMode mode) {
  switch (mode) {
    case climate::CLIMATE_MODE_HEAT_COOL:
      return 0;
    case climate::CLIMATE_MODE_COOL:
      return 1;
    case climate::CLIMATE_MODE_HEAT:
      return 2;
    case climate::CLIMATE_MODE_DRY:
      return 3;
    case climate::CLIMATE_MODE_FAN_ONLY:
      return 4;
    default:
      return 5;  // keep
  }
}

climate::ClimateMode BC7215ACClimate::lib_to_mode_(int m) {
  switch (m) {
    case 0:
      return climate::CLIMATE_MODE_HEAT_COOL;
    case 1:
      return climate::CLIMATE_MODE_COOL;
    case 2:
      return climate::CLIMATE_MODE_HEAT;
    case 3:
      return climate::CLIMATE_MODE_DRY;
    case 4:
      return climate::CLIMATE_MODE_FAN_ONLY;
    default:
      return climate::CLIMATE_MODE_COOL;
  }
}

// library fan: 0=Auto 1=Low 2=Med 3=High (4=Keep)
int BC7215ACClimate::fan_to_lib_(climate::ClimateFanMode fan) {
  switch (fan) {
    case climate::CLIMATE_FAN_AUTO:
      return 0;
    case climate::CLIMATE_FAN_LOW:
      return 1;
    case climate::CLIMATE_FAN_MEDIUM:
      return 2;
    case climate::CLIMATE_FAN_HIGH:
      return 3;
    default:
      return 4;  // keep
  }
}

climate::ClimateFanMode BC7215ACClimate::lib_to_fan_(int f) {
  switch (f) {
    case 0:
      return climate::CLIMATE_FAN_AUTO;
    case 1:
      return climate::CLIMATE_FAN_LOW;
    case 2:
      return climate::CLIMATE_FAN_MEDIUM;
    case 3:
      return climate::CLIMATE_FAN_HIGH;
    default:
      return climate::CLIMATE_FAN_AUTO;
  }
}

// There is no telemetry from the AC itself (open-loop IR blaster). "action" is
// inferred from the mode we last commanded / last decoded from the remote,
// not from measured compressor state.
void BC7215ACClimate::update_action_() {
  switch (this->mode) {
    case climate::CLIMATE_MODE_OFF:
      this->action = climate::CLIMATE_ACTION_OFF;
      break;
    case climate::CLIMATE_MODE_COOL:
      this->action = climate::CLIMATE_ACTION_COOLING;
      break;
    case climate::CLIMATE_MODE_HEAT:
      this->action = climate::CLIMATE_ACTION_HEATING;
      break;
    case climate::CLIMATE_MODE_DRY:
      this->action = climate::CLIMATE_ACTION_DRYING;
      break;
    case climate::CLIMATE_MODE_FAN_ONLY:
      this->action = climate::CLIMATE_ACTION_FAN;
      break;
    default:
      this->action = climate::CLIMATE_ACTION_IDLE;
      break;
  }
}

// ---------------------------------------------------------------------------
// traits / setup / dump_config
// ---------------------------------------------------------------------------
climate::ClimateTraits BC7215ACClimate::traits() {
  auto traits = climate::ClimateTraits();
  traits.add_supported_mode(climate::CLIMATE_MODE_OFF);
  traits.add_supported_mode(climate::CLIMATE_MODE_HEAT_COOL);
  traits.add_supported_mode(climate::CLIMATE_MODE_COOL);
  traits.add_supported_mode(climate::CLIMATE_MODE_HEAT);
  traits.add_supported_mode(climate::CLIMATE_MODE_DRY);
  traits.add_supported_mode(climate::CLIMATE_MODE_FAN_ONLY);

  traits.add_supported_fan_mode(climate::CLIMATE_FAN_AUTO);
  traits.add_supported_fan_mode(climate::CLIMATE_FAN_LOW);
  traits.add_supported_fan_mode(climate::CLIMATE_FAN_MEDIUM);
  traits.add_supported_fan_mode(climate::CLIMATE_FAN_HIGH);

  traits.set_visual_min_temperature(16);
  traits.set_visual_max_temperature(30);
  traits.set_visual_temperature_step(1);

  traits.add_feature_flags(climate::CLIMATE_SUPPORTS_ACTION);
  if (this->temperature_sensor_ != nullptr) {
    traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE);
  }
  return traits;
}

void BC7215ACClimate::setup() {
  pinMode(this->led_pin_, OUTPUT);
  this->led_off_();
  if (this->pair_pin_ >= 0) {
    pinMode(this->pair_pin_, INPUT_PULLUP);
  }

  // Brief boot blink (allowed per LED policy: boot / OTA / fatal error only).
  this->led_on_();
  delay(80);
  this->led_off_();

  this->bc7215_serial_.begin(19200, SWSERIAL_8N2, this->rx_pin_, this->tx_pin_);
  this->bc7215_ = new BC7215(this->bc7215_serial_, this->mod_pin_, this->busy_pin_);
  this->ac_ = new BC7215AC(*this->bc7215_);
  this->ac_->setCelsius();  // hardcoded: no Fahrenheit use case for this deployment

  this->pref_ = global_preferences->make_preference<BC7215ACSavedConfig>(fnv1_hash("bc7215_ac_config"));

  // Presence probe: wake, then request shutdown and see if the chip acks by
  // going busy. One-time ~100ms blocking delay at boot only, never in loop().
  this->bc7215_->setRx();
  delay(50);
  this->bc7215_->setTx();
  delay(50);
  this->bc7215_->setShutDown();

  this->state_ = OpState::STARTUP;
  this->startup_step_ = 0;
  this->state_time_ = millis();

  if (this->temperature_sensor_ != nullptr) {
    this->temperature_sensor_->add_on_state_callback([this](float state) {
      this->current_temperature = state;
      this->publish_state();
    });
    if (this->temperature_sensor_->has_state()) {
      this->current_temperature = this->temperature_sensor_->state;
    }
  }
}

void BC7215ACClimate::dump_config() {
  ESP_LOGCONFIG(TAG, "BC7215 AC Controller:");
  ESP_LOGCONFIG(TAG, "  MOD Pin: GPIO%d", this->mod_pin_);
  ESP_LOGCONFIG(TAG, "  BUSY Pin: GPIO%d", this->busy_pin_);
  ESP_LOGCONFIG(TAG, "  SoftwareSerial RX/TX: GPIO%d / GPIO%d", this->rx_pin_, this->tx_pin_);
  ESP_LOGCONFIG(TAG, "  LED Pin: GPIO%d", this->led_pin_);
  ESP_LOGCONFIG(TAG, "  Pair button Pin: GPIO%d", this->pair_pin_);
}

// ---------------------------------------------------------------------------
// main loop / state machine
// ---------------------------------------------------------------------------
void BC7215ACClimate::loop() {
  switch (this->state_) {
    case OpState::STARTUP:
      this->handle_startup_();
      break;
    case OpState::PAIRING:
      this->handle_pairing_();
      break;
    case OpState::WORKING:
      this->handle_working_();
      break;
    case OpState::SENDING:
      this->handle_sending_();
      break;
    case OpState::NOT_CONNECTED:
      this->handle_not_connected_();
      break;
  }
  this->update_pair_button_();
}

void BC7215ACClimate::handle_startup_() {
  switch (this->startup_step_) {
    case 0:
      if (!this->bc7215_->isBusy()) {
        this->bc7215_->setRx();
        this->bc7215_->setTx();
        this->startup_step_ = 1;
      } else if (millis() - this->state_time_ > 1000) {
        ESP_LOGE(TAG, "BC7215A not detected, check wiring (MOD/BUSY/RX/TX)");
        this->state_ = OpState::NOT_CONNECTED;
        this->led_time_ = millis();
      }
      break;
    case 1: {
      if (this->load_config_()) {
        ESP_LOGI(TAG, "Restored paired AC configuration from flash");
        this->mode = climate::CLIMATE_MODE_COOL;
        this->target_temperature = 25;
        this->fan_mode = climate::CLIMATE_FAN_AUTO;
        this->update_action_();
        this->publish_state();
        this->enter_working_();
      } else {
        ESP_LOGI(TAG, "No saved AC pairing found, entering pairing mode");
        this->enter_pairing_();
      }
      break;
    }
    default:
      break;
  }
}

void BC7215ACClimate::enter_pairing_() {
  this->state_ = OpState::PAIRING;
  this->led_time_ = millis();
  this->ac_->startCapture();
  ESP_LOGI(TAG, "Pairing: set remote to Cooling/25C, aim it at the receiver and press any button");
}

void BC7215ACClimate::handle_pairing_() {
  uint32_t now = millis();
  if (now - this->led_time_ >= 250) {
    this->led_time_ = now;
    this->led_toggle_();
  }

  if (this->ac_->signalCaptured()) {
    this->ac_->stopCapture();
    if (this->ac_->init()) {
      ESP_LOGI(TAG, "Pairing succeeded");
      this->match_cnt_ = 0;
      this->save_config_();
      this->mode = climate::CLIMATE_MODE_COOL;
      this->target_temperature = 25;
      this->fan_mode = climate::CLIMATE_FAN_AUTO;
      this->update_action_();
      this->publish_state();
      this->enter_working_();
    } else {
      ESP_LOGW(TAG, "Pairing failed (bad capture), listening again");
      this->ac_->startCapture();
    }
  }
}

void BC7215ACClimate::enter_working_() {
  this->state_ = OpState::WORKING;
  this->led_off_();
  this->ac_->startCapture();
}

void BC7215ACClimate::handle_working_() {
  if (!this->ac_->signalCaptured())
    return;

  this->ac_->stopCapture();
  this->led_on_();

  int t = -1, m = -1, f = -1, p = -1;
  if (this->ac_->parse(t, m, f, p)) {
    bool changed = false;
    bool celsius_ok = (t >= 16 && t <= 30);
    if (celsius_ok) {
      this->target_temperature = t;
      changed = true;
    }
    if (m >= 0 && m <= 4) {
      this->mode = this->lib_to_mode_(m);
      changed = true;
    }
    if (f >= 0 && f <= 3) {
      this->fan_mode = this->lib_to_fan_(f);
      changed = true;
    }
    if (p == 0) {
      this->mode = climate::CLIMATE_MODE_OFF;
      changed = true;
    }
    if (changed) {
      this->update_action_();
      this->publish_state();
      ESP_LOGD(TAG, "Decoded remote: T=%d M=%d F=%d P=%d", t, m, f, p);
    }
  } else {
    ESP_LOGD(TAG, "Captured signal did not parse (unrelated remote / noise)");
  }

  this->led_off_();
  this->ac_->startCapture();
  // Deliberately no idle heartbeat blink here: normal WORKING state stays dark.
}

void BC7215ACClimate::enter_sending_() {
  this->state_ = OpState::SENDING;
  this->state_time_ = millis();
  this->led_on_();
}

void BC7215ACClimate::handle_sending_() {
  if (!this->ac_->isBusy() || (millis() - this->state_time_ > 3000)) {
    this->led_off_();
    this->state_ = OpState::WORKING;
    this->ac_->startCapture();
  }
}

void BC7215ACClimate::handle_not_connected_() {
  uint32_t now = millis();
  if (now - this->led_time_ >= 500) {
    this->led_time_ = now;
    this->led_toggle_();
  }
}

void BC7215ACClimate::update_pair_button_() {
  if (this->pair_pin_ < 0)
    return;

  bool pressed = digitalRead(this->pair_pin_) == LOW;
  uint32_t now = millis();

  if (pressed && !this->btn_last_) {
    this->btn_pressed_time_ = now;
    this->long_press_handled_ = false;
  } else if (pressed && this->btn_last_ && !this->long_press_handled_) {
    if (now - this->btn_pressed_time_ >= 2000) {
      this->long_press_handled_ = true;
      ESP_LOGI(TAG, "Long press detected, entering pairing mode");
      if (this->state_ == OpState::WORKING || this->state_ == OpState::PAIRING) {
        this->ac_->stopCapture();
      }
      this->enter_pairing_();
    }
  }
  this->btn_last_ = pressed;
}

// ---------------------------------------------------------------------------
// control() - called by HA / API when the user changes mode/temp/fan
// ---------------------------------------------------------------------------
void BC7215ACClimate::control(const climate::ClimateCall &call) {
  if (this->ac_ == nullptr || !this->ac_->initOK) {
    ESP_LOGW(TAG, "Ignoring control(): not paired yet");
    return;
  }
  if (this->state_ != OpState::WORKING) {
    ESP_LOGW(TAG, "Ignoring control(): busy (pairing/sending/not connected)");
    return;
  }

  bool power_off_cmd = false;
  bool power_on_cmd = false;
  climate::ClimateMode new_mode = this->mode;

  if (call.get_mode().has_value()) {
    new_mode = *call.get_mode();
    if (new_mode == climate::CLIMATE_MODE_OFF) {
      power_off_cmd = true;
    } else if (this->mode == climate::CLIMATE_MODE_OFF) {
      // Powering on uses the AC's dedicated power button, same as a physical
      // remote. Any temp/fan/mode also present in this call is applied on a
      // follow-up control() call once WORKING resumes.
      power_on_cmd = true;
    }
  }

  this->ac_->stopCapture();

  if (power_off_cmd) {
    this->ac_->off();
    this->mode = climate::CLIMATE_MODE_OFF;
    this->update_action_();
    this->publish_state();
    this->enter_sending_();
    return;
  }

  if (power_on_cmd) {
    this->ac_->on();
    this->mode = new_mode;
    this->update_action_();
    this->publish_state();
    this->enter_sending_();
    return;
  }

  // Not a power transition: build a single setTo(temp, mode, fan, key) frame,
  // "keep" sentinels for anything not present in this call.
  int t = -1;
  int m = 5;  // keep
  int f = 4;  // keep
  int k = 4;  // key context, 4 = keep/default

  if (call.get_mode().has_value()) {
    m = this->mode_to_lib_(new_mode);
    this->mode = new_mode;
    k = 2;  // Mode button
  }
  if (call.get_target_temperature().has_value()) {
    t = (int) lroundf(*call.get_target_temperature());
    this->target_temperature = t;
    k = 0;  // Temp +/- button (direction is cosmetic only)
  }
  if (call.get_fan_mode().has_value()) {
    f = this->fan_to_lib_(*call.get_fan_mode());
    this->fan_mode = *call.get_fan_mode();
    k = 3;  // Fan button
  }

  this->ac_->setTo(t, m, f, k);
  this->update_action_();
  this->publish_state();
  this->enter_sending_();
}

// ---------------------------------------------------------------------------
// persistence
// ---------------------------------------------------------------------------
void BC7215ACClimate::save_config_() {
  BC7215ACSavedConfig cfg{};
  cfg.is_celsius = this->ac_->isCelsius();
  cfg.match_cnt = this->match_cnt_;
  memcpy(&cfg.ir_format, this->ac_->getFormatPkt(), sizeof(bc7215FormatPkt_t));
  // getDataPkt() points at a full bc7215DataMaxPkt_t-sized buffer internally
  // (BC7215AC::sampleData) even though it's returned via the variable-size
  // view type; copying sizeof(bc7215DataMaxPkt_t) mirrors bitcode-tech's own
  // EEPROM examples.
  memcpy(&cfg.ir_data, this->ac_->getDataPkt(), sizeof(bc7215DataMaxPkt_t));

  if (!this->pref_.save(&cfg)) {
    ESP_LOGW(TAG, "Failed to save AC pairing to flash");
  }
}

bool BC7215ACClimate::load_config_() {
  BC7215ACSavedConfig cfg;
  if (!this->pref_.load(&cfg))
    return false;

  if (cfg.is_celsius) {
    this->ac_->setCelsius();
  } else {
    this->ac_->setFahrenheit();
  }
  this->match_cnt_ = cfg.match_cnt;

  if (!this->ac_->init(cfg.ir_data, cfg.ir_format))
    return false;

  if (this->match_cnt_ > 0) {
    bool ok = false;
    for (int i = 0; i < this->match_cnt_; i++) {
      ok = this->ac_->matchNext();
    }
    if (!ok) {
      this->ac_->init(cfg.ir_data, cfg.ir_format);
      this->match_cnt_ = 0;
    }
  }
  return true;
}

}  // namespace bc7215_ac
}  // namespace esphome
