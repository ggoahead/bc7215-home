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

  this->pref_ = global_preferences->make_preference<BC7215ACSavedConfig>(fnv1_hash("bc7215_ac_config"), true);  // in_flash: ESP8266 default is RTC RAM, lost on power cut

  this->state_ = OpState::STARTUP;
  this->startup_step_ = 0;
  this->probe_attempts_ = 0;
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

// Presence probe: wake, then request shutdown and wait for the chip's ack.
// Retried because on a cold power-up the BC7215A is still booting when the
// ESP8266 first probes it and silently drops the command.
void BC7215ACClimate::handle_startup_() {
  uint32_t now = millis();
  switch (this->startup_step_) {
    case 0:
      this->bc7215_->setRx();
      this->state_time_ = now;
      this->startup_step_ = 1;
      break;
    case 1:
      if (now - this->state_time_ >= 50) {
        this->bc7215_->setTx();
        this->state_time_ = now;
        this->startup_step_ = 2;
      }
      break;
    case 2:
      if (now - this->state_time_ >= 50) {
        this->bc7215_->setShutDown();
        this->state_time_ = now;
        this->startup_step_ = 3;
      }
      break;
    case 3:
      if (!this->bc7215_->isBusy()) {
        if (this->probe_attempts_ > 0)
          ESP_LOGI(TAG, "BC7215A responded after %u retries", this->probe_attempts_);
        this->bc7215_->setRx();
        this->bc7215_->setTx();
        this->startup_step_ = 4;
      } else if (now - this->state_time_ > 1000) {
        if (++this->probe_attempts_ < PROBE_ATTEMPTS) {
          ESP_LOGD(TAG, "BC7215A not responding yet, retrying (%u/%u)", this->probe_attempts_, PROBE_ATTEMPTS);
          this->startup_step_ = 0;
        } else {
          ESP_LOGE(TAG, "BC7215A not detected, check wiring (MOD/BUSY/RX/TX); re-probing every %us",
                   (unsigned) (NOT_CONNECTED_REPROBE_MS / 1000));
          this->state_ = OpState::NOT_CONNECTED;
          this->led_time_ = now;
          this->state_time_ = now;
        }
      }
      break;
    case 4: {
      if (this->load_config_()) {
        ESP_LOGI(TAG, "Restored paired AC configuration from flash (protocol match #%d)", this->match_cnt_);
        // Real AC state is unknown after a reboot; after a power cut the AC is off too.
        this->mode = climate::CLIMATE_MODE_OFF;
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
  ESP_LOGI(TAG, "Pairing: with the AC on in Cooling/25C, aim the remote at the receiver and press Temp or Mode");
}

void BC7215ACClimate::start_pairing() {
  if (this->state_ != OpState::WORKING && this->state_ != OpState::PAIRING) {
    ESP_LOGW(TAG, "Ignoring pairing request: busy or BC7215A not connected");
    return;
  }
  this->ac_->stopCapture();
  this->enter_pairing_();
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
      // For protocols without a dedicated ON frame, on() replays this sample,
      // so a sample taken with the remote in OFF state would make power-on
      // send "off" again.
      int t = -1, m = -1, f = -1, p = -1;
      if (this->ac_->parse(t, m, f, p) && p == 0) {
        ESP_LOGW(TAG, "Pairing rejected: captured an OFF frame. Turn the AC on with the remote "
                      "(Cooling/25C), then press Temp or Mode (not Power)");
        this->ac_->startCapture();
        return;
      }
      ESP_LOGI(TAG, "Pairing succeeded (T=%d M=%d F=%d P=%d)", t, m, f, p);
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
  bool parsed = this->parse_capture_(t, m, f, p);
  // Parsing first swaps the library's base frame for the captured one
  // (bc7215_ac_replace_base), even when the capture doesn't parse, and
  // on()/off()/setTo() build every later frame from that base. A remote OFF
  // frame, a partial capture or another remote then breaks all commands until
  // reboot, so put the paired frame back, same as after a reboot.
  if (!this->load_config_())
    ESP_LOGW(TAG, "Failed to restore paired AC frame after capture");
  if (parsed) {
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

// Same steps as BC7215AC::parse(), which ignores bc7215_ac_replace_base()'s
// result: when the library rejects a capture, the base is still the paired
// frame and parse() decodes that, i.e. the pairing settings (Cooling/25C) with
// power on. Foreign IR (another remote, an iPhone's Face ID / camera) thus
// turned the AC "on" in HA without the real AC doing anything. Drop the capture
// if the IR protocol differs from the paired one or the library rejects it.
bool BC7215ACClimate::parse_capture_(int &t, int &m, int &f, int &p) {
  BC7215AC *ac = this->ac_;
  if (!ac->initOK || ac->sampleCount == 0)
    return false;

  const bc7215FormatPkt_t *base_fmt = bc7215_ac_get_base_fmt();
  uint8_t sig = ac->sampleFormat[0].signature.bits.sig;
  if (base_fmt == nullptr || sig != base_fmt->signature.bits.sig) {
    ESP_LOGD(TAG, "Ignoring capture: IR protocol 0x%02X, paired 0x%02X", sig,
             base_fmt == nullptr ? 0xFF : base_fmt->signature.bits.sig);
    return false;
  }

  bool replaced;
  if (ac->sampleCount == 1) {
    replaced = bc7215_ac_replace_base(ac->sampleStatus[0],
                                      reinterpret_cast<const bc7215DataVarPkt_t *>(&ac->sampleData[0]));
  } else {
    bc7215CombinedMsg_t msgs[4];
    for (int j = 0; j < ac->sampleCount; j++) {
      if (ac->sampleStatus[j] & 0x40) {  // "REV" bit: data arrives inverted
        for (int i = 0; i < (ac->sampleData[j].bitLen + 7) / 8; i++)
          ac->sampleData[j].data[i] = ~ac->sampleData[j].data[i];
        ac->sampleStatus[j] &= 0xbf;
      }
      msgs[j].bitLen = 0;
      msgs[j].body.msg.fmt = &ac->sampleFormat[j];
      msgs[j].body.msg.datPkt = reinterpret_cast<const bc7215DataVarPkt_t *>(&ac->sampleData[j]);
    }
    replaced = bc7215_ac_replace_base(ac->sampleCount, reinterpret_cast<const bc7215DataVarPkt_t *>(msgs));
  }
  if (!replaced) {
    ESP_LOGD(TAG, "Ignoring capture: rejected by the AC library (%u bits, %u segment(s))",
             ac->sampleData[0].bitLen, ac->sampleCount);
    return false;
  }

  int8_t t8 = -1, m8 = -1, f8 = -1, p8 = -1;
  bool celsius = ac->isCelsius();
  bool ok = celsius ? bc7215_ac_parse(&t8, &m8, &f8, &p8) : bc7215_ac_parse_f(&t8, &m8, &f8, &p8);
  t = t8 + (celsius ? 16 : 60);
  m = m8;
  f = f8;
  p = p8;
  return ok;
}

void BC7215ACClimate::enter_sending_() {
  this->state_ = OpState::SENDING;
  this->state_time_ = millis();
  this->led_on_();
}

void BC7215ACClimate::handle_sending_() {
  bool busy = this->ac_->isBusy();
  if (!busy || (millis() - this->state_time_ > 3000)) {
    if (busy)
      ESP_LOGW(TAG, "BC7215A did not confirm IR transmission within 3s");
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
  if (now - this->state_time_ >= NOT_CONNECTED_REPROBE_MS) {
    this->led_off_();
    this->state_ = OpState::STARTUP;
    this->startup_step_ = 0;
    this->probe_attempts_ = PROBE_ATTEMPTS - 1;  // one probe per cycle, then back here
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

  if (!this->pref_.save(&cfg) || !global_preferences->sync()) {
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

// One captured frame can match several protocol variants that differ in fields
// the sample didn't exercise (e.g. power bit). Step to the next candidate,
// wrapping back to the first one when the list runs out.
void BC7215ACClimate::try_next_protocol() {
  if (this->state_ != OpState::WORKING || this->ac_ == nullptr || !this->ac_->initOK) {
    ESP_LOGW(TAG, "Ignoring next-protocol request: not paired or busy");
    return;
  }
  BC7215ACSavedConfig cfg;
  if (!this->pref_.load(&cfg)) {
    ESP_LOGW(TAG, "Ignoring next-protocol request: no saved pairing");
    return;
  }

  this->ac_->stopCapture();
  if (this->ac_->matchNext()) {
    this->match_cnt_++;
    ESP_LOGI(TAG, "Switched to protocol match #%d", this->match_cnt_);
  } else {
    this->ac_->init(cfg.ir_data, cfg.ir_format);
    this->match_cnt_ = 0;
    ESP_LOGI(TAG, "No more protocol matches, back to match #0");
  }
  this->ac_->startCapture();

  cfg.match_cnt = this->match_cnt_;
  if (!this->pref_.save(&cfg) || !global_preferences->sync()) {
    ESP_LOGW(TAG, "Failed to save protocol match to flash");
  }
}

}  // namespace bc7215_ac
}  // namespace esphome
