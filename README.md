# bc7215-home

ESPHome `climate` external_component wrapping [bitcode-tech/bc7215ac](https://github.com/bitcode-tech/bc7215ac)
(Arduino/Stream-based library) for BC7215A universal IR AC control on ESP8266.

This component instead wraps bitcode-tech's own Arduino library, which has a
Stream-based interface and an official ESP8266 + SoftwareSerial example
(`ESP8266_AC_MQTT_HA.ino`). We keep the library and its wiring/pairing/EEPROM
logic, and replace the sketch's hand-rolled WiFi+MQTT+HA-discovery layer with
ESPHome's native `climate::Climate` + API, per the project's "all WiFi/API/OTA
goes through ESPHome" principle.

## Wiring (NodeMCU)

Identical to `bc7215ac`'s own ESP8266 example — MOD=D5(14), BUSY=D2(4),
SoftwareSerial RX=D1(5)/TX=D0(16), LED=D4(2), pair button=D3(0), DHT22=D6(12).

## Status / what's verified

- Python config schema (`climate.py`) validated against a real installed
  `esphome==2026.6.5` package: `esphome config example-livingroom-ac.yaml`
  passes.
- C++ (`bc7215_ac_climate.h/.cpp`) is written directly against the actual
  `bc7215.h` / `bc7215ac.h` / `climate.h` / `climate_traits.h` /
  `preference_backend.h` headers (pulled from the real repos / pip package),
  not from memory. **Not yet compiled** — this sandbox has no network access
  to PlatformIO's toolchain servers, so `esphome compile` gets as far as
  generating C++ and fails only at the "download Xtensa toolchain" step.
  First real build needs to happen on your T430 or dev machine.

## Design decisions to confirm

- **LED**: WORKING state stays fully dark (no idle heartbeat blink), per your
  LED policy. PAIRING blinks 250ms, NOT_CONNECTED (chip not detected) blinks
  500ms as a fault indicator, and the LED goes solid ON for the ~100–300ms of
  an actual IR transmission. If you want zero LED even during TX, that's a
  one-line removal in `enter_sending_()`/`handle_sending_()`.
- **Power on/off** uses the AC's dedicated on()/off() IR command, separate
  from temp/mode/fan changes — matches how a physical remote's power button
  usually works. If you power on and change temperature in the same HA
  interaction, only the power-on command fires; send the temperature change
  as a follow-up (mirrors physical remote behavior, not a bug).
- **`action`** (heating/cooling/idle shown in HA) is inferred from the mode
  we last commanded or decoded, not measured — there's no real telemetry from
  a universal IR blaster.
- Hardcoded to Celsius (`ac_->setCelsius()`); no reason to expose Fahrenheit
  for a Shanghai deployment.
- If you also want ESPHome's `ota:`/`safe_mode:` status LED, don't point it
  at GPIO2 — this component already drives that pin directly; two owners of
  one GPIO will fight.

## Pairing

Long-press the FLASH button (GPIO0, ≥2s) to (re-)enter pairing. Point the
real remote at the receiver, set it to Cooling/25°C, press any button. LED
blinks fast during pairing, goes dark on success. Pairing data persists in
flash (ESPHome preferences) across reboots and OTA updates.
