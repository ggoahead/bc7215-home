"""bc7215_ac climate platform.

Wraps the vendor bitcode-tech/bc7215ac Arduino library (Stream-based, works with
SoftwareSerial on ESP8266) inside ESPHome's native climate::Climate component.

Deliberately NOT based on timj-code/bc7215_ac_esphome: that project is built on
the ESP-IDF example from bc7215_ac_lib and is ESP32-only (confirmed by the
author). This component targets ESP8266 + Arduino framework + SoftwareSerial,
matching bitcode-tech/bc7215ac's own ESP8266 examples.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import climate, sensor
from esphome.const import CONF_ID, CONF_SENSOR

CODEOWNERS = ["@ggoahead"]

CONF_MOD_PIN = "mod_pin"
CONF_BUSY_PIN = "busy_pin"
CONF_RX_PIN = "rx_pin"
CONF_TX_PIN = "tx_pin"
CONF_LED_PIN = "led_pin"
CONF_PAIR_PIN = "pair_pin"

bc7215_ac_ns = cg.esphome_ns.namespace("bc7215_ac")
BC7215ACClimate = bc7215_ac_ns.class_(
    "BC7215ACClimate", climate.Climate, cg.Component
)

CONFIG_SCHEMA = climate.climate_schema(BC7215ACClimate).extend(
    {
        cv.Required(CONF_MOD_PIN): pins.internal_gpio_output_pin_number,
        cv.Required(CONF_BUSY_PIN): pins.internal_gpio_input_pin_number,
        # SoftwareSerial pins: RX = ESP pin receiving BC7215 TX, TX = ESP pin driving BC7215 RX.
        cv.Required(CONF_RX_PIN): pins.internal_gpio_input_pin_number,
        cv.Required(CONF_TX_PIN): pins.internal_gpio_output_pin_number,
        cv.Required(CONF_LED_PIN): pins.internal_gpio_output_pin_number,
        # FLASH button / long-press-to-pair pin. Defaults to NodeMCU D3 (GPIO0).
        cv.Optional(CONF_PAIR_PIN, default="GPIO0"): pins.internal_gpio_input_pin_number,
        # Optional: an existing sensor (e.g. dht.temperature) whose state feeds
        # current_temperature. The sensor itself stays a normal standalone entity.
        cv.Optional(CONF_SENSOR): cv.use_id(sensor.Sensor),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await climate.register_climate(var, config)

    cg.add(var.set_mod_pin(config[CONF_MOD_PIN]))
    cg.add(var.set_busy_pin(config[CONF_BUSY_PIN]))
    cg.add(var.set_rx_pin(config[CONF_RX_PIN]))
    cg.add(var.set_tx_pin(config[CONF_TX_PIN]))
    cg.add(var.set_led_pin(config[CONF_LED_PIN]))
    cg.add(var.set_pair_pin(config[CONF_PAIR_PIN]))

    if CONF_SENSOR in config:
        sens = await cg.get_variable(config[CONF_SENSOR])
        cg.add(var.set_temperature_sensor(sens))
