# esphome-seplos-bms

ESPHome component to monitor Seplos BMS via UART or RS485

## Supported devices

* 1101-SP15
* 1101-ZH26

## Untested devices

* EMU10xx
* 11XX Series

## Requirements

* [ESPHome 2022.4.0 or higher](https://github.com/esphome/esphome/releases).
* Generic ESP32 or ESP8266 board

## Installation

You can install this component with [ESPHome external components feature](https://esphome.io/components/external_components.html) like this:
```yaml
external_components:
  - source: github://syssi/esphome-seplos-bms@main
```

or just use the `esp32-example.yaml` as proof of concept:

```bash
# Install esphome
pip3 install esphome

# Clone this external component
git clone https://github.com/syssi/esphome-seplos-bms.git
cd esphome-seplos-bms

# Create a secrets.yaml containing some setup specific secrets
cat > secrets.yaml <<EOF
wifi_ssid: MY_WIFI_SSID
wifi_password: MY_WIFI_PASSWORD
EOF

# Validate the configuration, create a binary, upload it, and start logs
# If you use a esp8266 run the esp8266-examle.yaml
esphome run esp32-example.yaml
```

## Example response all sensors enabled

```
[I][seplos_bms:031]: Telemetry frame received
[D][sensor:124]: 'seplos-bms cell voltage 1': Sending state 3.28800 V with 3 decimals of accuracy
[D][sensor:124]: 'seplos-bms cell voltage 2': Sending state 3.30400 V with 3 decimals of accuracy
[D][sensor:124]: 'seplos-bms cell voltage 3': Sending state 3.31600 V with 3 decimals of accuracy
[D][sensor:124]: 'seplos-bms cell voltage 4': Sending state 3.28700 V with 3 decimals of accuracy
[D][sensor:124]: 'seplos-bms cell voltage 5': Sending state 3.31000 V with 3 decimals of accuracy
[D][sensor:124]: 'seplos-bms cell voltage 6': Sending state 3.30100 V with 3 decimals of accuracy
[D][sensor:124]: 'seplos-bms cell voltage 7': Sending state 3.29700 V with 3 decimals of accuracy
[D][sensor:124]: 'seplos-bms cell voltage 8': Sending state 3.29300 V with 3 decimals of accuracy
[D][sensor:124]: 'seplos-bms cell voltage 9': Sending state 3.30500 V with 3 decimals of accuracy
[D][sensor:124]: 'seplos-bms cell voltage 10': Sending state 3.31200 V with 3 decimals of accuracy
[D][sensor:124]: 'seplos-bms cell voltage 11': Sending state 3.30400 V with 3 decimals of accuracy
[D][sensor:124]: 'seplos-bms cell voltage 12': Sending state 3.31100 V with 3 decimals of accuracy
[D][sensor:124]: 'seplos-bms cell voltage 13': Sending state 3.30700 V with 3 decimals of accuracy
[D][sensor:124]: 'seplos-bms cell voltage 14': Sending state 3.29000 V with 3 decimals of accuracy
[D][sensor:124]: 'seplos-bms cell voltage 15': Sending state 3.29400 V with 3 decimals of accuracy
[D][sensor:124]: 'seplos-bms cell voltage 16': Sending state 3.28900 V with 3 decimals of accuracy
[D][sensor:124]: 'seplos-bms min cell voltage': Sending state 3.28700 V with 3 decimals of accuracy
[D][sensor:124]: 'seplos-bms max cell voltage': Sending state 3.31600 V with 3 decimals of accuracy
[D][sensor:124]: 'seplos-bms max voltage cell': Sending state 3.00000  with 0 decimals of accuracy
[D][sensor:124]: 'seplos-bms min voltage cell': Sending state 4.00000  with 0 decimals of accuracy
[D][sensor:124]: 'seplos-bms delta cell voltage': Sending state 0.02900 V with 3 decimals of accuracy
[D][sensor:124]: 'seplos-bms average cell voltage': Sending state 3.30050 V with 3 decimals of accuracy
[D][sensor:124]: 'seplos-bms temperature 1': Sending state 29.82000 °C with 0 decimals of accuracy
[D][sensor:124]: 'seplos-bms temperature 2': Sending state 29.76000 °C with 0 decimals of accuracy
[D][sensor:124]: 'seplos-bms temperature 3': Sending state 29.67000 °C with 0 decimals of accuracy
[D][sensor:124]: 'seplos-bms temperature 4': Sending state 29.82000 °C with 0 decimals of accuracy
[D][sensor:124]: 'seplos-bms environment temperature': Sending state 29.81000 °C with 0 decimals of accuracy
[D][sensor:124]: 'seplos-bms mosfet temperature': Sending state 29.78000 °C with 0 decimals of accuracy
[D][sensor:124]: 'seplos-bms current': Sending state -6.54000 A with 2 decimals of accuracy
[D][sensor:124]: 'seplos-bms total voltage': Sending state 52.80000 V with 2 decimals of accuracy
[D][sensor:124]: 'seplos-bms power': Sending state -345.31198 W with 2 decimals of accuracy
[D][sensor:124]: 'seplos-bms charging power': Sending state 0.00000 W with 2 decimals of accuracy
[D][sensor:124]: 'seplos-bms discharging power': Sending state 345.31198 W with 2 decimals of accuracy
[D][sensor:124]: 'seplos-bms residual capacity': Sending state 133.86000 Ah with 2 decimals of accuracy
[D][sensor:124]: 'seplos-bms battery capacity': Sending state 170.00000 Ah with 2 decimals of accuracy
[D][sensor:124]: 'seplos-bms state of charge': Sending state 78.70000 % with 1 decimals of accuracy
[D][sensor:124]: 'seplos-bms rated capacity': Sending state 180.00000 Ah with 2 decimals of accuracy
[D][sensor:124]: 'seplos-bms charging cycles': Sending state 70.00000  with 0 decimals of accuracy
[D][sensor:124]: 'seplos-bms state of health': Sending state 100.00000 % with 1 decimals of accuracy
[D][sensor:124]: 'seplos-bms port voltage': Sending state 52.79000 V with 2 decimals of accuracy
```

## Protocol

See [SEPLOS BMS Communication Protocol_V2.0.pdf](docs/SEPLOS%20BMS%20Communication%20Protocol_V2.0.pdf) and [Seplos 48v 100A BMS RS485 Protocol.pdf](docs/Seplos%2048v%20100A%20BMS%20RS485%20Protocol.pdf).

```
$ echo -ne "~20004642E00200FD37\r" | hexdump -ve '1/1 "%.2X."'
      7E.32.30.30.30.34.36.34.32.45.30.30.32.30.30.46.44.33.37.0D.

# Get pack #0 telemetry data (CID2 `0x42`)
TX -> "~20004642E00200FD37\r"
RX <- "~2000460010960001100CD70CE90CF40CD60CEF0CE50CE10CDC0CE90CF00CE80CEF0CEA0CDA0CDE0CD8060BA60BA00B970BA60BA50BA2FD5C14A0344E0A426803134650004603E8149F0000000000000000DC6C\r"

# Get system parameters (CID2 `0x47`)
TX -> "~200046470000FDA9\r"
RX <- ?

# Get protocol version (CID2 `0x4F`)
TX -> "~2000464F0000FD9A\r"
RX <- ?

# Get manufacturer info (CID2 `0x51`)
TX -> "~200046510000FDAE\r"
RX <- "~20004600C040313130312D5350313520020743414E50726F746F636F6C3A536F666172202020F046\r"

# Get management info (pylontech only?)
TX -> "~200046920000FDA9\r"
RX <- ?

# Get module serial number (pylontech only?)
TX -> "~200046930000FDA8\r"
RX <- ?
```

## Debugging

If this component doesn't work out of the box for your device please update your configuration to enable the debug output of the UART component and increase the log level to the see outgoing and incoming serial traffic:

```
logger:
  level: DEBUG

uart:
  id: uart0
  baud_rate: 9600
  tx_pin: ${tx_pin}
  rx_pin: ${rx_pin}
  rx_buffer_size: 384
  debug:
    direction: BOTH
    after:
      delimiter: "\r"
    sequence:
      - lambda: UARTDebug::log_string(direction, bytes);
```

## References

* https://github.com/Frankkkkk/python-pylontech/blob/master/pylontech/pylontech.py
* https://diysolarforum.com/threads/simple-seplos-bms-protocol-decode-bash-script.34993/
* https://github.com/celsworth/lxp-pylon-utils/tree/master/lib/pylon/packet
* https://github.com/meteosat007/solar-pylontech
