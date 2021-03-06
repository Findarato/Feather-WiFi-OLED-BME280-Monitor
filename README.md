# Web Accessable Temperature gague /w OLED readout

[![Build Status](https://travis-ci.org/Findarato/Feather-WiFi-OLED-BME280-Monitor.svg?branch=master)](https://travis-ci.org/Findarato/Feather-WiFi-OLED-BME280-Monitor)

## Now with Toggle switches

A :radio_button: Shows IP<br>
B :radio_button: Shows Readings

There are a few libraries you will need, and they are listed below. Ensure they are in your Arduino libraries folder and you should be able to compile the file.

If your into using the terminal

```bash
arduino --install-library "Adafruit GFX Library,ArduinoJson,Adafruit SSD1306,Adafruit BME280 Library,Adafruit Unified Sensor"
```

```bash
platformio ci --project-conf=platformio.ini ./src
```

Libraries Needed to be installed

- :octopus: [AdaFruitGFX][432f0407]
- :octopus: [ArduinoJSON][92f91ab3]
- :octopus: [AdaFruit SSD1306][e13d6d0d]
- :octopus: [Adafruit BME280 Library][b4a05a48]
- :octopus: [Adafruit Unified Sensor Driver][b47100f1]

[432f0407]: https://github.com/adafruit/Adafruit-GFX-Library "Github"
[92f91ab3]: https://github.com/bblanchon/ArduinoJson "Github"
[b47100f1]: https://github.com/adafruit/Adafruit_Sensor "Github"
[b4a05a48]: https://github.com/adafruit/Adafruit_BME280_Library "Github"
[e13d6d0d]: https://github.com/adafruit/Adafruit_SSD1306 "Github"
