# ESP8266 MQTT Thermostat
http://ihormelnyk.com/mqtt_thermostat

Simple MQTT OpenTherm Thermostat based on [OpenTherm Adapter](http://ihormelnyk.com/opentherm_adapter), [OpenTherm Library](http://ihormelnyk.com/opentherm_library), ESP8266 (WeMos D1 Mini) and PID Temperature Controller.

Watch out: needs version 2.5.0 of the ESP8266 library on Arduino.
Will throw "ISR not in IRAM"-error when using 2.5.1 or 2.5.2.
