# A serial to mqtt bridge for on ESP8266 and ESP32

This is a simple firmware for ESP8266 and ESP32 to act as an opinionated
serial to mqtt bridge, to connect microcontrollers, such as Arduinos and
Teensys to a MQTT broker over a serial connection.

This is not a unique project. Many have implemented similar solutions.
The code is aimed to be clean and simple to follow for beginner and
intermediate developers, following modern C++ best practices and
event-driven and reactive design patterns as much as possible.

By implementing the serial connection protocol it should work out of the
box as a drop in firmware for people who want MQTT connectability for an
Arduino or Teensy based project.
