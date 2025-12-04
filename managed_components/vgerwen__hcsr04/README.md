# HC SR04 ultrasonic driver

This repository contains an ESP-IDF driver for HC SR04 ultrasonic module with 2 bit data interface.This
module contains a ultrasonic transmitter and receiver and can measure distance. It runs on a ESP32 processor and is build using the ESP-IDF build system.

## Connecting the component

The driver uses 2 GPIO output pins of the ESP32.


| HC SR04 PIN  | ESP driver                 | Default GPIO
| ------------:|:--------------------------:| :-----------:|
| TRIGGER      | CONFIG_TRIGGER_PIN         | 33           |
| ECHO         | CONFIG_ECHO_PIN            | 32           |


These connections are made using **idf.py menuconfig** and choose the settings in the menu under **Component config --->HCSR04 menu**

## hcsr04 library

The libary only support the HC SR04 moduel  with a 2 bits control interface.
It uses the ```esp_driver_gpio``` component and ```esp_timer``` component.

# Usage

## API
The API of the driver is located in the include directory ```include/hcsr04_driver.h``` and has the following functions:

```C
esp_err_t  UltrasonicInit(void);
esp_err_t  UltrasonicMeasure(uint32_t max_distance, uint32_t *distance);
void       UltrasonicAssert(esp_err_t error_code);
```
The driver does not use interrupt handlers and API funcions run on the calling task. The ```UltrasonicMeasure``` function blocks during measurement of the distance. Depending on the measured distance the call blocks between 327 usec (2cm) for max. 6 msec (> 100cm).

**Measure algorith:**

- Generate start pulse of 10usec on trigger gpio pin
- Module generates 8 pulses of 40kHZ (approx 200usec)
- Measuure echo pulse width. Distance (cm) = T-echopulse width (usec) * 0,034/2

# Example

An example is proved in the directory examples/hcsr04-examples
To run the provided example:

```shell
cd examples/hcrs04-example
idf.py build
```
And flash it to the board:
``` shell
idf.py -p PORT flash monitor
```
The example code prints the string "afstand: <afstand> " on the terminal output. The <distance> is the distance in centimeter. If the distance is larger than 100cm and error code "ESP_ERR_ULTRASONIC_ECHO_TIMEOUT" is returned.


# License

This component is provided under MIT license, see [LICENSE](LICENSE.txt) file for details.

# Contributing

Please check for contribution guidelines.tbd