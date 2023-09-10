# I2C Basic Example

This examples shows how to use 2 ESP32 to work as master-slave. The master send 1 to the slave, then, it increase the number that arrives and put it in its buffer. Afterwards, the master read the slave buffer and sends that value.


## How to use example
We encourage the users to use the example as a template for the new projects.
A recommended way is to follow the instructions on a [docs page](https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html#start-a-new-project).

## Hardware Required

You need 2 ESP32 basic Devboard, this was tested in 2 NodeMCU.

### Pin Assignment:

To deploy the website in the SD Card you must have this pin connection for this example.

| ESP32-MASTER | ESP32-SLAVE | DESCRIPTION |
| ------------ | ----------- | ----------- |
| GPIO22       | GPIO5       | I2C SCL     |
| GPIO21       | GPIO4       | I2C SDA     |
| -            | GPIO2       | LED         |

## Log

* Last compile: September 09t 2023.
* Last test: September 09t 2023.
* Last compile espidf version: v5.1.1

## License

Apache License, Version 2.0, January 2004.

## Version

`v 1.0.0`
