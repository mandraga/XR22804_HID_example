
# XR22804_HID_example

This program shows how to use the HID interface with the Exar XR22804 chip under Linux. EDGE gpios, PWM, and the I2C bus.
The customer support was mute, so I had to figure it out from the datasheet and the HID documentation. Not so easy when starting froms scratch.
The logic analyser was usefull.

It uses the Hidapi library.

## Run it

```
sudo apt-get install libhidapi-dev
make
sudo ./a.out
```

