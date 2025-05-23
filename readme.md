# IMU to USB

Acceleration: units G
Gyro: units dps

Baud rate: 1_000_000

Test:
```sh
$ stty -F /dev/ttyUSB0 1000000
$ tail -f /dev/ttyUSB0
```
