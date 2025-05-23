# IMU to USB

Acceleration: units G
Gyro: units dps
Precision: 9 digits
Frequency: 833 Hz
Range: 2G, 150 dps

Baud rate: 1_000_000

Test (this is buffering):
```sh
$ stty -F /dev/ttyUSB0 1000000
$ tail -f /dev/ttyUSB0
```

or

```sh
$ picocom -e c -b 1000000 --imap lfcrlf /dev/ttyUSB0
```

a.k.a. `make com`.

Make sure to read unbuffered to get minimum lag.
