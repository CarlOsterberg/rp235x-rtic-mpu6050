# Read sensor data from MPU6050 over i2c on rp pico 235x using RTIC

This program writes information over UART on Gpio0/pin1.
The UART settings are: baud rate 460_800, 8 bits, no parity and 1 stop bit.

Connect the MPU6050 sensor SDA to GPIO4, SCL to GPIO5 and sensor INT to GPIO6.

### Installing dependencies

```sh
rustup target install thumbv8m.main-none-eabihf
cargo install flip-link
cargo install probe-rs-tools # If debugging with SWD
```

## Flashing and running the code

There are two ways, either you can use the picotool or probe-rs. The picotool
requires fewer tools, but you need to put the device into mass storage mode somehow.
probe-rs is easier to use, but you need a debugger supporting swd and to solder the debug pins.
For details see the [cargo config](.cargo/config.toml).

## Material

MPU6050 links
* [Datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)
* [Register map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)

Sensor fusion video inspiration
* [Accelerometers and Gyroscopes](https://www.youtube.com/watch?v=RZd6XDx5VXo)
* [Complementary filter](https://www.youtube.com/watch?v=BUW2OdAtzBw)

## License

The contents of this repository are dual-licensed under the _[MIT](LICENSE-MIT) OR [Apache-2.0](LICENSE-APACHE-2.0)_ License. That means you can chose either the MIT licence or the
Apache-2.0 license when you re-use this code. See [`LICENSE-MIT`](LICENSE-MIT) or [`LICENSE-APACHE-2.0`](LICENSE-APACHE-2.0) for more
information on each specific license.
