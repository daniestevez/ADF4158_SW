# Software for ADF4158

This repository contains Rust software to configure an
[ADF4158](https://www.analog.com/en/products/adf4158.html) PLL waveform
synthesizer. The repository is structured as a Rust workspace containing the
following crates:

- `adf4158`. A `no-std` library crate that contains functions to configure all
  the ADF4158 registers.

- `longan-nano-adf4158`. A crate that builds firmware for a [Longan
  Nano](https://wiki.sipeed.com/hardware/en/longan/Nano/Longan_nano.html) board
  that uses the `adf4158` crate to configure an ADF4158 using three GPIOs of the
  Longan Nano.

## Building

This project uses `just` to build and flash the firmware. Run `just firmware` to
build the `firmware.bin` file. Run `just flash` to build the `firmware.bin` file
and flash it. This requires `rust-objdump` from
[cargo-binutils](https://github.com/rust-embedded/cargo-binutils) and
`dfu-util`.

## License

Licensed under either of

 * Apache License, Version 2.0
   ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
 * MIT license
   ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.
