# xmip-core-transport-serial

Serial transport: bytes on a serial port, delimited or fixed-length frames, one frame is one Stream. A technology of [xmip-core-transport](https://github.com/IlleNilsson/xmip-core-transport).

## Toolchain

`rust-toolchain.toml` pins the toolchain for the whole estate. Do not change it
here.

## Verification

The included workflow is manual-only and calls the versioned shared workflow at
`IlleNilsson/.github@v1`.

## The port

The `port` feature, on by default, opens the device through `serialport`. Build
with `--no-default-features` on a box with no serial stack: the framing, which
is what a Location configures and the tests hold, needs no device.
