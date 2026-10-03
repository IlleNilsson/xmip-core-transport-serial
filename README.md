# xmip-core-transport-serial

Serial transport: bytes on a serial port, delimited or fixed-length frames, one frame is one Stream. A technology of [xmip-core-transport](https://github.com/IlleNilsson/xmip-core-transport).

`SerialTransport::port` is the port a line is open on, which IO-Link names in its own origin; until 2026-09-28 IO-Link cut it out of this technology's origin itself.

## Acknowledgement

Acceptance is at-most-once here. A frame read off the line is gone from the
device's buffer, and a raw serial line has no reply to hold back until the
receive cycle ends, so nobody is told how it ended. Each frame arrives whole. A
protocol on the line that answers, such as M-Bus or HART, answers in its own
terms.

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
