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

A line opens its port at its first use and keeps it open, shared by every clone
of it, for as long as its Location uses it, so a device's answer that comes
between a write and the read after it waits in the port's buffer; until
2026-11-15 the port was opened for every write and every read, and such an
answer was lost. A port that fails is closed, and the next use opens it again.
Each read sets the wait it is handed on the open port, a protocol's read as a
`Line` its own and a Receive Location's its `timeout`; each write waits the
`timeout`. A read the wait runs out on with nothing is nothing arrived: `None`
to a line, an empty receive to a Receive Location. A frame begun and cut off by
the wait is an error.

What the port is opened through is handed in (ADR-0068): `Opens` opens a
`Port`, which takes each wait set on it. `SystemPorts`, the operating system's,
is the default; the loopback line is a port of the same kind, and a test hands
in a fake one (`SerialTransport::opened_by`), so the read and write paths a
device takes are the ones the tests drive.
