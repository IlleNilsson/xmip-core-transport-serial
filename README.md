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

The `port` feature, on by default, opens the device: on Windows through the
system's own calls, for overlapped I/O (`src/system.rs`, the crate's one file of
unsafe code), and on Unix through `serialport`. Build with
`--no-default-features` on a box with no serial stack: the framing, which is
what a Location configures and the tests hold, needs no device.

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
`Port`, which takes each wait and each speed set on it and gives the `Watch` a
receive waits on. `SystemPorts`, the operating system's, is the default; the
loopback line is a port of the same kind, and a test hands in a fake one
(`SerialTransport::opened_by`), so the read and write paths a device takes are
the ones the tests drive. The fake is a device on a bench (`bench::Bench`),
under the `test-support` feature for another crate's tests.

## One port per device on a node

The operating system opens a serial port for one handle at a time —
exclusively on Windows, and as `serialport` opens it everywhere else — so
until 2026-11-15, when each Location kept a port of its own, several M-Bus
meters on one bus, or a Send and a Receive Location on one port, left every
Location but the first refused, "opening COM3", every cycle. On a node, a
device has one port, shared by every Location naming its path (ADR-0064,
amendment 2026-11-15): the runtime hands each transport it builds what the
node's Locations share (`transport::Sharing`, through
`Configured::shared_by`), and the line takes its device's port from it,
opened through what the first line naming it opens with. A line built
outside a node keeps one of its own.

Each exchange holds the port from its transmit through its receive
(`SerialTransport::exchanging`, an `Exchange`), so one Location's answer
never reaches another: a protocol on the line exchanges through
`Line::exchange`, which holds it across both, and one whose exchange is
several frames each way — an IO-Link ISDU — holds it across them all. A read
and a write held alone take it for themselves. A port that fails is closed
for every Location on the device, and the next exchange of any of them opens
it again, once; Locations on two devices hold two ports and never wait on
each other. Each exchange runs at its own Location's speed, set on the port
(`Port::speed`) where another Location left it at another.

A receive holds the port only while a frame arrives (the owner, 2026-11-15).
It looks at the port without waiting; until bytes are there it waits on the
port's `Watch` holding nothing, and once they are there it takes the port and
holds it until the frame is whole or cut, so a Send Location on the port waits
at most for a frame in flight. Until 2026-11-15 a Receive Location's read held
the port for its whole `timeout`, and a send on the port waited up to that
long. The watch is the platform's own wait for a readable port, never a read
and never a sleep: `poll` on Unix, on a copy of `serialport`'s descriptor; on
Windows `WaitCommEvent` for `EV_RXCHAR`, overlapped. `serialport` opens a COM
port for synchronous I/O, on which Windows runs one call at a time, so a wait
on it held every write; on Windows the crate opens the port itself, set as
`serialport` sets it: eight data bits, no parity, one stop bit, no flow
control. Each read, write and wait for a first byte on it is waited for by
the estate's one overlapped wait, `net::system::overlapped::waited`, which
the port's close stops; `src/system.rs` keeps only the port's own calls. One
receive waits on a port's watch at a time, as Windows takes one
`WaitCommEvent` on a port, and another waits for what that one sees; a port
closed while a receive waits on its watch ends that wait first, so nothing
holds the device open once it is closed. The bench drives all of it with no
device; on Unix it is driven on a real descriptor too, a pseudo-terminal pair
(`src/shared_port/pseudo_terminal.rs`, run in the Linux guest).
