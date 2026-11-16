//! What a serial line is open on, handed in (ADR-0068): the operating
//! system's port a Location names, the in-process wire both ends of a
//! loopback share, or a test's fake. A [`crate::SerialTransport`] opens its
//! device's port through [`Opens`] at its first use and keeps it open for as
//! long as it is used, so what a device sends between a write and the read
//! after it waits in the port until it is read; on a node, every Location
//! naming the device goes through that one port (`shared_port.rs`). A
//! receive waits for the port's first byte on its [`Watch`], holding
//! nothing, so a write on the port goes meanwhile.

use std::collections::VecDeque;
use std::io::{self, Read, Write};
use std::sync::{Arc, Mutex};
use std::time::Duration;
use xcore::lock::Locked;

use transport::error::Result;

/// An open port: bytes in and out, each read and each write waiting at
/// most what was last set on it, at the speed last set on it, and watched
/// for bytes coming apart from its reads ([`Port::watch`]).
pub trait Port: Read + Write + Send {
    /// Each read and each write that follows waits at most `wait`: the
    /// platform's timeout on the open port. A read the wait runs out on with
    /// nothing fails [`io::ErrorKind::TimedOut`], or reads nothing.
    ///
    /// # Errors
    /// Where the port refused the wait.
    fn wait(&mut self, wait: Duration) -> io::Result<()>;

    /// Each read and each write that follows runs at `baud`: what a
    /// Location's exchange sets where another Location on the device left
    /// the port at another speed, as meters on one M-Bus line may each
    /// speak their own.
    ///
    /// # Errors
    /// Where the port refused the speed.
    fn speed(&mut self, baud: u32) -> io::Result<()>;

    /// What waits for bytes to come to this port while other threads write
    /// and read it: what a receive waits on for its first byte without
    /// holding the port, so another Location on the device writes
    /// meanwhile (ADR-0064, amendment 2026-11-15). Asked once, as the port
    /// opens.
    ///
    /// # Errors
    /// Where the platform gives the port none.
    fn watch(&self) -> io::Result<Arc<dyn Watch>>;
}

/// A port's bytes, waited for apart from its reads: the platform's own wait
/// for a port to be readable — `WaitCommEvent` on Windows, `poll` on Unix —
/// never a read, so what comes stays in the port for the read that takes
/// it. One receive waits on a watch at a time (`shared_port.rs`).
pub trait Watch: Send + Sync {
    /// Whether bytes came to the port within `wait`, none of them read:
    /// `false` when the wait ran out with nothing come; `true` when bytes
    /// came, or may have, so the port's reader looks and waits again where
    /// another took them. Once [`Watch::cancel`]led, `true` at once.
    ///
    /// # Errors
    /// Where the port failed while it was waited on.
    fn readable(&self, wait: Duration) -> io::Result<bool>;

    /// Every wait on this watch, the one under way and each after it, ended
    /// at once: its port is closing.
    fn cancel(&self);
}

/// Where a line's port comes from.
pub trait Opens: Send + Sync {
    /// `port` at `baud`, open.
    ///
    /// # Errors
    /// Where the port could not be opened.
    fn open(&self, port: &str, baud: u32) -> Result<Box<dyn Port>>;

    /// Whether both ends of the line are in this process, so what one end
    /// writes the other reads: a loopback has a far end here, a device
    /// does not.
    fn in_process(&self) -> bool {
        false
    }
}

/// The operating system's serial ports — a COM port, `/dev/ttyUSB0`, an
/// RS-485 adapter — opened on a build with the `port` feature through the
/// system's own calls on Windows and through `serialport` on Unix
/// (`system.rs`), and refused on a build without it.
#[derive(Clone, Copy, Debug, Default)]
pub struct SystemPorts;

/// Worth asking again: a port busy or unplugged now may be free or back on
/// the next use.
#[cfg(all(feature = "port", any(windows, unix)))]
fn refused(port: &str, error: &dyn std::fmt::Display) -> transport::error::TransportError {
    transport::error::TransportError::retryable(format!("opening {port}: {error}"))
}

cfg_select! {
    all(feature = "port", windows) => {
        impl Opens for SystemPorts {
            fn open(&self, port: &str, baud: u32) -> Result<Box<dyn Port>> {
                crate::system::ComPort::open(port, baud)
                    .map(|opened| Box::new(opened) as Box<dyn Port>)
                    .map_err(|e| refused(port, &e))
            }
        }
    }
    all(feature = "port", unix) => {
        impl Opens for SystemPorts {
            fn open(&self, port: &str, baud: u32) -> Result<Box<dyn Port>> {
                serialport::new(port, baud)
                    .open_native()
                    .map(|opened| Box::new(opened) as Box<dyn Port>)
                    .map_err(|e| refused(port, &e))
            }
        }

        /// `serialport`'s timeout bounds every read and write that follows,
        /// the `poll` before each; a read it runs out on with nothing fails
        /// `TimedOut` (serialport 4.10). Its baud rate is the open port's
        /// own, set on it in place. Its watch polls a copy of its
        /// descriptor (`system.rs`).
        impl Port for serialport::TTYPort {
            fn wait(&mut self, wait: Duration) -> io::Result<()> {
                serialport::SerialPort::set_timeout(self, wait).map_err(io::Error::from)
            }

            fn speed(&mut self, baud: u32) -> io::Result<()> {
                serialport::SerialPort::set_baud_rate(self, baud).map_err(io::Error::from)
            }

            fn watch(&self) -> io::Result<Arc<dyn Watch>> {
                Ok(Arc::new(crate::system::Polled::of(self)?))
            }
        }
    }
    _ => {
        impl Opens for SystemPorts {
            fn open(&self, port: &str, _baud: u32) -> Result<Box<dyn Port>> {
                Err(transport::error::protocol_error(format!(
                    "built without the port feature: no serial stack on this box for {port}"
                )))
            }
        }
    }
}

/// An in-memory wire both ends of a loopback hold: what one end writes, the
/// other reads, in order. Nothing on it waits: a read of it empty reads
/// nothing, and a watch of it empty sees nothing, at once; and it carries
/// any speed.
#[derive(Clone, Default)]
pub(crate) struct Wire(Arc<Mutex<VecDeque<u8>>>);

impl Read for Wire {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        self.0.locked().read(buf)
    }
}

impl Write for Wire {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.0.locked().write(buf)
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

impl Port for Wire {
    fn wait(&mut self, _wait: Duration) -> io::Result<()> {
        Ok(())
    }

    fn speed(&mut self, _baud: u32) -> io::Result<()> {
        Ok(())
    }

    fn watch(&self) -> io::Result<Arc<dyn Watch>> {
        Ok(Arc::new(self.clone()))
    }
}

/// Whether the wire holds what one end wrote, now.
impl Watch for Wire {
    fn readable(&self, _wait: Duration) -> io::Result<bool> {
        Ok(!self.0.locked().is_empty())
    }

    fn cancel(&self) {}
}

impl Opens for Wire {
    fn open(&self, _port: &str, _baud: u32) -> Result<Box<dyn Port>> {
        Ok(Box::new(self.clone()))
    }

    fn in_process(&self) -> bool {
        true
    }
}
