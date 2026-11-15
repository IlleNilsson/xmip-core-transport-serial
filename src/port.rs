//! What a serial line is open on, handed in (ADR-0068): the operating
//! system's port a Location names, the in-process wire both ends of a
//! loopback share, or a test's fake. A [`crate::SerialTransport`] opens its
//! port through [`Opens`] at its first use and keeps it open for as long as
//! it is used, so what a device sends between a write and the read after it
//! waits in the port until it is read.

use std::collections::VecDeque;
use std::io::{self, Read, Write};
use std::sync::{Arc, Mutex};
use std::time::Duration;
use xcore::lock::Locked;

use transport::error::Result;

/// An open port: bytes in and out, each read and each write waiting at
/// most what was last set on it.
pub trait Port: Read + Write + Send {
    /// Each read and each write that follows waits at most `wait`: the
    /// platform's timeout on the open port. A read the wait runs out on with
    /// nothing fails [`io::ErrorKind::TimedOut`], or reads nothing.
    ///
    /// # Errors
    /// Where the port refused the wait.
    fn wait(&mut self, wait: Duration) -> io::Result<()>;
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
/// RS-485 adapter — opened through `serialport` on a build with the `port`
/// feature, and refused on a build without it.
#[derive(Clone, Copy, Debug, Default)]
pub struct SystemPorts;

cfg_select! {
    feature = "port" => {
        impl Opens for SystemPorts {
            /// Worth asking again: a port busy or unplugged now may be free
            /// or back on the next use.
            fn open(&self, port: &str, baud: u32) -> Result<Box<dyn Port>> {
                serialport::new(port, baud)
                    .open()
                    .map(|opened| Box::new(opened) as Box<dyn Port>)
                    .map_err(|e| {
                        transport::error::TransportError::retryable(format!(
                            "opening {port}: {e}"
                        ))
                    })
            }
        }

        /// `serialport`'s timeout bounds every read and write that follows:
        /// on Windows the port's read and write total timeouts, elsewhere
        /// the `poll` before each; a read it runs out on with nothing fails
        /// `TimedOut` on both (serialport 4.10).
        impl Port for Box<dyn serialport::SerialPort> {
            fn wait(&mut self, wait: Duration) -> io::Result<()> {
                self.set_timeout(wait).map_err(io::Error::from)
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
/// nothing, at once.
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
}

impl Opens for Wire {
    fn open(&self, _port: &str, _baud: u32) -> Result<Box<dyn Port>> {
        Ok(Box::new(self.clone()))
    }

    fn in_process(&self) -> bool {
        true
    }
}
