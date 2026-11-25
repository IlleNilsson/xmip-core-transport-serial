//! A pseudo-terminal pair as a device on a line (ADR-0064, amendment
//! 2026-11-15): `serialport` makes it (`TTYPort::pair`), a line opens its
//! line's end by path as it opens a device's port (`SystemPorts`), and the
//! device's end is held here, read and written as a device reads and writes
//! its line. It is what a test of the port on a real descriptor drives
//! (`shared_port/pseudo_terminal.rs`), and the far end a node in a process of
//! its own writes to: a serial Send Location's frame, or a protocol's master
//! on a serial line answered by the device the protocol's own crate stands
//! (the owner, 2026-11-25: *just do your best for now*). Unix's, for tests,
//! under `test-support`; never a production build.

use std::io::{BufReader, Write};
use std::time::Duration;

use serialport::{SerialPort, TTYPort};
use transport::Taken;
use transport::error::{Result, classify, protocol_error};
use transport::held::Held;
use transport::loopback::FarEnd;

use crate::{Framing, read_frame};

/// A pseudo-terminal: the device's end, and the line's end a line opens by
/// its path.
pub struct Terminal {
    device: BufReader<TTYPort>,
    /// The line's end, held open for as long as the device's: a
    /// pseudo-terminal's device end reads only a hang-up while no line end
    /// is open, which a device waiting for a node to open its line would
    /// meet first.
    line: TTYPort,
}

impl Terminal {
    /// A fresh pair, each read on the device's end waiting at most `wait`.
    ///
    /// # Errors
    /// Where the system gives no pseudo-terminal.
    pub fn pair(wait: Duration) -> Result<Self> {
        let (mut device, line) =
            TTYPort::pair().map_err(|e| classify("making a pseudo-terminal", &e.into()))?;
        if line.name().is_none() {
            return Err(protocol_error("a pseudo-terminal whose line has no path"));
        }
        device
            .set_timeout(wait)
            .map_err(|e| classify("setting the device's wait", &e.into()))?;
        Ok(Self {
            device: BufReader::new(device),
            line,
        })
    }

    /// The path a line opens to reach the device: `/dev/pts/3`.
    #[must_use]
    pub fn path(&self) -> String {
        self.line.name().unwrap_or_default()
    }

    /// The next frame the line wrote, framed as `framing` frames it.
    ///
    /// # Errors
    /// Where nothing came within the wait, or the line closed inside a
    /// frame.
    pub fn read_frame(&mut self, framing: &Framing) -> Result<Vec<u8>> {
        read_frame(&mut self.device, framing)
    }

    /// `bytes` written to the line, as the device sends them.
    ///
    /// # Errors
    /// Where the line took them not.
    pub fn write(&mut self, bytes: &[u8]) -> Result<()> {
        let device = self.device.get_mut();
        device
            .write_all(bytes)
            .and_then(|()| device.flush())
            .map_err(|e| classify("writing to the line", &e))
    }

    /// The far end of a line that writes one frame, framed as `framing`
    /// frames it: the device taking it whole.
    #[must_use]
    pub fn far_end(mut self, framing: Framing) -> Box<dyn FarEnd> {
        let path = self.path();
        let origin = format!("serial://{path}");
        Box::new(Held::new(path, move || {
            Ok(Taken::new(origin, self.read_frame(&framing)?))
        }))
    }

    /// The far end of a protocol's master on this line: every frame it
    /// writes, framed as `framing` frames it, answered as `hear` answers it,
    /// until `held` says what the device holds once a whole Stream reached
    /// it — the device of the protocol's own crate, which a loopback stands
    /// on the serial technology's bus in process.
    #[must_use]
    pub fn answering<H, D>(mut self, framing: Framing, mut hear: H, held: D) -> Box<dyn FarEnd>
    where
        H: FnMut(&[u8]) -> Result<Option<Vec<u8>>> + Send + 'static,
        D: Fn() -> Option<Taken> + Send + 'static,
    {
        Box::new(Held::new(self.path(), move || {
            loop {
                let frame = self.read_frame(&framing)?;
                if let Some(answer) = hear(&frame)? {
                    self.write(&answer)?;
                }
                if let Some(taken) = held() {
                    return Ok(taken);
                }
            }
        }))
    }
}
