//! A device on a bench, the fake every test of a line's port drives: handed
//! in as what opens the port, so the tests read and write as a device is
//! read and written, with no device and no sleep. Under `test-support` for
//! another crate's tests, such as the runtime's, which shows a node's
//! Locations on one device sharing its one port.

use std::collections::VecDeque;
use std::io::{ErrorKind, Read, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, Weak};
use std::time::Duration;
use xcore::lock::Locked;
use xcore::wake::{Revision, Wake};

use transport::error::{Result, TransportError};

use crate::{Framing, Opens, Port, SerialTransport, Watch};

/// A device on a bench, handed in as what opens a line's port. Its port
/// opens for one handle at a time, as the operating system's does: a
/// second open while one is open — the port, or its watch — is refused.
/// What it sends lands in the port open at the time and is lost where none
/// is, as on a real line. It keeps what was written to it, every wait and
/// speed set on a port it opened and every wait a watch was handed, and a
/// port it breaks fails its next read. Where it answers, each frame written
/// to it is answered at once with the frame in capitals, over whatever it
/// sent before that was not read yet: an answer a frame was written over
/// is lost, as two speakers on one pair of wires lose it.
///
/// Its silence is over at once: a read or a watch of a port it sent nothing
/// times out at once, unless it [`Bench::waits`], when a watch waits until
/// it sends, the port closes, or the wait runs out.
#[derive(Clone, Default)]
pub struct Bench {
    ports: Arc<Mutex<Vec<Weak<Sent>>>>,
    pub(crate) written: Arc<Mutex<Vec<u8>>>,
    pub(crate) waits: Arc<Mutex<Vec<Duration>>>,
    pub(crate) watched: Arc<Mutex<Vec<Duration>>>,
    pub(crate) speeds: Arc<Mutex<Vec<u32>>>,
    pub(crate) broken: Arc<AtomicBool>,
    pub(crate) answers: Arc<AtomicBool>,
    /// Whether a watch waits for the device, rather than its silence being
    /// over at once.
    idles: Arc<AtomicBool>,
    /// Raised as each watch begins to wait for the device.
    watching: Arc<Revision>,
}

impl Bench {
    /// A device that answers every frame written to it.
    #[must_use]
    pub fn answering() -> Self {
        let bench = Self::default();
        bench.answers.store(true, Ordering::Relaxed);
        bench
    }

    /// A device whose silence lasts: a watch of its port waits until it
    /// sends.
    #[must_use]
    pub fn waits() -> Self {
        let bench = Self::default();
        bench.idles.store(true, Ordering::Relaxed);
        bench
    }

    /// The device sends `bytes`, into the port open now.
    pub fn sends(&self, bytes: &[u8]) {
        self.sent(bytes, false);
    }

    /// `bytes` into the port open now, over what was sent before and not
    /// read yet where `over`, and whatever waits on it told.
    fn sent(&self, bytes: &[u8], over: bool) {
        for port in self.ports.locked().iter().filter_map(Weak::upgrade) {
            let mut sent = port.bytes.locked();
            if over {
                sent.clear();
            }
            sent.extend(bytes);
            port.came.notify_all();
        }
    }

    /// How many times a port was opened on the bench.
    #[must_use]
    pub fn opened(&self) -> usize {
        self.ports.locked().len()
    }

    /// What was written to the device, in order.
    #[must_use]
    pub fn written(&self) -> Vec<u8> {
        self.written.locked().clone()
    }

    /// Waited, at most `within`, until `count` watches have begun to wait
    /// for the device since the bench was set up; whether they had.
    #[must_use]
    pub fn watching(&self, count: u64, within: Duration) -> bool {
        self.watching.wait(count.saturating_sub(1), within) >= count
    }

    /// A line on the bench at `COM9`, 2400 baud, its frames `length` bytes
    /// long.
    #[must_use]
    pub fn line(&self, length: usize) -> SerialTransport {
        self.at("COM9", 2400, length)
    }

    /// A line on the bench at `port` and `baud`, its frames `length` bytes
    /// long.
    #[must_use]
    pub fn at(&self, port: &str, baud: u32, length: usize) -> SerialTransport {
        SerialTransport::new(port, baud)
            .framed(Framing::Fixed(length))
            .opened_by(Arc::new(self.clone()))
    }
}

impl Opens for Bench {
    fn open(&self, port: &str, _baud: u32) -> Result<Box<dyn Port>> {
        let mut ports = self.ports.locked();
        if ports.iter().any(|open| open.strong_count() > 0) {
            return Err(TransportError::retryable(format!(
                "opening {port}: the port is open to another handle"
            )));
        }
        let sent = Arc::default();
        ports.push(Arc::downgrade(&sent));
        Ok(Box::new(Benched {
            sent,
            bench: self.clone(),
        }))
    }
}

/// What the device sent a port, not yet read off it; told as it sends, and
/// whether the port is closing.
#[derive(Default)]
struct Sent {
    bytes: Mutex<VecDeque<u8>>,
    came: Wake,
    closing: AtomicBool,
}

/// A port the bench opened.
struct Benched {
    sent: Arc<Sent>,
    bench: Bench,
}

/// What the device sent, in order; a read with nothing there times out, as
/// `serialport`'s does.
impl Read for Benched {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        if self.bench.broken.swap(false, Ordering::Relaxed) {
            return Err(ErrorKind::BrokenPipe.into());
        }
        match self.sent.bytes.locked().read(buf)? {
            0 => Err(ErrorKind::TimedOut.into()),
            read => Ok(read),
        }
    }
}

impl Write for Benched {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        self.bench.written.locked().extend_from_slice(buf);
        if self.bench.answers.load(Ordering::Relaxed) {
            self.bench.sent(&buf.to_ascii_uppercase(), true);
        }
        Ok(buf.len())
    }

    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

impl Port for Benched {
    fn wait(&mut self, wait: Duration) -> std::io::Result<()> {
        self.bench.waits.locked().push(wait);
        Ok(())
    }

    fn speed(&mut self, baud: u32) -> std::io::Result<()> {
        self.bench.speeds.locked().push(baud);
        Ok(())
    }

    fn watch(&self) -> std::io::Result<Arc<dyn Watch>> {
        Ok(Arc::new(Watched {
            sent: Arc::clone(&self.sent),
            bench: self.bench.clone(),
        }))
    }
}

/// A watch of a port the bench opened, holding the port open as the
/// operating system's watch holds its device.
struct Watched {
    sent: Arc<Sent>,
    bench: Bench,
}

impl Watch for Watched {
    fn readable(&self, wait: Duration) -> std::io::Result<bool> {
        self.bench.watched.locked().push(wait);
        let closing = || self.sent.closing.load(Ordering::Relaxed);
        let sent = self.sent.bytes.locked();
        if !sent.is_empty() || closing() {
            return Ok(true);
        }
        if !self.bench.idles.load(Ordering::Relaxed) {
            return Ok(false);
        }
        self.bench.watching.raise();
        let (_sent, ran_out) =
            self.sent
                .came
                .wait_timeout_while(&self.sent.bytes, sent, wait, |sent| {
                    sent.is_empty() && !closing()
                });
        Ok(!ran_out)
    }

    fn cancel(&self) {
        let _sent = self.sent.bytes.locked();
        self.sent.closing.store(true, Ordering::Relaxed);
        self.sent.came.notify_all();
    }
}
