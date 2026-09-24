#![forbid(unsafe_code)]

//! Streams that arrive on a serial port. One frame is one Stream.
//!
//! A serial line has no message boundary of its own, so the boundary is
//! configured: a delimiter the frame ends with, or a fixed length. Everything
//! between boundaries is the Stream; the port's bytes are never interpreted
//! here. The port itself — a COM port, `/dev/ttyUSB0`, an RS-485 adapter — is
//! opened through the `port` feature; the framing is what the tests hold and
//! is available on a box with no serial stack at all, over an in-memory line
//! both ends of a loopback share.
//!
//! The origin URI is the port: `serial:///dev/ttyUSB0?baud=9600`.
//!
//! Two lines, one boundary. A protocol on a serial line — M-Bus, HART —
//! writes frames to a [`Line`] and reads them from it; a port is one, and
//! [`bus::Bus`] is the other: a multi-drop bus in process, with addressed
//! devices on it, for every test and every box without a port.

pub mod bus;

use std::io::BufRead;
use std::sync::{Arc, Mutex, PoisonError};
use std::time::Duration;

use transport::error::{Result, classify, protocol_error};
use transport::line::Line;
use transport::loopback::{FarEnd, LOOPBACK_TIMEOUT, Loopback};
use transport::{Arrived, Directions, Transport};

/// Where one frame ends. Not comparable: a measured frame's rule is a
/// function, and two function pointers are not reliably equal.
#[derive(Clone, Debug)]
pub enum Framing {
    /// The frame ends with these bytes, which are not part of it.
    Delimited(Vec<u8>),
    /// Every frame is exactly this long.
    Fixed(usize),
    /// The frame says its own length in its first bytes, as M-Bus and HART
    /// frames do: given the bytes read so far, the protocol answers the
    /// frame's whole length, or `None` while it needs another byte to tell.
    Measured(Measure),
}

/// How long a frame is, from the bytes read of it so far; see
/// [`Framing::Measured`].
pub type Measure = fn(&[u8]) -> Result<Option<usize>>;

/// Read one frame from `reader` under `framing`.
///
/// # Errors
/// The line closed inside a frame, or a delimited frame outgrew [`MAX_FRAME`].
pub fn read_frame(reader: &mut impl BufRead, framing: &Framing) -> Result<Vec<u8>> {
    match framing {
        Framing::Fixed(length) => {
            let mut frame = vec![0u8; *length];
            reader
                .read_exact(&mut frame)
                .map_err(|e| classify("reading a fixed frame", &e))?;
            Ok(frame)
        }
        Framing::Measured(measure) => {
            let mut frame = Vec::new();
            loop {
                let mut byte = [0u8; 1];
                reader
                    .read_exact(&mut byte)
                    .map_err(|e| classify("reading a measured frame", &e))?;
                frame.push(byte[0]);
                if let Some(length) = measure(&frame)? {
                    if length > MAX_FRAME {
                        return Err(protocol_error("a frame over the size Xmip will read"));
                    }
                    let mut rest = vec![0u8; length.saturating_sub(frame.len())];
                    reader
                        .read_exact(&mut rest)
                        .map_err(|e| classify("reading a measured frame", &e))?;
                    frame.extend(rest);
                    return Ok(frame);
                }
            }
        }
        Framing::Delimited(delimiter) => {
            let last = *delimiter
                .last()
                .ok_or_else(|| protocol_error("an empty delimiter"))?;
            let mut frame = Vec::new();
            loop {
                let mut chunk = Vec::new();
                let read = reader
                    .read_until(last, &mut chunk)
                    .map_err(|e| classify("reading a delimited frame", &e))?;
                if read == 0 {
                    return Err(protocol_error("the line closed inside a frame"));
                }
                frame.extend_from_slice(&chunk);
                if frame.ends_with(delimiter) {
                    frame.truncate(frame.len() - delimiter.len());
                    return Ok(frame);
                }
                if frame.len() > MAX_FRAME {
                    return Err(protocol_error("a frame over the size Xmip will read"));
                }
            }
        }
    }
}

/// The most a delimited frame may be before the line is judged broken.
pub const MAX_FRAME: usize = 1024 * 1024;

/// An in-memory wire: what one end writes, the other reads, in order.
type Wire = Arc<Mutex<Vec<u8>>>;

#[derive(Clone)]
pub struct SerialTransport {
    port: String,
    baud: u32,
    framing: Framing,
    timeout: Duration,
    /// Set on a loopback: the line stands in for the device.
    line: Option<Wire>,
}

impl SerialTransport {
    /// `port` at `baud`, frames ending in `\r\n`.
    #[must_use]
    pub fn new(port: impl Into<String>, baud: u32) -> Self {
        Self {
            port: port.into(),
            baud,
            framing: Framing::Delimited(b"\r\n".to_vec()),
            timeout: Duration::from_secs(5),
            line: None,
        }
    }

    #[must_use]
    pub fn framed(mut self, framing: Framing) -> Self {
        self.framing = framing;
        self
    }

    #[must_use]
    pub const fn timing_out_after(mut self, timeout: Duration) -> Self {
        self.timeout = timeout;
        self
    }

    #[must_use]
    pub fn origin(&self) -> String {
        format!("serial://{}?baud={}", self.port, self.baud)
    }

    /// One frame from any byte source, framed as this port frames.
    ///
    /// # Errors
    /// As [`read_frame`].
    pub fn read_one(&self, reader: &mut impl BufRead) -> Result<Arrived> {
        let frame = read_frame(reader, &self.framing)?;
        Ok(Arrived::new(self.origin(), frame))
    }

    /// `bytes` as one frame on the line: the delimiter appended, or the fixed
    /// length enforced.
    ///
    /// # Errors
    /// A fixed-length frame of the wrong length.
    pub fn framed_bytes(&self, bytes: &[u8]) -> Result<Vec<u8>> {
        match &self.framing {
            Framing::Delimited(delimiter) => {
                let mut out = bytes.to_vec();
                out.extend_from_slice(delimiter);
                Ok(out)
            }
            Framing::Fixed(length) if bytes.len() == *length => Ok(bytes.to_vec()),
            Framing::Measured(_) => Ok(bytes.to_vec()),
            Framing::Fixed(length) => Err(protocol_error(format!(
                "a frame of {} bytes on a line framed at {length}",
                bytes.len()
            ))),
        }
    }

    /// One frame off the in-memory line, the rest left for the next.
    fn read_line(&self, line: &Wire) -> Result<Arrived> {
        let mut wire = line.lock().unwrap_or_else(PoisonError::into_inner);
        let mut reader: &[u8] = &wire;
        let arrived = self.read_one(&mut reader)?;
        let consumed = wire.len() - reader.len();
        wire.drain(..consumed);
        Ok(arrived)
    }

    /// `bytes` framed onto the in-memory line.
    fn write_line(&self, line: &Wire, bytes: &[u8]) -> Result<()> {
        let framed = self.framed_bytes(bytes)?;
        line.lock()
            .unwrap_or_else(PoisonError::into_inner)
            .extend_from_slice(&framed);
        Ok(())
    }

    #[cfg(feature = "port")]
    fn open(&self) -> Result<Box<dyn serialport::SerialPort>> {
        serialport::new(&self.port, self.baud)
            .timeout(self.timeout)
            .open()
            .map_err(|e| {
                transport::error::TransportError::retryable(format!("opening {}: {e}", self.port))
            })
    }

    #[cfg(feature = "port")]
    fn receive_port(&self) -> Result<Arrived> {
        let port = self.open()?;
        let mut reader = std::io::BufReader::new(port);
        self.read_one(&mut reader)
    }

    #[cfg(not(feature = "port"))]
    fn receive_port(&self) -> Result<Arrived> {
        Err(self.no_port())
    }

    /// The device cannot be opened on a build without the `port` feature.
    #[cfg(not(feature = "port"))]
    fn no_port(&self) -> transport::error::TransportError {
        protocol_error(format!(
            "built without the port feature: no serial stack on this box for {}",
            self.port
        ))
    }

    #[cfg(feature = "port")]
    fn send_port(&self, bytes: &[u8]) -> Result<()> {
        use std::io::Write;
        let mut port = self.open()?;
        port.write_all(&self.framed_bytes(bytes)?)
            .map_err(|e| classify("writing to the port", &e))?;
        port.flush().map_err(|e| classify("flushing the port", &e))
    }

    #[cfg(not(feature = "port"))]
    fn send_port(&self, _bytes: &[u8]) -> Result<()> {
        Err(self.no_port())
    }
}

impl Transport for SerialTransport {
    fn name(&self) -> &'static str {
        "serial"
    }

    fn directions(&self) -> Directions {
        Directions::BOTH
    }

    fn receive(&self) -> Result<Vec<Arrived>> {
        let arrived = match &self.line {
            Some(line) => self.read_line(line)?,
            None => self.receive_port()?,
        };
        Ok(vec![arrived])
    }

    fn send(&self, _target: &str, bytes: &[u8]) -> Result<()> {
        match &self.line {
            Some(line) => self.write_line(line, bytes),
            None => self.send_port(bytes),
        }
    }
}

impl SerialTransport {
    /// Both ends on one in-memory line: frames close with end-of-transmission,
    /// the loopback timeout stands where the device's would.
    #[must_use]
    pub fn loopback() -> Self {
        let mut line = Self::new("loopback", 9600)
            .framed(Framing::Delimited(vec![0x04]))
            .timing_out_after(LOOPBACK_TIMEOUT);
        line.line = Some(Wire::default());
        line
    }

    /// The framing that carries `payload`: this line's, unless the payload
    /// holds the delimiter, when it is framed by length instead — as a line
    /// with fixed records would be.
    fn framing_for(&self, payload: &[u8]) -> Framing {
        match &self.framing {
            Framing::Delimited(delimiter)
                if !delimiter.is_empty()
                    && payload.windows(delimiter.len()).any(|w| w == delimiter) =>
            {
                Framing::Fixed(payload.len())
            }
            framing => framing.clone(),
        }
    }
}

/// A serial port is a line: a frame goes out as the protocol wrote it, and
/// comes back framed as this port frames — [`Framing::Measured`] for a
/// protocol that says its own length.
impl Line for SerialTransport {
    fn name(&self) -> String {
        self.port.clone()
    }

    fn transmit(&self, frame: &[u8]) -> Result<()> {
        self.send("", frame)
    }

    fn receive(&self, _timeout: Duration) -> Result<Option<Vec<u8>>> {
        match Transport::receive(self) {
            Ok(arrived) => Ok(arrived.into_iter().next().map(|one| one.bytes)),
            // An in-memory line with nothing on it is silence, not a fault.
            Err(_)
                if self
                    .line
                    .as_ref()
                    .is_some_and(|line| lock_line(line).is_empty()) =>
            {
                Ok(None)
            }
            Err(error) => Err(error),
        }
    }
}

fn lock_line(line: &Wire) -> std::sync::MutexGuard<'_, Vec<u8>> {
    line.lock().unwrap_or_else(PoisonError::into_inner)
}

/// The line one frame was written to. Nothing waits: the round is in order.
struct Written(SerialTransport);

impl FarEnd for Written {
    fn address(&self) -> &str {
        &self.0.port
    }

    fn take_one(self: Box<Self>) -> Result<Arrived> {
        Transport::receive(&self.0)?
            .into_iter()
            .next()
            .ok_or_else(|| protocol_error("written, but nothing came off the line"))
    }
}

impl Loopback for SerialTransport {
    fn far_end(&self) -> Result<Box<dyn FarEnd>> {
        if self.line.is_none() {
            return Err(protocol_error("a device, not a loopback line"));
        }
        Ok(Box::new(Written(self.clone())))
    }

    fn send_to(&self, address: &str, payload: &[u8]) -> Result<()> {
        self.send(address, payload)
    }

    fn unblock(&self, _address: &str) {}

    /// In order on one thread: a line does not listen, so the write goes
    /// first and the read finds it, both ends framed for this payload.
    fn round(&self, payload: &[u8]) -> Result<Arrived> {
        self.clone()
            .framed(self.framing_for(payload))
            .round_in_order(payload)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use transport::payload::edge_payloads;

    #[test]
    fn delimited_frames_end_at_the_delimiter_and_keep_partial_matches() {
        let line = SerialTransport::new("COM3", 9600);
        let mut reader = std::io::BufReader::new(&b"first\r\nsec\rond\r\n"[..]);
        assert_eq!(line.read_one(&mut reader).expect("frame").bytes, b"first");
        assert_eq!(
            line.read_one(&mut reader).expect("frame").bytes,
            b"sec\rond"
        );
        assert!(line.read_one(&mut reader).is_err(), "the line closed");
        assert_eq!(line.framed_bytes(b"go").expect("framed"), b"go\r\n");
        assert_eq!(line.origin(), "serial://COM3?baud=9600");
    }

    #[test]
    fn fixed_frames_are_exactly_their_length() {
        let line = SerialTransport::new("/dev/ttyUSB0", 115_200).framed(Framing::Fixed(4));
        let mut reader = std::io::BufReader::new(&b"abcdefgh"[..]);
        assert_eq!(line.read_one(&mut reader).expect("frame").bytes, b"abcd");
        assert_eq!(line.read_one(&mut reader).expect("frame").bytes, b"efgh");
        assert!(line.framed_bytes(b"abc").is_err());
        assert_eq!(line.framed_bytes(b"abcd").expect("framed"), b"abcd");
    }

    #[test]
    fn a_line_has_no_artefact_to_claim() {
        let line = SerialTransport::new("COM1", 9600).timing_out_after(Duration::from_millis(10));
        assert!(line.claims().is_none());
        assert_eq!(Transport::name(&line), "serial");
        assert!(line.far_end().is_err(), "a device is not a loopback line");
    }

    #[test]
    fn a_loopback_round_frames_the_line_and_reads_it_back() {
        let loopback = SerialTransport::loopback();
        let arrived = loopback.round(b"line\r\nbreak").expect("delimited");
        assert_eq!(arrived.bytes, b"line\r\nbreak");
        assert_eq!(arrived.origin_uri, "serial://loopback?baud=9600");
        // End-of-transmission closes a frame; a payload that carries one is
        // framed by length instead.
        let arrived = loopback.round(b"eot\x04inside").expect("fixed");
        assert_eq!(arrived.bytes, b"eot\x04inside");
        assert!(matches!(
            loopback.framing_for(b"eot\x04inside"),
            Framing::Fixed(10)
        ));
        assert!(loopback.ceiling().is_none());
        assert!(loopback.refuses(b"eot\x04inside").is_none());
    }

    #[test]
    fn the_loopback_line_carries_frames_in_order_through_the_transport_trait() {
        let loopback = SerialTransport::loopback();
        loopback.send("loopback", b"first").expect("writing");
        loopback.send("loopback", b"second").expect("writing");
        assert_eq!(
            Transport::receive(&loopback).expect("reading")[0].bytes,
            b"first"
        );
        assert_eq!(
            Transport::receive(&loopback).expect("reading")[0].bytes,
            b"second"
        );
        assert!(Transport::receive(&loopback).is_err(), "the line is empty");
    }

    #[test]
    fn a_measured_frame_is_read_to_the_length_its_header_says() {
        // A frame that opens with its length: 0x02 then two bytes. A length
        // of zero opens no frame, and the protocol says so.
        fn measure(read: &[u8]) -> Result<Option<usize>> {
            match read.first() {
                Some(0) => Err(protocol_error("a frame of no length")),
                length => Ok(length.map(|length| usize::from(*length) + 1)),
            }
        }
        let port = SerialTransport::new("COM4", 2400).framed(Framing::Measured(measure));
        let mut reader = std::io::BufReader::new(&b"\x02ab\x01c\x00"[..]);
        assert_eq!(port.read_one(&mut reader).expect("frame").bytes, b"\x02ab");
        assert_eq!(port.read_one(&mut reader).expect("frame").bytes, b"\x01c");
        assert!(port.read_one(&mut reader).is_err(), "no length");
        assert!(port.read_one(&mut reader).is_err(), "the line closed");
    }

    #[test]
    fn a_loopback_wire_is_a_line_and_its_silence_is_none() {
        let wire = SerialTransport::loopback();
        assert_eq!(Line::name(&wire), "loopback");
        Line::transmit(&wire, b"frame").expect("sent");
        assert_eq!(
            Line::receive(&wire, Duration::ZERO).expect("read"),
            Some(b"frame".to_vec())
        );
        assert_eq!(Line::receive(&wire, Duration::ZERO).expect("silence"), None);
    }

    #[test]
    fn the_loopback_returns_the_edges_whole() {
        let loopback = SerialTransport::loopback();
        for (name, bytes) in edge_payloads() {
            let arrived = loopback
                .round(&bytes)
                .unwrap_or_else(|error| panic!("{name}: {error}"));
            assert_eq!(arrived.bytes, bytes, "{name}");
        }
    }
}
