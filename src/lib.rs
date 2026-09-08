#![forbid(unsafe_code)]

//! Streams that arrive on a serial port. One frame is one Stream.
//!
//! A serial line has no message boundary of its own, so the boundary is
//! configured: a delimiter the frame ends with, or a fixed length. Everything
//! between boundaries is the Stream; the port's bytes are never interpreted
//! here. The port itself — a COM port, `/dev/ttyUSB0`, an RS-485 adapter — is
//! opened through the `port` feature; the framing is what the tests hold and
//! is available on a box with no serial stack at all.
//!
//! The origin URI is the port: `serial:///dev/ttyUSB0?baud=9600`.

use std::io::BufRead;
use std::time::Duration;

use transport::error::{Result, classify, protocol_error};
use transport::{Arrived, Directions, Transport};

/// Where one frame ends.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Framing {
    /// The frame ends with these bytes, which are not part of it.
    Delimited(Vec<u8>),
    /// Every frame is exactly this long.
    Fixed(usize),
}

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

pub struct SerialTransport {
    port: String,
    baud: u32,
    framing: Framing,
    timeout: Duration,
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
            Framing::Fixed(length) => Err(protocol_error(format!(
                "a frame of {} bytes on a line framed at {length}",
                bytes.len()
            ))),
        }
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
}

impl Transport for SerialTransport {
    fn name(&self) -> &'static str {
        "serial"
    }

    fn directions(&self) -> Directions {
        Directions::BOTH
    }

    #[cfg(feature = "port")]
    fn receive(&self) -> Result<Vec<Arrived>> {
        let port = self.open()?;
        let mut reader = std::io::BufReader::new(port);
        Ok(vec![self.read_one(&mut reader)?])
    }

    #[cfg(not(feature = "port"))]
    fn receive(&self) -> Result<Vec<Arrived>> {
        Err(protocol_error(
            "built without the port feature: no serial stack on this box",
        ))
    }

    #[cfg(feature = "port")]
    fn send(&self, _target: &str, bytes: &[u8]) -> Result<()> {
        use std::io::Write;
        let mut port = self.open()?;
        port.write_all(&self.framed_bytes(bytes)?)
            .map_err(|e| classify("writing to the port", &e))?;
        port.flush().map_err(|e| classify("flushing the port", &e))
    }

    #[cfg(not(feature = "port"))]
    fn send(&self, _target: &str, _bytes: &[u8]) -> Result<()> {
        Err(protocol_error(
            "built without the port feature: no serial stack on this box",
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
        assert_eq!(line.name(), "serial");
    }
}
