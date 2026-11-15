#![forbid(unsafe_code)]

//! Streams that arrive on a serial port. One frame is one Stream.
//!
//! A serial line has no message boundary of its own, so the boundary is
//! configured (`framing.rs`): a delimiter the frame ends with, or a fixed
//! length, or the length the frame says of itself. Everything
//! between boundaries is the Stream; the port's bytes are never interpreted
//! here. The port itself — a COM port, `/dev/ttyUSB0`, an RS-485 adapter — is
//! opened through the `port` feature; the framing is what the tests hold and
//! is available on a box with no serial stack at all, over an in-memory line
//! both ends of a loopback share.
//!
//! **The port is opened once and kept open** for as long as the line is
//! used, by every clone of it: a device's answer to a write lands in the
//! port's buffer while it is open, and is lost where no handle is. Each read
//! sets its own wait on the open port, each write the port's own timeout; a
//! read the wait runs out on with nothing is nothing arrived, never an error.
//! What the port is opened through is handed in ([`Opens`], ADR-0068): the
//! operating system's ([`SystemPorts`]) unless another is.
//!
//! The origin URI is the port: `serial:///dev/ttyUSB0?baud=9600`.
//!
//! A protocol on a serial line — M-Bus, HART — writes frames to a [`Line`]
//! and reads them from it, and a port is one. The multi-drop bus a protocol is
//! tested on, with addressed devices on it, is the SDK's simulator,
//! `sdk::serial::Bus`: the same boundary, in process.
//!
//! **Acceptance is at-most-once here** ([`AT_MOST_ONCE`]): a frame read off
//! the line is gone, and a raw line has no reply to defer.

use std::io::{BufRead, BufReader, ErrorKind};
use std::sync::{Arc, Mutex};
use std::time::Duration;
use xcore::lock::Locked;

use transport::ArrivalIdentity;
use transport::arrived::next_arrival;
use transport::error::{Result, classify, protocol_error};
use transport::held::Held;
use transport::line::Line;
use transport::loopback::{FarEnd, LOOPBACK_TIMEOUT, Loopback};
use transport::{Acknowledgement, Arrived, Configured, Directions, Taken, Transport};

mod framing;
mod port;
mod settings;

pub use framing::{Framing, MAX_FRAME, Measure, read_frame};
pub use port::{Opens, Port, SystemPorts};

use crate::port::Wire;

/// Why a frame read off a serial line is not acknowledged after the cycle.
pub const AT_MOST_ONCE: &str = "a serial line has no reply: a frame read off it is gone from \
                                the device's buffer, and the device does not send it again";

/// How long a read or a write waits on the port until a Location says
/// otherwise; a protocol reading the port as a [`Line`] hands each read its
/// own wait.
pub const TIMEOUT: Duration = Duration::from_secs(5);

/// An open port, and what was read off it ahead of the frame being read.
type Open = BufReader<Box<dyn Port>>;

#[derive(Clone)]
pub struct SerialTransport {
    port: String,
    baud: u32,
    framing: Framing,
    timeout: Duration,
    /// What the port is opened through: the operating system's, unless
    /// another is handed in.
    opens: Arc<dyn Opens>,
    /// The port once opened, shared by every clone and kept open for as
    /// long as the line is used.
    open: Arc<Mutex<Option<Open>>>,
}

impl SerialTransport {
    /// `port` at `baud`, frames ending in `\r\n`, on the operating system's
    /// port of that name.
    #[must_use]
    pub fn new(port: impl Into<String>, baud: u32) -> Self {
        Self {
            port: port.into(),
            baud,
            framing: Framing::Delimited(b"\r\n".to_vec()),
            timeout: TIMEOUT,
            opens: Arc::new(SystemPorts),
            open: Arc::default(),
        }
    }

    /// The same line on the port `opens` opens — a provider's, or a test's
    /// fake (ADR-0068) — in place of the operating system's.
    #[must_use]
    pub fn opened_by(mut self, opens: Arc<dyn Opens>) -> Self {
        self.opens = opens;
        self.open = Arc::default();
        self
    }

    #[must_use]
    pub fn framed(mut self, framing: Framing) -> Self {
        self.framing = framing;
        self
    }

    #[must_use]
    pub fn origin(&self) -> String {
        format!("serial://{}?baud={}", self.port, self.baud)
    }

    /// The port the line is open on — `COM3`, `/dev/ttyUSB0`, `loopback` —
    /// which a protocol riding on the line names in its own origin.
    #[must_use]
    pub fn port(&self) -> &str {
        &self.port
    }

    /// One frame from any byte source, framed as this port frames: whole,
    /// and at-most-once ([`AT_MOST_ONCE`]).
    ///
    /// # Errors
    /// As [`read_frame`].
    pub fn read_one(&self, reader: &mut impl BufRead) -> Result<Arrived> {
        let frame = read_frame(reader, &self.framing)?;
        Ok(Arrived::whole(
            self.origin(),
            frame,
            Acknowledgement::at_most_once(AT_MOST_ONCE),
        ))
    }

    /// `bytes` as one frame on the line: the delimiter appended, or the fixed
    /// length enforced.
    ///
    /// # Errors
    /// A fixed-length frame of the wrong length.
    fn framed_bytes(&self, bytes: &[u8]) -> Result<Vec<u8>> {
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

    /// The port, opened at the line's first use and kept open for the rest.
    fn opened<'a>(&self, open: &'a mut Option<Open>) -> Result<&'a mut Open> {
        let port = match open.take() {
            Some(port) => port,
            None => BufReader::new(self.opens.open(&self.port, self.baud)?),
        };
        Ok(open.insert(port))
    }

    /// One frame off the line, each read of the port waiting at most
    /// `wait`; `None` when nothing arrived within it. A frame begun and cut
    /// off by the wait is an error: something arrived, and it was no frame.
    /// A failure of the port itself closes it, for the next use to open it
    /// again.
    fn read(&self, wait: Duration) -> Result<Option<Arrived>> {
        let mut open = self.open.locked();
        let port = self.opened(&mut open)?;
        let begun = port
            .get_mut()
            .wait(wait)
            .and_then(|()| port.fill_buf().map(|read| !read.is_empty()));
        match begun {
            Ok(true) => self.read_one(port).map(Some),
            Ok(false) => Ok(None),
            Err(e) if e.kind() == ErrorKind::TimedOut => Ok(None),
            Err(e) => {
                *open = None;
                Err(classify("reading the port", &e))
            }
        }
    }

    /// `bytes` framed onto the line, the write waiting at most the port's
    /// own timeout. A failure closes the port, for the next use to open it
    /// again.
    fn write(&self, bytes: &[u8]) -> Result<()> {
        let framed = self.framed_bytes(bytes)?;
        let mut open = self.open.locked();
        let port = self.opened(&mut open)?.get_mut();
        put(port.as_mut(), self.timeout, &framed).map_err(|e| {
            *open = None;
            classify("writing to the port", &e)
        })
    }
}

/// `framed` written to `port` and flushed, each waiting at most `wait`.
fn put(port: &mut dyn Port, wait: Duration, framed: &[u8]) -> std::io::Result<()> {
    port.wait(wait)?;
    port.write_all(framed)?;
    port.flush()
}

impl Transport for SerialTransport {
    fn name(&self) -> &'static str {
        "serial"
    }

    fn directions(&self) -> Directions {
        Directions::BOTH
    }

    fn arrivals(&self) -> transport::Arrivals {
        transport::Arrivals::Ordered("one line or bus, answered in the order it speaks")
    }

    fn waiting(&self) -> transport::Waiting {
        transport::Waiting::Waits("a read of the line waits for a frame or its timeout")
    }

    /// One frame off the port or the loopback line, or none where nothing
    /// arrived within the port's timeout. Acceptance is at-most-once here:
    /// a raw line has no reply to defer ([`AT_MOST_ONCE`]).
    fn receive_up_to(&self, _most: usize) -> Result<Vec<Arrived>> {
        Ok(self.read(self.timeout)?.into_iter().collect())
    }

    fn sending(&self) -> transport::Sending {
        transport::Sending::Gathered(
            "a frame's delimiter or length is chosen by whether the body holds the delimiter",
        )
    }

    fn send_from(&self, _target: &str, outgoing: transport::Outgoing<'_>) -> Result<()> {
        self.write(&outgoing.gather()?)
    }
}

impl SerialTransport {
    /// Both ends on one in-memory line, read and written as a port is:
    /// frames close with end-of-transmission, the loopback timeout stands
    /// where the device's would.
    #[must_use]
    pub fn loopback() -> Self {
        Self::new("loopback", 9600)
            .framed(Framing::Delimited(vec![0x04]))
            .opened_by(Arc::new(Wire::default()))
            .timing_out_after(LOOPBACK_TIMEOUT)
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
/// protocol that says its own length. Transmit and receive go through the
/// one port, kept open between them, so an answer that comes before the
/// receive is read by it. A read waits what the protocol hands it, a write
/// the port's own timeout.
impl Line for SerialTransport {
    fn name(&self) -> String {
        self.port.clone()
    }

    fn transmit(&self, frame: &[u8]) -> Result<()> {
        self.write(frame)
    }

    /// `None` where nothing arrived within `timeout`; a frame cut off by it
    /// is an error.
    fn receive(&self, timeout: Duration) -> Result<Option<Vec<u8>>> {
        self.read(timeout)?
            .map(|arrived| arrived.taken().map(|taken| taken.bytes))
            .transpose()
    }
}

impl Loopback for SerialTransport {
    fn arrival_identity(&self) -> ArrivalIdentity {
        ArrivalIdentity::Unnamed("a serial line has one far end, and names it nowhere")
    }

    /// The line one frame was written to. Nothing waits: the round is in
    /// order.
    fn far_end(&self) -> Result<Box<dyn FarEnd>> {
        if !self.opens.in_process() {
            return Err(protocol_error("a device, not a loopback line"));
        }
        let line = self.clone();
        Ok(Box::new(Held::new(self.port.clone(), move || {
            next_arrival(
                Transport::receive(&line)?,
                "written, but nothing came off the line",
            )?
            .taken()
        })))
    }

    fn send_to(&self, address: &str, payload: &[u8]) -> Result<()> {
        self.send(address, payload)
    }

    /// A line does not listen: its two ends exchange in order, and there is
    /// nothing for a failed write to release.
    fn exchanges_in_order(&self) -> bool {
        true
    }

    /// In order on one thread: a line does not listen, so the write goes
    /// first and the read finds it, both ends framed for this payload.
    fn round(&self, payload: &[u8]) -> Result<Taken> {
        self.clone()
            .framed(self.framing_for(payload))
            .round_in_order(payload)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::assert_matches;
    use std::collections::VecDeque;
    use std::io::{Read, Write};
    use std::sync::Weak;
    use std::sync::atomic::{AtomicBool, Ordering};
    use transport::payload::edge_payloads;

    /// The bytes of the one frame `read` read, taken.
    fn frame(read: Result<Arrived>) -> Vec<u8> {
        read.expect("frame").taken().expect("taken").bytes
    }

    #[test]
    fn delimited_frames_end_at_the_delimiter_and_keep_partial_matches() {
        let line = SerialTransport::new("COM3", 9600);
        let mut reader = std::io::BufReader::new(&b"first\r\nsec\rond\r\n"[..]);
        let first = line.read_one(&mut reader).expect("frame");
        assert!(!first.defers(), "a serial line is at-most-once");
        assert_eq!(first.taken().expect("taken").bytes, b"first");
        assert_eq!(frame(line.read_one(&mut reader)), b"sec\rond");
        assert!(line.read_one(&mut reader).is_err(), "the line closed");
        assert_eq!(line.framed_bytes(b"go").expect("framed"), b"go\r\n");
        assert_eq!(line.origin(), "serial://COM3?baud=9600");
        assert_eq!(line.port(), "COM3");
    }

    #[test]
    fn fixed_frames_are_exactly_their_length() {
        let line = SerialTransport::new("/dev/ttyUSB0", 115_200).framed(Framing::Fixed(4));
        let mut reader = std::io::BufReader::new(&b"abcdefgh"[..]);
        assert_eq!(frame(line.read_one(&mut reader)), b"abcd");
        assert_eq!(frame(line.read_one(&mut reader)), b"efgh");
        assert!(line.framed_bytes(b"abc").is_err());
        assert_eq!(line.framed_bytes(b"abcd").expect("framed"), b"abcd");
    }

    #[test]
    fn a_line_has_no_artifact_to_claim() {
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
        assert_matches!(loopback.framing_for(b"eot\x04inside"), Framing::Fixed(10));
        assert!(loopback.ceiling().is_none());
        assert!(loopback.refuses(b"eot\x04inside").is_none());
    }

    #[test]
    fn the_loopback_line_carries_frames_in_order_through_the_transport_trait() {
        let loopback = SerialTransport::loopback();
        loopback.send("loopback", b"first").expect("writing");
        loopback.send("loopback", b"second").expect("writing");
        for sent in [&b"first"[..], b"second"] {
            let read = Transport::receive(&loopback).expect("reading");
            assert_eq!(frame(next_arrival(read, "a frame")), sent);
        }
        let quiet = Transport::receive(&loopback).expect("nothing there is no error");
        assert!(quiet.is_empty(), "the line is empty");
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
        assert_eq!(frame(port.read_one(&mut reader)), b"\x02ab");
        assert_eq!(frame(port.read_one(&mut reader)), b"\x01c");
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

    /// A device on a bench, handed in as what opens the line's port: what
    /// it sends lands in each port open at the time and is lost where none
    /// is, as on a real line. It keeps what was written to it and every
    /// wait set on a port it opened, and a port it breaks fails its next
    /// read.
    #[derive(Clone, Default)]
    struct Bench {
        ports: Arc<Mutex<Vec<Weak<Sent>>>>,
        written: Arc<Mutex<Vec<u8>>>,
        waits: Arc<Mutex<Vec<Duration>>>,
        broken: Arc<AtomicBool>,
    }

    impl Bench {
        /// The device sends `bytes`, into every port open now.
        fn sends(&self, bytes: &[u8]) {
            for port in self.ports.locked().iter().filter_map(Weak::upgrade) {
                port.locked().extend(bytes);
            }
        }

        /// How many times a port was opened on the bench.
        fn opened(&self) -> usize {
            self.ports.locked().len()
        }

        /// A line on the bench, its frames `length` bytes long.
        fn line(&self, length: usize) -> SerialTransport {
            SerialTransport::new("COM9", 2400)
                .framed(Framing::Fixed(length))
                .opened_by(Arc::new(self.clone()))
        }
    }

    impl Opens for Bench {
        fn open(&self, _port: &str, _baud: u32) -> Result<Box<dyn Port>> {
            let sent = Arc::default();
            self.ports.locked().push(Arc::downgrade(&sent));
            Ok(Box::new(Benched {
                sent,
                bench: self.clone(),
            }))
        }
    }

    /// What the device sent a port, not yet read off it.
    type Sent = Mutex<VecDeque<u8>>;

    /// A port the bench opened.
    struct Benched {
        sent: Arc<Sent>,
        bench: Bench,
    }

    /// What the device sent, in order; a read with nothing there times
    /// out, as `serialport`'s does.
    impl Read for Benched {
        fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
            if self.bench.broken.swap(false, Ordering::Relaxed) {
                return Err(ErrorKind::BrokenPipe.into());
            }
            match self.sent.locked().read(buf)? {
                0 => Err(ErrorKind::TimedOut.into()),
                read => Ok(read),
            }
        }
    }

    impl Write for Benched {
        fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
            self.bench.written.locked().extend_from_slice(buf);
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
    }

    /// What the device sends between a write and the read after it waits
    /// in the port, kept open across both; a port opened for each had no
    /// handle open when it came, and lost it.
    #[test]
    fn an_answer_sent_before_the_read_is_read_off_the_port_kept_open() {
        let bench = Bench::default();
        let line = bench.line(3);
        Line::transmit(&line, b"ask").expect("sent");
        bench.sends(b"yes");
        let answer = Line::receive(&line, Duration::from_millis(250)).expect("read");
        assert_eq!(answer, Some(b"yes".to_vec()));
        assert_eq!(*bench.written.locked(), b"ask");
        assert_eq!(bench.opened(), 1, "opened once, kept open");
    }

    /// A read the wait runs out on with nothing is nothing arrived, to a
    /// line and to a Receive Location alike, and leaves the port open; a
    /// frame begun and cut off by the wait is an error.
    #[test]
    fn a_read_that_times_out_with_nothing_is_nothing_arrived() {
        let bench = Bench::default();
        let line = bench.line(3);
        let quiet = Line::receive(&line, Duration::from_millis(250)).expect("quiet");
        assert_eq!(quiet, None);
        assert!(Transport::receive(&line).expect("quiet").is_empty());
        bench.sends(b"ye");
        let cut = Line::receive(&line, Duration::from_millis(250));
        assert!(cut.is_err(), "a frame cut off is no silence");
        assert_eq!(bench.opened(), 1, "a timeout leaves the port open");
    }

    /// Each read sets the wait it is handed on the open port, a write the
    /// port's own timeout, and a Receive Location's read that too.
    #[test]
    fn the_wait_handed_to_each_read_is_set_on_the_open_port() {
        let bench = Bench::default();
        let line = bench.line(3);
        Line::receive(&line, Duration::from_millis(250)).expect("quiet");
        Line::transmit(&line, b"ask").expect("sent");
        Line::receive(&line, Duration::from_millis(40)).expect("quiet");
        Transport::receive(&line).expect("quiet");
        let waits = [
            Duration::from_millis(250),
            TIMEOUT,
            Duration::from_millis(40),
            TIMEOUT,
        ];
        assert_eq!(*bench.waits.locked(), waits);
    }

    /// A port that fails is closed, and the next use opens it again.
    #[test]
    fn a_port_that_fails_is_opened_again_at_the_next_use() {
        let bench = Bench::default();
        let line = bench.line(3);
        Line::transmit(&line, b"ask").expect("sent");
        bench.broken.store(true, Ordering::Relaxed);
        assert!(Line::receive(&line, Duration::from_millis(250)).is_err());
        Line::transmit(&line, b"ask").expect("sent again");
        assert_eq!(bench.opened(), 2);
    }

    /// A port no box has fails to open, worth asking again, and is asked
    /// again at the next use rather than kept failed.
    #[cfg(feature = "port")]
    #[test]
    fn a_port_no_box_has_fails_to_open_and_is_asked_again() {
        let port = SerialTransport::new("xmip-no-such-port", 2400);
        let error = Line::receive(&port, Duration::from_millis(250)).expect_err("no port");
        assert!(error.retryable, "{error}");
        assert!(error.message.contains("xmip-no-such-port"), "{error}");
        assert!(Transport::receive(&port).is_err(), "refused again");
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
