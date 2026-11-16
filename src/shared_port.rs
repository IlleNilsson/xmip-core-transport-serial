//! One device's port, shared by every Location on a node that names it
//! (ADR-0064, amendment 2026-11-15).
//!
//! The operating system opens a serial port for one handle at a time —
//! exclusively on Windows, and `serialport` opens it so everywhere else — so
//! while each Location opened its own, several M-Bus meters on one bus, or a
//! Send and a Receive Location on one port, left every Location but the
//! first refused, "opening COM3", at each cycle. On a node a line takes its
//! device's one port from what the node's Locations share
//! (`transport::Sharing`), by the path its Location names; a line built
//! outside a node keeps one of its own.
//!
//! The port is opened at its first use, through what the first line naming
//! it opens with ([`crate::Opens`]), and kept open; a port that fails is
//! closed, and the next exchange of any Location on the device opens it
//! again, once for all of them. An [`Exchange`] holds the port from its first
//! frame written to its last read, so no other Location writes or reads
//! between them and an answer reaches only the Location that asked;
//! Locations on two devices hold two ports and never wait on each other.
//! Each exchange runs at its own Location's speed, set on the port where
//! another Location left it at another.
//!
//! **A receive holds the port only while a frame arrives** ([`arriving`];
//! the owner, 2026-11-15). Until bytes are there it waits on the port's
//! [`Watch`], holding nothing, so another Location writes and exchanges
//! meanwhile and a writer waits at most for a frame in flight; once they
//! are there it takes the port and holds it until the frame is whole or
//! cut. One receive waits on the watch at a time, as Windows takes one
//! `WaitCommEvent` on a port; another waiting receive waits for what that
//! one sees. A port closed while a receive waits on its watch ends the
//! wait, and the close waits for it, so nothing holds the device open once
//! the port is closed.

use std::io::{BufRead, BufReader, ErrorKind};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;
use xcore::clock::{Clock, SystemClock, between};
use xcore::lock::Locked;
use xcore::wake::Wake;

use transport::error::{Result, classify};

use crate::{Framing, Opens, Port, SerialTransport, Watch, read_frame};

/// One device's port: what opens it, the port once open, and the receives
/// waiting for its next bytes.
pub(crate) struct SharedPort {
    opens: Arc<dyn Opens>,
    open: Mutex<Option<Open>>,
    watching: Mutex<Watching>,
    /// Told whenever a receive takes the watch or lets it go.
    watched: Wake,
}

/// An open port, what was read off it ahead of the frame being read, what
/// waits for its next bytes, and the speed it runs at.
struct Open {
    port: BufReader<Box<dyn Port>>,
    watch: Arc<dyn Watch>,
    baud: u32,
}

/// Who waits on a port's watch.
#[derive(Default)]
struct Watching {
    /// Whether a receive waits on the watch now: one at a time.
    led: bool,
    /// How many of its waits ended with bytes come, or the port closing: a
    /// reason for every receive waiting to look again.
    woken: u64,
}

/// How a receive holding no port waits for its next bytes.
enum Waits {
    /// On the port's watch, which no other receive waits on.
    Watching(Arc<dyn Watch>),
    /// For the receive on the watch to see something, past its count of
    /// wakes then.
    Following(u64),
}

impl SharedPort {
    /// A device's port, opened through `opens` at its first use.
    pub(crate) fn new(opens: Arc<dyn Opens>) -> Self {
        Self {
            opens,
            open: Mutex::new(None),
            watching: Mutex::default(),
            watched: Wake::new(),
        }
    }

    /// What the port is opened through.
    pub(crate) const fn opens(&self) -> &Arc<dyn Opens> {
        &self.opens
    }
}

/// One exchange on a device's port: the port held from the first frame
/// written to the last frame read, so no other Location naming the device
/// writes or reads between them, and an answer reaches only the Location
/// that asked. Every other Location on the device waits until it is
/// dropped; a write waits at most its line's own timeout, a read what it is
/// handed.
pub struct Exchange<'a> {
    line: &'a SerialTransport,
    open: MutexGuard<'a, Option<Open>>,
}

impl<'a> Exchange<'a> {
    /// `line`'s device's port, held.
    pub(crate) fn held(line: &'a SerialTransport) -> Self {
        Self {
            line,
            open: line.shared.open.locked(),
        }
    }

    /// `frame` on the line as `framing` frames it. A failure of the port
    /// closes it, for the next exchange on the device to open it again.
    ///
    /// # Errors
    /// A fixed-length frame of the wrong length, or a port that could not
    /// be opened or written.
    pub fn transmit(&mut self, framing: &Framing, frame: &[u8]) -> Result<()> {
        let framed = framing.frame(frame)?;
        let wait = self.line.timeout;
        let port = self.opened()?.get_mut();
        put(port.as_mut(), wait, &framed).map_err(|e| {
            self.close();
            classify("writing to the port", &e)
        })
    }

    /// The next frame off the line as `framing` frames it, each read of the
    /// port waiting at most `wait`, the port held throughout; `None` when
    /// nothing arrived within it. A frame begun and cut off by the wait is
    /// an error: something arrived, and it was no frame. A failure of the
    /// port itself closes it, for the next exchange on the device to open it
    /// again.
    ///
    /// # Errors
    /// A port that could not be opened or read, or a frame cut off.
    pub fn receive(&mut self, framing: &Framing, wait: Duration) -> Result<Option<Vec<u8>>> {
        if !self.begun(wait, wait)? {
            return Ok(None);
        }
        read_frame(self.opened()?, framing).map(Some)
    }

    /// Whether a frame has begun on the port: bytes read ahead of the last
    /// frame, or bytes there within `look`; the port's wait left at `wait`
    /// for the rest of it. A failure of the port closes it.
    fn begun(&mut self, look: Duration, wait: Duration) -> Result<bool> {
        match looked(self.opened()?, look, wait) {
            Ok(begun) => Ok(begun),
            Err(e) if e.kind() == ErrorKind::TimedOut => Ok(false),
            Err(e) => {
                self.close();
                Err(classify("reading the port", &e))
            }
        }
    }

    /// How this receive waits for the port's next bytes once it lets the
    /// port go: on the port's watch where no other receive does, for the one
    /// that does where one does. Decided with the port held, so a close
    /// waits for the receive it made the watch's; whoever waits for the
    /// watch to be taken is told.
    fn waits(&self) -> Waits {
        let shared = &self.line.shared;
        let mut watching = shared.watching.locked();
        match self.open.as_ref() {
            Some(open) if !watching.led => {
                watching.led = true;
                shared.watched.notify_all();
                Waits::Watching(Arc::clone(&open.watch))
            }
            _ => Waits::Following(watching.woken),
        }
    }

    /// The port closed, for the next exchange on the device to open it
    /// again: the receive waiting on its watch told to stop and waited for,
    /// so nothing holds the device open once this returns.
    fn close(&mut self) {
        let Some(open) = self.open.take() else {
            return;
        };
        open.watch.cancel();
        let shared = &self.line.shared;
        let watching = shared.watching.locked();
        drop(
            shared
                .watched
                .wait_while(&shared.watching, watching, |w| w.led),
        );
        drop(open);
    }

    /// The port, opened at the device's first exchange and kept open for
    /// the rest, at this line's speed. A port that refuses the speed is
    /// closed.
    fn opened(&mut self) -> Result<&mut BufReader<Box<dyn Port>>> {
        let (path, baud) = (self.line.port(), self.line.baud);
        let refused = match self.open.as_mut() {
            Some(open) if open.baud != baud => match open.port.get_mut().speed(baud) {
                Ok(()) => {
                    open.baud = baud;
                    None
                }
                Err(e) => Some(e),
            },
            _ => None,
        };
        if let Some(e) = refused {
            self.close();
            return Err(classify("setting the port's speed", &e));
        }
        let open = if let Some(open) = self.open.take() {
            open
        } else {
            let port = self.line.shared.opens.open(path, baud)?;
            let watch = port
                .watch()
                .map_err(|e| classify("watching the port", &e))?;
            Open {
                port: BufReader::new(port),
                watch,
                baud,
            }
        };
        Ok(&mut self.open.insert(open).port)
    }
}

/// The next frame off `line` as `framing` frames it, within `wait`, the
/// port held only while the frame arrives: until bytes are there the
/// receive waits on the port's watch holding nothing, so another Location
/// on the device writes and exchanges meanwhile, waiting at most for a
/// frame in flight (the owner, 2026-11-15). `None` when nothing arrived
/// within `wait`; a frame begun and cut off by it is an error, each read of
/// the frame waiting at most `wait`.
///
/// # Errors
/// A port that could not be opened, read or watched, or a frame cut off.
pub(crate) fn arriving(
    line: &SerialTransport,
    framing: &Framing,
    wait: Duration,
) -> Result<Option<Vec<u8>>> {
    let clock = SystemClock;
    let began = clock.monotonic_nanos();
    loop {
        let mut held = Exchange::held(line);
        if held.begun(Duration::ZERO, wait)? {
            return read_frame(held.opened()?, framing).map(Some);
        }
        let left = wait.saturating_sub(between(began, clock.monotonic_nanos()));
        if left.is_zero() {
            return Ok(None);
        }
        let waits = held.waits();
        drop(held);
        let look = match waits {
            Waits::Watching(watch) => line.shared.watched_on(watch, left)?,
            Waits::Following(seen) => line.shared.followed(seen, left),
        };
        if !look {
            return Ok(None);
        }
    }
}

impl SharedPort {
    /// `watch` waited on for at most `left` by the one receive on it:
    /// whether to look again — bytes came, or the port closes — rather than
    /// nothing having come. Every receive following it is told, once the
    /// watch is let go.
    fn watched_on(&self, watch: Arc<dyn Watch>, left: Duration) -> Result<bool> {
        let seen = watch.readable(left);
        drop(watch);
        let mut watching = self.watching.locked();
        watching.led = false;
        if matches!(seen, Ok(true)) {
            watching.woken = watching.woken.wrapping_add(1);
        }
        self.watched.notify_all();
        drop(watching);
        seen.map_err(|e| classify("watching the port", &e))
    }

    /// Waited, at most `left`, for the receive on the watch to see
    /// something past `seen`, or to let the watch go: whether to look again
    /// rather than nothing having come within `left`.
    fn followed(&self, seen: u64, left: Duration) -> bool {
        let watching = self.watching.locked();
        let (_watching, ran_out) =
            self.watched
                .wait_timeout_while(&self.watching, watching, left, |w| w.led && w.woken == seen);
        !ran_out
    }
}

/// Whether a frame has begun on `port`: bytes read ahead, or bytes there
/// within `look`; its wait left at `wait` once one has.
fn looked(
    port: &mut BufReader<Box<dyn Port>>,
    look: Duration,
    wait: Duration,
) -> std::io::Result<bool> {
    if port.buffer().is_empty() {
        port.get_mut().wait(look)?;
        if port.fill_buf()?.is_empty() {
            return Ok(false);
        }
        if look == wait {
            return Ok(true);
        }
    }
    port.get_mut().wait(wait)?;
    Ok(true)
}

/// `framed` written to `port` and flushed, each waiting at most `wait`.
fn put(port: &mut dyn Port, wait: Duration, framed: &[u8]) -> std::io::Result<()> {
    port.wait(wait)?;
    port.write_all(framed)?;
    port.flush()
}

/// The Unix path on a real descriptor, a pseudo-terminal's.
#[cfg(all(test, unix, feature = "port"))]
mod pseudo_terminal;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::TIMEOUT;
    use crate::bench::Bench;
    use std::sync::atomic::Ordering;
    use transport::arrived::next_arrival;
    use transport::line::Line;
    use transport::{Arrived, Configured, Shared, Sharing, Transport};

    /// What each read of the tests waits, which the bench never runs out.
    const WAIT: Duration = Duration::from_millis(250);

    /// What one node's Locations share.
    fn node() -> Arc<dyn Sharing> {
        Arc::new(Shared::default())
    }

    /// `line`'s exchange of `asked`, answered.
    fn answered(line: &SerialTransport, asked: &[u8]) -> Vec<u8> {
        Line::exchange(line, asked, WAIT)
            .expect("exchanged")
            .expect("answered")
    }

    /// Off a node, a second Location on one device is refused its port, as
    /// the operating system refuses it; on a node, two Locations naming the
    /// device go through its one port, opened once, each exchange in its
    /// turn: while one holds the port the other cannot take it.
    #[test]
    fn two_locations_on_one_device_share_its_port_and_exchange_in_turn() {
        let bench = Bench::answering();
        let (alone, beside) = (bench.line(3), bench.line(3));
        assert_eq!(answered(&alone, b"ask"), b"ASK");
        let refused = Line::exchange(&beside, b"bid", WAIT).expect_err("refused");
        assert!(refused.retryable, "{refused}");
        drop(alone);

        let node = node();
        let first = bench.line(3).shared_by(&node);
        let second = bench.line(3).shared_by(&node);
        assert_eq!(answered(&first, b"ask"), b"ASK");
        assert_eq!(answered(&second, b"bid"), b"BID");
        assert_eq!(answered(&first, b"one"), b"ONE");
        assert_eq!(bench.opened(), 2, "the one alone, then one for both");
        let held = first.exchanging();
        assert!(second.shared.open.try_lock().is_err(), "the second waits");
        drop(held);
        assert!(second.shared.open.try_lock().is_ok(), "and takes its turn");
    }

    /// Each Location's exchange is answered to it alone, however the two
    /// Locations' exchanges fall between their threads: the device answers
    /// the last frame it heard, so a frame written between another
    /// Location's transmit and its receive would take that answer. The
    /// window between a transmit and a receive not held together is narrow,
    /// so each thread exchanges often enough to land in it: an exchange that
    /// let the port go between the two failed this every run of five, in a
    /// tenth of a second at most.
    #[test]
    fn an_answer_reaches_only_the_location_that_asked() {
        const ROUNDS: usize = 20_000;
        let bench = Bench::answering();
        let node = node();
        let lines = [
            bench.line(3).shared_by(&node),
            bench.line(3).shared_by(&node),
        ];
        std::thread::scope(|scope| {
            for (line, asked) in lines.iter().zip([b"ask", b"bid"]) {
                scope.spawn(move || {
                    for round in 0..ROUNDS {
                        let answer = answered(line, asked);
                        assert_eq!(answer, asked.to_ascii_uppercase(), "round {round}");
                    }
                });
            }
        });
        assert_eq!(bench.written.locked().len(), 2 * ROUNDS * 3);
        assert_eq!(bench.opened(), 1);
    }

    /// A port that fails is closed for every Location on its device, and
    /// the next exchange of any of them opens it again, once for all.
    #[test]
    fn a_failed_port_is_opened_again_once_for_all_its_locations() {
        let bench = Bench::answering();
        let node = node();
        let first = bench.line(3).shared_by(&node);
        let second = bench.line(3).shared_by(&node);
        assert_eq!(answered(&first, b"ask"), b"ASK");
        bench.broken.store(true, Ordering::Relaxed);
        assert!(
            Line::exchange(&first, b"ask", WAIT).is_err(),
            "the port broke"
        );
        for line in [&second, &first, &second] {
            assert_eq!(answered(line, b"bid"), b"BID");
        }
        assert_eq!(bench.opened(), 2, "opened again once, for both");
    }

    /// Locations on two devices hold two ports: an exchange on one device
    /// goes on while another device's port is held.
    #[test]
    fn locations_on_two_devices_do_not_wait_on_each_other() {
        let (one, two) = (Bench::answering(), Bench::answering());
        let node = node();
        let meter = one.at("COM1", 2400, 3).shared_by(&node);
        let sensor = two.at("COM2", 2400, 3).shared_by(&node);
        let mut held = meter.exchanging();
        held.transmit(&Framing::Fixed(3), b"ask").expect("sent");
        assert!(
            sensor.shared.open.try_lock().is_ok(),
            "the other port is free"
        );
        assert_eq!(answered(&sensor, b"bid"), b"BID");
        let answer = held.receive(&Framing::Fixed(3), WAIT).expect("read");
        assert_eq!(answer, Some(b"ASK".to_vec()));
        drop(held);
        assert_eq!((one.opened(), two.opened()), (1, 1));
    }

    /// Each Location's exchange runs at its own speed: the port is opened
    /// at the first one's and set to another's only where it differs.
    #[test]
    fn each_location_on_a_device_exchanges_at_its_own_speed() {
        let bench = Bench::answering();
        let node = node();
        let slow = bench.at("COM9", 2400, 3).shared_by(&node);
        let fast = bench.at("COM9", 9600, 3).shared_by(&node);
        for line in [&slow, &slow, &fast, &fast, &slow] {
            assert_eq!(answered(line, b"ask"), b"ASK");
        }
        assert_eq!(*bench.speeds.locked(), [9600, 2400]);
        assert_eq!(bench.opened(), 1);
    }

    /// How long a test waits for a receive to wait on a watch: never run
    /// out, and only a bound on a test that fails.
    const WITHIN: Duration = Duration::from_secs(10);

    /// A send on a device's port goes while a receive on it waits idle for
    /// its first byte, which waits on the port's watch holding nothing
    /// (the owner, 2026-11-15). In order: the receive waits on the watch;
    /// the send is written and returns while the receive still waits; and
    /// what the device sends after it is what the receive reads. A receive
    /// that held the port while it waited kept the send until its wait ran
    /// out, and read nothing.
    #[test]
    fn a_send_goes_at_once_while_a_receive_waits_idle() {
        let bench = Bench::waits();
        let node = node();
        let receiving = bench.line(3).shared_by(&node);
        let sending = bench.line(3).shared_by(&node);
        std::thread::scope(|scope| {
            let received = scope.spawn(|| Transport::receive(&receiving));
            assert!(bench.watching(1, WITHIN), "the receive waits on the watch");
            sending.send("COM9", b"out").expect("sent");
            assert_eq!(bench.written(), b"out");
            assert!(!received.is_finished(), "the receive still waits");
            bench.sends(b"in!");
            let arrived = received.join().expect("joined").expect("received");
            let taken = next_arrival(arrived, "a frame").and_then(Arrived::taken);
            assert_eq!(taken.expect("read whole").bytes, b"in!");
        });
        assert_eq!(bench.opened(), 1, "one port for both");
    }

    /// Two receives waiting idle on one port: one waits on its watch, the
    /// other for what that one sees, and each frame the device sends is
    /// read by one of them while the other waits on for the next.
    #[test]
    fn two_receives_waiting_on_one_port_each_read_a_frame() {
        let bench = Bench::waits();
        let node = node();
        let lines = [
            bench.line(3).shared_by(&node),
            bench.line(3).shared_by(&node),
        ];
        std::thread::scope(|scope| {
            let waiting: Vec<_> = lines
                .iter()
                .map(|line| scope.spawn(move || Line::receive(line, TIMEOUT)))
                .collect();
            assert!(bench.watching(1, WITHIN), "one waits on the watch");
            bench.sends(b"one");
            assert!(bench.watching(2, WITHIN), "then the other");
            bench.sends(b"two");
            let mut read: Vec<_> = waiting
                .into_iter()
                .map(|one| one.join().expect("joined").expect("read").expect("a frame"))
                .collect();
            read.sort();
            assert_eq!(read, [b"one".to_vec(), b"two".to_vec()]);
        });
    }

    /// A port closed while a receive waits idle on its watch — another
    /// Location's exchange broke it — ends the wait, and the close waits for
    /// it, so nothing holds the device and the port opens again at once for
    /// the receive, which reads what the device sends next.
    #[test]
    fn a_port_closed_under_a_waiting_receive_opens_again_for_it() {
        let bench = Bench::waits();
        let node = node();
        let receiving = bench.line(3).shared_by(&node);
        let asking = bench.line(3).shared_by(&node);
        std::thread::scope(|scope| {
            let received = scope.spawn(|| Line::receive(&receiving, TIMEOUT));
            assert!(bench.watching(1, WITHIN), "the receive waits on the watch");
            bench.broken.store(true, Ordering::Relaxed);
            let broke = Line::exchange(&asking, b"ask", WAIT);
            assert!(broke.is_err(), "the port broke: {broke:?}");
            assert!(bench.watching(2, WITHIN), "on the port opened again");
            bench.sends(b"yes");
            let read = received.join().expect("joined").expect("read");
            assert_eq!(read, Some(b"yes".to_vec()));
        });
        assert_eq!(bench.opened(), 2, "opened again once");
    }
}
