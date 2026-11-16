//! A serial port's Unix path on a real descriptor (ADR-0064, amendment
//! 2026-11-15): a pseudo-terminal pair, made by `serialport`
//! (`TTYPort::pair`), its terminal end opened by path as a device is
//! (`SystemPorts`) and watched with `poll` on a copy of its descriptor, and
//! the test at the other end as the device. In order of events, no sleep:
//! each receive waits on the port's watch before the device sends.

use std::io::{Read, Write};
use std::sync::Arc;
use std::time::Duration;

use serialport::{SerialPort, TTYPort};
use transport::arrived::next_arrival;
use transport::line::Line;
use transport::{Arrived, Configured, Shared, Sharing, Transport};
use xcore::lock::Locked;

use crate::{Framing, SerialTransport, TIMEOUT};

/// How long the device's end waits for what the line writes, and a test
/// for a receive to wait on its watch: never run out, a bound on a test
/// that fails.
const WITHIN: Duration = Duration::from_secs(10);

/// A pseudo-terminal: the device's end, and the path of the line's end,
/// which a line opens itself, as it opens a device.
fn terminal() -> (TTYPort, String) {
    let (mut device, line) = TTYPort::pair().expect("a pseudo-terminal");
    let path = line.name().expect("the line's end has a path");
    drop(line);
    device.set_timeout(WITHIN).expect("the device's wait");
    (device, path)
}

/// Two Locations' lines on the device at `path`, on one node, their frames
/// three bytes long.
fn on_one_node(path: &str) -> [SerialTransport; 2] {
    let node: Arc<dyn Sharing> = Arc::new(Shared::default());
    let line = || {
        SerialTransport::new(path, 9600)
            .framed(Framing::Fixed(3))
            .shared_by(&node)
    };
    [line(), line()]
}

/// Waited, at most [`WITHIN`], until a receive on `line`'s port waits on its
/// watch: whether one does.
fn watched(line: &SerialTransport) -> bool {
    let shared = &line.shared;
    let watching = shared.watching.locked();
    let (watching, _) =
        shared
            .watched
            .wait_timeout_while(&shared.watching, watching, WITHIN, |w| !w.led);
    watching.led
}

/// What the device's end reads: `length` bytes.
fn heard(device: &mut TTYPort, length: usize) -> Vec<u8> {
    let mut heard = vec![0; length];
    device.read_exact(&mut heard).expect("the device hears it");
    heard
}

/// A send on the port goes while a Receive Location's receive waits idle
/// on its watch: the device hears it while the receive still waits, and
/// the receive reads what the device sends after it.
#[test]
fn a_send_goes_while_a_receive_waits_idle_on_a_terminal() {
    let (mut device, path) = terminal();
    let [receiving, sending] = on_one_node(&path);
    std::thread::scope(|scope| {
        let received = scope.spawn(|| Transport::receive(&receiving));
        assert!(watched(&receiving), "the receive waits on the watch");
        sending.send(&path, b"out").expect("sent");
        assert_eq!(heard(&mut device, 3), b"out");
        assert!(!received.is_finished(), "the receive still waits");
        device.write_all(b"in!").expect("the device sends");
        let arrived = received.join().expect("joined").expect("received");
        let taken = next_arrival(arrived, "a frame").and_then(Arrived::taken);
        assert_eq!(taken.expect("read whole").bytes, b"in!");
    });
}

/// A receive waiting idle on its watch wakes when the device sends, and
/// reads the frame; with nothing sent, its wait runs out with nothing.
#[test]
fn an_idle_receive_wakes_when_bytes_come_to_a_terminal() {
    let (mut device, path) = terminal();
    let [line, _] = on_one_node(&path);
    let quiet = Line::receive(&line, Duration::from_millis(20)).expect("quiet");
    assert_eq!(quiet, None, "nothing came");
    std::thread::scope(|scope| {
        let received = scope.spawn(|| Line::receive(&line, TIMEOUT));
        assert!(watched(&line), "the receive waits on the watch");
        device.write_all(b"abc").expect("the device sends");
        let read = received.join().expect("joined").expect("read");
        assert_eq!(read, Some(b"abc".to_vec()));
    });
}

/// A port closed while a receive waits idle on its watch ends that wait,
/// and the close lets the terminal go, so the receive opens it again at
/// once and reads what the device sends next. A close that did not end the
/// wait left the receive waiting out its wait, reading nothing.
#[test]
fn a_close_ends_a_receive_waiting_on_a_terminal_and_lets_it_go() {
    let (mut device, path) = terminal();
    let [receiving, closing] = on_one_node(&path);
    std::thread::scope(|scope| {
        let received = scope.spawn(|| Line::receive(&receiving, TIMEOUT));
        assert!(watched(&receiving), "the receive waits on the watch");
        closing.exchanging().close();
        assert!(watched(&receiving), "on the terminal opened again");
        device.write_all(b"yes").expect("the device sends");
        let read = received.join().expect("joined").expect("read");
        assert_eq!(read, Some(b"yes".to_vec()));
    });
}
