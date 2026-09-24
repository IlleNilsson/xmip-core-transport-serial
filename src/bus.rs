//! A multi-drop serial bus, in process: one pair of wires, a master, and as
//! many addressed devices as a test or a configuration puts on it.
//!
//! RS-485 and the field buses on it — M-Bus, HART, Modbus RTU — are
//! multi-drop: every device hears every frame, and the protocol's address
//! says which one answers. A line that echoes cannot show any of what a bus is
//! for, so this one keeps the parts that matter (open problem 24):
//!
//! - **Addressing.** Every attached [`Device`] hears every frame and decides
//!   from the protocol's own address whether it is meant; one that is not
//!   stays silent.
//! - **Silence and collision.** No answer is silence, and the master's read
//!   finds nothing. Two answers to one frame collide: the master reads the two
//!   overlaid, which no protocol's checksum passes.
//! - **Turnaround.** An answer is on the line only after the bus's turnaround;
//!   a master that polls before then reads nothing yet.
//! - **Unsolicited frames.** A device may speak unasked — HART burst mode —
//!   through [`Bus::speak`].
//! - **Faults.** A character lost on the way to the devices, and a break
//!   condition the master's next read meets.
//!
//! A test and a node without a serial port use it alike: it is a [`Line`],
//! the same boundary a real port is.

use std::collections::VecDeque;
use std::sync::{Arc, Mutex, PoisonError};
use std::time::{Duration, Instant};

use transport::error::{Result, protocol_error};
use transport::line::Line;

/// Something on the bus with an address: a meter, a field device.
pub trait Device: Send + Sync {
    /// What this device puts on the line after hearing `frame`: its answer,
    /// or `None` when the frame is not for it or asks for no answer.
    ///
    /// # Errors
    /// The frame was for this device and it could not act on it.
    fn hear(&self, frame: &[u8]) -> Result<Option<Vec<u8>>>;
}

/// One frame waiting for the master, and when it is on the line.
struct Pending {
    bytes: Vec<u8>,
    ready: Instant,
}

/// What the master's next read meets that is not a frame.
#[derive(Default)]
struct Faults {
    /// Lose one character from every `n`th frame the master sends.
    lose_every: Option<u32>,
    /// The master's next read meets a break condition.
    broken: bool,
}

/// An in-process multi-drop bus.
pub struct Bus {
    name: String,
    turnaround: Duration,
    devices: Mutex<Vec<Arc<dyn Device>>>,
    to_master: Mutex<VecDeque<Pending>>,
    faults: Mutex<Faults>,
    sent: Mutex<u32>,
}

impl Bus {
    /// An empty bus called `name`, which the origin URIs of what crosses it
    /// carry. No turnaround: an answer is there as soon as it is given.
    #[must_use]
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            turnaround: Duration::ZERO,
            devices: Mutex::new(Vec::new()),
            to_master: Mutex::new(VecDeque::new()),
            faults: Mutex::new(Faults::default()),
            sent: Mutex::new(0),
        }
    }

    /// The time between the end of a frame and the start of its answer.
    #[must_use]
    pub const fn with_turnaround(mut self, turnaround: Duration) -> Self {
        self.turnaround = turnaround;
        self
    }

    /// Put `device` on the bus.
    pub fn attach(&self, device: Arc<dyn Device>) {
        lock(&self.devices).push(device);
    }

    /// A device speaks unasked: `frame` is on the line for the master after
    /// the turnaround, as a HART device in burst mode puts its variable.
    pub fn speak(&self, frame: Vec<u8>) {
        self.queue(frame);
    }

    /// Lose one character, the middle one, from every `every`th frame the
    /// master sends, before any device hears it.
    pub fn lose_a_character_every(&self, every: u32) {
        lock(&self.faults).lose_every = Some(every.max(1));
    }

    /// Hold the line in a break condition: the master's next read meets it.
    pub fn break_the_line(&self) {
        lock(&self.faults).broken = true;
    }

    fn queue(&self, bytes: Vec<u8>) {
        lock(&self.to_master).push_back(Pending {
            bytes,
            ready: Instant::now() + self.turnaround,
        });
    }

    /// `frame` as the devices hear it, and whether a fault changed it.
    fn heard(&self, frame: &[u8]) -> (Vec<u8>, bool) {
        let mut sent = lock(&self.sent);
        *sent += 1;
        let lose = lock(&self.faults)
            .lose_every
            .is_some_and(|every| (*sent).is_multiple_of(every));
        if lose && !frame.is_empty() {
            let mut heard = frame.to_vec();
            heard.remove(frame.len() / 2);
            return (heard, true);
        }
        (frame.to_vec(), false)
    }
}

impl Line for Bus {
    fn name(&self) -> String {
        self.name.clone()
    }

    /// The master's frame on the bus: every device hears it, and what they
    /// answer is what the master reads next — one answer whole, two or more
    /// overlaid.
    fn transmit(&self, frame: &[u8]) -> Result<()> {
        let (heard, damaged) = self.heard(frame);
        let devices = lock(&self.devices).clone();
        let mut answers = Vec::new();
        for device in devices {
            match device.hear(&heard) {
                Ok(Some(answer)) => answers.push(answer),
                Ok(None) => {}
                // A device that cannot read a damaged frame ignores it, as a
                // real one does; an intact frame it cannot act on is an error.
                Err(_) if damaged => {}
                Err(error) => return Err(error),
            }
        }
        match answers.len() {
            0 => {}
            1 => self.queue(answers.remove(0)),
            _ => self.queue(collided(&answers)),
        }
        Ok(())
    }

    /// The next frame on the line within `timeout`, `None` when nothing
    /// comes: silence, or an answer still inside its turnaround.
    fn receive(&self, timeout: Duration) -> Result<Option<Vec<u8>>> {
        if std::mem::take(&mut lock(&self.faults).broken) {
            return Err(protocol_error("a break condition on the line"));
        }
        let deadline = Instant::now() + timeout;
        loop {
            let ready = {
                let mut waiting = lock(&self.to_master);
                match waiting.front() {
                    None => return Ok(None),
                    Some(next) if next.ready <= Instant::now() => {
                        return Ok(waiting.pop_front().map(|pending| pending.bytes));
                    }
                    Some(next) => next.ready,
                }
            };
            if ready > deadline {
                return Ok(None);
            }
            std::thread::sleep(ready.saturating_duration_since(Instant::now()));
        }
    }
}

/// Two or more answers on the line at once: each byte the OR of theirs, as
/// drivers pulling one pair of wires leave it, the longest answer's length.
fn collided(answers: &[Vec<u8>]) -> Vec<u8> {
    let length = answers.iter().map(Vec::len).max().unwrap_or(0);
    (0..length)
        .map(|at| {
            answers
                .iter()
                .filter_map(|answer| answer.get(at))
                .fold(0, |line, byte| line | byte)
        })
        .collect()
}

fn lock<T>(mutex: &Mutex<T>) -> std::sync::MutexGuard<'_, T> {
    mutex.lock().unwrap_or_else(PoisonError::into_inner)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A device that answers `[address, byte + 1]` to `[address, byte]`, and
    /// refuses `[address, 0xff]`.
    struct Echo(u8);

    impl Device for Echo {
        fn hear(&self, frame: &[u8]) -> Result<Option<Vec<u8>>> {
            match frame {
                [address, 0xff] if *address == self.0 => Err(protocol_error("refused")),
                [address, byte] if *address == self.0 => Ok(Some(vec![*address, byte + 1])),
                _ => Ok(None),
            }
        }
    }

    fn bus(addresses: &[u8]) -> Bus {
        let bus = Bus::new("rs485");
        for address in addresses {
            bus.attach(Arc::new(Echo(*address)));
        }
        bus
    }

    #[test]
    fn the_addressed_device_answers_and_the_others_stay_silent() {
        let bus = bus(&[1, 2, 3]);
        bus.transmit(&[2, 10]).expect("sent");
        assert_eq!(
            bus.receive(Duration::ZERO).expect("read"),
            Some(vec![2, 11])
        );
        bus.transmit(&[9, 10]).expect("sent");
        assert_eq!(
            bus.receive(Duration::ZERO).expect("read"),
            None,
            "nobody at 9"
        );
        assert_eq!(bus.name(), "rs485");
    }

    /// A device at `address` that always answers `answer`.
    struct Fixed(u8, Vec<u8>);

    impl Device for Fixed {
        fn hear(&self, frame: &[u8]) -> Result<Option<Vec<u8>>> {
            Ok((frame.first() == Some(&self.0)).then(|| self.1.clone()))
        }
    }

    #[test]
    fn two_devices_answering_one_frame_collide_into_what_neither_sent() {
        // Answers that differ overlay into bytes neither device sent. Two
        // identical answers overlay into themselves, on a real pair of wires
        // too, which is why a protocol's address, not the bus, keeps two
        // devices from answering at once.
        let bus = Bus::new("rs485");
        bus.attach(Arc::new(Fixed(5, vec![0x05, 0x11])));
        bus.attach(Arc::new(Fixed(5, vec![0x40, 0x02, 0x7f])));
        bus.transmit(&[5, 0x10]).expect("sent");
        assert_eq!(
            bus.receive(Duration::ZERO).expect("read"),
            Some(vec![0x45, 0x13, 0x7f])
        );
    }

    #[test]
    fn a_device_speaks_unasked_and_is_read_in_order() {
        let bus = bus(&[1]);
        bus.speak(vec![0xaa]);
        bus.transmit(&[1, 1]).expect("sent");
        assert_eq!(
            bus.receive(Duration::ZERO).expect("burst"),
            Some(vec![0xaa])
        );
        assert_eq!(
            bus.receive(Duration::ZERO).expect("answer"),
            Some(vec![1, 2])
        );
    }

    #[test]
    fn a_master_that_polls_before_the_turnaround_reads_nothing_yet() {
        let bus = bus(&[1]).with_turnaround(Duration::from_millis(40));
        bus.transmit(&[1, 1]).expect("sent");
        assert_eq!(bus.receive(Duration::ZERO).expect("early"), None);
        assert_eq!(
            bus.receive(Duration::from_millis(500)).expect("waited"),
            Some(vec![1, 2])
        );
    }

    #[test]
    fn a_lost_character_leaves_the_device_silent_and_a_break_is_met() {
        let bus = bus(&[1]);
        bus.lose_a_character_every(2);
        bus.transmit(&[1, 1]).expect("first arrives whole");
        assert_eq!(bus.receive(Duration::ZERO).expect("read"), Some(vec![1, 2]));
        bus.transmit(&[1, 1]).expect("second loses a character");
        assert_eq!(bus.receive(Duration::ZERO).expect("read"), None);
        bus.break_the_line();
        assert!(bus.receive(Duration::ZERO).is_err(), "a break condition");
        assert_eq!(bus.receive(Duration::ZERO).expect("once"), None);
        assert!(bus.transmit(&[1, 0xff]).is_err(), "an intact frame refused");
    }
}
