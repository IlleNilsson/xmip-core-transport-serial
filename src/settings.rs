//! The settings a serial Location takes, declared once and read through
//! (ADR-0064, amendment 2026-09-26), and the device's port it takes from
//! what the Locations of its node share (amendment 2026-11-15).

use std::sync::Arc;

use transport::error::{Result, protocol_error};
use transport::{Configured, Sharing};
use xcore::settings::{Applies, Kind, Presence, Read, Setting, Settings};

use crate::shared_port::SharedPort;
use crate::{Framing, MAX_FRAME, SerialTransport, TIMEOUT};

impl Configured for SerialTransport {
    /// The address is the port: `COM3`, `/dev/ttyUSB0`.
    #[expect(
        clippy::cast_possible_wrap,
        reason = "MAX_FRAME is one mebibyte, far inside an i64"
    )]
    const SETTINGS: &'static Settings = &Settings {
        technology: env!("CARGO_PKG_NAME"),
        settings: &[
            Setting {
                name: "baud",
                kind: Kind::Integer {
                    minimum: 1,
                    maximum: 4_294_967_295,
                },
                presence: Presence::Required,
                meaning: "The line's speed in bits per second, such as 9600.",
                applies: Applies::Both,
            },
            Setting {
                name: "delimiter",
                kind: Kind::Text,
                presence: Presence::Optional,
                meaning: "The bytes a frame ends with, not part of it; a carriage return and a \
                          line feed when neither this nor a length is given.",
                applies: Applies::Both,
            },
            Setting {
                name: "length",
                kind: Kind::Integer {
                    minimum: 1,
                    maximum: MAX_FRAME as i64,
                },
                presence: Presence::Optional,
                meaning: "The fixed length of every frame, in place of a delimiter.",
                applies: Applies::Both,
            },
            transport::configured::timeout(
                Applies::Both,
                Some(TIMEOUT),
                "How long a read or a write waits on the port.",
            ),
        ],
    };

    /// The schemes a Location's address is entered with to choose this technology.
    const SCHEMES: &'static [&'static str] = &["serial"];

    fn timeout_kept(&mut self) -> Option<&mut dyn transport::configured::TimeoutKept> {
        Some(&mut self.timeout)
    }

    /// The port every Location naming this line's device on the node goes
    /// through, by the path it names: the one the first of them opened,
    /// through what that one opens with.
    fn shared_by(mut self, sharing: &Arc<dyn Sharing>) -> Self {
        let opens = Arc::clone(self.shared.opens());
        self.shared = sharing.one(&self.port, || SharedPort::new(Arc::clone(&opens)));
        self
    }

    fn configured(address: &str, settings: &Read) -> Result<Self> {
        let baud = u32::try_from(settings.integer("baud"))
            .map_err(|_| protocol_error("the baud rate is out of range"))?;
        let transport = Self::new(address, baud);
        match (
            settings.optional_text("delimiter"),
            settings.optional_integer("length"),
        ) {
            (Some(_), Some(_)) => Err(protocol_error(
                "a frame ends at a delimiter or at a length, not both",
            )),
            (Some(delimiter), None) => {
                Ok(transport.framed(Framing::Delimited(delimiter.as_bytes().to_vec())))
            }
            (None, Some(length)) => {
                let length = usize::try_from(length)
                    .map_err(|_| protocol_error("the frame length is out of range"))?;
                Ok(transport.framed(Framing::Fixed(length)))
            }
            (None, None) => Ok(transport),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::assert_matches;
    use transport::Transport;
    use xcore::settings::Given;

    #[test]
    fn serial_declares_its_settings_and_reads_through_them() {
        assert_eq!(SerialTransport::SETTINGS.problems(), Vec::<String>::new());
        let given = [
            ("baud".to_string(), Given::Integer(19_200)),
            ("length".to_string(), Given::Integer(12)),
        ];
        let built = <SerialTransport as Configured>::open("COM3", Applies::Receive, &given)
            .expect("configured");
        assert_eq!(built.origin(), "serial://COM3?baud=19200");
        assert_matches!(built.framing, Framing::Fixed(12));
        assert_eq!(built.timeout, TIMEOUT);
        assert_eq!(built.name(), "serial");
        let Err(refused) = <SerialTransport as Configured>::open("COM3", Applies::Send, &[]) else {
            panic!("the baud rate is required");
        };
        assert!(refused.message.contains("\"baud\""), "{refused}");
    }
}
