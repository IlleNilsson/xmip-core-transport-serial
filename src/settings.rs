//! The settings a serial Location takes, read through its declaration
//! (`crate::declaration`; ADR-0064, amendments 2026-09-26 and 2026-11-26),
//! and the device's port it takes from what the Locations of its node
//! share (amendment 2026-11-15).

use std::sync::Arc;

use transport::error::{Result, protocol_error};
use transport::{Configured, Sharing};
use xcore::settings::{Read, Settings};

use crate::shared_port::SharedPort;
use crate::{Framing, SerialTransport, declaration};

impl Configured for SerialTransport {
    const SETTINGS: &'static Settings = declaration::SETTINGS;
    const SCHEMES: &'static [&'static str] = declaration::SCHEMES;

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
    use crate::declaration::TIMEOUT;
    use std::assert_matches;
    use transport::Transport;
    use xcore::settings::{Applies, Given};

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
