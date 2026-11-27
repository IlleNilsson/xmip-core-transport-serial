//! This technology's declaration: the transport guide's "The declaration",
//! `module/core/capability/transport/doc/adding-a-transport.md`. It is the
//! same on every operating system, one with no serial port included: a
//! Location's settings are suggested and checked wherever it is designed.

use std::time::Duration;

use xcore::settings::{Applies, Kind, Presence, Setting, Settings};

/// How long a read or a write waits on the port until a Location says
/// otherwise; a protocol reading the port as a `Line` hands each read its
/// own wait.
pub const TIMEOUT: Duration = Duration::from_secs(5);

/// The most a frame may be before the line is judged broken: a delimited
/// frame's and a fixed length's.
pub const MAX_FRAME: usize = 1024 * 1024;

/// Every setting a Location takes beyond its address, the port: `COM3`,
/// `/dev/ttyUSB0`.
#[expect(
    clippy::cast_possible_wrap,
    reason = "MAX_FRAME is one mebibyte, far inside an i64"
)]
pub const SETTINGS: &Settings = &Settings {
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

/// This technology's schemes, as [`transport::Configured::SCHEMES`] defines
/// them.
pub const SCHEMES: &[&str] = &["serial"];
