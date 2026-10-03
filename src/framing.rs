//! Where one frame ends on a line that has no message boundary of its own:
//! a delimiter, a fixed length, or the length the frame says in its first
//! bytes — and reading one frame by that rule from any byte source.

use std::io::BufRead;

use transport::error::{Result, classify, protocol_error};

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

/// The most a delimited frame may be before the line is judged broken.
pub const MAX_FRAME: usize = 1024 * 1024;

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
