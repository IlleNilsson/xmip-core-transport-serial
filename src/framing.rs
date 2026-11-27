//! Where one frame ends on a line that has no message boundary of its own:
//! a delimiter, a fixed length, the length the frame says in its first
//! bytes, or SLIP's ends and escapes — and reading one frame by that rule
//! from any byte source.

use std::io::BufRead;

use transport::error::{Result, classify, protocol_error};

use crate::declaration::MAX_FRAME;

/// SLIP's frame end (RFC 1055).
const END: u8 = 0xc0;
/// SLIP's escape: the next byte stands for an end or an escape.
const ESC: u8 = 0xdb;
/// After [`ESC`], an end in the frame.
const ESC_END: u8 = 0xdc;
/// After [`ESC`], an escape in the frame.
const ESC_ESC: u8 = 0xdd;

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
    /// SLIP, RFC 1055: a frame between two end bytes, an end or an escape
    /// inside it escaped, as a wireless M-Bus receiver stick's host
    /// controller interface frames its messages. An end with nothing before
    /// it opens the next frame and is no frame of its own.
    Slip,
}

impl Framing {
    /// `bytes` as one frame on a line framed so: the delimiter appended,
    /// the fixed length enforced, or the frame between SLIP's ends with
    /// what it holds of them escaped; a measured frame says its own length.
    ///
    /// # Errors
    /// A fixed-length frame of the wrong length.
    pub fn frame(&self, bytes: &[u8]) -> Result<Vec<u8>> {
        match self {
            Self::Delimited(delimiter) => {
                let mut out = bytes.to_vec();
                out.extend_from_slice(delimiter);
                Ok(out)
            }
            Self::Fixed(length) if bytes.len() == *length => Ok(bytes.to_vec()),
            Self::Measured(_) => Ok(bytes.to_vec()),
            Self::Slip => Ok(slipped(bytes)),
            Self::Fixed(length) => Err(protocol_error(format!(
                "a frame of {} bytes on a line framed at {length}",
                bytes.len()
            ))),
        }
    }
}

/// `bytes` between two SLIP ends, each end and escape in them escaped.
fn slipped(bytes: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(bytes.len() + 2);
    out.push(END);
    for byte in bytes {
        match *byte {
            END => out.extend_from_slice(&[ESC, ESC_END]),
            ESC => out.extend_from_slice(&[ESC, ESC_ESC]),
            other => out.push(other),
        }
    }
    out.push(END);
    out
}

/// How long a frame is, from the bytes read of it so far; see
/// [`Framing::Measured`].
pub type Measure = fn(&[u8]) -> Result<Option<usize>>;

/// Read one frame from `reader` under `framing`.
///
/// # Errors
/// The line closed inside a frame, a delimited or SLIP frame outgrew
/// [`MAX_FRAME`], or a SLIP escape stood for nothing it escapes.
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
        Framing::Slip => read_slipped(reader),
    }
}

/// One SLIP frame off `reader`, its escapes undone: what lies before its
/// end, the ends before it skipped.
fn read_slipped(reader: &mut impl BufRead) -> Result<Vec<u8>> {
    let mut escaped = Vec::new();
    loop {
        let read = reader
            .read_until(END, &mut escaped)
            .map_err(|e| classify("reading a SLIP frame", &e))?;
        if read == 0 {
            return Err(protocol_error("the line closed inside a frame"));
        }
        if escaped.last() == Some(&END) {
            escaped.pop();
            if !escaped.is_empty() {
                return unslipped(&escaped);
            }
        }
        if escaped.len() > MAX_FRAME {
            return Err(protocol_error("a frame over the size Xmip will read"));
        }
    }
}

/// `escaped`, the bytes between two SLIP ends, with each escape undone.
fn unslipped(escaped: &[u8]) -> Result<Vec<u8>> {
    let mut frame = Vec::with_capacity(escaped.len());
    let mut bytes = escaped.iter().copied();
    while let Some(byte) = bytes.next() {
        frame.push(if byte == ESC {
            match bytes.next() {
                Some(ESC_END) => END,
                Some(ESC_ESC) => ESC,
                _ => return Err(protocol_error("a SLIP escape of nothing it escapes")),
            }
        } else {
            byte
        });
    }
    Ok(frame)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_slip_frame_escapes_its_ends_and_escapes_and_reads_back() {
        let message = [0x01, END, 0x02, ESC, 0x03];
        let framed = Framing::Slip.frame(&message).expect("framed");
        assert_eq!(
            framed,
            [END, 0x01, ESC, ESC_END, 0x02, ESC, ESC_ESC, 0x03, END]
        );
        let mut reader = std::io::BufReader::new(&framed[..]);
        assert_eq!(
            read_frame(&mut reader, &Framing::Slip).expect("read"),
            message
        );
    }

    /// IMST's Ping request and response as its specification writes them,
    /// one after the other: the end that opens each is no frame.
    #[test]
    fn ends_with_nothing_between_them_open_the_next_frame() {
        let line = [
            END, 0x01, 0x01, 0x16, 0x07, END, END, 0x01, 0x02, 0x00, 0xa0, 0xaf, END,
        ];
        let mut reader = std::io::BufReader::new(&line[..]);
        let first = read_frame(&mut reader, &Framing::Slip).expect("first");
        assert_eq!(first, [0x01, 0x01, 0x16, 0x07]);
        let second = read_frame(&mut reader, &Framing::Slip).expect("second");
        assert_eq!(second, [0x01, 0x02, 0x00, 0xa0, 0xaf]);
        assert!(read_frame(&mut reader, &Framing::Slip).is_err(), "closed");
    }

    #[test]
    fn a_slip_frame_cut_short_or_wrongly_escaped_is_refused() {
        let cut = [END, 0x01, 0x02];
        let mut reader = std::io::BufReader::new(&cut[..]);
        assert!(
            read_frame(&mut reader, &Framing::Slip).is_err(),
            "cut short"
        );
        let wrong = [END, 0x01, ESC, 0x02, END];
        let mut reader = std::io::BufReader::new(&wrong[..]);
        let error = read_frame(&mut reader, &Framing::Slip).expect_err("no such escape");
        assert!(error.message.contains("escape"), "{error}");
        let trailing = [END, 0x01, ESC, END];
        let mut reader = std::io::BufReader::new(&trailing[..]);
        assert!(
            read_frame(&mut reader, &Framing::Slip).is_err(),
            "an escape at the end"
        );
    }
}
