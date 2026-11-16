//! The serial port's own calls to the system, behind [`crate::Port`] and
//! [`crate::Watch`] (ADR-0064, amendment 2026-11-15): on Windows the COM
//! port, and on Unix the descriptor a watch lends to `poll`.
//!
//! **Windows.** `serialport` opens a COM port for synchronous I/O
//! (serialport 4.10.1, `COMPort::open`), and Windows runs one call at a time
//! on such a handle: a thread waiting on it for the first byte — a read, or
//! `WaitCommEvent` — holds every other thread's write until it returns
//! (Microsoft, *Serial Communications*, Allen Denver, "Nonoverlapped I/O":
//! "if one thread were waiting for a `ReadFile` function to return, any
//! other thread that issued a `WriteFile` function would be blocked"; and
//! *Monitoring Communications Events*: `WaitCommEvent` "should be executed
//! as an overlapped operation so the other threads of the process can
//! perform I/O operations during the wait"). So the port is opened here for
//! overlapped I/O (`FILE_FLAG_OVERLAPPED`), set as `serialport` sets it as
//! it opens — eight data bits, no parity, one stop bit, no flow control,
//! DTR and RTS off — its timeouts (`COMMTIMEOUTS`) as `serialport` sets
//! them, and watched for `EV_RXCHAR`. Each read, each write and each wait
//! for a first byte is an overlapped operation begun here and waited for by
//! the estate's one overlapped wait, `net::system::overlapped::waited`; a
//! watch's wait is ended by its port's close as well.
//!
//! **Unix.** `serialport`'s port, which polls before each read and write,
//! is kept; its watch polls a copy of its descriptor beside a pipe its
//! port's close writes to (`transport::socket::ready`), lending the copy,
//! which `serialport` hands out only as a raw descriptor.
#![allow(unsafe_code)]

#[cfg(windows)]
pub(crate) use com::ComPort;
#[cfg(unix)]
pub(crate) use descriptor::Polled;

#[cfg(windows)]
mod com {
    use std::io;
    use std::os::windows::io::{AsRawHandle, FromRawHandle, OwnedHandle};
    use std::ptr;
    use std::sync::Arc;
    use std::time::Duration;

    use net::system::overlapped::{Event, On, milliseconds, waited};
    use windows_sys::Win32::Devices::Communication::{
        COMMTIMEOUTS, DCB, EV_RXCHAR, GetCommState, NOPARITY, ONESTOPBIT, SetCommMask,
        SetCommState, SetCommTimeouts, WaitCommEvent,
    };
    use windows_sys::Win32::Foundation::{
        GENERIC_READ, GENERIC_WRITE, HANDLE, INVALID_HANDLE_VALUE,
    };
    use windows_sys::Win32::Storage::FileSystem::{
        CreateFileW, FILE_FLAG_OVERLAPPED, FlushFileBuffers, OPEN_EXISTING, ReadFile, WriteFile,
    };
    use windows_sys::core::BOOL;

    use crate::{Port, Watch};

    /// The DCB flags `serialport` sets as it opens a port: `fBinary` on
    /// (bit 0), and off `fParity` (1), `fOutxCtsFlow` (2), `fOutxDsrFlow`
    /// (3), `fDtrControl` (4-5), `fDsrSensitivity` (6), `fOutX` (8), `fInX`
    /// (9), `fErrorChar` (10), `fNull` (11), `fRtsControl` (12-13) and
    /// `fAbortOnError` (14); `fTXContinueOnXoff` (7) as the driver has it.
    const FLAGS_SET: u32 = 0x7f7f;

    /// The outcome of a call returning a `BOOL`: its failure in the last
    /// error.
    fn done(called: BOOL) -> io::Result<()> {
        if called == 0 {
            Err(io::Error::last_os_error())
        } else {
            Ok(())
        }
    }

    /// A COM port open for overlapped I/O.
    pub(crate) struct ComPort {
        /// The port's handle, which its watch holds as well, so the handle
        /// outlives every wait on it.
        handle: Arc<OwnedHandle>,
        /// The event each read and each write ends on, one at a time: the
        /// port is read and written under its line's lock.
        io: Event,
    }

    impl ComPort {
        /// `path` — `COM3`, or a device path as Windows writes it — open at
        /// `baud`, set as `serialport` sets a port, watched for `EV_RXCHAR`.
        ///
        /// # Errors
        /// Where Windows refused the port or its settings.
        pub(crate) fn open(path: &str, baud: u32) -> io::Result<Self> {
            let mut name: Vec<u16> = Vec::with_capacity(path.len() + 5);
            if !path.starts_with('\\') {
                name.extend(r"\\.\".encode_utf16());
            }
            name.extend(path.encode_utf16());
            name.push(0);
            // SAFETY: `name` ends in NUL and lives across the call; no
            // sharing, attributes or template.
            let handle = unsafe {
                CreateFileW(
                    name.as_ptr(),
                    GENERIC_READ | GENERIC_WRITE,
                    0,
                    ptr::null(),
                    OPEN_EXISTING,
                    FILE_FLAG_OVERLAPPED,
                    ptr::null_mut(),
                )
            };
            if handle == INVALID_HANDLE_VALUE {
                return Err(io::Error::last_os_error());
            }
            // SAFETY: the handle was just opened, and nothing else owns or
            // closes it: `OwnedHandle` closes it once, as the last holder
            // drops it.
            let handle = Arc::new(unsafe { OwnedHandle::from_raw_handle(handle) });
            let port = Self {
                handle,
                io: Event::new()?,
            };
            port.set(|dcb| {
                dcb.BaudRate = baud;
                dcb.ByteSize = 8;
                dcb.Parity = NOPARITY;
                dcb.StopBits = ONESTOPBIT;
                dcb.XonChar = 17;
                dcb.XoffChar = 19;
                dcb.ErrorChar = 0;
                dcb.EofChar = 26;
                dcb._bitfield = (dcb._bitfield & !FLAGS_SET) | 1;
            })?;
            port.timeouts(crate::TIMEOUT)?;
            // SAFETY: the handle is open while `port` lives.
            done(unsafe { SetCommMask(port.raw(), EV_RXCHAR) })?;
            Ok(port)
        }

        fn raw(&self) -> HANDLE {
            self.handle.as_raw_handle()
        }

        /// The line's settings, read, changed by `change` and written back.
        fn set(&self, change: impl FnOnce(&mut DCB)) -> io::Result<()> {
            // SAFETY: all zero is a DCB before `GetCommState` fills it.
            let mut dcb: DCB = unsafe { std::mem::zeroed() };
            dcb.DCBlength = u32::try_from(size_of::<DCB>()).unwrap_or(u32::MAX);
            // SAFETY: the handle is open; `dcb` is writable, its length set.
            done(unsafe { GetCommState(self.raw(), &raw mut dcb) })?;
            change(&mut dcb);
            // SAFETY: the handle is open; `dcb` is read for the call alone.
            done(unsafe { SetCommState(self.raw(), &raw const dcb) })
        }

        /// Each read and each write waiting at most `wait`, as `serialport`
        /// sets it: a read ends as soon as a byte is there, or with none
        /// once `wait` passed; a wait of nothing reads what is there, at
        /// once.
        fn timeouts(&self, wait: Duration) -> io::Result<()> {
            let constant = if wait.is_zero() {
                0
            } else {
                milliseconds(wait)
            };
            let multiplier = if wait.is_zero() { 0 } else { u32::MAX };
            let timeouts = COMMTIMEOUTS {
                ReadIntervalTimeout: u32::MAX,
                ReadTotalTimeoutMultiplier: multiplier,
                ReadTotalTimeoutConstant: constant,
                WriteTotalTimeoutMultiplier: 0,
                WriteTotalTimeoutConstant: constant,
            };
            // SAFETY: the handle is open; `timeouts` is read for the call.
            done(unsafe { SetCommTimeouts(self.raw(), &raw const timeouts) })
        }
    }

    /// What came, within the port's read timeout; nothing within it times
    /// out, as `serialport`'s read does.
    impl io::Read for ComPort {
        fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
            let (handle, length) = (self.raw(), u32::try_from(buf.len()).unwrap_or(u32::MAX));
            let into = buf.as_mut_ptr();
            let read = waited(On::File(handle), &self.io, None, None, |overlapped| {
                // SAFETY: `buf` is writable for `length` bytes and outlives
                // the read, which `waited` waits for to its end.
                unsafe { ReadFile(handle, into, length, ptr::null_mut(), overlapped) }
            })?;
            if read == 0 {
                Err(io::ErrorKind::TimedOut.into())
            } else {
                Ok(read)
            }
        }
    }

    impl io::Write for ComPort {
        fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
            let (handle, length) = (self.raw(), u32::try_from(buf.len()).unwrap_or(u32::MAX));
            waited(On::File(handle), &self.io, None, None, |overlapped| {
                // SAFETY: `buf` is readable for `length` bytes and outlives
                // the write, which `waited` waits for to its end.
                unsafe { WriteFile(handle, buf.as_ptr(), length, ptr::null_mut(), overlapped) }
            })
        }

        fn flush(&mut self) -> io::Result<()> {
            // SAFETY: the handle is open while the port lives.
            done(unsafe { FlushFileBuffers(self.raw()) })
        }
    }

    impl Port for ComPort {
        fn wait(&mut self, wait: Duration) -> io::Result<()> {
            self.timeouts(wait)
        }

        fn speed(&mut self, baud: u32) -> io::Result<()> {
            self.set(|dcb| dcb.BaudRate = baud)
        }

        fn watch(&self) -> io::Result<Arc<dyn Watch>> {
            Ok(Arc::new(ComWatch {
                handle: Arc::clone(&self.handle),
                came: Event::new()?,
                closing: Event::new()?,
            }))
        }
    }

    /// A COM port's `EV_RXCHAR`, waited for with `WaitCommEvent` beside the
    /// port's reads and writes, one wait at a time.
    struct ComWatch {
        handle: Arc<OwnedHandle>,
        /// The event a wait ends on.
        came: Event,
        /// Set once, as the port closes: every wait after it ends at once.
        closing: Event,
    }

    impl Watch for ComWatch {
        /// Bytes came, or may have: the wait ended with an event, or as its
        /// port closes; nothing came where it timed out.
        fn readable(&self, wait: Duration) -> io::Result<bool> {
            let handle = self.handle.as_raw_handle();
            let mut events = 0u32;
            let waiting = Some(&self.closing);
            let came = waited(
                On::File(handle),
                &self.came,
                Some(wait),
                waiting,
                |overlapped| {
                    // SAFETY: the handle is open while `self` lives; `events`
                    // and the OVERLAPPED live until the wait has ended, which
                    // `waited` waits for.
                    unsafe { WaitCommEvent(handle, &raw mut events, overlapped) }
                },
            );
            match came {
                Ok(_) => Ok(true),
                Err(e) if e.kind() == io::ErrorKind::TimedOut => Ok(false),
                Err(e) if e.kind() == io::ErrorKind::Interrupted => Ok(true),
                Err(e) => Err(e),
            }
        }

        fn cancel(&self) {
            self.closing.set();
        }
    }
}

#[cfg(unix)]
mod descriptor {
    use std::io::{self, PipeReader, PipeWriter, Write};
    use std::os::fd::{AsFd, AsRawFd, BorrowedFd};
    use std::time::Duration;

    use crate::Watch;

    /// A serial port's descriptor, polled beside the port's reads and
    /// writes: a copy of it, and the pipe its port's close writes to.
    pub(crate) struct Polled {
        port: serialport::TTYPort,
        closing: (PipeReader, PipeWriter),
    }

    impl Polled {
        /// `port`'s watch, on a copy of its descriptor.
        ///
        /// # Errors
        /// Where the descriptor could not be copied, or no pipe made.
        pub(crate) fn of(port: &serialport::TTYPort) -> io::Result<Self> {
            Ok(Self {
                port: port.try_clone_native().map_err(io::Error::from)?,
                closing: io::pipe()?,
            })
        }
    }

    impl Watch for Polled {
        fn readable(&self, wait: Duration) -> io::Result<bool> {
            // SAFETY: the descriptor is the copy `self.port` owns, open while
            // `self` lives, lent for this one wait.
            let port = unsafe { BorrowedFd::borrow_raw(self.port.as_raw_fd()) };
            let ready = transport::socket::ready(&[port, self.closing.0.as_fd()], Some(wait))?;
            Ok(ready.contains(&true))
        }

        fn cancel(&self) {
            // A byte in the pipe, never read: every poll after it ends at
            // once. A pipe that will not take it is full of them already.
            let _ = (&self.closing.1).write(&[0]);
        }
    }
}
