//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The SD card as the ring_saver uses it (doc/specs/avionics-port.md "The ring_saver", "SD and flash file layout"):
//! the recordings directory, `/sdcard/recordings` on the board, holding one directory per session and the global
//! session.csv. The saver owns every file under it, so nothing else appends there while it runs.
//!
//! [`Fs`] is the few calls the saver makes, each naming its file by a path relative to that directory with `/`
//! between the parts (`000123/000123_health.csv`, `session.csv`). It exists so that the host tests can cut the
//! card's power at any call; [`StdFs`] serves the board (IDF's FatFs VFS under `std::fs`) and the host tests alike.
//!
//! What the saver relies on, as FatFs gives it: an append lands at the file's end, and the bytes written since the
//! last `fsync` may be lost at a cut, whole or in part, never the ones before it. So what a file holds after a cut or
//! a failed call is what it held before, then a prefix of what was appended since: the saver resumes past it.

use std::fmt::Debug;
use std::fs::{File, OpenOptions};
use std::io::{self, ErrorKind, Read, Seek, SeekFrom, Write};
use std::path::PathBuf;

/// The calls the ring_saver makes on the card. A file opened for appending is closed by dropping it.
pub trait Fs {
    /// An open file.
    type File;
    /// What a failed call reports (`std::io::Error` for [`StdFs`]).
    type Error: Debug;

    /// Creates `directory` and every missing parent; an existing one is fine.
    fn create_dir_all(&mut self, directory: &str) -> Result<(), Self::Error>;

    /// Reads the end of `file` into `buffer`: its last `buffer.len()` bytes, or all of them when it is shorter.
    /// Returns its size in bytes, 0 when it does not exist, so `buffer[..size.min(buffer.len())]` holds its end.
    ///
    /// The saver looks at a file's end before its first append in a run: to end a line a cut left torn, to know
    /// whether the file held a whole line, and which boot the global session.csv's last row names. After a failed
    /// call it asks the size alone (an empty buffer), to append only what did not land.
    fn tail(&mut self, file: &str, buffer: &mut [u8]) -> Result<u64, Self::Error>;

    /// Opens `file` for appending, creating it when it does not exist.
    fn open_append(&mut self, file: &str) -> Result<Self::File, Self::Error>;

    /// Appends all of `bytes` to `file`.
    fn write(&mut self, file: &mut Self::File, bytes: &[u8]) -> Result<(), Self::Error>;

    /// Makes everything appended to `file` so far durable: a cut no longer loses it.
    fn fsync(&mut self, file: &mut Self::File) -> Result<(), Self::Error>;
}

/// The card through `std::fs`, under a root directory: the board's `/sdcard/recordings`, or a host test's temporary
/// directory.
#[derive(Clone, Debug)]
pub struct StdFs {
    /// The recordings directory every path is relative to.
    root: PathBuf,
}

impl StdFs {
    /// The card's recordings under `root`. Nothing is touched until the first call.
    pub fn new(root: impl Into<PathBuf>) -> StdFs {
        StdFs { root: root.into() }
    }
}

impl Fs for StdFs {
    type File = File;
    type Error = io::Error;

    fn create_dir_all(&mut self, directory: &str) -> io::Result<()> {
        std::fs::create_dir_all(self.root.join(directory))
    }

    fn tail(&mut self, file: &str, buffer: &mut [u8]) -> io::Result<u64> {
        let mut handle = match File::open(self.root.join(file)) {
            Ok(handle) => handle,
            Err(error) if error.kind() == ErrorKind::NotFound => return Ok(0),
            Err(error) => return Err(error),
        };
        let size = handle.metadata()?.len();
        let read = size.min(buffer.len() as u64);
        if read > 0 {
            handle.seek(SeekFrom::Start(size - read))?;
            handle.read_exact(&mut buffer[..read as usize])?;
        }
        Ok(size)
    }

    fn open_append(&mut self, file: &str) -> io::Result<File> {
        OpenOptions::new().create(true).append(true).open(self.root.join(file))
    }

    fn write(&mut self, file: &mut File, bytes: &[u8]) -> io::Result<()> {
        file.write_all(bytes)
    }

    fn fsync(&mut self, file: &mut File) -> io::Result<()> {
        file.sync_all()
    }
}

#[cfg(test)]
mod tests {
    use super::{Fs, StdFs};
    use std::path::PathBuf;

    /// An empty directory of the test's own under the system temp dir.
    fn scratch(name: &str) -> PathBuf {
        let root = std::env::temp_dir().join(format!("coludo-std-fs-{}-{name}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);
        root
    }

    #[test]
    fn appends_and_reads_the_end_under_its_root() {
        let root = scratch("append");
        let mut card = StdFs::new(&root);
        let path = "000123/recorder.log";
        let mut end = [0u8; 4];
        card.create_dir_all("000123").unwrap();
        card.create_dir_all("000123").unwrap(); // an existing directory is fine
        assert_eq!(card.tail(path, &mut end).unwrap(), 0, "missing");
        let mut file = card.open_append(path).unwrap();
        assert_eq!(card.tail(path, &mut end).unwrap(), 0, "empty");
        card.write(&mut file, b"one\n").unwrap();
        card.write(&mut file, b"tw").unwrap();
        card.fsync(&mut file).unwrap();
        drop(file);
        assert_eq!(card.tail(path, &mut end).unwrap(), 6);
        assert_eq!(&end, b"e\ntw", "the last four bytes");
        assert_eq!(card.tail(path, &mut []).unwrap(), 6, "the size alone");
        let mut file = card.open_append(path).unwrap();
        card.write(&mut file, b"o\n").unwrap();
        drop(file);
        assert_eq!(std::fs::read(root.join(path)).unwrap(), b"one\ntwo\n");
        // negative: a buffer longer than the file takes it whole and leaves the rest untouched
        let mut long = [b'x'; 12];
        assert_eq!(card.tail(path, &mut long).unwrap(), 8);
        assert_eq!(&long, b"one\ntwo\nxxxx");
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn a_file_in_a_missing_directory_is_an_error_not_a_guess() {
        let root = scratch("missing");
        let mut card = StdFs::new(&root);
        assert!(
            card.open_append("000123/x.csv").is_err(),
            "the saver creates the directory first"
        );
        assert_eq!(card.tail("000123/x.csv", &mut [0; 8]).unwrap(), 0);
    }
}
