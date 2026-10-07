//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! A memory SD card for the ring_saver's tests: the recordings directory as [`recorder::Fs`] sees it, with its power
//! cut on demand. It keeps FatFs's promise and no more: an append lands at the file's end, an `fsync` makes the
//! file's bytes durable, and at a cut the bytes written since the last `fsync` survive as [`Keep`] says (FatFs
//! updates a file's size only when it syncs, so in practice none of them do). After a cut every call fails, as on
//! a dead board, until [`Card::power_on`]; a card that never comes back is a dead SD. Short of a cut, one call can
//! fail with the card still powered ([`Card::fail_after`]), and a file can be locked against appends, as FatFs's
//! lock table refuses a write-open while an HTTP pull reads the file ([`Card::lock`]).
//!
//! Each handle is a clone sharing one card, so the test keeps one while the saver owns another. Beside it, the
//! readings of a card file the saver tests share: its lines, which verify, the tools' duplicate filter, its headers.

use std::cell::RefCell;
use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;
use std::rc::Rc;

/// What a cut leaves of the bytes written since a file's last `fsync`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Keep {
    /// None of them: FatFs, which writes a file's size only when it syncs.
    Synced,
    /// Every completed write; the call the power went in did nothing.
    Written,
    /// Every completed write, and this many bytes of the write the power went in: a torn last line.
    Torn(usize),
}

/// Why a card call failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CardError {
    /// The power was cut: this call, or an earlier one.
    PowerCut,
    /// The file's directory does not exist (FatFs creates no parent).
    NoDirectory,
    /// One call failed, the card still powered.
    Failed,
    /// The file is locked: FR_LOCKED, EACCES through the VFS.
    Locked,
}

/// One file: the bytes an `fsync` made durable, and those written since.
#[derive(Clone, Debug, Default)]
struct File {
    /// Durable.
    synced: Vec<u8>,
    /// Written since the last `fsync`.
    written: Vec<u8>,
}

/// The card's state.
#[derive(Debug, Default)]
struct State {
    /// Every file by its path.
    files: BTreeMap<String, File>,
    /// Every directory by its path.
    directories: BTreeSet<String>,
    /// The calls that change the card so far: create_dir_all, open_append, write, fsync.
    operations: u32,
    /// The call the power goes in, and what it leaves.
    cut: Option<(u32, Keep)>,
    /// The call that fails with the power on, and what it leaves.
    fault: Option<(u32, Keep)>,
    /// The file no append may open.
    locked: Option<String>,
    /// Whether the power is off.
    dead: bool,
    /// The `tail` calls so far.
    looks: u32,
}

/// A memory SD card. See the module notes.
#[derive(Clone, Debug, Default)]
pub struct Card(Rc<RefCell<State>>);

/// An open file: its path.
#[derive(Debug)]
pub struct Handle(String);

impl Card {
    /// An empty card.
    pub fn new() -> Card {
        Card::default()
    }

    /// Cuts the power in the call that changes the card after `operations` more, leaving what `keep` says.
    pub fn cut_after(&self, operations: u32, keep: Keep) {
        let mut state = self.0.borrow_mut();
        state.cut = Some((state.operations + operations, keep));
    }

    /// Fails the call that changes the card after `operations` more, the card staying powered: a write lands what
    /// `keep` says of its bytes (none for [`Keep::Synced`]), and the bytes before it stay.
    pub fn fail_after(&self, operations: u32, keep: Keep) {
        let mut state = self.0.borrow_mut();
        state.fault = Some((state.operations + operations, keep));
    }

    /// Locks `file` against appends (None unlocks): an HTTP pull holds it open for reading.
    pub fn lock(&self, file: Option<&str>) {
        self.0.borrow_mut().locked = file.map(String::from);
    }

    /// The `tail` calls so far: how often the saver looked at a file.
    pub fn looks(&self) -> u32 {
        self.0.borrow().looks
    }

    /// A card that fails every call from now on: a dead SD.
    pub fn kill(&self) {
        self.0.borrow_mut().dead = true;
    }

    /// The power comes back: what the cut left is all there is, and every call works again.
    pub fn power_on(&self) {
        let mut state = self.0.borrow_mut();
        state.dead = false;
        state.cut = None;
        for file in state.files.values_mut() {
            let written = std::mem::take(&mut file.written);
            file.synced.extend_from_slice(&written);
        }
    }

    /// The calls that changed the card so far.
    pub fn operations(&self) -> u32 {
        self.0.borrow().operations
    }

    /// Whether the power is off.
    pub fn dead(&self) -> bool {
        self.0.borrow().dead
    }

    /// Every file and the bytes it holds now, by path.
    pub fn files(&self) -> BTreeMap<String, Vec<u8>> {
        let state = self.0.borrow();
        state
            .files
            .iter()
            .map(|(path, file)| (path.clone(), [file.synced.as_slice(), &file.written].concat()))
            .collect()
    }

    /// One file's bytes now; empty when it does not exist.
    pub fn file(&self, path: &str) -> Vec<u8> {
        self.files().remove(path).unwrap_or_default()
    }

    /// Writes the card's files under `root`, as a pulled card for the host tools.
    pub fn write_to(&self, root: &Path) {
        for (path, bytes) in self.files() {
            let target = root.join(&path);
            std::fs::create_dir_all(target.parent().unwrap()).unwrap();
            std::fs::write(target, bytes).unwrap();
        }
    }

    /// Starts a call that changes the card: fails once the power is off, cuts it in the call it was set for
    /// (dropping what [`Keep`] does not leave), and fails the call a fault was set for. Returns what the failing
    /// call leaves and its error, when this call fails.
    fn begin(&self) -> Result<Option<(Keep, CardError)>, CardError> {
        let mut state = self.0.borrow_mut();
        if state.dead {
            return Err(CardError::PowerCut);
        }
        let operation = state.operations;
        state.operations += 1;
        if let Some((at, keep)) = state.cut
            && at == operation
        {
            state.dead = true;
            if keep == Keep::Synced {
                for file in state.files.values_mut() {
                    file.written.clear();
                }
            }
            return Ok(Some((keep, CardError::PowerCut)));
        }
        if let Some((at, keep)) = state.fault
            && at == operation
        {
            state.fault = None;
            return Ok(Some((keep, CardError::Failed)));
        }
        Ok(None)
    }
}

/// The parent directory of `path` ("" at the root).
fn parent(path: &str) -> &str {
    path.rfind('/').map_or("", |slash| &path[..slash])
}

impl recorder::Fs for Card {
    type File = Handle;
    type Error = CardError;

    fn create_dir_all(&mut self, directory: &str) -> Result<(), CardError> {
        if let Some((_, error)) = self.begin()? {
            return Err(error);
        }
        let mut state = self.0.borrow_mut();
        let mut path = directory;
        while !path.is_empty() {
            state.directories.insert(path.to_string());
            path = parent(path);
        }
        Ok(())
    }

    fn tail(&mut self, file: &str, buffer: &mut [u8]) -> Result<u64, CardError> {
        let mut state = self.0.borrow_mut();
        if state.dead {
            return Err(CardError::PowerCut);
        }
        state.looks += 1;
        let Some(file) = state.files.get(file) else {
            return Ok(0);
        };
        let bytes = [file.synced.as_slice(), &file.written].concat();
        let read = bytes.len().min(buffer.len());
        buffer[..read].copy_from_slice(&bytes[bytes.len() - read..]);
        Ok(bytes.len() as u64)
    }

    fn open_append(&mut self, file: &str) -> Result<Handle, CardError> {
        if self.0.borrow().locked.as_deref() == Some(file) {
            return Err(CardError::Locked);
        }
        if let Some((_, error)) = self.begin()? {
            return Err(error);
        }
        let mut state = self.0.borrow_mut();
        let directory = parent(file);
        if !directory.is_empty() && !state.directories.contains(directory) {
            return Err(CardError::NoDirectory);
        }
        state.files.entry(file.to_string()).or_default();
        Ok(Handle(file.to_string()))
    }

    fn write(&mut self, file: &mut Handle, bytes: &[u8]) -> Result<(), CardError> {
        let failed = self.begin()?;
        let landed = match failed {
            None => bytes.len(),
            Some((Keep::Torn(torn), _)) => torn.min(bytes.len()),
            Some(_) => 0,
        };
        let mut state = self.0.borrow_mut();
        let target = state.files.get_mut(&file.0).expect("an open file exists");
        target.written.extend_from_slice(&bytes[..landed]);
        match failed {
            None => Ok(()),
            Some((_, error)) => Err(error),
        }
    }

    fn fsync(&mut self, file: &mut Handle) -> Result<(), CardError> {
        if let Some((_, error)) = self.begin()? {
            return Err(error);
        }
        let mut state = self.0.borrow_mut();
        let target = state.files.get_mut(&file.0).expect("an open file exists");
        let written = std::mem::take(&mut target.written);
        target.synced.extend_from_slice(&written);
        Ok(())
    }
}

/// The routing a card file's lines carry: its name, or None for recorder.log.
pub fn routing_of(path: &str) -> Option<&str> {
    let name = path.rsplit('/').next().unwrap();
    (name != "recorder.log").then_some(name)
}

/// The lines of `bytes`, each with its `\n`, and a last one without when it has none.
pub fn lines(bytes: &[u8]) -> Vec<&[u8]> {
    bytes.split_inclusive(|&byte| byte == b'\n').collect()
}

/// The payload of one line of the file at `path` when it verifies there.
pub fn payload<'a>(path: &str, line: &'a [u8]) -> Option<&'a [u8]> {
    let body = line.strip_suffix(b"\n")?;
    recorder_wire::verify(routing_of(path).map(str::as_bytes), body)
}

/// The file at `path` as the tools read it after their duplicate filter (tools/recorder_wire.py `unique`): a
/// stamped line that verifies is dropped when its (uptime, OPEN) came before; every other line stays.
pub fn unique(path: &str, bytes: &[u8]) -> Vec<u8> {
    let mut seen = BTreeSet::new();
    let mut kept = Vec::new();
    for line in lines(bytes) {
        if let Some(payload) = payload(path, line) {
            let digits = payload.iter().take_while(|byte| byte.is_ascii_digit()).count();
            let open = recorder_wire::unwrap(line.strip_suffix(b"\n").unwrap()).unwrap().open;
            if digits > 0 && !seen.insert((payload[..digits].to_vec(), open)) {
                continue;
            }
        }
        kept.extend_from_slice(line);
    }
    kept
}

/// Whether `bytes` holds `needle`.
pub fn contains(bytes: &[u8], needle: &[u8]) -> bool {
    bytes.windows(needle.len()).any(|window| window == needle)
}

/// The header lines of the file at `path`: lines that verify and carry no uptime.
pub fn headers(path: &str, bytes: &[u8]) -> usize {
    lines(bytes)
        .into_iter()
        .filter_map(|line| payload(path, line))
        .filter(|payload| !payload.first().is_some_and(u8::is_ascii_digit))
        .count()
}
