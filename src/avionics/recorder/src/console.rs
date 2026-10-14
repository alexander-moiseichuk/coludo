//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The flight recorder's USB console: its card from the host (tools/recorder_console.py is the host's end of it).
//! One command a line, one reply line or a run of them ending in a `<command> ::` line, `err <command>: <why>` on a
//! refusal. The console has no driver, so its reads never wait; the thread looks again every [`POLL`], below the link's
//! loop (crate `LINK_PRIORITY`): a walk over a session's files must never hold the UART reader.
//!
//! **Every reply starts with an empty line.** With no driver, output nobody reads is dropped, except the last 64 bytes
//! the USB FIFO still holds; they leave in front of the next write, so a reply's first line came glued to a status
//! line's head from a minute before (`I (137624) ... 55231 lindf 15327166464 15624077312`) and the host never saw it.
//! The empty line ends such a fragment, and the host reads it as a line of its own.
//!
//! - `ls [dir]`: a directory's entries, one `ls <name>;<size>` each (`<name>/` and no size for a directory), then
//!   `ls :: <count> entries`.
//! - `df`: `df <free bytes> <total bytes>` of the card's volume.
//! - `rm <path>`: a file, or a directory with all it holds; `rm :: <path>`. The card's root and the session being
//!   filed now are refused (its lines would go on landing in files the receiver no longer knows).
//! - `get <path>`: a whole file, `get <size>`, then its bytes in base64 lines of [`GET_CHUNK`] bytes, then
//!   `get :: <crc32>` (zlib's CRC-32 in hex, as Python's `binascii.crc32`): the console's text is not byte-exact (it
//!   turns `\n` into `\r\n`), base64 is.
//! - `files [dir]`: every file under the card (or `<dir>`), one `file <path>;<size>;<crc32>` each, then
//!   `files :: <count> files, <bytes> bytes`: a file being filed is read up to where it ends at that moment.
//! - `read <path> <offset> <length>`: at most 4 KB of a file, one line `read <offset> <length> <hex>`.

use std::io::{Read as _, Seek as _, SeekFrom};
use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use base64::Engine as _;
use base64::engine::general_purpose::STANDARD;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;

use crate::ROOT;

/// How often the console thread looks for typed bytes.
const POLL: Duration = Duration::from_millis(100);

/// The console thread's stack: a command and its directory walk.
const STACK: usize = 8 << 10;

/// The console thread's priority: below the link's loop.
const PRIORITY: u8 = 2;

/// The bytes of a file a `get` line carries (4 KB of base64).
pub const GET_CHUNK: usize = 3072;

/// Prints one reply line the way the logger prints its lines (esp-idf-svc's `EspLogger`): through newlib's `stdout`
/// FILE, in one `fwrite`, which holds the FILE's lock for the whole line. `println!` writes the descriptor itself, past
/// that lock, so it could land between the pieces of a log line longer than the FILE's buffer.
macro_rules! say {
    ($($argument:tt)*) => {{
        let mut line = format!($($argument)*);
        line.push('\n');
        // SAFETY: this task's newlib reentrancy structure and its stdout FILE live as long as the task; `line` is a
        // live buffer of `line.len()` bytes.
        unsafe {
            let stdout = (*esp_idf_svc::sys::__getreent())._stdout;
            esp_idf_svc::sys::fwrite(line.as_ptr().cast(), 1, line.len() as _, stdout);
        }
    }};
}

/// The session the filer files now (crate `Filer`): `rm` refuses it.
pub static FILING: Mutex<String> = Mutex::new(String::new());

/// Set by `rm`: the filer forgets what it knew of the card's directories and files (crate `Filer`), since the files
/// it had looked at may be gone.
pub static FORGET: AtomicBool = AtomicBool::new(false);

/// Starts the console thread.
///
/// Errors: the thread would not spawn.
pub fn spawn() -> anyhow::Result<()> {
    ThreadSpawnConfiguration {
        name: Some(c"console"),
        priority: PRIORITY,
        ..Default::default()
    }
    .set()?;
    let spawned = std::thread::Builder::new().stack_size(STACK).spawn(|| {
        let mut held = Vec::new();
        let mut chunk = [0u8; 64];
        loop {
            match std::io::stdin().read(&mut chunk) {
                Ok(count) if count > 0 => held.extend_from_slice(&chunk[..count]),
                _ => std::thread::sleep(POLL),
            }
            while let Some(end) = held.iter().position(|&byte| byte == b'\n') {
                let line: Vec<u8> = held.drain(..=end).collect();
                run(&String::from_utf8_lossy(&line));
            }
        }
    });
    ThreadSpawnConfiguration::default().set()?;
    spawned?;
    Ok(())
}

/// One command line.
fn run(line: &str) {
    let mut words = line.split_whitespace();
    let command = words.next();
    let arguments: Vec<&str> = words.collect();
    if command.is_none() {
        return;
    }
    say!("");
    match (command, arguments.as_slice()) {
        (None, _) => {}
        (Some("ls"), []) => ls(""),
        (Some("ls"), [directory]) => ls(directory),
        (Some("df"), []) => df(),
        (Some("rm"), [path]) => rm(path),
        (Some("get"), [path]) => get(path),
        (Some("files"), []) => files(""),
        (Some("files"), [directory]) => files(directory),
        (Some("read"), [path, offset, length]) => read(path, offset, length),
        (Some(other), _) => say!(
            "err {other}: the commands are `ls [dir]`, `df`, `rm <path>`, `get <path>`, `files [dir]` and `read \
             <path> <offset> <length>`"
        ),
    }
}

/// `path` under the card's root; None for a path that climbs out of it.
fn on_card(path: &str) -> Option<String> {
    let path = path.trim_matches('/');
    (!path.split('/').any(|part| part == "..")).then(|| {
        if path.is_empty() {
            ROOT.to_string()
        } else {
            format!("{ROOT}/{path}")
        }
    })
}

/// The console's `ls [dir]`.
fn ls(directory: &str) {
    let Some(path) = on_card(directory) else {
        say!("err ls: {directory} is not on the card");
        return;
    };
    let entries = match std::fs::read_dir(&path) {
        Ok(entries) => entries,
        Err(error) => {
            say!("err ls: {path}: {error}");
            return;
        }
    };
    let mut count = 0usize;
    for entry in entries.flatten() {
        let name = entry.file_name().to_string_lossy().into_owned();
        match entry.metadata() {
            Ok(metadata) if metadata.is_dir() => say!("ls {name}/"),
            Ok(metadata) => say!("ls {name};{}", metadata.len()),
            Err(error) => say!("err ls: {name}: {error}"),
        }
        count += 1;
    }
    say!("ls :: {count} entries");
}

/// The console's `df`.
fn df() {
    let (mut total, mut free) = (0u64, 0u64);
    let Ok(root) = std::ffi::CString::new(ROOT) else {
        return;
    };
    // SAFETY: a NUL-terminated mount point and two u64s to fill.
    match esp_idf_svc::sys::esp!(unsafe { esp_idf_svc::sys::esp_vfs_fat_info(root.as_ptr(), &mut total, &mut free) }) {
        Ok(()) => say!("df {free} {total}"),
        Err(error) => say!("err df: {error} (no card mounted?)"),
    }
}

/// The console's `rm <path>`.
fn rm(path: &str) {
    let Some(full) = on_card(path).filter(|full| full != ROOT) else {
        say!("err rm: {path} is the card's root, or not on it");
        return;
    };
    let filing = FILING.lock().map(|session| session.clone()).unwrap_or_default();
    if !filing.is_empty() && path.trim_matches('/').split('/').next() == Some(filing.as_str()) {
        say!("err rm: {path} is the session being filed now ({filing})");
        return;
    }
    let removed = match std::fs::metadata(&full) {
        Ok(metadata) if metadata.is_dir() => std::fs::remove_dir_all(&full),
        Ok(_) => std::fs::remove_file(&full),
        Err(error) => Err(error),
    };
    match removed {
        Ok(()) => {
            FORGET.store(true, Ordering::Release);
            say!("rm :: {path}");
        }
        Err(error) => say!("err rm: {full}: {error}"),
    }
}

/// The console's `get <path>`.
fn get(path: &str) {
    let Some(full) = on_card(path) else {
        say!("err get: {path} is not on the card");
        return;
    };
    let mut file = match std::fs::File::open(&full) {
        Ok(file) => file,
        Err(error) => {
            say!("err get: {full}: {error}");
            return;
        }
    };
    let size = file.metadata().map(|metadata| metadata.len()).unwrap_or(0);
    say!("get {size}");
    let mut buffer = vec![0u8; GET_CHUNK];
    let mut crc = 0u32;
    loop {
        // a whole chunk a line (a short read is topped up), so every line but the last is GET_CHUNK bytes
        let mut got = 0;
        while got < buffer.len() {
            match file.read(&mut buffer[got..]) {
                Ok(0) => break,
                Ok(count) => got += count,
                Err(error) => {
                    say!("err get: {full}: {error}");
                    return;
                }
            }
        }
        if got == 0 {
            break;
        }
        crc = recorder_wire::crc32(crc, &buffer[..got]);
        say!("{}", STANDARD.encode(&buffer[..got]));
        if got < buffer.len() {
            break;
        }
    }
    say!("get :: {crc:08x}");
}

/// The console's `files [dir]`.
fn files(directory: &str) {
    let Some(start) = on_card(directory) else {
        say!("err files: {directory} is not on the card");
        return;
    };
    let mut pending = vec![std::path::PathBuf::from(&start)];
    let (mut count, mut total) = (0u64, 0u64);
    let mut buffer = vec![0u8; 4096];
    while let Some(directory) = pending.pop() {
        let entries = match std::fs::read_dir(&directory) {
            Ok(entries) => entries,
            Err(error) => {
                say!("err files: {}: {error}", directory.display());
                continue;
            }
        };
        for entry in entries.flatten() {
            let path = entry.path();
            if entry.file_type().is_ok_and(|kind| kind.is_dir()) {
                pending.push(path);
                continue;
            }
            let summed = std::fs::File::open(&path).and_then(|mut file| {
                let (mut size, mut crc) = (0u64, 0u32);
                loop {
                    let read = file.read(&mut buffer)?;
                    if read == 0 {
                        return Ok((size, crc));
                    }
                    size += read as u64;
                    crc = recorder_wire::crc32(crc, &buffer[..read]);
                }
            });
            let shown = path.strip_prefix(ROOT).unwrap_or(&path).display().to_string();
            match summed {
                Ok((size, crc)) => {
                    say!("file {shown};{size};{crc:08x}");
                    count += 1;
                    total += size;
                }
                Err(error) => say!("err files: {shown}: {error}"),
            }
        }
    }
    say!("files :: {count} files, {total} bytes");
}

/// The console's `read <path> <offset> <length>`.
fn read(path: &str, offset: &str, length: &str) {
    let (Ok(offset), Ok(length)) = (offset.parse::<u64>(), length.parse::<usize>()) else {
        say!("err read: <offset> and <length> are numbers");
        return;
    };
    let Some(full) = on_card(path) else {
        say!("err read: {path} is not on the card");
        return;
    };
    let mut bytes = vec![0u8; length.min(4096)];
    let got = std::fs::File::open(&full).and_then(|mut file| {
        file.seek(SeekFrom::Start(offset))?;
        let mut got = 0;
        while got < bytes.len() {
            let count = file.read(&mut bytes[got..])?;
            if count == 0 {
                break;
            }
            got += count;
        }
        Ok(got)
    });
    match got {
        Ok(got) => {
            let hex: String = bytes[..got].iter().map(|byte| format!("{byte:02x}")).collect();
            say!("read {offset} {got} {hex}");
        }
        Err(error) => say!("err read: {full}: {error}"),
    }
}
