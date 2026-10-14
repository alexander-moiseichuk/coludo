//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The entry file (M21, doc/specs/avionics-port.md): `/sdcard/entry.gcl`, one GCL line per line, `#` comments. Once
//! bring-up is done, each line runs in order through the executor (origin [`Origin::Entry`], the one point of
//! injection), under the normal gates, so changes still run only in SETTING; a console-only command is refused, since
//! the file sits on a card the hub can pull. Each line and its answer go to recorder.log. It replaces MicroPython's
//! run-any-script flexibility for bench tests, soaks and HITL setup (`ring load seconds=600`, `video start`,
//! `pace seconds=60`).
//!
//! **A bench mode never rides into a flight:** while the file is on the card, `verify` and `arm` refuse with
//! `entry-file` ([`problem`]); `entry clear confirm` (or `rm confirm path=entry.gcl`) takes it away.
//!
//! The file is written over CC, no card reader needed ([`register_commands`], SETTING only): `entry add text=<line>`
//! appends a line (the hub carries its spaces base64), `entry show` lists them, `entry clear confirm` removes the
//! file. They run from the next boot.

use std::time::Duration;

use esp_idf_svc::hal::cpu::Core;
use firmware::gcl::Origin;
use firmware::sd;

use std::io::Write as _;

use firmware::gcl::{self, Command, Gate, Kind, Parameter, Reach};

use crate::commands::Executor;
use firmware::threads::{self, Spec, Stack};

/// The file's name at the card's root.
pub const NAME: &str = "entry.gcl";

/// The most of the file read: a page of lines, not a program.
const MOST_BYTES: u64 = 16 << 10;

/// The runner's stack (PSRAM): a line, its answer, the log.
const STACK_SIZE: usize = 8 << 10;

/// The runner's priority on core 0: below the executor, which runs the lines.
const PRIORITY: u8 = 2;

/// The file's path on the card.
fn path() -> String {
    format!("{}/{NAME}", sd::MOUNT)
}

/// Why `verify` and `arm` refuse while the file is on the card; None when it is not.
pub fn problem() -> Option<String> {
    std::path::Path::new(&path()).exists().then(|| {
        format!(
            "{} is on the card: a bench mode never rides into a flight (entry clear confirm)",
            path()
        )
    })
}

/// The lines to run: trimmed, no blank lines, no `#` comments.
fn lines(text: &str) -> Vec<String> {
    text.lines()
        .map(str::trim)
        .filter(|line| !line.is_empty() && !line.starts_with('#'))
        .map(str::to_string)
        .collect()
}

/// Runs the entry file's lines on a thread of their own, in order, through `executor`, when the card holds one; at
/// once otherwise. Call it once bring-up is done and the card is mounted.
///
/// Errors: the runner would not spawn.
pub fn start(executor: Executor) -> anyhow::Result<()> {
    let path = path();
    let text = match std::fs::metadata(&path) {
        Err(_) => return Ok(()),
        Ok(metadata) if metadata.len() > MOST_BYTES => {
            log::error!(
                "entry :: {path} holds {} bytes, more than {MOST_BYTES}: not run",
                metadata.len()
            );
            return Ok(());
        }
        Ok(_) => match std::fs::read_to_string(&path) {
            Ok(text) => text,
            Err(error) => {
                log::error!("entry :: {path} would not read: {error}");
                return Ok(());
            }
        },
    };
    let lines = lines(&text);
    log::warn!(
        "entry :: {path}: {} lines to run; verify and arm refuse while it is on the card",
        lines.len()
    );
    threads::spawn(
        Spec {
            name: c"entry",
            stack_size: STACK_SIZE,
            stack: Stack::Psram,
            priority: PRIORITY,
            core: Core::Core0,
        },
        move || {
            // the console and the hub get their turn first: the bring-up's own lines are still settling
            std::thread::sleep(Duration::from_secs(1));
            for (number, line) in lines.iter().enumerate() {
                log::info!("entry :: {} > {line}", number + 1);
                let answer = executor
                    .ask(line, Origin::Entry)
                    .unwrap_or_else(|| "(no answer)".into());
                log::info!("entry :: {} < {answer}", number + 1);
            }
            log::info!("entry :: done");
        },
    )?;
    Ok(())
}

/// The entry file's commands (crate::gcl), registered when the card mounts: they write the file the next boot runs.
pub fn register_commands() {
    gcl::register(vec![
        Command {
            owner: "entry",
            words: "entry add",
            about: "append a line to the card's entry.gcl, run at the next boot (verify and arm refuse while it is there)",
            gate: Gate::Setting,
            reach: Reach::Everywhere,
            parameters: &[Parameter {
                name: "text",
                kind: Kind::Text,
                default: None,
                about: "one GCL line, e.g. `ring load seconds=600` (the hub carries its spaces)",
            }],
            bare_words: &[],
            run: Box::new(|arguments| {
                let line = arguments.text("text").trim().replace(['\r', '\n'], " ");
                let appended = std::fs::OpenOptions::new()
                    .create(true)
                    .append(true)
                    .open(path())
                    .and_then(|mut file| writeln!(file, "{line}"));
                match appended {
                    Ok(()) => cc_protocol::build("ok", &["entry", &line], &[]),
                    Err(error) => cc_protocol::build("err", &["entry", &error.to_string()], &[]),
                }
            }),
        },
        Command {
            owner: "entry",
            words: "entry show",
            about: "the card's entry.gcl lines, as the next boot runs them",
            gate: Gate::Open,
            reach: Reach::Everywhere,
            parameters: &[],
            bare_words: &[],
            run: Box::new(|_| match std::fs::read_to_string(path()) {
                Ok(text) => {
                    let json = serde_json::json!({ "lines": lines(&text) }).to_string();
                    cc_protocol::build("ok", &[&json], &[])
                }
                Err(_) => cc_protocol::build("ok", &[r#"{"lines":[]}"#], &[]),
            }),
        },
        Command {
            owner: "entry",
            words: "entry clear",
            about: "remove the card's entry.gcl: the next boot runs nothing, verify and arm no longer refuse",
            gate: Gate::Setting,
            reach: Reach::Everywhere,
            parameters: &[],
            bare_words: &[gcl::CONFIRM],
            run: Box::new(|_| match std::fs::remove_file(path()) {
                Ok(()) => cc_protocol::build("ok", &["entry", "cleared"], &[]),
                Err(error) => cc_protocol::build("err", &["entry", &error.to_string()], &[]),
            }),
        },
    ]);
}
