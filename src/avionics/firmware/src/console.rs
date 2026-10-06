//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The device console on UART0 (the board's CH343P USB-serial port): the CC commands, plus a few that only
//! an operator at the cable may run.
//!
//! `wifi <ssid> <password>` stores a network's password (values follow the CC encoding, so a password with
//! spaces travels as `base64:`; the network list itself is the board config's); `panic` panics on purpose, to prove a panic reboots;
//! `c6reset` browns the Wi-Fi co-processor out on purpose, to prove a failing C6 link does not reboot the P4;
//! `sdtest [megabytes] [kilobytes per second]` writes to the microSD (flat out, or paced) and reports the rate and
//! the worst single write; `video start [width height fps kbit/s]` / `video stop` / `video stats` drive the
//! camera recorder; `load [s]` / `pace [s]` price CPU and the flight loop's timing, `flashwrite [s]` loads the
//! flash ring in the background (src/diagnostics.rs).

use std::io::{BufRead, Write};
use std::sync::Arc;

use esp_idf_svc::hal::gpio::{AnyIOPin, InputPin, OutputPin};
use esp_idf_svc::hal::uart::{Uart, UartDriver};
use esp_idf_svc::io::vfs::BlockingStdIo;

use crate::board::Board;
use crate::commands::Executor;
use crate::config::Credentials;

/// The P4 pin that drives the C6's EN (the Function-EV pinout; esp_hosted's slave reset).
const C6_ENABLE_PIN: i32 = 54;

/// The console thread's stack: the commands build JSON, nothing deeper.
const STACK_SIZE: usize = 8192;

/// Take over UART0 for blocking line reads and serve the console on its own thread, forever.
pub fn start(
    uart: impl Uart + 'static,
    transmit: impl OutputPin + 'static,
    receive: impl InputPin + 'static,
    board: Arc<Board>,
    executor: Executor,
    credentials: Credentials,
) -> anyhow::Result<()> {
    // drain what the boot already printed: installing the driver mid-line cuts the line off
    // SAFETY: a polling wait on the console UART's transmit FIFO.
    unsafe { esp_idf_svc::sys::uart_wait_tx_idle_polling(0) };
    let driver = UartDriver::new(
        uart,
        transmit,
        receive,
        Option::<AnyIOPin>::None,
        Option::<AnyIOPin>::None,
        &Default::default(),
    )?;
    let standard_io = BlockingStdIo::uart(driver)?;
    std::thread::Builder::new()
        .name("console".into())
        .stack_size(STACK_SIZE)
        .spawn(move || {
            let _standard_io = standard_io; // stdin blocks only while this lives
            let mut line = String::new();
            loop {
                line.clear();
                if std::io::stdin().lock().read_line(&mut line).is_err() {
                    continue;
                }
                if let Some(reply) = operator(line.trim(), &board, &credentials).or_else(|| executor.ask(line.trim())) {
                    println!("{reply}");
                    let _ = std::io::stdout().flush();
                }
            }
        })?;
    Ok(())
}

/// The console-only commands; None hands the line on to the CC commands.
fn operator(line: &str, board: &Board, credentials: &Credentials) -> Option<String> {
    let message = cc_protocol::parse(line).ok()?;
    match message.command.as_deref()? {
        "panic" => panic!("deliberate panic from the console"),
        "c6reset" => {
            // the C6's EN is P4 GPIO54 (esp_hosted drives it as an output): 100 ms low = a C6 brown-out
            // SAFETY: level writes on the pin esp_hosted already configured as an output.
            unsafe {
                esp_idf_svc::sys::gpio_set_level(C6_ENABLE_PIN, 0);
                std::thread::sleep(std::time::Duration::from_millis(100));
                esp_idf_svc::sys::gpio_set_level(C6_ENABLE_PIN, 1);
            }
            Some(cc_protocol::build("ok", &["c6reset"], &[]))
        }
        "sdtest" => {
            let megabytes = message
                .args
                .first()
                .and_then(|value| value.parse().ok())
                .unwrap_or(16usize);
            let rate = message.args.get(1).and_then(|value| value.parse().ok());
            Some(match (board.sd_mounted, firmware::sd::write_test(megabytes, rate)) {
                (false, _) => cc_protocol::build("err", &["no-sd"], &[]),
                (true, Ok(test)) => cc_protocol::build(
                    "ok",
                    &["sdtest"],
                    &[
                        ("rate_mbs", &format!("{:.2}", test.rate)),
                        ("worst_ms", &format!("{:.1}", test.worst_write_ms)),
                    ],
                ),
                (true, Err(error)) => cc_protocol::build("err", &["sdtest", &error.to_string()], &[]),
            })
        }
        "video" => Some(video(&message.args, board)),
        "load" => Some(crate::diagnostics::load(seconds(&message.args, 5))),
        "flashwrite" => Some(crate::diagnostics::flash_write(seconds(&message.args, 30))),
        "pace" => {
            let source = if message.args.iter().any(|arg| arg == "gptimer") {
                crate::diagnostics::Source::Gptimer
            } else {
                crate::diagnostics::Source::EspTimer
            };
            Some(crate::diagnostics::pace(seconds(&message.args, 10), source))
        }
        "wifi" => Some(match message.args.as_slice() {
            [ssid, password] => match credentials.set_password(ssid, password) {
                Ok(()) => cc_protocol::build("ok", &[ssid], &[]),
                Err(error) => cc_protocol::build("err", &["store", &error.to_string()], &[]),
            },
            _ => cc_protocol::build("err", &["usage", "wifi <ssid> <password>"], &[]),
        }),
        _ => None,
    }
}

/// `video start [width height fps kbit/s] [bars] [bayer=N]` (default 1920 1080 25 8000; `bars`: the sensor's
/// colour-bar test pattern; `bayer=`: the ISP's Bayer order, 0 RGGB .. 3 BGGR, default the OV5647's BGGR) |
/// `video stop` | `video stats`.
fn video(args: &[String], board: &Board) -> String {
    let number = |index: usize, default: u32| args.get(index).and_then(|value| value.parse().ok()).unwrap_or(default);
    match args.first().map(String::as_str) {
        Some("start") if !board.sd_mounted => cc_protocol::build("err", &["no-sd"], &[]),
        Some("start") => {
            let settings = firmware::video::Settings {
                width: number(1, 1920) as u16,
                height: number(2, 1080) as u16,
                fps: number(3, 25) as u8,
                kilobits: number(4, 8000),
                test_pattern: args.iter().any(|arg| arg == "bars"),
                bayer: args
                    .iter()
                    .find_map(|arg| arg.strip_prefix("bayer=").and_then(|value| value.parse().ok()))
                    .unwrap_or(firmware::video::OV5647_BAYER),
            };
            let session = board.session().unwrap_or_else(|| "uncounted".into());
            match firmware::video::start(&session, board.boot_id.unwrap_or(0) as u32, &settings) {
                Ok(path) => cc_protocol::build("ok", &["video", &path], &[]),
                Err(error) => cc_protocol::build("err", &["video", &error.to_string()], &[]),
            }
        }
        Some("stop") => match firmware::video::stop() {
            Ok(()) => video_stats("stopped"),
            Err(error) => cc_protocol::build("err", &["video", &error.to_string()], &[]),
        },
        Some("stats") => video_stats("running"),
        _ => cc_protocol::build(
            "err",
            &["usage", "video start [width height fps kbit/s] | stop | stats"],
            &[],
        ),
    }
}

/// The recorder's counters as one `ok video <state> key=value...` line.
fn video_stats(state: &str) -> String {
    let stats = firmware::video::stats();
    let average = stats.encode_microseconds_total / stats.frames_encoded.max(1) as u64;
    let values = [
        ("captured", stats.frames_captured.to_string()),
        ("missed", stats.frames_missed.to_string()),
        ("encoded", stats.frames_encoded.to_string()),
        ("idr", stats.frames_idr.to_string()),
        ("dropped", stats.frames_dropped.to_string()),
        ("encode_errors", stats.encode_errors.to_string()),
        ("timeouts", stats.capture_timeouts.to_string()),
        ("write_errors", stats.write_errors.to_string()),
        ("written", stats.bytes_written.to_string()),
        ("encode_us_avg", average.to_string()),
        ("encode_us_max", stats.encode_microseconds_max.to_string()),
        ("write_us_max", stats.write_microseconds_max.to_string()),
        ("ring_max", stats.ring_used_max.to_string()),
    ];
    let named: Vec<(&str, &str)> = values.iter().map(|(key, value)| (*key, value.as_str())).collect();
    cc_protocol::build("ok", &["video", state], &named)
}

/// The first argument as whole seconds (1..=600), else `default`.
fn seconds(args: &[String], default: u64) -> u64 {
    args.first()
        .and_then(|value| value.parse().ok())
        .unwrap_or(default)
        .clamp(1, 600)
}
