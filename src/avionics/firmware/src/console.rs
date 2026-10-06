//! The device console on UART0 (the board's CH343P USB-serial port): the CC commands, plus a few that only
//! an operator at the cable may run.
//!
//! `wifi <ssid> <password>` stores the network to join (values follow the CC encoding, so a password with
//! spaces travels as `base64:`); `reboot` restarts; `panic` panics on purpose, to prove a panic reboots;
//! `c6reset` browns the Wi-Fi co-processor out on purpose, to prove a failing C6 link does not reboot the P4;
//! `sdtest [megabytes] [kilobytes per second]` writes to the microSD (flat out, or paced) and reports the rate and
//! the worst single write.

use std::io::{BufRead, Write};
use std::sync::Arc;

use esp_idf_svc::hal::gpio::{AnyIOPin, InputPin, OutputPin};
use esp_idf_svc::hal::uart::{Uart, UartDriver};
use esp_idf_svc::io::vfs::BlockingStdIo;

use crate::board::Board;
use crate::commands;

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
                if let Some(reply) = operator(line.trim(), &board).or_else(|| commands::handle(line.trim(), &board)) {
                    println!("{reply}");
                    let _ = std::io::stdout().flush();
                }
            }
        })?;
    Ok(())
}

/// The console-only commands; None hands the line on to the CC commands.
fn operator(line: &str, board: &Board) -> Option<String> {
    let message = cc_protocol::parse(line).ok()?;
    match message.command.as_deref()? {
        "panic" => panic!("deliberate panic from the console"),
        "reboot" => {
            println!("ok rebooting");
            let _ = std::io::stdout().flush();
            esp_idf_svc::hal::reset::restart();
        }
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
        "wifi" => Some(match message.args.as_slice() {
            [ssid, password] => match board.set_credentials(ssid, password) {
                Ok(()) => cc_protocol::build("ok", &[ssid], &[]),
                Err(error) => cc_protocol::build("err", &["store", &error.to_string()], &[]),
            },
            _ => cc_protocol::build("err", &["usage", "wifi <ssid> <password>"], &[]),
        }),
        _ => None,
    }
}
