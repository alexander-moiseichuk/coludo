//! The device console on UART0 (the board's CH343P USB-serial port): the CC commands, plus a few that only
//! an operator at the cable may run.
//!
//! `wifi <ssid> <password>` stores the network to join (values follow the CC encoding, so a password with
//! spaces travels as `base64:`); `reboot` restarts; `panic` panics on purpose, to prove a panic reboots.

use std::io::{BufRead, Write};
use std::sync::Arc;

use esp_idf_svc::hal::gpio::{AnyIOPin, InputPin, OutputPin};
use esp_idf_svc::hal::uart::{Uart, UartDriver};
use esp_idf_svc::io::vfs::BlockingStdIo;

use crate::board::Board;
use crate::commands;

/// The console thread's stack: the commands build JSON, nothing deeper.
const STACK_SIZE: usize = 8192;

/// Take over UART0 for blocking line reads and serve the console on its own thread, forever.
pub fn start(
    uart: impl Uart + 'static,
    transmit: impl OutputPin + 'static,
    receive: impl InputPin + 'static,
    board: Arc<Board>,
) -> anyhow::Result<()> {
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
