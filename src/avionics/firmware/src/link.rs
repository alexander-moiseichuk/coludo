//! The CC link: join Wi-Fi through the on-board ESP32-C6 (esp_hosted over SDIO), dial the hub, serve its
//! lines, and re-dial forever (src/glider cc_client.Client).

use std::io::{BufRead, BufReader, Write};
use std::net::TcpStream;
use std::time::Duration;

use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::modem::Modem;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::wifi::{BlockingWifi, ClientConfiguration, Configuration, EspWifi};

use crate::board::Board;
use crate::commands;

/// The hub's board port (src/glider cc_client: `cc_port`).
const HUB_PORT: u16 = 1234;
/// No line from the hub this long -> it is gone; drop and re-dial. The hub polls every ~2 s.
const SILENT: Duration = Duration::from_secs(60);
/// The pause between failed attempts.
const BACKOFF: Duration = Duration::from_secs(2);

/// Join, dial, serve -- forever. The hub is the `.1` of the board's subnet (the gateway), as on src/glider.
pub fn run(
    modem: Modem<'static>,
    event_loop: EspSystemEventLoop,
    partition: EspDefaultNvsPartition,
    board: &Board,
) -> ! {
    let mut wifi = BlockingWifi::wrap(
        EspWifi::new(modem, event_loop.clone(), Some(partition)).expect("the C6 Wi-Fi driver"),
        event_loop,
    )
    .expect("the Wi-Fi event loop");
    let mut told = false;
    let mut parked = false;
    loop {
        std::thread::sleep(BACKOFF);
        // SAFETY: reads the flag hosted_no_reboot sets when it parks the failed C6 transport.
        if unsafe { esp_idf_svc::sys::hosted_link_down() } {
            // every Wi-Fi call would now wait out a 5 s RPC timeout, for nothing: the link is gone until the next
            // boot (a reboot restores it; on the ground the operator runs `reboot` on the console)
            if !parked {
                log::error!("link :: the C6 link is parked -- Wi-Fi is off until the next boot");
                parked = true;
            }
            continue;
        }
        if !wifi.is_connected().unwrap_or(false) {
            let Some((ssid, password)) = board.credentials() else {
                if !told {
                    log::warn!("link :: no Wi-Fi credentials -- on the console: wifi <ssid> <password>");
                    told = true;
                }
                continue;
            };
            if let Err(error) = join(&mut wifi, &ssid, &password) {
                log::warn!("link :: join {ssid} failed: {error}");
                continue;
            }
        }
        let address = match wifi.wifi().sta_netif().get_ip_info() {
            Ok(info) => info,
            Err(error) => {
                log::warn!("link :: no address: {error}");
                continue;
            }
        };
        let hub = (address.subnet.gateway, HUB_PORT);
        log::info!("link :: {} -> hub {}:{}", address.ip, hub.0, hub.1);
        match TcpStream::connect(hub).and_then(|stream| serve(stream, board)) {
            Ok(()) => log::info!("link :: hub closed the link"),
            Err(error) => log::warn!("link :: {error}"),
        }
    }
}

/// Join the network: configure, start, associate, and wait for an address.
fn join(wifi: &mut BlockingWifi<EspWifi<'static>>, ssid: &str, password: &str) -> anyhow::Result<()> {
    let configuration = ClientConfiguration {
        ssid: ssid
            .try_into()
            .map_err(|_| anyhow::anyhow!("ssid longer than 32 bytes"))?,
        password: password
            .try_into()
            .map_err(|_| anyhow::anyhow!("password longer than 64 bytes"))?,
        ..Default::default()
    };
    wifi.set_configuration(&Configuration::Client(configuration))?;
    if !wifi.is_started()? {
        wifi.start()?;
        // modem sleep (the C6's default) parks every incoming packet until the next DTIM beacon: ~150 ms per
        // hub round trip instead of ~2 ms. The pad link wants the latter; in flight Wi-Fi is off anyway.
        // SAFETY: a plain driver call on the started Wi-Fi.
        esp_idf_svc::sys::esp!(unsafe {
            esp_idf_svc::sys::esp_wifi_set_ps(esp_idf_svc::sys::wifi_ps_type_t_WIFI_PS_NONE)
        })?;
    }
    // the C6 keeps the last network in its own NVS and may already be connecting to it; it then answers
    // ESP_ERR_WIFI_CONN, which here means "in progress", so wait for the address all the same
    match wifi.wifi_mut().connect() {
        Err(error) if error.code() == esp_idf_svc::sys::ESP_ERR_WIFI_CONN => {}
        other => other?,
    }
    wifi.wait_netif_up()?;
    Ok(())
}

/// Answer the hub's lines until it closes the link or goes silent.
///
/// Reads and writes share the one socket through `&TcpStream`: `try_clone()` dups the descriptor, and lwIP
/// sockets do not dup (ENOSYS).
fn serve(stream: TcpStream, board: &Board) -> std::io::Result<()> {
    stream.set_read_timeout(Some(SILENT))?;
    stream.set_nodelay(true)?; // a reply is one small segment; never hold it back for the ACK of the last
    let mut writer = &stream;
    let mut reader = BufReader::new(&stream);
    let mut line = String::new();
    loop {
        line.clear();
        if reader.read_line(&mut line)? == 0 {
            return Ok(());
        }
        if let Some(mut reply) = commands::handle(line.trim(), board) {
            reply.push('\n'); // one write per line: two would wait out the hub's delayed ACK (~150 ms)
            writer.write_all(reply.as_bytes())?;
        }
    }
}
