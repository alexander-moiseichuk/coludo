//! The CC link: join Wi-Fi through the on-board ESP32-C6 (esp_hosted over SDIO), dial the hub, serve its lines,
//! and re-dial forever (src/glider drivers/wifi.py + tasks/cc_link.py + cc_client.Client).
//!
//! The networks are the board config's (`wifi.networks`, board_config::wifi), tried round-robin with each one's
//! own retry clock; a password comes from the `creds` store, else the config's fallback. The hub is the config's
//! `cc_host`, the `.1` of the board's own subnet when it is unset, and nothing when it is '' (standalone: Wi-Fi
//! still joins, for the file server).

use std::io::{BufRead, BufReader, Write};
use std::net::TcpStream;
use std::time::{Duration, Instant};

use board_config::{Hub, Wifi};
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::modem::Modem;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::wifi::{BlockingWifi, ClientConfiguration, Configuration, EspWifi};

use crate::commands::Executor;
use crate::config::Credentials;

/// No line from the hub this long -> it is gone; drop and re-dial. The hub polls every ~2 s.
const SILENT: Duration = Duration::from_secs(60);
/// The link's tick: how often it looks for a network whose retry clock has run out (plan B hunts at 1 s).
const TICK: Duration = Duration::from_secs(1);
/// How long a cancelled association settles before the next one starts (plan B `_DISCONNECT_SETTLE_MS`).
const DISCONNECT_SETTLE: Duration = Duration::from_millis(300);

/// Join, dial, serve -- forever.
pub fn run(
    modem: Modem<'static>,
    event_loop: EspSystemEventLoop,
    partition: EspDefaultNvsPartition,
    wifi: Wifi,
    credentials: Credentials,
    executor: &Executor,
) -> ! {
    let mut driver = BlockingWifi::wrap(
        EspWifi::new(modem, event_loop.clone(), Some(partition)).expect("the C6 Wi-Fi driver"),
        event_loop,
    )
    .expect("the Wi-Fi event loop");
    // the microSD over HTTP (recordings, logs) -- only now: the Wi-Fi driver brought up the TCP/IP stack it needs
    let _files = crate::files::serve()
        .inspect_err(|error| log::warn!("files :: {error}"))
        .ok();
    if !wifi.policy_enabled || wifi.networks.is_empty() {
        log::warn!("link :: Wi-Fi disabled by the board config (policy, or no network)");
        loop {
            std::thread::sleep(Duration::from_secs(3600));
        }
    }
    let mut rotation = Rotation::new(wifi.networks.len());
    let mut parked = false;
    loop {
        std::thread::sleep(TICK);
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
        if !driver.is_connected().unwrap_or(false) {
            let Some(index) = rotation.next(&wifi, Instant::now()) else {
                continue; // every network is inside its own retry window
            };
            let network = &wifi.networks[index];
            let password = credentials
                .password(&network.ssid)
                .unwrap_or_else(|| network.password.clone());
            if let Err(error) = join(&mut driver, &network.ssid, &password) {
                log::warn!("link :: join {} failed: {error}", network.ssid);
                continue;
            }
            log::info!("link :: joined {}", network.ssid);
        }
        let address = match driver.wifi().sta_netif().get_ip_info() {
            Ok(info) => info.ip,
            Err(error) => {
                log::warn!("link :: no address: {error}");
                continue;
            }
        };
        let host = match &wifi.hub {
            Hub::Off => continue, // standalone: stay joined, never dial
            // plan B cc_link._network_host: the `.1` of the board's own subnet is the AP, the hub by convention
            Hub::Gateway => {
                let [first, second, third, _] = address.octets();
                format!("{first}.{second}.{third}.1")
            }
            Hub::Host(host) => host.clone(),
        };
        log::info!("link :: {address} -> hub {host}:{}", wifi.hub_port);
        match TcpStream::connect((host.as_str(), wifi.hub_port)).and_then(|stream| serve(stream, executor)) {
            Ok(()) => log::info!("link :: hub closed the link"),
            Err(error) => log::warn!("link :: {error}"),
        }
    }
}

/// Which network to try next (plan B wifi._next_network): round-robin from the last one tried, over the networks
/// whose own `retry_ms` has run out since their last attempt.
struct Rotation {
    /// When each network was last tried; None = never.
    attempted: Vec<Option<Instant>>,
    /// Where the round-robin resumes.
    start: usize,
}

impl Rotation {
    /// A rotation over `count` networks, none tried yet.
    fn new(count: usize) -> Self {
        Self {
            attempted: vec![None; count],
            start: 0,
        }
    }

    /// The index of the next network to try, stamping its clock; None while every one is inside its window.
    fn next(&mut self, wifi: &Wifi, now: Instant) -> Option<usize> {
        let count = self.attempted.len();
        let index = (0..count).map(|step| (self.start + step) % count).find(|&index| {
            self.attempted[index].is_none_or(|last| {
                now.duration_since(last) >= Duration::from_millis(u64::from(wifi.networks[index].retry_ms))
            })
        })?;
        self.attempted[index] = Some(now);
        self.start = (index + 1) % count;
        Some(index)
    }
}

/// Join one network: configure, start, associate, and wait for an address.
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
    if wifi.is_started()? {
        // an attempt at an absent network stays pending, and the next connect then answers ESP_ERR_WIFI_CONN
        // forever (plan B wifi._clear_pending, measured on this C6): disconnect clears it, harmless when idle.
        // The RPC to the C6 is asynchronous, so the cancel settles before the next attempt starts.
        let _ = wifi.wifi_mut().disconnect();
        std::thread::sleep(DISCONNECT_SETTLE);
    }
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
fn serve(stream: TcpStream, executor: &Executor) -> std::io::Result<()> {
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
        if let Some(mut reply) = executor.ask(line.trim()) {
            reply.push('\n'); // one write per line: two would wait out the hub's delayed ACK (~150 ms)
            writer.write_all(reply.as_bytes())?;
        }
    }
}
