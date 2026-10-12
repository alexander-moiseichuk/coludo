//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The CC link: join Wi-Fi through the on-board ESP32-C6 (esp_hosted over SDIO), dial the hub, serve its lines,
//! and re-dial forever (src/glider drivers/wifi.py + tasks/cc_link.py + cc_client.Client) -- until the stage machine
//! detects the launch.
//!
//! **After the launch** (`launched` in the flight word, crate::status) the link never rejoins, hunts or dials: the
//! glider leaves range, and a session still up runs on until it drops by itself. The radio is never switched off (a
//! Wi-Fi stop and start through the C6 is an unproven path that can park the link until a reboot). An operator hold
//! clears `launched`, so a held bench board keeps its normal reconnects. Plan B keyed its hunting pause on the
//! airborne stage instead (drivers/wifi.py).
//!
//! The networks are the board config's (`wifi.networks`, board_config::wifi), tried round-robin with each one's
//! own retry clock; a password comes from the `creds` store, else the config's fallback. The hub is the config's
//! `cc_host`, the `.1` of the board's own subnet when it is unset, and nothing when it is '' (standalone: Wi-Fi
//! still joins, for the file server).

use std::io::{BufRead, BufReader, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::time::{Duration, Instant};

use board_config::{Hub, Network, Wifi};
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::modem::Modem;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::wifi::{AuthMethod, BlockingWifi, ClientConfiguration, Configuration, EspWifi, WifiEvent};

use crate::commands::Executor;
use crate::config::Credentials;
use crate::status::FLIGHT;
use firmware::gcl::Origin;

/// No line from the hub this long -> it is gone; drop the session (and re-dial, unless launched). The hub polls every
/// ~2 s.
const SILENT: Duration = Duration::from_secs(60);
/// How long a dial waits for the hub to accept. A host that never answers would otherwise hold the dial for over a
/// minute (lwIP's 12 SYN retries), and the launch check with it.
const DIAL_TIMEOUT: Duration = Duration::from_secs(3);
/// The link's tick: how often it looks for a network whose retry clock has run out (plan B hunts at 1 s).
const TICK: Duration = Duration::from_secs(1);
/// How long a cancelled association settles before the next one starts (plan B `_DISCONNECT_SETTLE_MS`).
const DISCONNECT_SETTLE: Duration = Duration::from_millis(300);
/// How long a join waits for its address (esp-idf-svc's own connect timeout), and how often it looks.
const JOIN_TIMEOUT: Duration = Duration::from_secs(15);
const JOIN_POLL: Duration = Duration::from_millis(50);

/// The station's last disconnect reason (IDF's `wifi_err_reason_t`, at least 1), 0 since a join cleared it: a join
/// that drops ends at once instead of waiting out [`JOIN_TIMEOUT`] (the boot's first association on this C6 drops
/// ~2.4 s in and the second joins in ~4 s: the board joined at 27 s, the operator asked why, 2026-10-11).
static DISCONNECTED: AtomicU32 = AtomicU32::new(0);

/// Whether the TCP/IP stack is up: the Wi-Fi driver brings it up in [`run`], and a socket opened before it asserts in
/// lwIP (the sim-link's UDP socket waits for it, crate::sim_link).
static STACK_UP: AtomicBool = AtomicBool::new(false);

/// Whether the TCP/IP stack is up, so a socket can be opened (it may not hold an address yet).
pub fn stack_up() -> bool {
    STACK_UP.load(Ordering::Acquire)
}

/// Join, dial, serve -- forever.
pub fn run(
    modem: Modem<'static>,
    event_loop: EspSystemEventLoop,
    partition: EspDefaultNvsPartition,
    wifi: Wifi,
    credentials: Credentials,
    executor: &Executor,
    beacon: crate::beacon::Beacon,
) -> ! {
    let events = event_loop.clone();
    let mut driver = match EspWifi::new(modem, event_loop.clone(), Some(partition))
        .and_then(|wifi| BlockingWifi::wrap(wifi, event_loop))
    {
        Ok(driver) => driver,
        Err(error) => without_driver(error),
    };
    // the C6 must not keep the networks: wipe what earlier boots stored in its own NVS (SSID and password in
    // clear) and hold the configuration in RAM from now on, as MicroPython does (network_wlan.c)
    // SAFETY: plain driver calls on the initialised (not started) Wi-Fi, remoted to the C6.
    unsafe {
        let _ = esp_idf_svc::sys::esp!(esp_idf_svc::sys::esp_wifi_restore())
            .inspect_err(|error| log::warn!("link :: wifi restore: {error}"));
        let _ = esp_idf_svc::sys::esp!(esp_idf_svc::sys::esp_wifi_set_storage(
            esp_idf_svc::sys::wifi_storage_t_WIFI_STORAGE_RAM
        ))
        .inspect_err(|error| log::warn!("link :: wifi storage: {error}"));
    }
    // the station's drops, for join(): the subscription lives as long as this loop, forever
    let _disconnects = events
        .subscribe::<WifiEvent, _>(|event| {
            if let WifiEvent::StaDisconnected(info) = event {
                DISCONNECTED.store(u32::from(info.reason()).max(1), Ordering::Release);
            }
        })
        .inspect_err(|error| log::warn!("link :: no disconnect events, a dropped join waits its timeout: {error}"))
        .ok();
    STACK_UP.store(true, Ordering::Release);
    // the microSD over HTTP (recordings, logs) -- only now: the Wi-Fi driver brought up the TCP/IP stack it needs
    let _files = crate::files::serve(beacon.clone())
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
    // the recovery beacon once DONE (M11b): the access point and its page server, kept until power-off or a reset
    let mut beaconing = None;
    let mut beacon_failed = false;
    let mut gave_up = false;
    let mut first = true;
    loop {
        // the first look at once, the boot's join not a tick later
        if !std::mem::take(&mut first) {
            std::thread::sleep(TICK);
        }
        // SAFETY: reads the flag hosted_no_reboot sets when it parks the failed C6 transport.
        if unsafe { esp_idf_svc::sys::hosted_link_down() } {
            // every Wi-Fi call would now wait out a 5 s RPC timeout, for nothing: the link is gone until the next
            // boot (a reboot restores it; on the ground the operator runs `reboot` on the console)
            if !parked {
                log::error!("link :: the C6 link is parked -- Wi-Fi is off until the next boot");
                crate::panics_logger::oops(
                    "c6-link-parked",
                    "esp_hosted asked for a host restart (an SDIO command to the C6 failed); hosted_no_reboot parked \
                     the transport instead. Wi-Fi is off until the next boot; the flight goes on.",
                );
                parked = true;
            }
            continue;
        }
        if beaconing.is_some() {
            continue; // the beacon holds the radio until power-off or a reset
        }
        if beacon_due() {
            match beacon.start(&mut driver) {
                Ok(server) => beaconing = Some(server),
                Err(error) if !beacon_failed => {
                    log::error!("beacon :: did not start, tried again each second: {error}");
                    beacon_failed = true;
                }
                Err(_) => {}
            }
            continue;
        }
        if !reconnect_allowed(&mut gave_up) {
            continue; // launched: no hunt, no join, no dial
        }
        // joined = associated AND holding an address (plan B's isconnected() follows the lease): an AP that
        // associates but never leases must not pin the board -- join() disconnects it and the rotation moves on
        if !driver.is_up().unwrap_or(false) {
            let Some(index) = rotation.next(&wifi, Instant::now()) else {
                continue; // every network is inside its own retry window
            };
            let network = &wifi.networks[index];
            let password = credentials
                .password(&network.ssid)
                .unwrap_or_else(|| network.password.clone());
            if let Err(error) = join(&mut driver, network, &password) {
                log::warn!("link :: join {} failed: {error}", network.ssid);
                continue;
            }
            log::info!("link :: joined {}", network.ssid);
        }
        let address = match driver.wifi().sta_netif().get_ip_info() {
            Ok(info) if !info.ip.is_unspecified() => info.ip,
            Ok(_) => continue, // no lease (plan B _network_host: no hub for 0.0.0.0)
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
        if !reconnect_allowed(&mut gave_up) {
            // the launch came while the join waited for its address (up to 15 s, never cancelled: plan B abandons
            // that wait and clears the attempt, drivers/wifi.py connect): no dial
            continue;
        }
        log::info!("link :: {address} -> hub {host}:{}", wifi.hub_port);
        match dial(&host, wifi.hub_port).and_then(|stream| serve(stream, executor)) {
            Ok(()) => log::info!("link :: hub closed the link"),
            Err(error) => log::warn!("link :: {error}"),
        }
    }
}

/// No Wi-Fi driver: the C6 never answered at boot (a reset left its SDIO link wedged, 7F 2026-10-12, or the C6 is
/// gone). The link is gone until the next boot, as when hosted_no_reboot parks a failed transport; the flight goes on.
/// Never a panic: that made every boot a reboot ~30 s in, a flight's included, until a chip reset.
fn without_driver(error: esp_idf_svc::sys::EspError) -> ! {
    log::error!("link :: no C6 Wi-Fi driver ({error}) -- Wi-Fi is off until the next boot");
    crate::panics_logger::oops(
        "c6-driver-failed",
        &format!("the C6 Wi-Fi driver did not start ({error}). Wi-Fi is off until the next boot; the flight goes on."),
    );
    loop {
        std::thread::sleep(Duration::from_secs(3600));
    }
}

/// Whether the link may rejoin, hunt or dial now, by a fresh snapshot of the flight word
/// ([`sequencer::Flight::reconnect_allowed`]); the first refusal after a launch logs that the link gives up.
///
/// Args: `gave_up` the refusal is already logged: set by a refusal, cleared once reconnects are allowed again (a
/// hold or `stage setting`), so the next launch logs again.
///
/// Returns: false from the stage machine's launch until a hold or a return to SETTING, true otherwise.
fn reconnect_allowed(gave_up: &mut bool) -> bool {
    // HITL rides the Wi-Fi (doc/specs/sim-link.md): the simulated sensors arrive over it, so a board in a HITL
    // flight joins again after a reboot in flight, or the warm-started flight would fly blind
    let allowed = FLIGHT.load().reconnect_allowed() || crate::sim_link::running();
    if !allowed && !*gave_up {
        log::info!("link :: launch detected -- no more joins or dials (a session still up runs until it drops)");
    }
    *gave_up = !allowed;
    allowed
}

/// Dial the hub, bounded by [`DIAL_TIMEOUT`]. `cc_host` may be a name: lwIP resolves it to one address.
///
/// Args: `host` the hub's address or name; `port` its port.
///
/// Returns: the connected stream; the resolver's or the connect's error, a timeout included.
fn dial(host: &str, port: u16) -> std::io::Result<TcpStream> {
    let address = (host, port)
        .to_socket_addrs()?
        .next()
        .ok_or_else(|| std::io::Error::new(std::io::ErrorKind::NotFound, format!("{host} resolves to no address")))?;
    TcpStream::connect_timeout(&address, DIAL_TIMEOUT)
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

/// Join one network: configure, start, associate (unless the launch landed meanwhile), and wait for an address.
fn join(wifi: &mut BlockingWifi<EspWifi<'static>>, network: &Network, password: &str) -> anyhow::Result<()> {
    let configuration = ClientConfiguration {
        ssid: network
            .ssid
            .as_str()
            .try_into()
            .map_err(|_| anyhow::anyhow!("ssid longer than 32 bytes"))?,
        password: password
            .try_into()
            .map_err(|_| anyhow::anyhow!("password longer than 64 bytes"))?,
        // no password = an open network; the default threshold (WPA2) would never join one, MicroPython's
        // zeroed config (threshold OPEN) does
        auth_method: if password.is_empty() {
            AuthMethod::None
        } else {
            AuthMethod::WPA2Personal
        },
        ..Default::default()
    };
    if !wifi.is_started()? {
        // the station mode with the network before the start: a mode change on a started radio raises the station's
        // start again, and its connect with it
        wifi.set_configuration(&Configuration::Client(configuration.clone()))?;
        wifi.start()?;
        // modem sleep (the C6's default) parks every incoming packet until the next DTIM beacon: ~150 ms per
        // hub round trip instead of ~2 ms. The pad link wants the latter.
        // SAFETY: a plain driver call on the started Wi-Fi.
        esp_idf_svc::sys::esp!(unsafe {
            esp_idf_svc::sys::esp_wifi_set_ps(esp_idf_svc::sys::wifi_ps_type_t_WIFI_PS_NONE)
        })?;
    }
    // an attempt at an absent network stays pending, and the next connect then answers ESP_ERR_WIFI_CONN forever
    // (plan B wifi._clear_pending, measured on this C6); the station's start raises a connect of its own, before any
    // configuration, which failed ESP_ERR_WIFI_SSID and left the boot's first join to drop and wait out its 15 s
    // (joined at 27 s). Disconnect clears either, harmless when idle; the RPC to the C6 is asynchronous, so the
    // cancel settles before the configuration
    let _ = wifi.wifi_mut().disconnect();
    std::thread::sleep(DISCONNECT_SETTLE);
    wifi.set_configuration(&Configuration::Client(configuration))?;
    if let Some(dbm) = network.tx_power_dbm {
        // the radio's cap in quarter-dBm (MicroPython's txpower): the operator's signal-level knob
        // SAFETY: a plain driver call on the started Wi-Fi.
        esp_idf_svc::sys::esp!(unsafe { esp_idf_svc::sys::esp_wifi_set_max_tx_power((dbm * 4.0) as i8) })?;
    }
    // run() checked the launch before the RPCs and the settle above: a launch that landed since gets no association
    // (the disconnect above already cleared any attempt still pending)
    if !FLIGHT.load().reconnect_allowed() && !crate::sim_link::running() {
        anyhow::bail!("launched before the association");
    }
    // the C6 keeps the last network in its own NVS and may already be connecting to it; it then answers
    // ESP_ERR_WIFI_CONN, which here means "in progress", so wait for the address all the same
    DISCONNECTED.store(0, Ordering::Release);
    match wifi.wifi_mut().connect() {
        Err(error) if error.code() == esp_idf_svc::sys::ESP_ERR_WIFI_CONN => {}
        other => other?,
    }
    // the address, or the drop: a join that drops ends now, and the link's next tick tries again
    let started = Instant::now();
    while !wifi.wifi().is_up()? {
        let reason = DISCONNECTED.load(Ordering::Acquire);
        anyhow::ensure!(reason == 0, "dropped (reason {reason})");
        anyhow::ensure!(
            started.elapsed() < JOIN_TIMEOUT,
            "no address within {} s",
            JOIN_TIMEOUT.as_secs()
        );
        std::thread::sleep(JOIN_POLL);
    }
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
        if let Some(mut reply) = executor.ask(line.trim(), Origin::Hub) {
            reply.push('\n'); // one write per line: two would wait out the hub's delayed ACK (~150 ms)
            writer.write_all(reply.as_bytes())?;
        }
        if beacon_due() {
            return Ok(()); // DONE: the radio turns into the beacon (the hub polls every ~2 s, so it ends that soon)
        }
    }
}

/// Whether the recovery beacon is due: DONE, and no HITL flight, whose frames ride this Wi-Fi and whose host restores
/// the board after DONE over the hub (crate::beacon).
fn beacon_due() -> bool {
    FLIGHT.load().stage == sequencer::Stage::Done && !crate::sim_link::running()
}

/// The P4 pin that drives the C6's EN (the Function-EV pinout; esp_hosted's slave reset).
const C6_ENABLE_PIN: i32 = 54;

/// The C6 link's console command (the Glider Command Language), registered at boot.
pub fn register_commands() {
    firmware::gcl::register(vec![firmware::gcl::Command {
        owner: "c6",
        words: "c6reset",
        about: "brown the Wi-Fi co-processor out for 100 ms: proves a failing C6 link parks, never reboots the P4",
        gate: firmware::gcl::Gate::Setting,
        reach: firmware::gcl::Reach::Everywhere,
        parameters: &[],
        bare_words: &[firmware::gcl::CONFIRM],
        run: Box::new(|_| {
            // the C6's EN is P4 GPIO54 (esp_hosted drives it as an output): 100 ms low = a C6 brown-out
            // SAFETY: level writes on the pin esp_hosted already configured as an output.
            unsafe {
                esp_idf_svc::sys::gpio_set_level(C6_ENABLE_PIN, 0);
                std::thread::sleep(Duration::from_millis(100));
                esp_idf_svc::sys::gpio_set_level(C6_ENABLE_PIN, 1);
            }
            cc_protocol::build("ok", &["c6reset"], &[])
        }),
    }]);
}
