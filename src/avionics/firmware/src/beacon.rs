//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The recovery beacon (M11b; doc/specs/avionics-port.md M11b): at DONE the link thread, the radio's one owner, turns
//! the C6 from a station into an open access point `<BOARD>_glider` on channel 3 at the maximum TX power, and serves
//! one page on port 80 for a searcher's phone: the last good fix with a map link, the elevation from every source (the
//! GNSS, each barometer, the laser: how high the glider hangs, should it land in a tree), the battery where an INA226
//! is fitted, and the flash->SD copy still to go. The page refreshes itself every 5 s. Power-off or a reset ends it;
//! any power-up after DONE is a new session in SETTING.
//!
//! The figures are read from the databoard's channels at each request, every source's latest value whatever its age
//! (a fix lost in the trees is still the best guess), with that age shown.
//!
//! The page opens by itself (a captive portal, operator 2026-10-11): the AP's address is 192.168.4.1, its DHCP server
//! offers the portal's URI (option 114, RFC 8910) and the board as the DNS server, a DNS responder answers every name
//! with the board, and every other path redirects to the page, so a phone's connectivity check (Android's
//! /generate_204, iOS's /hotspot-detect.html, Windows' /connecttest.txt) lands on it and the phone offers to open it.

use std::ffi::CStr;
use std::fmt::Write as _;
use std::net::UdpSocket;

use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::task::thread::{MallocCap, ThreadSpawnConfiguration};
use esp_idf_svc::handle::RawHandle;
use esp_idf_svc::http::Method;
use esp_idf_svc::http::server::{Configuration as HttpConfiguration, EspHttpServer};
use esp_idf_svc::io::Write as _;
use esp_idf_svc::sys;
use esp_idf_svc::wifi::{AccessPointConfiguration, AuthMethod, BlockingWifi, Configuration, EspWifi};

use crate::devices::Topology;

/// The beacon's channel (operator, 2026-10-06): the separate recorder's beacon takes 6 (M20), so the two never share
/// one.
const CHANNEL: u8 = 3;

/// The radio's TX cap at the beacon, quarter-dBm: IDF's ceiling, 21 dBm.
const MAXIMUM_TX_POWER: i8 = 84;

/// The page's port: a phone opens the access point's address as it is.
const PORT: u16 = 80;

/// Phones at once.
const MAXIMUM_CONNECTIONS: u16 = 4;

/// The access point's address, the one the ESP-IDF examples and most beacons use.
const ADDRESS: [u8; 4] = [192, 168, 4, 1];

/// The captive portal's URI, as DHCP option 114 carries it (IDF keeps the pointer: a static).
const PORTAL: &CStr = c"http://192.168.4.1/";

/// The DNS responder's stack (PSRAM: a slow core-0 thread that never calls esp_mmu_map/unmap).
const DNS_STACK: usize = 4 << 10;

/// A DNS answer's lifetime, seconds: short, the board is the only name server while it lasts.
const DNS_TTL_S: u32 = 60;

/// A 1S LiPo's open-circuit voltage (mV) against its charge (%), for the page's charge estimate: the usual table,
/// coarse, under load it reads low.
const LIPO_CHARGE: [(f32, f32); 9] = [
    (3300.0, 0.0),
    (3500.0, 10.0),
    (3600.0, 20.0),
    (3700.0, 35.0),
    (3800.0, 50.0),
    (3900.0, 65.0),
    (4000.0, 80.0),
    (4100.0, 90.0),
    (4200.0, 100.0),
];

/// Above this the INA226 reads no 1S LiPo (the bench's 5 V USB): no charge estimate.
const LIPO_CEILING_MV: f32 = 4350.0;

/// The beacon of one board.
#[derive(Clone)]
pub struct Beacon {
    /// The access point's name, `<BOARD>_glider`.
    ssid: String,
    /// The board's quantities.
    topology: &'static Topology,
    /// The battery's capacity, mAh (the INA226 entry's `battery_mah`), for the remaining time.
    battery_mah: Option<f32>,
}

impl Beacon {
    /// The beacon of board `board_id` reading `topology`, its battery `battery_mah` when the config gives one.
    pub fn new(board_id: &str, topology: &'static Topology, battery_mah: Option<f32>) -> Beacon {
        Beacon {
            ssid: format!("{board_id}_glider"),
            topology,
            battery_mah,
        }
    }

    /// The access point up on `wifi` (the station dropped) and the page served: the server lives as long as the board.
    ///
    /// Errors: the AP would not configure or start, or the server would not start.
    pub fn start(&self, wifi: &mut BlockingWifi<EspWifi<'static>>) -> anyhow::Result<EspHttpServer<'static>> {
        let _ = wifi.wifi_mut().disconnect();
        let access_point = AccessPointConfiguration {
            ssid: self
                .ssid
                .as_str()
                .try_into()
                .map_err(|_| anyhow::anyhow!("{} is longer than 32 bytes", self.ssid))?,
            channel: CHANNEL,
            auth_method: AuthMethod::None,
            max_connections: MAXIMUM_CONNECTIONS,
            ..Default::default()
        };
        wifi.set_configuration(&Configuration::AccessPoint(access_point))?;
        if !wifi.is_started()? {
            wifi.start()?;
        }
        // SAFETY: a plain driver call on the started Wi-Fi.
        sys::esp!(unsafe { sys::esp_wifi_set_max_tx_power(MAXIMUM_TX_POWER) })?;
        portal(wifi.wifi().ap_netif().handle())?;
        spawn_dns()?;
        let address = wifi.wifi().ap_netif().get_ip_info().map(|info| info.ip.to_string());
        let mut server = EspHttpServer::new(&HttpConfiguration {
            http_port: PORT,
            // the file server's control port is the default one: this one takes the next
            ctrl_port: 32769,
            stack_size: 8192,
            core: Some(Core::Core0),
            // a PSRAM stack, as the file server's (crate::files): it never calls esp_mmu_map/unmap
            task_caps: sys::MALLOC_CAP_SPIRAM | sys::MALLOC_CAP_8BIT,
            ..Default::default()
        })?;
        let beacon = self.clone();
        server.fn_handler::<anyhow::Error, _>("/", Method::Get, move |request| {
            let page = beacon.page();
            request
                .into_response(200, None, &[("Content-Type", "text/html; charset=utf-8")])?
                .write_all(page.as_bytes())?;
            Ok(())
        })?;
        // every other path to the page: the phones' connectivity checks find a portal and offer to open it
        server.fn_handler::<anyhow::Error, _>("/*", Method::Get, |request| {
            request
                .into_response(302, Some("Found"), &[("Location", "http://192.168.4.1/")])?
                .write_all(b"the beacon's page: http://192.168.4.1/\n")?;
            Ok(())
        })?;
        log::info!(
            "beacon :: {} open on channel {CHANNEL}, the page on http://{}/",
            self.ssid,
            address.as_deref().unwrap_or("?")
        );
        Ok(server)
    }

    /// The page, read now (also on the file server as `/beacon`: the bench sees it before DONE).
    pub fn page(&self) -> String {
        // SAFETY: a plain read of the monotonic timer.
        let now_us = unsafe { sys::esp_timer_get_time() } as u64;
        let age = |pushed: Option<u64>| pushed.map(|at| now_us.saturating_sub(at) / 1_000_000);
        let mut page = String::with_capacity(4096);
        let _ = write!(
            page,
            "<!doctype html><html><head><meta charset=\"utf-8\"><meta name=\"viewport\" \
             content=\"width=device-width,initial-scale=1\"><meta http-equiv=\"refresh\" content=\"5\">\
             <title>{name}</title><style>body{{font-family:sans-serif;margin:16px;font-size:18px}}\
             td{{padding:4px 12px 4px 0}}h2{{margin:20px 0 6px}}</style></head><body><h1>{name}</h1>",
            name = escape(&self.ssid)
        );
        // the last good fix, whatever its age: the newest any source pushed
        page.push_str("<h2>Where</h2>");
        let fix = self.topology.position("position").and_then(|parameter| {
            parameter
                .sources()
                .filter_map(|source| {
                    let channel = parameter.channel(source)?;
                    Some((channel.latest()?, channel.pushed_at()))
                })
                .max_by_key(|(_, pushed)| *pushed)
        });
        match fix {
            Some(([latitude, longitude], pushed)) => {
                let (latitude, longitude) = (f64::from(latitude) * 1e-7, f64::from(longitude) * 1e-7);
                let _ = write!(
                    page,
                    "<p>{latitude:.6}, {longitude:.6} ({} s ago)<br><a href=\"https://maps.google.com/?q={latitude:.6},\
                     {longitude:.6}\">open in Google Maps</a></p>",
                    age(pushed).unwrap_or_default()
                );
            }
            None => page.push_str("<p>no GNSS fix this boot</p>"),
        }
        // every source's height: each barometer's and the GNSS's elevation above the pad, the laser's AGL
        page.push_str("<h2>How high</h2><table>");
        for (quantity, label) in [("elevation", "above the pad"), ("agl", "above the ground (laser)")] {
            let Some(parameter) = self.topology.scalar(quantity) else {
                continue;
            };
            for source in parameter.sources() {
                let Some(channel) = parameter.channel(source) else {
                    continue;
                };
                match channel.latest() {
                    Some(metres) => {
                        let _ = write!(
                            page,
                            "<tr><td>{}</td><td>{metres:.1} m {label}</td><td>{} s ago</td></tr>",
                            escape(source),
                            age(channel.pushed_at()).unwrap_or_default()
                        );
                    }
                    None => {
                        let _ = write!(page, "<tr><td>{}</td><td>no reading</td></tr>", escape(source));
                    }
                }
            }
        }
        page.push_str("</table>");
        // the battery, where an INA226 measures it
        let latest = |quantity: &str| {
            self.topology.scalar(quantity).and_then(|parameter| {
                parameter
                    .sources()
                    .find_map(|source| parameter.channel(source)?.latest())
            })
        };
        if let Some(millivolts) = latest("voltage") {
            let milliamps = latest("current");
            let _ = write!(page, "<h2>Battery</h2><p>{:.2} V", millivolts / 1000.0);
            if let Some(milliamps) = milliamps {
                let _ = write!(page, ", {milliamps:.0} mA");
            }
            match charge(millivolts) {
                Some(percent) => {
                    let _ = write!(page, ", about {percent:.0} % (1S LiPo)");
                    if let (Some(capacity), Some(milliamps)) = (self.battery_mah, milliamps.filter(|draw| *draw > 1.0))
                    {
                        let hours = capacity * percent / 100.0 / milliamps;
                        let _ = write!(page, ", about {:.1} h left", hours);
                    }
                }
                None => page.push_str(" (no 1S LiPo: on USB?)"),
            }
            page.push_str("</p>");
        }
        // the recording still to copy from the flash to the card
        if let Some((blocks, seconds)) = crate::recorder::backlog() {
            let _ = write!(page, "<h2>Recording</h2><p>{blocks} blocks to copy to the card");
            if let Some(seconds) = seconds {
                let _ = write!(page, " ({seconds:.0} s of recording)");
            }
            page.push_str(": keep the glider powered until 0</p>");
        }
        page.push_str("</body></html>");
        page
    }
}

/// The AP `netif` re-addressed to [`ADDRESS`], its DHCP server offering the board as the DNS server and [`PORTAL`] as
/// the captive portal (the server stopped for the change, then started again).
fn portal(netif: *mut sys::esp_netif_t) -> anyhow::Result<()> {
    let address = sys::esp_ip4_addr {
        addr: u32::from_le_bytes(ADDRESS),
    };
    let information = sys::esp_netif_ip_info_t {
        ip: address,
        netmask: sys::esp_ip4_addr {
            addr: u32::from_le_bytes([255, 255, 255, 0]),
        },
        gw: address,
    };
    let mut dns = sys::esp_netif_dns_info_t::default();
    dns.ip.u_addr.ip4 = address;
    dns.ip.type_ = 0; // IPADDR_TYPE_V4
    let mut offer_dns: u8 = 1;
    // SAFETY: plain esp_netif calls on the AP's live netif; the option values outlive the calls, the portal's URI is
    // a static, as IDF keeps its pointer.
    unsafe {
        let _ = sys::esp_netif_dhcps_stop(netif);
        sys::esp!(sys::esp_netif_set_ip_info(netif, &information))?;
        sys::esp!(sys::esp_netif_set_dns_info(
            netif,
            sys::esp_netif_dns_type_t_ESP_NETIF_DNS_MAIN,
            &mut dns
        ))?;
        sys::esp!(sys::esp_netif_dhcps_option(
            netif,
            sys::esp_netif_dhcp_option_mode_t_ESP_NETIF_OP_SET,
            sys::esp_netif_dhcp_option_id_t_ESP_NETIF_DOMAIN_NAME_SERVER,
            (&mut offer_dns as *mut u8).cast(),
            1
        ))?;
        sys::esp!(sys::esp_netif_dhcps_option(
            netif,
            sys::esp_netif_dhcp_option_mode_t_ESP_NETIF_OP_SET,
            sys::esp_netif_dhcp_option_id_t_ESP_NETIF_CAPTIVEPORTAL_URI,
            PORTAL.as_ptr().cast_mut().cast(),
            PORTAL.to_bytes().len() as u32
        ))?;
        sys::esp!(sys::esp_netif_dhcps_start(netif))?;
    }
    Ok(())
}

/// The DNS responder (core 0, priority 2): every A query answered with [`ADDRESS`], every other one with no answer,
/// forever.
fn spawn_dns() -> anyhow::Result<()> {
    let socket = UdpSocket::bind(("0.0.0.0", 53))?;
    ThreadSpawnConfiguration {
        name: Some(c"beacon_dns"),
        stack_size: DNS_STACK,
        priority: 2,
        pin_to_core: Some(Core::Core0),
        stack_alloc_caps: MallocCap::Spiram | MallocCap::Cap8bit,
        ..Default::default()
    }
    .set()?;
    let spawned = std::thread::Builder::new().stack_size(DNS_STACK).spawn(move || {
        let mut query = [0u8; 512];
        loop {
            let Ok((length, sender)) = socket.recv_from(&mut query) else {
                continue;
            };
            if let Some(answer) = answer(&query[..length]) {
                let _ = socket.send_to(&answer, sender);
            }
        }
    });
    ThreadSpawnConfiguration::default().set()?;
    spawned?;
    Ok(())
}

/// The answer to one DNS `query` (RFC 1035): its id and question echoed, an A record of [`ADDRESS`] for an A query,
/// no record for any other type; None for a datagram that is no single-question query.
fn answer(query: &[u8]) -> Option<Vec<u8>> {
    if query.len() < 12 || query[2] & 0x80 != 0 || u16::from_be_bytes([query[4], query[5]]) != 1 {
        return None;
    }
    // the question's name: labels up to the zero length
    let mut end = 12;
    loop {
        let label = usize::from(*query.get(end)?);
        end += 1 + label;
        if label == 0 {
            break;
        }
        if label & 0xc0 != 0 {
            return None; // a pointer has no place in a question
        }
    }
    let question = query.get(12..end + 4)?;
    let address_query = u16::from_be_bytes([question[question.len() - 4], question[question.len() - 3]]) == 1;
    let mut answer = Vec::with_capacity(12 + question.len() + 16);
    answer.extend_from_slice(&query[..2]); // the id
    answer.extend_from_slice(&[0x81, 0x80]); // a response, recursion desired and available, no error
    answer.extend_from_slice(&[0, 1, 0, u8::from(address_query), 0, 0, 0, 0]);
    answer.extend_from_slice(question);
    if address_query {
        answer.extend_from_slice(&[0xc0, 0x0c, 0, 1, 0, 1]); // the question's name, type A, class IN
        answer.extend_from_slice(&DNS_TTL_S.to_be_bytes());
        answer.extend_from_slice(&[0, 4]);
        answer.extend_from_slice(&ADDRESS);
    }
    Some(answer)
}

/// A 1S LiPo's charge (%) at `millivolts`, interpolated in [`LIPO_CHARGE`]; None above [`LIPO_CEILING_MV`].
fn charge(millivolts: f32) -> Option<f32> {
    if millivolts > LIPO_CEILING_MV {
        return None;
    }
    let (lowest, _) = LIPO_CHARGE[0];
    if millivolts <= lowest {
        return Some(0.0);
    }
    for pair in LIPO_CHARGE.windows(2) {
        let [(low_mv, low), (high_mv, high)] = [pair[0], pair[1]];
        if millivolts <= high_mv {
            return Some(low + (high - low) * (millivolts - low_mv) / (high_mv - low_mv));
        }
    }
    Some(100.0)
}

/// `text` safe inside HTML.
fn escape(text: &str) -> String {
    text.replace('&', "&amp;").replace('<', "&lt;").replace('>', "&gt;")
}
