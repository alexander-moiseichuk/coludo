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

use std::fmt::Write as _;

use esp_idf_svc::hal::cpu::Core;
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
