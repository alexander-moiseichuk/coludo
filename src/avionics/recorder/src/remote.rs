//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The remote beacon (M20, doc/specs/avionics-port.md M19/M20; crates/beacon): the glider writes a report to
//! beacon.csv every few seconds, an ordinary stream this recorder files, and the beacon replays its last row. The row
//! sets the radio (the access point's name, empty for off; its password, empty for an open one; its channel, its TX
//! cap, its on/off cycle) and the page's figures, the same page the glider serves at DONE. Every report also goes to
//! the card's root beacon.csv, and after a restart its last row is read back ([`last_report`]), so a recorder that
//! loses power and comes back beacons again.
//!
//! The access point is a captive portal at 192.168.4.1 (crates/beacon `esp`); the page notes how old the glider's
//! report is (the figures' ages grow with it), or that it came back from the card with its age unknown.

use std::sync::{Arc, Mutex, PoisonError};
use std::time::Instant;

use beacon::{Radio, Report, Snapshot};
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::modem::Modem;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use esp_idf_svc::http::server::{Configuration as HttpConfiguration, EspHttpServer};
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::wifi::{BlockingWifi, EspWifi};

/// The DNS responder's priority: below the link's loop (crate `LINK_PRIORITY`).
const PRIORITY: u8 = 2;

/// How long an access point that did not start waits before the next try, at least, seconds.
const RETRY_S: u32 = 10;

/// The glider's last report, and when it came: None when it was read back from the card after a restart.
type Latest = Arc<Mutex<Option<(Report, Option<Instant>)>>>;

/// The remote beacon: the last report, and the access point it asks for.
pub struct RemoteBeacon {
    /// What the Wi-Fi driver is made from, the first time a report turns the beacon on.
    parts: Option<(Modem<'static>, EspSystemEventLoop, Option<EspDefaultNvsPartition>)>,
    /// The Wi-Fi driver, once made.
    wifi: Option<BlockingWifi<EspWifi<'static>>>,
    /// The page server, once started (it lives as long as the recorder).
    server: Option<EspHttpServer<'static>>,
    /// The glider's last report: the page reads it.
    latest: Latest,
    /// The radio the access point runs, or ran before its silence; None while the beacon is off.
    radio: Option<Radio>,
    /// The access point is on the air.
    on: bool,
    /// When it went on the air, or silent.
    since: Instant,
}

impl RemoteBeacon {
    /// Off, until a report turns it on; the Wi-Fi driver is made from `modem`, `event_loop` and `nvs` then.
    pub fn new(modem: Modem<'static>, event_loop: EspSystemEventLoop, nvs: Option<EspDefaultNvsPartition>) -> Self {
        RemoteBeacon {
            parts: Some((modem, event_loop, nvs)),
            wifi: None,
            server: None,
            latest: Arc::new(Mutex::new(None)),
            radio: None,
            on: false,
            since: Instant::now(),
        }
    }

    /// The glider's `report`, received now (`received` Some) or read back from the card (None).
    pub fn report(&mut self, report: Report, received: Option<Instant>) {
        *self.latest.lock().unwrap_or_else(PoisonError::into_inner) = Some((report, received));
    }

    /// The access point as the last report's radio asks: off for an empty name; on, or silent, by its cycle, staying on
    /// while a phone is connected (spec M20: a searcher who found it keeps it); started again only when what goes on
    /// the air changed (name, password, channel, TX cap), never for the same one again: a new cycle applies as it is.
    /// Called from the link's loop, often.
    pub fn step(&mut self) {
        let wanted = self
            .latest
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .as_ref()
            .map(|(report, _)| report.radio.clone())
            .filter(|radio| !radio.ssid.is_empty());
        let Some(radio) = wanted else {
            if self.radio.take().is_some() && self.on {
                self.stop("the glider turned it off");
            }
            return;
        };
        let changed = self.radio.as_ref().is_none_or(|running| !same_air(running, &radio));
        let cycle_over = |seconds: u32| seconds > 0 && self.since.elapsed().as_secs() >= u64::from(seconds);
        if self.on && changed {
            self.stop("its name, password, channel or power changed");
            self.start(radio);
        } else if self.on && radio.off_s > 0 && cycle_over(radio.on_s) && !self.phone_connected() {
            self.radio = Some(radio);
            self.stop("silent for its cycle");
        } else if !self.on && (changed || cycle_over(radio.off_s.max(RETRY_S))) {
            self.start(radio);
        } else {
            // the same air: a new cycle (on/off seconds) only, kept without a restart
            self.radio = Some(radio);
        }
    }

    /// Whether a station is joined to the access point now (a phone on the page).
    fn phone_connected(&self) -> bool {
        let mut stations = esp_idf_svc::sys::wifi_sta_list_t::default();
        // SAFETY: a plain driver query on the started access point, into a list it fills.
        let listed = unsafe { esp_idf_svc::sys::esp_wifi_ap_get_sta_list(&mut stations) };
        listed == esp_idf_svc::sys::ESP_OK && stations.num > 0
    }

    /// The access point up as `radio` says, the page served (the server and the DNS responder start once).
    fn start(&mut self, radio: Radio) {
        self.since = Instant::now();
        self.radio = Some(radio.clone());
        match self.bring_up(&radio) {
            Ok(()) => {
                self.on = true;
                log::info!(
                    "beacon :: {} on channel {} at {} dBm, on {} s / silent {} s",
                    radio.ssid,
                    radio.channel,
                    radio.power_dbm,
                    radio.on_s,
                    radio.off_s
                );
            }
            // tried again after the silence (at least RETRY_S), as if it had run: one that never starts must not spin
            Err(error) => log::warn!("beacon :: {} did not start: {error}", radio.ssid),
        }
    }

    /// [`RemoteBeacon::start`]'s driver work.
    fn bring_up(&mut self, radio: &Radio) -> anyhow::Result<()> {
        if self.wifi.is_none() {
            let (modem, event_loop, nvs) = self
                .parts
                .take()
                .ok_or_else(|| anyhow::anyhow!("the Wi-Fi driver failed before"))?;
            let driver = EspWifi::new(modem, event_loop.clone(), nvs)?;
            self.wifi = Some(BlockingWifi::wrap(driver, event_loop)?);
        }
        let wifi = self.wifi.as_mut().expect("made above");
        beacon::esp::start(wifi, radio)?;
        if self.server.is_none() {
            beacon::esp::spawn_dns(ThreadSpawnConfiguration {
                priority: PRIORITY,
                ..Default::default()
            })?;
            let latest = self.latest.clone();
            // IDF's server task at its own priority (5, above the link's loop): a page is built in microseconds
            self.server = Some(beacon::esp::serve(HttpConfiguration::default(), move || page(&latest))?);
        }
        Ok(())
    }

    /// What the beacon does now, for the status line: `off`, or its name and `on` or `silent`, and where its report
    /// came from: `live` from the link, `card` read back after a restart.
    pub fn state(&self) -> String {
        let source = match self.latest.lock().unwrap_or_else(PoisonError::into_inner).as_ref() {
            Some((_, Some(_))) => "live",
            Some((_, None)) => "card",
            None => "none",
        };
        match &self.radio {
            None => format!("off ({source})"),
            Some(radio) => format!("{} {} ({source})", radio.ssid, if self.on { "on" } else { "silent" }),
        }
    }

    /// The access point off the air, for `why`.
    fn stop(&mut self, why: &str) {
        self.on = false;
        self.since = Instant::now();
        if let Some(wifi) = self.wifi.as_mut()
            && let Err(error) = wifi.stop()
        {
            log::warn!("beacon :: did not stop: {error}");
        }
        log::info!("beacon :: off the air: {why}");
    }
}

/// Whether `running` and `wanted` put the same access point on the air: name, password, channel and TX cap (the
/// cycle is no part of it).
fn same_air(running: &Radio, wanted: &Radio) -> bool {
    (&running.ssid, &running.password, running.channel, running.power_dbm)
        == (&wanted.ssid, &wanted.password, wanted.channel, wanted.power_dbm)
}

/// The page from the glider's `latest` report, its figures aged by the report's own age.
fn page(latest: &Latest) -> String {
    let latest = latest.lock().unwrap_or_else(PoisonError::into_inner);
    let Some((report, received)) = latest.as_ref() else {
        return beacon::page("beacon", &Snapshot::default(), Some("no report from the glider yet"));
    };
    let (snapshot, note) = match received {
        Some(at) => {
            let seconds = at.elapsed().as_secs();
            (
                report.snapshot.clone().aged(seconds),
                format!("from the recorder: the glider's last report, {seconds} s ago"),
            )
        }
        None => (
            report.snapshot.clone(),
            "from the recorder: the glider's last report, read back from its card after a restart (its age unknown)"
                .to_string(),
        ),
    };
    beacon::page(&report.radio.ssid, &snapshot, Some(&note))
}

/// The glider's last report on the card under `root`: the last line of the root's beacon.csv, where the recorder
/// keeps each report's wire line (crate `Filer::line`); None without one.
pub fn last_report(root: &str) -> Option<Report> {
    let lines = tail(&format!("{root}/{}", beacon::FILE))?;
    lines
        .rsplit(|&byte| byte == b'\n')
        .filter(|line| !line.is_empty())
        .find_map(|line| {
            let line = recorder_wire::classify(line);
            (line.status == recorder_wire::Status::Good).then_some(())?;
            Report::parse(line.payload)
        })
}

/// The last 2 KB of the file at `path`.
fn tail(path: &str) -> Option<Vec<u8>> {
    use std::io::{Read as _, Seek as _, SeekFrom};
    let mut file = std::fs::File::open(path).ok()?;
    let size = file.metadata().ok()?.len();
    file.seek(SeekFrom::Start(size.saturating_sub(2048))).ok()?;
    let mut bytes = Vec::new();
    file.read_to_end(&mut bytes).ok()?;
    Some(bytes)
}
