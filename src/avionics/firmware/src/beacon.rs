//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The recovery beacons (M11b, M20; doc/specs/avionics-port.md), the page and the report shared with the remote
//! recorder through crates/beacon:
//!
//! - This board's own: at DONE the link thread, the radio's one owner, turns the C6 co-processor from a station into
//!   the open access point `<BOARD>_glider` on channel 3 at the maximum TX power, a captive portal serving one page
//!   for a searcher's phone ([`Beacon::start`]): the last good fix with a map link, the elevation from every source
//!   (the GNSS, each barometer, the laser: how high the glider hangs, should it land in a tree), the battery where an
//!   INA226 is fitted, and the flash->SD copy still to go. Power-off or a reset ends it; any power-up after DONE is a
//!   new session in SETTING.
//! - The remote recorder's (receiver/, an ESP32-C6 on the recorder's UART, M20): every [`REPORT_EVERY`] the
//!   beacon_feed thread writes the same page's figures and that beacon's radio to beacon.csv, an ordinary recorder
//!   stream ([`spawn_feed`]). The recorder replays the last row: `<BOARD>_beacon` three channels above this board's own
//!   (operator 2026-10-11), a minute on and a minute silent. It comes on BEFORE launch, with the camera: armed in
//!   SETTING and standing on the rod (crates/sequencer `video::Trigger`, the same look), or any stage past SETTING (a
//!   launch, a warm start in the air, `stage done`); then it stays on until power-off, so a glider that dies in flight
//!   owes it nothing. Until then its name is empty, which keeps it off: a new session in SETTING turns it off. Never
//!   in a HITL flight, whose frames ride this board's Wi-Fi.
//!
//! The figures are read from the databoard's channels at each request and each report, every source's latest value
//! whatever its age (a fix lost in the trees is still the best guess), with that age.

use std::time::{Duration, Instant};

use ::beacon::{Backlog, Battery, Height, Point, Radio, Report, Snapshot};
use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::task::thread::{MallocCap, ThreadSpawnConfiguration};
use esp_idf_svc::http::server::{Configuration as HttpConfiguration, EspHttpServer};
use esp_idf_svc::sys;
use esp_idf_svc::wifi::{BlockingWifi, EspWifi};
use recorder::Telemetry;
use sequencer::Stage;
use sequencer::video::{Duty, Trigger};

use crate::devices::Topology;
use crate::status::FLIGHT;

/// This board's beacon channel (operator, 2026-10-06).
const CHANNEL: u8 = 3;

/// The remote beacon's channel: three above this board's, so the two never share one (operator, 2026-10-11).
const REMOTE_CHANNEL: u8 = CHANNEL + 3;

/// This board's TX cap at the beacon, dBm: IDF's ceiling.
const POWER_DBM: u8 = 21;

/// The remote beacon's TX cap, dBm: the C6's ceiling.
const REMOTE_POWER_DBM: u8 = 20;

/// The remote beacon's cycle, seconds on then silent: its battery's share (operator, 2026-10-11).
const REMOTE_ON_S: u32 = 60;
const REMOTE_OFF_S: u32 = 60;

/// How often the beacon_feed thread writes a report (and at once when the remote beacon comes on).
pub const REPORT_EVERY: Duration = Duration::from_secs(5);

/// How often the beacon_feed thread looks for the rod: the trigger's 8 looks in a row are 2 s at this, as the video's.
const LOOK_EVERY: Duration = Duration::from_millis(250);

/// The beacon_feed thread's stack (PSRAM): a report's text.
const FEED_STACK: usize = 8 << 10;

/// The recorder's device key the beacon.csv stream is registered under (crate::recorder `device_stream`).
pub const STREAM: &str = "beacon";

/// The beacons of one board.
#[derive(Clone)]
pub struct Beacon {
    /// The board's id: the access points' names.
    board_id: String,
    /// The board's quantities.
    topology: &'static Topology,
    /// The battery's capacity, mAh (the INA226 entry's `battery_mah`), for the time left.
    battery_mah: Option<f32>,
}

impl Beacon {
    /// The beacons of board `board_id` reading `topology`, its battery `battery_mah` when the config gives one.
    pub fn new(board_id: &str, topology: &'static Topology, battery_mah: Option<f32>) -> Beacon {
        Beacon {
            board_id: board_id.to_string(),
            topology,
            battery_mah,
        }
    }

    /// This board's access point, `<BOARD>_glider`.
    fn radio(&self) -> Radio {
        Radio {
            ssid: format!("{}_glider", self.board_id),
            password: String::new(),
            channel: CHANNEL,
            power_dbm: POWER_DBM,
            on_s: 0,
            off_s: 0,
        }
    }

    /// This board's access point up on `wifi` (the station dropped) and the page served: the server lives as long as
    /// the board.
    ///
    /// Errors: the AP would not configure or start, or the DNS responder or the server would not start.
    pub fn start(&self, wifi: &mut BlockingWifi<EspWifi<'static>>) -> anyhow::Result<EspHttpServer<'static>> {
        let radio = self.radio();
        ::beacon::esp::start(wifi, &radio)?;
        // core 0, PSRAM stacks: slow threads that never call esp_mmu_map/unmap
        ::beacon::esp::spawn_dns(ThreadSpawnConfiguration {
            priority: 2,
            pin_to_core: Some(Core::Core0),
            stack_alloc_caps: MallocCap::Spiram | MallocCap::Cap8bit,
            ..Default::default()
        })?;
        let beacon = self.clone();
        let server = ::beacon::esp::serve(
            HttpConfiguration {
                // the file server's control port is the default one: this one takes the next
                ctrl_port: 32769,
                stack_size: 8192,
                core: Some(Core::Core0),
                task_caps: sys::MALLOC_CAP_SPIRAM | sys::MALLOC_CAP_8BIT,
                ..Default::default()
            },
            move || beacon.page(),
        )?;
        let address = wifi.wifi().ap_netif().get_ip_info().map(|info| info.ip.to_string());
        log::info!(
            "beacon :: {} open on channel {CHANNEL}, the page on http://{}/",
            radio.ssid,
            address.as_deref().unwrap_or("?")
        );
        Ok(server)
    }

    /// The page, read now (also on the file server as `/beacon`: the bench sees it before DONE).
    pub fn page(&self) -> String {
        ::beacon::page(&self.radio().ssid, &self.snapshot(), None)
    }

    /// The report for the remote beacon: its radio (`on`, or an empty name), and the page's figures read now.
    fn report(&self, on: bool) -> Report {
        Report {
            radio: Radio {
                ssid: if on {
                    format!("{}_beacon", self.board_id)
                } else {
                    String::new()
                },
                password: String::new(),
                channel: REMOTE_CHANNEL,
                power_dbm: REMOTE_POWER_DBM,
                on_s: REMOTE_ON_S,
                off_s: REMOTE_OFF_S,
            },
            snapshot: self.snapshot(),
        }
    }

    /// The page's figures, read now from the databoard, the mission and the recorder.
    fn snapshot(&self) -> Snapshot {
        // SAFETY: a plain read of the monotonic timer.
        let now_us = unsafe { sys::esp_timer_get_time() } as u64;
        let age = |pushed: Option<u64>| pushed.map_or(0, |at| now_us.saturating_sub(at) / 1_000_000);
        // the last good fix, whatever its age: the newest any source pushed
        let fix = self.topology.position("position").and_then(|parameter| {
            parameter
                .sources()
                .filter_map(|source| {
                    let channel = parameter.channel(source)?;
                    Some((channel.latest()?, channel.pushed_at()))
                })
                .max_by_key(|(_, pushed)| *pushed)
        });
        let heights = |quantity: &str| -> Vec<Height> {
            let Some(parameter) = self.topology.scalar(quantity) else {
                return Vec::new();
            };
            parameter
                .sources()
                .filter_map(|source| {
                    let channel = parameter.channel(source)?;
                    Some(Height {
                        source: source.to_string(),
                        metres: channel.latest(),
                        age_s: age(channel.pushed_at()),
                    })
                })
                .collect()
        };
        let latest = |quantity: &str| {
            self.topology.scalar(quantity).and_then(|parameter| {
                parameter
                    .sources()
                    .find_map(|source| parameter.channel(source)?.latest())
            })
        };
        Snapshot {
            fix: fix.map(|([latitude, longitude], pushed)| (Point { latitude, longitude }, age(pushed))),
            pad: crate::flight::launch_e7().map(|(latitude, longitude)| Point { latitude, longitude }),
            elevations: heights("elevation"),
            agl: heights("agl"),
            battery: latest("voltage").map(|millivolts| Battery {
                millivolts,
                milliamps: latest("current"),
                capacity_mah: self.battery_mah,
            }),
            backlog: crate::recorder::backlog().map(|(blocks, seconds)| Backlog {
                blocks: u64::from(blocks),
                seconds,
            }),
        }
    }
}

/// The beacon_feed thread (core 0, priority 2, a PSRAM stack: it never touches the flash): a look for the rod every
/// [`LOOK_EVERY`], and a report to beacon.csv every [`REPORT_EVERY`] and at once when the remote beacon comes on,
/// forever (see the module notes). Without a recorder, or with its stream unregistered, there is nothing to feed.
///
/// Errors: the thread would not spawn.
pub fn spawn_feed(beacon: Beacon) -> anyhow::Result<()> {
    let Some(mut telemetry) = crate::recorder::get()
        .zip(crate::recorder::device_stream(STREAM))
        .and_then(|(recorder, stream)| Telemetry::new(recorder, stream))
    else {
        log::warn!("beacon :: no recorder stream: the remote beacon gets no report");
        return Ok(());
    };
    let accel = beacon.topology.vector("accel");
    ThreadSpawnConfiguration {
        name: Some(c"beacon_feed"),
        stack_size: FEED_STACK,
        priority: 2,
        pin_to_core: Some(Core::Core0),
        stack_alloc_caps: MallocCap::Spiram | MallocCap::Cap8bit,
        ..Default::default()
    }
    .set()?;
    let spawned = std::thread::Builder::new().stack_size(FEED_STACK).spawn(move || {
        let mut trigger = Trigger::new(accel.is_some());
        let mut on = false;
        let mut reported: Option<Instant> = None;
        let mut overflowing = false;
        loop {
            // SAFETY: a plain read of the monotonic timer.
            let now_us = unsafe { sys::esp_timer_get_time() } as u64;
            let flight = FLIGHT.load();
            if !on && !crate::sim_link::running() {
                let reading = accel
                    .and_then(|parameter| parameter.read(now_us))
                    .map(|reading| reading.value);
                let on_the_rod = trigger.look(flight, reading, None) == Some(Duty::StartFlight);
                if on_the_rod || flight.launched || flight.stage != Stage::Setting {
                    on = true;
                    reported = None;
                    log::info!(
                        "beacon :: the remote beacon on until power-off ({})",
                        flight.stage.name()
                    );
                }
            }
            if reported.is_none_or(|at| at.elapsed() >= REPORT_EVERY) {
                reported = Some(Instant::now());
                let report = beacon.report(on);
                let heights = report.height_texts();
                // telemetry raises (the error policy): the recorder counts the refused row, the first of a run logged
                match telemetry.push(now_us, &report.cells(&heights)) {
                    Ok(()) => overflowing = false,
                    Err(overflow) if !overflowing => {
                        log::warn!("beacon :: beacon.csv row: {overflow}");
                        overflowing = true;
                    }
                    Err(_) => {}
                }
            }
            std::thread::sleep(LOOK_EVERY);
        }
    });
    ThreadSpawnConfiguration::default().set()?;
    spawned?;
    Ok(())
}
