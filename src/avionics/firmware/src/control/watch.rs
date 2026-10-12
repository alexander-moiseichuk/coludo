//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The lapse watches (M9): a tick at which an IMU channel is not fresh while its part is healthy.

use super::*;

/// The IMU channels the consumer side watches ([`Watch`]): each IMU's `accel` and `rate`. Six: a bench board carries
/// three accelerometers (the LSM6DSO32, the BMI323, the BNO055's accel) beside two rates, and four left the BMI323's
/// rate unwatched on the taster.
pub(super) const WATCHED: usize = 6;
/// Each watched channel's lapses, by [`WATCH_NAMES`]' order: ticks at which the channel was not fresh in its
/// quantity's shared window while its part was healthy -- what a consumer (the stage machine, the attitude filter) saw
/// as a gap (M9).
pub(super) static LAPSES: [AtomicU32; WATCHED] = [const { AtomicU32::new(0) }; WATCHED];
/// The watched channels' names, `<source>.<quantity>`, once the control thread starts.
pub(super) static WATCH_NAMES: OnceLock<Vec<String>> = OnceLock::new();
/// One IMU channel the consumer side watches for lapses (M9): a tick at which the channel is not fresh in its
/// quantity's shared window while its part is healthy, as the stage machine and the attitude filter read it. Counted
/// from the channel's first push, so a boot's bring-up is no lapse.
pub(super) struct Watch {
    /// The source's channel.
    channel: &'static databoard::Channel<[f32; 3]>,
    /// The quantity's shared freshness window, microseconds.
    window_us: u32,
    /// The source's table entry, whose health gates the count.
    entry: &'static DeviceEntry<'static>,
    /// Its counter in [`LAPSES`].
    lapses: &'static AtomicU32,
}

impl Watch {
    /// Every source's `accel` and `rate` channel, by quantity in rank order, at most [`WATCHED`]: the SPI IMU's on
    /// this thread and the BMI323's on its bus owner alike. Their names published for `inspect`.
    pub(super) fn every(topology: &'static Topology, table: &'static Table<'static>) -> Vec<Watch> {
        let mut watches = Vec::new();
        let mut names = Vec::new();
        for quantity in ["accel", "rate"] {
            let Some(parameter) = topology.vector(quantity) else {
                continue;
            };
            for source in parameter.sources() {
                let (Some(channel), Some(entry), Some(lapses)) =
                    (parameter.channel(source), table.find(source), LAPSES.get(watches.len()))
                else {
                    continue;
                };
                names.push(format!("{source}.{quantity}"));
                watches.push(Watch {
                    channel,
                    window_us: parameter.window_us(),
                    entry,
                    lapses,
                });
            }
        }
        let _ = WATCH_NAMES.set(names);
        watches
    }

    /// One look at `read_us`: a lapse when the part is healthy and the channel, pushed before, is not fresh.
    pub(super) fn look(&self, read_us: u64) {
        if self.entry.healthy()
            && self.channel.pushed_at().is_some()
            && self.channel.recent(self.window_us, read_us).is_none()
        {
            self.lapses.fetch_add(1, Ordering::Relaxed);
        }
    }
}

/// `source`'s watched channels' lapses, by quantity (`inspect` of an IMU); empty before the thread starts.
pub fn lapses_of(source: &str) -> Vec<(String, u32)> {
    WATCH_NAMES
        .get()
        .map(|names| {
            names
                .iter()
                .zip(&LAPSES)
                .filter_map(|(name, count)| {
                    let (owner, quantity) = name.split_once('.')?;
                    (owner == source).then(|| (quantity.to_string(), count.load(Ordering::Relaxed)))
                })
                .collect()
        })
        .unwrap_or_default()
}

/// Each watched channel's lapses as `source.quantity:count`, comma-separated; empty before the thread starts.
pub(super) fn lapses() -> String {
    WATCH_NAMES
        .get()
        .map(|names| {
            names
                .iter()
                .zip(&LAPSES)
                .map(|(name, count)| format!("{name}:{}", count.load(Ordering::Relaxed)))
                .collect::<Vec<_>>()
                .join(",")
        })
        .unwrap_or_default()
}
