//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The seqlock under real threads: a writer pushing as fast as it can while readers read, the way the bus owners
//! and the control thread share a parameter on the board. A reading is either whole or absent, never torn: every
//! value pushed has all its words equal to its push time, so a reader that saw words from two pushes would see them
//! differ.

use std::sync::Barrier;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;

use databoard::Parameter;

/// How many readings the writer pushes.
const PUSHES: u32 = 200_000;

#[test]
fn readers_never_see_a_torn_reading() {
    let mut parameter: Parameter<[f32; 4]> = Parameter::new("quaternion");
    parameter.add_source("imu", 0, 0, false).unwrap();
    parameter.add_source("backup", 1, 0, false).unwrap();
    let parameter = &parameter;
    let done = AtomicBool::new(false);
    let done = &done;
    let start = Barrier::new(4); // the writer and three readers set off together, so their work overlaps
    let start = &start;
    thread::scope(|scope| {
        scope.spawn(move || {
            start.wait();
            let mut writer = parameter.writer("imu").unwrap();
            for push in 1..=PUSHES {
                // f32 holds every integer up to 2**24 exactly: the words and the stamp name the same push
                writer.push([push as f32; 4], u64::from(push));
            }
            done.store(true, Ordering::Release);
        });
        let readers: Vec<_> = (0..3)
            .map(|_| {
                scope.spawn(move || {
                    let mut whole = 0_u32;
                    let mut last = 0_u64;
                    start.wait();
                    // until the writer is done, and at least once after it (a slow start still checks a reading)
                    while !done.load(Ordering::Acquire) || whole == 0 {
                        let channel = parameter.channel("imu").unwrap();
                        if let (Some(value), Some(stamp)) = (channel.latest(), channel.pushed_at()) {
                            assert!(value.iter().all(|word| *word == value[0]), "torn: {value:?}");
                            assert!(stamp >= last, "time ran back: {stamp} after {last}");
                            last = stamp;
                            whole += 1;
                        }
                        if let Some(reading) = parameter.read(u64::from(PUSHES)) {
                            assert!(
                                reading.value.iter().all(|word| *word == reading.value[0]),
                                "torn: {reading:?}"
                            );
                            assert_eq!(
                                reading.age_us,
                                PUSHES - reading.value[0] as u32,
                                "value and stamp apart"
                            );
                        }
                    }
                    whole
                })
            })
            .collect();
        for reader in readers {
            assert!(reader.join().unwrap() > 0, "a reader saw no whole reading at all");
        }
    });
    assert_eq!(parameter.read(u64::from(PUSHES)).unwrap().value, [PUSHES as f32; 4]);
}

#[test]
fn the_primary_learns_while_readers_apply_the_offset() {
    // the writer side learns, readers only apply: an offset a reader sees is always one the primary stored whole
    let mut parameter: Parameter<f32> = Parameter::new("altitude");
    parameter.add_source("icp", 0, 50, true).unwrap();
    parameter.add_source("bmp", 1, 50, false).unwrap();
    let parameter = &parameter;
    thread::scope(|scope| {
        scope.spawn(move || {
            let mut backup = parameter.writer("bmp").unwrap();
            let mut primary = parameter.writer("icp").unwrap();
            // a push a millisecond: 50 s of board time, so the primary looks at the backup a hundred times (one look
            // per LEARN_PERIOD_US), each store racing the readers
            for step in 0..50_000_u64 {
                backup.push(102.5, step * 1_000);
                primary.push(100.0, step * 1_000);
            }
        });
        scope.spawn(move || {
            for _ in 0..50_000 {
                for (source, offset) in parameter.offsets() {
                    assert_eq!((source, offset), ("bmp", -2.5), "a constant bias learns exactly");
                }
            }
        });
    });
    assert_eq!(parameter.offsets().collect::<Vec<_>>(), [("bmp", -2.5)]);
}
