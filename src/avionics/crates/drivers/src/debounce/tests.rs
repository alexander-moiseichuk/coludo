//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Unit tests of the generic debounce, at both levels: the evidence (armed and polled), the level rule, the restart
//! rule, the two holds, the return, the deadline (its origin, its floor, and the run it carries), the departure and
//! the settling, the edge contract and the clock. The separation switch's own physics are in separation/tests.rs.

use super::{Debounce, Edges, MINIMUM_SAMPLES, SETTLE_WINDOWS, Verdict, elapsed_us};
use board_config::LONGEST_DEBOUNCE_MS;

/// The read spacing these tests use, µs: the control tick.
const TICK_US: u32 = 10_000;

/// The debounce most tests use, µs: the separation switch's default.
const DEBOUNCE_US: u32 = 10_000;

/// One read: its time (µs), its level and what the interrupt saw since the previous read.
type Read = (u32, bool, Edges);

/// Feed `reads` to a fresh debounce of `debounce_us` whose owner's latch stands for `latched` throughout. Returns the
/// verdicts, each with its read's time.
fn verdicts(debounce_us: u32, latched: bool, reads: &[Read]) -> Vec<(u32, Verdict)> {
    let mut debounce = Debounce::new(debounce_us);
    reads
        .iter()
        .filter_map(|&(now_us, level, edges)| {
            debounce
                .step(now_us, level, edges, latched)
                .map(|verdict| (now_us, verdict))
        })
        .collect()
}

/// `level` read on every tick from `from_us` to `to_us`, each with `edges` ([`Edges::Quiet`] or [`Edges::Polled`]).
fn held(level: bool, edges: Edges, from_us: u32, to_us: u32) -> Vec<Read> {
    (from_us..=to_us)
        .step_by(TICK_US as usize)
        .map(|now_us| (now_us, level, edges))
        .collect()
}

/// One edge seen since the previous read, stamped `at_us`.
fn edge(at_us: u32) -> Edges {
    Edges::Seen {
        first_us: at_us,
        last_us: at_us,
    }
}

/// Chatter seen by a read at `now_us` on the tick: edges from 4 ms to 1 ms before it, so no level holds a debounce
/// between two reads' edges (1 ms to the next read, 6 ms to its first edge).
fn chatter(now_us: u32) -> Edges {
    Edges::Seen {
        first_us: now_us.wrapping_sub(4000),
        last_us: now_us.wrapping_sub(1000),
    }
}

/// A held verdict.
fn held_verdict(level: bool, departed_us: u32, settled_us: u32) -> Verdict {
    Verdict::Held {
        level,
        departed_us,
        settled_us,
    }
}

#[test]
fn the_armed_hold_is_timed_from_the_last_edge_to_the_microsecond() {
    for latched in [true, false] {
        let level = !latched;
        let seen = Edges::Seen {
            first_us: 3000,
            last_us: 7000,
        };
        // the line left at 3 ms and last changed at 7 ms: the hold needs a quiet read 10 ms after 7, not after the read
        let reads = [
            (10_000, level, seen),
            (16_999, level, Edges::Quiet),
            (17_000, level, Edges::Quiet),
        ];
        assert_eq!(
            verdicts(DEBOUNCE_US, latched, &reads),
            [(17_000, held_verdict(level, 3000, 7000))],
            "latched {latched}"
        );
        // NEGATIVE: a microsecond short is no verdict, however many quiet reads
        let short: Vec<Read> = (11_000..=16_999)
            .step_by(1000)
            .map(|now_us| (now_us, level, Edges::Quiet))
            .collect();
        assert!(verdicts(DEBOUNCE_US, latched, &[&reads[..1], &short].concat()).is_empty());
    }
}

#[test]
fn quiet_reads_after_a_restart_need_the_full_debounce_from_its_last_edge() {
    // an edge mid-settle restarts the run since its last stamp: the quiet reads before it count for nothing
    let reads = [
        (10_000, false, edge(3000)),
        (12_000, false, Edges::Quiet),
        (
            20_000,
            false,
            Edges::Seen {
                first_us: 18_000,
                last_us: 19_000,
            },
        ),
        (25_000, false, Edges::Quiet),
        (28_999, false, Edges::Quiet),
        (29_000, false, Edges::Quiet),
    ];
    assert_eq!(
        verdicts(DEBOUNCE_US, true, &reads),
        [(29_000, held_verdict(false, 3000, 19_000))]
    );
    // NEGATIVE: the read with the edge never concludes, even 10 ms past the settle's first edge
    assert!(verdicts(DEBOUNCE_US, true, &reads[..5]).is_empty());
}

#[test]
fn a_debounce_of_0_concludes_at_the_first_quiet_agreeing_read() {
    for latched in [true, false] {
        // an edge starts the settle; the next read, quiet and agreeing, concludes it
        let reads = [(10_000, !latched, edge(4000)), (20_000, !latched, Edges::Quiet)];
        assert_eq!(
            verdicts(0, latched, &reads),
            [(20_000, held_verdict(!latched, 4000, 4000))]
        );
        // the level rule alike, a microsecond later
        let reads = [(10_000, !latched, Edges::Quiet), (10_001, !latched, Edges::Quiet)];
        assert_eq!(
            verdicts(0, latched, &reads),
            [(10_001, held_verdict(!latched, 10_000, 10_000))]
        );
        // NEGATIVE: the read that starts a settle never concludes it, the level rule's included
        let mut debounce = Debounce::new(0);
        assert_eq!(debounce.step(10_000, !latched, Edges::Quiet, latched), None);
        assert!(debounce.settling());
    }
}

#[test]
fn the_polled_hold_needs_three_reads_spanning_the_debounce() {
    for latched in [true, false] {
        let level = !latched;
        let reads = [
            (0, level, Edges::Polled),
            (5000, level, Edges::Polled),
            (10_000, level, Edges::Polled),
            (20_000, level, Edges::Polled),
        ];
        // the third read comes at 10 ms, but the run spans 10 ms only: the fourth concludes a 20 ms debounce
        assert_eq!(
            verdicts(20_000, latched, &reads),
            [(20_000, held_verdict(level, 0, 0))],
            "latched {latched}"
        );
        // NEGATIVE: the span alone is not enough either -- two reads 30 ms apart are two reads
        let sparse = [
            (0, level, Edges::Polled),
            (30_000, level, Edges::Polled),
            (40_000, level, Edges::Polled),
        ];
        assert_eq!(
            verdicts(20_000, latched, &sparse),
            [(40_000, held_verdict(level, 0, 0))]
        );
        assert!(verdicts(20_000, latched, &sparse[..2]).is_empty());
        // at the default on the tick: the third read, 20 ms after the first, where an armed input decides at 10
        let polled = held(level, Edges::Polled, 0, 50_000);
        assert_eq!(
            verdicts(DEBOUNCE_US, latched, &polled).first(),
            Some(&(20_000, held_verdict(level, 0, 0)))
        );
        let quiet = held(level, Edges::Quiet, 0, 50_000);
        assert_eq!(
            verdicts(DEBOUNCE_US, latched, &quiet).first(),
            Some(&(10_000, held_verdict(level, 0, 0)))
        );
    }
}

#[test]
fn the_level_rule_starts_a_settle_with_no_edge_at_either_level() {
    for edges in [Edges::Quiet, Edges::Polled] {
        for latched in [true, false] {
            let mut debounce = Debounce::new(DEBOUNCE_US);
            assert_eq!(debounce.step(100_000, !latched, edges, latched), None);
            assert!(
                debounce.settling(),
                "a level other than the latched one starts a settle"
            );
            // armed, one quiet read a debounce later concludes it; polled, the third read
            let verdict_us = if edges == Edges::Quiet { 110_000 } else { 120_000 };
            for now_us in (110_000..verdict_us).step_by(TICK_US as usize) {
                assert_eq!(debounce.step(now_us, !latched, edges, latched), None);
            }
            // dated by the read that disagreed, which is also where the held level began: the change's own time is
            // unknown to it
            assert_eq!(
                debounce.step(verdict_us, !latched, edges, latched),
                Some(held_verdict(!latched, 100_000, 100_000)),
                "{edges:?}, latched {latched}"
            );
            assert!(!debounce.settling());
        }
    }
}

#[test]
fn an_idle_read_of_the_latched_level_starts_nothing() {
    // NEGATIVE: no edge and the latched level, whichever the latch and the input: no settle, however long
    for edges in [Edges::Quiet, Edges::Polled] {
        for latched in [true, false] {
            let mut debounce = Debounce::new(DEBOUNCE_US);
            for tick in 0..30 {
                assert_eq!(debounce.step(tick * TICK_US, latched, edges, latched), None);
                assert!(!debounce.settling(), "{edges:?}, latched {latched}, tick {tick}");
            }
        }
    }
}

#[test]
fn an_edge_at_the_latched_level_settles_back_to_it() {
    // a blip that came and went between two reads: the edges start a settle, the latched level holds, and the verdict
    // is the latched level (the owner records nothing), departed at the blip's first edge, settled at its last
    for latched in [true, false] {
        let blip = Edges::Seen {
            first_us: 4000,
            last_us: 6000,
        };
        let reads = [(10_000, latched, blip), (20_000, latched, Edges::Quiet)];
        assert_eq!(
            verdicts(DEBOUNCE_US, latched, &reads),
            [(20_000, held_verdict(latched, 4000, 6000))]
        );
    }
}

#[test]
fn a_read_of_the_other_level_restarts_the_run_since_that_read() {
    // a lost edge: a quiet read whose level is not the run's restarts it from the read itself, armed as polled
    for (edges, departure, departed_us, verdict_us) in [
        (Edges::Quiet, edge(2000), 2000, 30_000),
        (Edges::Polled, Edges::Polled, 10_000, 40_000),
    ] {
        let mut reads = vec![(10_000, false, departure), (20_000, true, edges)];
        reads.extend(held(true, edges, 30_000, 60_000));
        assert_eq!(
            verdicts(DEBOUNCE_US, true, &reads),
            [(verdict_us, held_verdict(true, departed_us, 20_000))],
            "{edges:?}"
        );
        // NEGATIVE: the disagreeing read itself concludes nothing, even with no debounce
        assert!(verdicts(0, true, &reads[..2]).is_empty(), "{edges:?}");
    }
}

#[test]
fn an_edge_between_two_agreeing_reads_restarts_the_run() {
    // NEGATIVE: reads of one level 20 ms apart, but the line moved between the second and the third
    let mut debounce = Debounce::new(20_000);
    assert_eq!(debounce.step(0, false, edge(0), true), None);
    assert_eq!(debounce.step(10_000, false, Edges::Quiet, true), None);
    let bounce = Edges::Seen {
        first_us: 15_000,
        last_us: 17_000,
    };
    assert_eq!(debounce.step(20_000, false, bounce, true), None, "the restart rule");
    assert!(debounce.settling());
    // the run restarted at the bounce's last edge: 20 ms after it concludes, the departure kept from the first edge
    assert_eq!(debounce.step(30_000, false, Edges::Quiet, true), None);
    assert_eq!(debounce.step(36_999, false, Edges::Quiet, true), None);
    assert_eq!(
        debounce.step(37_000, false, Edges::Quiet, true),
        Some(held_verdict(false, 0, 17_000))
    );
}

#[test]
fn the_departure_is_the_first_edge_and_survives_every_restart() {
    // the line leaves the latched level at 3 ms and wobbles, each read seeing an edge, sometimes back at the latched
    // level; it then holds the other level from 33 ms: departed at the very first edge, settled at the last
    for latched in [true, false] {
        let other = !latched;
        let mut reads = vec![
            (10_000, other, edge(3000)),
            (20_000, latched, edge(12_000)),
            (30_000, other, edge(21_000)),
            (40_000, other, edge(33_000)),
        ];
        reads.extend(held(other, Edges::Quiet, 50_000, 60_000));
        assert_eq!(
            verdicts(DEBOUNCE_US, latched, &reads),
            [(50_000, held_verdict(other, 3000, 33_000))],
            "latched {latched}"
        );
        // polled, the same wobble read by the reads alone: departed at the first read that disagreed, settled at the
        // first read of the final run -- both on the reads' grid
        let polled: Vec<Read> = reads
            .iter()
            .map(|&(now_us, level, _)| (now_us, level, Edges::Polled))
            .collect();
        assert_eq!(
            verdicts(DEBOUNCE_US, latched, &polled),
            [(50_000, held_verdict(other, 10_000, 30_000))],
            "polled, latched {latched}"
        );
    }
}

#[test]
fn a_concluded_settle_ends_the_episode() {
    // a departure that returned and held the latched level through a verdict: the next departure is a new episode,
    // dated by its own edge, not the first one's
    let mut debounce = Debounce::new(DEBOUNCE_US);
    let mut found = Vec::new();
    let reads = [
        (10_000, true, edge(4000)),
        (20_000, true, Edges::Quiet),
        (30_000, true, Edges::Quiet),
        (40_000, false, edge(37_000)),
        (50_000, false, Edges::Quiet),
    ];
    for (now_us, level, edges) in reads {
        found.extend(debounce.step(now_us, level, edges, true));
    }
    assert_eq!(
        found,
        [held_verdict(true, 4000, 4000), held_verdict(false, 37_000, 37_000)]
    );
    // NEGATIVE: a return shorter than the debounce is the same episode, dated by the first departure
    let short = [
        (10_000, true, edge(8000)),
        (20_000, false, edge(17_999)),
        (30_000, false, Edges::Quiet),
    ];
    assert_eq!(
        verdicts(DEBOUNCE_US, true, &short),
        [(30_000, held_verdict(false, 8000, 17_999))]
    );
}

#[test]
fn a_return_that_held_the_debounce_concludes_on_the_read_of_its_next_edge() {
    // the line went home at 8 ms and left again at 18 ms, with no read between to see it held: the edge's read
    // concludes the return, at the latched level (departed at the blip before it), and its edge departs anew
    for latched in [true, false] {
        let mut debounce = Debounce::new(DEBOUNCE_US);
        assert_eq!(debounce.step(10_000, latched, edge(8000), latched), None);
        assert_eq!(
            debounce.step(20_000, !latched, edge(18_000), latched),
            Some(held_verdict(latched, 8000, 8000)),
            "latched {latched}"
        );
        assert!(
            debounce.settling(),
            "the edge that ended the return starts the next settle"
        );
        assert_eq!(
            debounce.step(30_000, !latched, Edges::Quiet, latched),
            Some(held_verdict(!latched, 18_000, 18_000)),
            "latched {latched}"
        );
        // NEGATIVE: a run away from the latch that held as long concludes only on a quiet read -- the edge's read
        // restarts it, and the departure stays the first edge's
        let away = [
            (10_000, !latched, edge(8000)),
            (20_000, latched, edge(18_000)),
            (30_000, latched, Edges::Quiet),
        ];
        assert_eq!(
            verdicts(DEBOUNCE_US, latched, &away),
            [(30_000, held_verdict(latched, 8000, 18_000))],
            "latched {latched}"
        );
        // NEGATIVE: home a microsecond short of the debounce is no return
        let short = [
            (10_000, latched, edge(8000)),
            (20_000, !latched, edge(17_999)),
            (30_000, !latched, Edges::Quiet),
        ];
        assert_eq!(
            verdicts(DEBOUNCE_US, latched, &short),
            [(30_000, held_verdict(!latched, 8000, 17_999))],
            "latched {latched}"
        );
    }
}

#[test]
fn an_edge_stamped_before_the_level_rule_s_read_pulls_the_departure_back() {
    // the tick took its edges, then the line moved, then the tick read the level: that read sees the new level with no
    // edge (the level rule, dated at the read), and the edge's stamp, a µs earlier, reaches the next read. The
    // departure is the edge's: the settling is never before it, so the duration never wraps
    for latched in [true, false] {
        let reads = [
            (10_000, !latched, Edges::Quiet),
            (20_000, !latched, edge(9999)),
            (30_000, !latched, Edges::Quiet),
        ];
        assert_eq!(
            verdicts(DEBOUNCE_US, latched, &reads),
            [(30_000, held_verdict(!latched, 9999, 9999))],
            "latched {latched}"
        );
    }
    // NEGATIVE: a later edge never pushes the departure forward
    let reads = [
        (10_000, false, Edges::Quiet),
        (20_000, false, edge(15_000)),
        (30_000, false, Edges::Quiet),
    ];
    assert_eq!(
        verdicts(DEBOUNCE_US, true, &reads),
        [(30_000, held_verdict(false, 10_000, 15_000))]
    );
}

#[test]
fn chatter_runs_out_at_ten_windows_and_the_run_carries_on_away() {
    // chatter seen on every read: no quiet read, so no hold, and no level held a debounce between two reads' edges, so
    // no return; the settle gives up 10 x 10 ms after its first read. The deadline ends the episode, not the run
    assert_eq!(SETTLE_WINDOWS, 10);
    for home_last in [false, true] {
        let mut debounce = Debounce::new(DEBOUNCE_US);
        let mut found = Vec::new();
        for tick in 1..=11 {
            let now_us = tick * TICK_US;
            let level = (tick % 2 == 1) == home_last;
            found.extend(
                debounce
                    .step(now_us, level, chatter(now_us), true)
                    .map(|verdict| (now_us, verdict)),
            );
        }
        assert_eq!(
            found,
            [(110_000, Verdict::Unsettled)],
            "the deadline is 10 x 10 = 100 ms after the first read"
        );
        if home_last {
            // NEGATIVE: home, nothing is carried over: a quiet read of the latched level starts nothing
            assert!(!debounce.settling());
            assert_eq!(debounce.step(120_000, true, Edges::Quiet, true), None);
            assert!(!debounce.settling());
            // the other level does, and dates a new departure from its own read
            assert_eq!(debounce.step(130_000, false, Edges::Quiet, true), None);
            assert_eq!(
                debounce.step(140_000, false, Edges::Quiet, true),
                Some(held_verdict(false, 130_000, 130_000))
            );
        } else {
            // away, the run since the deadline read's last edge carries on, departed there: a quiet read a debounce
            // after that edge concludes it, as it would have with no deadline
            assert!(debounce.settling());
            assert_eq!(debounce.step(118_999, false, Edges::Quiet, true), None);
            assert_eq!(
                debounce.step(119_000, false, Edges::Quiet, true),
                Some(held_verdict(false, 109_000, 109_000))
            );
        }
    }
}

#[test]
fn the_deadline_counts_from_the_settle_s_first_read_not_its_first_edge() {
    // the first edge 4 ms before the settle's first read, chatter away from the latch on every read after it, and a
    // read off the tick at 101 ms: the deadline is 100 ms after the first read (105 ms), not after the edge (101 ms)
    let mut reads = vec![(5000, false, edge(1000))];
    reads.extend(
        (15_000..=95_000)
            .step_by(TICK_US as usize)
            .map(|now_us| (now_us, false, chatter(now_us))),
    );
    reads.extend([(101_000, false, chatter(101_000)), (105_000, false, chatter(105_000))]);
    assert_eq!(verdicts(DEBOUNCE_US, true, &reads), [(105_000, Verdict::Unsettled)]);
}

#[test]
fn the_deadline_waits_for_three_reads() {
    // the floor: a deadline of 0 or 10 ms has passed by the second read, but a settle concludes Unsettled on its
    // third at the earliest -- plan B's arithmetic ran its 0 and 1 ms debounces out before a third read could come
    assert_eq!(MINIMUM_SAMPLES, 3);
    for debounce_us in [0, 1000] {
        // armed chatter away from the latch, a blip home between every two reads (home held at no read: no return)
        let chatter = [
            (0, false, edge(0)),
            (
                10_000,
                false,
                Edges::Seen {
                    first_us: 5000,
                    last_us: 6000,
                },
            ),
            (
                20_000,
                false,
                Edges::Seen {
                    first_us: 15_000,
                    last_us: 16_000,
                },
            ),
        ];
        assert_eq!(
            verdicts(debounce_us, true, &chatter),
            [(20_000, Verdict::Unsettled)],
            "debounce {debounce_us} µs"
        );
        assert!(verdicts(debounce_us, true, &chatter[..2]).is_empty());
        // polled chatter the reads see
        let polled = [
            (0, false, Edges::Polled),
            (10_000, true, Edges::Polled),
            (20_000, false, Edges::Polled),
        ];
        assert_eq!(
            verdicts(debounce_us, true, &polled),
            [(20_000, Verdict::Unsettled)],
            "polled, debounce {debounce_us} µs"
        );
        // and a held level settles at any debounce: polled on its third read, armed on its first quiet one
        for (edges, verdict_us) in [(Edges::Polled, 20_000), (Edges::Quiet, 10_000)] {
            assert_eq!(
                verdicts(debounce_us, true, &held(false, edges, 0, 40_000)).first(),
                Some(&(verdict_us, held_verdict(false, 0, 0))),
                "{edges:?}, debounce {debounce_us} µs"
            );
        }
    }
}

#[test]
fn an_edge_passed_on_every_read_never_settles() {
    // NEGATIVE, the edge contract pinned: a polled caller that passed an edge on every read would restart every run;
    // held at the other level from the first read, each settle runs out to its deadline, Unsettled, and carries the
    // run into the next, which runs out in turn
    let reads: Vec<Read> = (0..=45)
        .map(|tick| (tick * TICK_US, false, edge(tick * TICK_US)))
        .collect();
    assert_eq!(
        verdicts(DEBOUNCE_US, true, &reads),
        [
            (100_000, Verdict::Unsettled),
            (200_000, Verdict::Unsettled),
            (300_000, Verdict::Unsettled),
            (400_000, Verdict::Unsettled)
        ]
    );
    // the same reads polled: the level rule settles them at once
    let polled: Vec<Read> = reads
        .iter()
        .map(|&(now_us, level, _)| (now_us, level, Edges::Polled))
        .collect();
    assert_eq!(
        verdicts(DEBOUNCE_US, true, &polled).first(),
        Some(&(20_000, held_verdict(false, 0, 0)))
    );
}

#[test]
fn the_clock_may_wrap() {
    // the u32 µs clock wraps every 71.6 minutes: a settle across the wrap, departed and settled before it, concluded
    // after it -- and a quiet read before the wrap, where the hold's end (since + debounce) has wrapped to a small
    // number, is not past it: the hold is an elapsed time
    let start = u32::MAX - 5000;
    let reads = [
        (
            start,
            false,
            Edges::Seen {
                first_us: start - 3000,
                last_us: start - 1000,
            },
        ),
        (start + 1000, false, Edges::Quiet),
        (start.wrapping_add(8000), false, Edges::Quiet),
        (start.wrapping_add(9000), false, Edges::Quiet),
    ];
    assert_eq!(
        verdicts(DEBOUNCE_US, true, &reads),
        [(
            start.wrapping_add(9000),
            held_verdict(false, start - 3000, start - 1000)
        )]
    );
    // polled alike
    let polled: Vec<Read> = (0..3)
        .map(|tick| (start.wrapping_add(tick * TICK_US), false, Edges::Polled))
        .collect();
    assert_eq!(
        verdicts(DEBOUNCE_US, true, &polled),
        [(start.wrapping_add(20_000), held_verdict(false, start, start))]
    );
    // chatter across the wrap still runs out 100 ms after its first read
    let chattering: Vec<Read> = (0..=10)
        .map(|tick| {
            let now_us = start.wrapping_add(tick * TICK_US);
            (now_us, tick % 2 == 0, chatter(now_us))
        })
        .collect();
    assert_eq!(
        verdicts(DEBOUNCE_US, true, &chattering),
        [(start.wrapping_add(100_000), Verdict::Unsettled)]
    );
}

#[test]
fn elapsed_us_reads_the_wrap_as_a_short_interval() {
    assert_eq!(elapsed_us(5, u32::MAX - 4), 10);
    assert_eq!(elapsed_us(1_000_000, 0), 1_000_000);
    // NEGATIVE: an earlier `now` is a negative interval, not a long one, across the wrap too
    assert_eq!(elapsed_us(0, 1), -1);
    assert_eq!(elapsed_us(u32::MAX - 4, 5), -10);
}

#[test]
fn a_debounce_at_the_validator_s_bound_and_one_past_it() {
    // board_config refuses a debounce_ms past LONGEST_DEBOUNCE_MS (a latch held back a second or more); the debounce
    // itself needs no bound: at the bound and one ms past it, a held level settles on its span and chatter on its
    // deadline, armed and polled
    let longest_us = u32::try_from(LONGEST_DEBOUNCE_MS * 1000).expect("the bound fits the µs clock");
    for (debounce_us, verdict_us) in [(longest_us, 1_000_000), (longest_us + 1000, 1_010_000)] {
        for edges in [Edges::Quiet, Edges::Polled] {
            assert_eq!(
                verdicts(debounce_us, true, &held(false, edges, 0, 2_000_000)),
                [(verdict_us, held_verdict(false, 0, 0))],
                "{edges:?}, debounce {debounce_us} µs"
            );
        }
        let chatter: Vec<Read> = (0..=1100)
            .map(|tick| (tick * TICK_US, tick % 2 == 0, edge(tick * TICK_US)))
            .collect();
        let deadline_us = debounce_us * SETTLE_WINDOWS;
        assert_eq!(
            verdicts(debounce_us, true, &chatter).first(),
            Some(&(deadline_us.next_multiple_of(TICK_US), Verdict::Unsettled)),
            "debounce {debounce_us} µs"
        );
    }
}

#[test]
fn a_window_past_the_clock_s_half_range_never_reads_as_run_out() {
    // NEGATIVE: a window past i32::MAX µs once read its deadline as passed on the second read, so every settle ended
    // Unsettled and the latch never moved. The deadline is an elapsed time: such a window never runs out, even past the
    // clock's half range (chatter read every minute for 40 minutes), and a held level still settles on its span (here
    // 214 748 365 µs, about 3.6 minutes)
    let debounce_us = 214_748_365;
    let mut chattering = Debounce::new(debounce_us);
    for minute in 0..=40 {
        let now_us = minute * 60_000_000;
        assert_eq!(
            chattering.step(now_us, minute % 2 == 0, chatter(now_us), true),
            None,
            "minute {minute}"
        );
    }
    assert!(chattering.settling());
    assert!(u64::from(debounce_us) * u64::from(SETTLE_WINDOWS) > i32::MAX as u64);
    let reads = [
        (0, false, edge(0)),
        (10_000, false, Edges::Quiet),
        (20_000, false, Edges::Quiet),
        (1_000_000, false, Edges::Quiet),
        (debounce_us - 1, false, Edges::Quiet),
        (debounce_us, false, Edges::Quiet),
    ];
    assert_eq!(
        verdicts(debounce_us, true, &reads),
        [(debounce_us, held_verdict(false, 0, 0))]
    );
    // and the longest of all never concludes, and never overflows a count
    let mut longest = Debounce::new(u32::MAX);
    for tick in 0..10_000 {
        assert_eq!(
            longest.step(tick * TICK_US, tick % 2 == 0, edge(tick * TICK_US), true),
            None
        );
    }
    assert!(longest.settling());
}
