//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The ring gate (doc/specs/avionics-port.md M7, "Ring gate"; §3.7 "The ring_eraser and newest wins"): `arm` and
//! `verify` require the flash ring to take a flight -- its bring-up scan has run, its eraser runs, and the eraser holds
//! its reserve (about 1 % of the ring, flash_ring's `Geometry::reserve_blocks`) erased ahead of the writer. A refusal
//! plan B never had: its recorder probe asked only that its two RAM rings exist (tasks/recorder.py:30-48). The
//! verdict keeps that probe's shape, `recorder rings: <why>`, reported under the recorder's name, so CC shows it as it
//! showed plan B's.
//!
//! Not refusals: no card (`no-sd`, a health token), a saver behind, and newest wins erasing an unsaved block
//! (`ring-full`). Newest wins keeps one block more than the reserve erased, so without a card the gate does not
//! flicker as the writer opens blocks.

/// The start of every verdict: plan B's probe names the rings (tasks/recorder.py:45 `'recorder rings: %s'`).
const RINGS: &str = "recorder rings";

/// Why the flash ring cannot take a flight, as the recorder's probe verdict `recorder rings: <why>`; None when it
/// can.
///
/// `scanned` says the writer's bring-up scan has run, `eraser_running` that the ring_eraser thread started,
/// `erased_ahead` counts the erased blocks ahead of the writer (flash_ring's `Figures::erased_ahead`), and
/// `reserve_blocks` is the reserve the eraser keeps (`Geometry::reserve_blocks`). The first check that fails names
/// the why, in this order: a ring never scanned has no figures to trust, and one with no eraser never refills its
/// reserve, whatever it holds now.
pub fn ring_gate(scanned: bool, eraser_running: bool, erased_ahead: u32, reserve_blocks: u32) -> Option<String> {
    if !scanned {
        return Some(format!("{RINGS}: the bring-up scan has not run"));
    }
    if !eraser_running {
        return Some(format!("{RINGS}: the eraser is not running"));
    }
    (erased_ahead < reserve_blocks).then(|| {
        format!("{RINGS}: {erased_ahead} blocks erased ahead of the writer, below the {reserve_blocks}-block reserve")
    })
}

#[cfg(test)]
mod tests {
    use super::ring_gate;

    /// The board's reserve: 1 % of its 6144 blocks (flash_ring `Geometry::BOARD`).
    const BOARD_RESERVE: u32 = 61;

    #[test]
    fn a_scanned_ring_with_its_eraser_and_its_reserve_passes() {
        assert_eq!(
            ring_gate(true, true, BOARD_RESERVE, BOARD_RESERVE),
            None,
            "exactly the reserve"
        );
        assert_eq!(ring_gate(true, true, BOARD_RESERVE + 1, BOARD_RESERVE), None);
        assert_eq!(ring_gate(true, true, 6144, BOARD_RESERVE), None, "a fresh ring");
        assert_eq!(ring_gate(true, true, 1, 1), None, "a small ring's one-block reserve");
    }

    #[test]
    fn a_ring_never_scanned_is_refused_first() {
        assert_eq!(
            ring_gate(false, true, 6144, BOARD_RESERVE).as_deref(),
            Some("recorder rings: the bring-up scan has not run")
        );
        assert_eq!(
            ring_gate(false, false, 0, BOARD_RESERVE).as_deref(),
            Some("recorder rings: the bring-up scan has not run"),
            "before the eraser and the figures"
        );
    }

    #[test]
    fn a_ring_without_its_eraser_is_refused_even_with_its_reserve() {
        assert_eq!(
            ring_gate(true, false, 6144, BOARD_RESERVE).as_deref(),
            Some("recorder rings: the eraser is not running")
        );
        assert_eq!(
            ring_gate(true, false, 0, BOARD_RESERVE).as_deref(),
            Some("recorder rings: the eraser is not running"),
            "before the figures"
        );
    }

    #[test]
    fn a_ring_below_its_reserve_is_refused_with_both_counts() {
        assert_eq!(
            ring_gate(true, true, BOARD_RESERVE - 1, BOARD_RESERVE).as_deref(),
            Some("recorder rings: 60 blocks erased ahead of the writer, below the 61-block reserve")
        );
        assert_eq!(
            ring_gate(true, true, 0, 1).as_deref(),
            Some("recorder rings: 0 blocks erased ahead of the writer, below the 1-block reserve")
        );
    }
}
