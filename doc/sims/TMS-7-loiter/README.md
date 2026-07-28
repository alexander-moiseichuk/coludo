# TMS-7 loiter — the fly-long glide law, before/after (HOST sim)

🎬 **[`tms7_loiter_host.mp4`](tms7_loiter_host.mp4)** — three segments, HOST-sim provenance
(`tools/virtual_flight.py`, the real `guidance`/`governor`/`pid`/`mixer` over the quality-2
worst-case polar, calm, 5 % noise):

1. **F15, the OLD racetrack law** ([3D report](f15_racetrack_old.html), [plan](f15_racetrack_old.svg))
   — steer-at-point + bank cap: 184 m leg swings across the zone, touchdown decided by phase luck
   (that sweep's median miss: 129 m).
2. **F15, loiter orbit + endgame spiral** ([3D report](f15_loiter.html), [plan](f15_loiter.svg)) —
   the tangent-capture orbit (R 30 m) holds around the centre, then the spiral collapses it as the
   altitude runs out: **18 m from the centre, IN-ZONE**.
3. **E16, same law** ([3D report](e16_loiter.html), [plan](e16_loiter.svg)) — **17 m, IN-ZONE**:
   the small motor lands in the zone too.

Time aloft 121–148 % of the polar ceiling in every sweep case — objective ① (fly long) never paid
for the precision. Full law + tuning history: doc/specs/coludo.md "Gliding" and the guidance commit.

> **SUPERSEDED (7/27) — the numbers above are ~100 m optimistic.** The pending on-board confirmation
> is done: [`TMS-7-phase5_refactor`](../TMS-7-phase5_refactor/) flew this same law on the board and
> got **121 m (F15) / 90 m (E16)**, not 18 / 17 m. These captures predate the harness correction
> documented in [`TMS-7-physics_refresh`](../TMS-7-physics_refresh/) — the host servo applied every
> command instantly, plus the circular-noise and control-rate defects. `physics_refresh` re-flew the
> same F15 case on the corrected harness and got 119.2 m, which the board then independently
> confirmed at 121.2 m. **The loiter LAW is unchanged and is what flew**; what was optimistic is the
> harness it was measured on. Keep this set for the before/after of the law shape (racetrack vs
> orbit+spiral), not for its absolute misses.
