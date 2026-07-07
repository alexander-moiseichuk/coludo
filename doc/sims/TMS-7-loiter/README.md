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
for the precision. Full law + tuning history: specs/coludo.md "Gliding" and the guidance commit.

**Pending for this set:** the ON-BOARD confirmation matrix (glide-energy program step 3) replaces
these host captures with device telemetry; until then treat the numbers as sim-predicted.
