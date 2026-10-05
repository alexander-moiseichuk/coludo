# GNSS bench 2026-10-04: why no fix on 10-03

On 10-03 no airframe fixed: **0 of 11,334 GGA rows** on TMS-7C and TMS-7D, with the receivers powered and
talking for 14.7 and 23.6 min (see
[20261003](../../../launches/20261003/README.md#gnss-is-a-bigger-antenna-needed)). The flight init switches
GSV, GSA and the antenna text off, so those logs could not tell a bad configuration from a weak signal. This
bench separates the two.

## Setup

* **Checker:** an ESP32-C6 SuperMini running [`src/gnss_bench/main.py`](../../../src/gnss_bench/README.md),
  with an **ATGM336H** and a **NEO-6M** on it. Open sky in the operator's yard, nothing else powered.
* **Our flight init**, command for command: the ATGM at 10 Hz RMC + 1 Hz GGA (`drivers/atgm336h.py`, hz 10,
  every 10-03 config), the NEO at 5 Hz RMC + 1 Hz GGA (`drivers/neo6mv2.py`, hz 5). Then **slow diagnostics**
  that only choose what is printed: ATGM `PCAS03,10,0,99,99,1,0,0,99,0,0,,,0,0` (GSA, GSV and the antenna
  text every 99th fix), NEO `PUBX,40,GSA,0,50,…` and `PUBX,40,GSV,0,50,…`. That is one burst every ~10 s,
  and the flight rates stay intact (checked on the bench beforehand: ATGM 250 RMC + 25 GGA in 25 s, NEO 125 RMC
  + 25 GGA).
* **A session per power-on.** Both modules get the same antenna type. Each module gets the init once it talks,
  then:
  * a status line every 10 s: RMC valid, GGA quality and satellites used, satellites in view, the best four
    C/N0 and the antenna text;
  * at the first 3D fix, a `fix3d` record with the last 100 NMEA lines;
  * every 30 s, each module's moment file (`.x`) rewritten with its last 100 lines, both at one instant.

| session | antenna (the operator's words) | run |
|---|---|---|
| 1 | "plugged no external antenna" | 17 min |
| 2, 3, 4 | sub-second battery-connector bounces: `start` only, or empty (`neo6.3` is 0 bytes) | — |
| 5 | "small 15x5 mm antenna" | 28 min |
| 6 | "usual antenna during runs - 12x12 mm expected from small UAVs" (**what the airframes flew**) | 16 min |
| 7 | "larger (and heaviest) antenna from neo6Mv2 package" | 12 min |
| 8 | "largest (by size) BT-580 advertized as 32 db but usually it is 28 db (and in some exceptional conditions 32 db)" | 12 min |
| 9 | the USB read-out power-on (ignored) | — |

Sessions 5–8 ran back to back on the evening of 10-04, from 23:51 to 01:04 UTC.

## Results

* **3D fix** is the checker's rule: the first GGA with quality ≥ 1 and ≥ 4 satellites.
* **RMC valid** is the fix a flight can use. The flight code takes a position only from an RMC that says
  `A`. Two kinds of reading bound when that first happened: the status lines, every ~10 s, and the
  `situation` line of the `fix3d` record, the module's summary at its 3D fix. The bound is `(after, by]`:
  after the reading before the first `A`, by that first `A`. A `≤` means the first reading already had it.
* The two differ on the ATGM. In sessions 6–8 its RMC still said `V` at its 3D fix; in session 5, the cold
  start, it already said `A`. On the NEO, every `fix3d` situation says `rmc=A`, so its 3D fix was a usable
  one.
* **Top-4** is the best mean of the four strongest C/N0 values, in dB-Hz, over every satellite a module
  reports: GPS, QZSS (inside its `$GPGSV`) and BeiDou on the ATGM, GPS and SBAS on the NEO. It is each
  receiver's best signal, not a like-for-like comparison of the two.
  [The same-satellite table](#same-satellite-one-read-out) is that.
* **CEP50** is the scatter of the fixed positions about their median.

| session | antenna | ATGM 3D fix | ATGM RMC valid | ATGM top-4 | ATGM antenna text | ATGM CEP50 | NEO 3D fix | NEO RMC valid | NEO top-4 | NEO CEP50 |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | none | **never** (17 min, ≤ 6 in view) | never | **26.0** | OPEN | — | **never** (≤ 2 in view) | never | — | — |
| 5 | small 15x5 mm | 37.5 s (cold) | (31.9, 37.5] s | 43.5 | OK | 1.3 m | 37.5 s (cold) | (31.9, 37.5] s | 42.0 | 3.2 m |
| 6 | 12x12 mm patch | 22.5 s | **(32.1, 42.1] s** | **35.5** | **OPEN** | 1.5 m | 3.7 s | ≤ 3.7 s | 42.5 | 3.3 m |
| 7 | NEO kit | 16.2 s | (22.0, 32.0] s | 45.0 | OK | 0.9 m | 3.5 s | ≤ 3.5 s | 45.8 | 2.0 m |
| 8 | BT-580 | 14.2 s | (21.1, 31.7] s | 42.0 | OK | 1.2 m | 1.0 s | ≤ 1.0 s | 45.0 | 2.2 m |

The full table, with in-view, used and best-satellite counts, is [`summary.csv`](summary.csv).

### Same satellite, one read-out

Every 30 s the checker writes both moment files at one instant, each with its module's last 100 lines. Each
module prints its GSV in a burst every ~10 s, so the latest bursts in a pair of moment files can be up to
~10 s apart: in session 7, the ATGM's came 9.3 s after the NEO's. The sky barely moves in that time. So a
GPS satellite tracked by both compares the two receivers on one signal, GPS only. **This is the
like-for-like comparison.** The NEO's C/N0 minus the ATGM's, per common PRN at its latest report, then
averaged:

| session | antenna | common GPS PRNs | NEO − ATGM |
|---|---|---|---|
| 5 | small 15x5 mm | 10 | **+2.3 dB** |
| 6 | 12x12 mm patch | 8 | **+5.8 dB** |
| 7 | NEO kit | 7 | **+3.9 dB** |
| 8 | BT-580 | 9 | **+1.1 dB** |

## Conclusions

* **Our configuration is cleared: a usable fix in ≤ ~42 s.** On the flight init, both modules reached a valid
  RMC within ~42 s with every real antenna, the cold start included.
  * The NEO-6M's came with its 3D fix: 37.5 s cold, 1–4 s warm.
  * The ATGM336H's GGA showed a 3D position at 14–38 s. Cold, in session 5, its RMC was already valid at
    that fix (37.5 s). Warm, in sessions 6–8, the RMC still said `V` at the 3D fix (14–23 s) and turned
    valid later, between ~21 and ~42 s.
  * The ATGM accepts the out-of-spec GGA divider 10: it ran 10 Hz RMC with 1 Hz GGA, as configured.
* **10-03 was signal-starved.** The 12x12 mm passive patch the airframes flew gives the ATGM **35.5 dB-Hz in
  open sky**, the worst of every real antenna, and the module reports `ANTENNA OPEN`. On the airframe it
  likely started cold at every power-on, had the Luckfox, the camera and Wi-Fi beside it, and faced the
  horizon on the rod. It plausibly stayed below what decoding the ephemeris needs. With no antenna
  (26 dB-Hz) the ATGM never fixes, which is what the 10-03 logs look like.
* **A passive feed costs ~8–10 dB** on the ATGM: 35.5 against 43.5 (small) and 45.0 (NEO kit), and 6.5
  against the BT-580. `ANTENNA OPEN` on a connected antenna means the module sees no DC load: a passive
  antenna, with no amplifier.
* **The small 15x5 mm active antenna is the light winner.** At 43.5 dB-Hz and OK, it is as good as the big
  ones.
* **The NEO-6M reads +1 to +6 dB over the ATGM on the same satellite**, and +5.8 dB on the passive patch.
  After the cold start in session 5, it re-fixed in 1–4 s on every antenna, its RMC valid at once. The
  ATGM's usable fix took ~21–42 s on the same antennas, so the NEO's lead is larger than the 3D-fix times
  show.
* **The BT-580 buys nothing here**: 42.0 / 45.0 dB-Hz against 45.0 / 45.8 for the NEO kit and 43.5 / 42.0 for
  the small antenna.

## Caveats

* **One session per antenna.** The sky moves between sessions, and the runs differ in length (the ATGM's most
  satellites in view: 30, 20, 18, 17).
* **Only session 5 is a cold start.** The backup cells make the later sessions warm or hot. Compare their C/N0,
  not their fix times.
* **The ATGM's RMC bound is up to 10.6 s wide** (session 8's (21.1, 31.7]). In sessions 6–8 its `fix3d`
  situation still said `V` and came before the bound, so only the status lines, every ~10 s, set it. A
  situation that says `A` narrows a bound: session 5 on both modules, and the NEO's warm fixes.
* **No crossed swap.** Each module had its own antenna of the type, so unit-to-unit variance is inside the
  NEO − ATGM numbers. And two receivers' C/N0 estimates are not calibrated against each other.
* **Installation loss on the airframe is not measured.** Here each antenna was in the open, with only the C6
  beside it.

## Next steps

* **Sky diagnostics: logged on the pad, not in flight.** The operator's decision: the flight firmware logs
  the sky on the pad, and not from BOOSTING to DONE
  ([spec: Sky diagnostics](../../specs/coludo.md#sky-diagnostics--implemented-1004-pad-only)). The flight drivers
  send the bench's slow dividers after their init, and `gnss.py` logs one row per burst to `gnss_sky.csv`:
  satellites in view and used, the fix mode, the top four C/N0 and the antenna code. `inspect gnss` shows
  the same for the pad check.
* **An active antenna on the ATGM**, the small 15x5 mm one, **or the NEO-6M**.
* **Keep GNSS powered on the pad**, so the first fix comes warm or hot.
* **The installation test:** a module and its antenna on the assembled airframe, outdoors, with the Luckfox,
  the camera and Wi-Fi on. Compare its top-4 with this table, or with `inspect gnss` on the pad.
* **The crossed swap:** swap the two 12x12 mm patches between the modules.

## Files

* `raw/`: the checker's files, verbatim as read out (sizes checked against the board's listing). `336.N` is
  the ATGM and `neo6.N` the NEO, for session N; `.x` is the last moment file.
  * **They stay local, git-ignored** (`.gitignore`), and are better not used. The bench sat at the
    operator's home, and the fixed status lines, the `fix3d` blocks and the moment files hold that position
    at metre precision.
  * Only this README's numbers came from them. `src/control/test/test_gnss_bench.py` tests the report on
    synthetic sessions only; [Provenance](#provenance) pins each raw file without publishing it.
  * This README and `summary.csv` carry no position.
* [`summary.csv`](summary.csv): the report table, ';'-separated, one row per session and module, with the RMC
  bound as `rmc_valid_after_s` / `rmc_valid_by_s` (`after` empty: valid from the first reading).

Reproduce, from a tree that has `raw/`:

```
python3 tools/gnss_bench_report.py doc/benches/gnss-20261004/raw \
    --label 1=none 5=small-15x5 6=patch-12x12 7=neo-kit 8=bt-580 -o doc/benches/gnss-20261004/summary.csv
```

## Provenance

Each raw file, pinned without publishing it: an **HMAC-SHA256 keyed with a pepper**, 32 random bytes the tool
made on its first use in `raw/.pepper`. The pepper is owner-only, never printed, and stays local with `raw/`:
git-ignored with the folder and again by name, should `raw/` ever be committed. Lost, these digests can no
longer be checked, so it is kept with the raw files.

**Why not a plain SHA-256.** A plain hash of a position, or of a record whose one unknown is a position, can
be brute-forced. The record format is public ([`main.py`](../../../src/gnss_bench/main.py)), and a few square
kilometres at the logged 0.00001' (~2 cm) are ~10^10 candidates, seconds for one GPU. A published plain hash
would confirm a guessed home; a keyed one confirms nothing to anyone without the pepper.

Equal digests mean equal files: in sessions 2 and 4 both files hold the same `start` line.

```
336.1;7445b72f7fccf85c27e7ddc484c5c7126efb6e12a2c04ff74a9c0aefec6e5a8f
336.1.x;f0c9065c1c5829fac4adb768a65342c505ae3c540c4e9a8ffb35e4953ff23338
neo6.1;dfbb0d6b25e0055dc2e22bb38eddbd4c2fc6b6332cd6aa0cc5f3d271bac7e2c2
neo6.1.x;038c4d204ff2e4c307cf0ba4cb5d301a14cfe71c1be88deb0a74f0c61e7160a8
336.2;c9d178e5405ca97a421ae91c4ed26d511737ec7d5af43de5551fe25fd898413e
neo6.2;c9d178e5405ca97a421ae91c4ed26d511737ec7d5af43de5551fe25fd898413e
336.3;fdb151ffe13138a6bf4fd22c468417523d5d6b67ecabdf2dcb644f486b33a824
neo6.3;eff9f07c16e487763470209a5f40d8b02ffaa844e85230beac1e321e64807485
336.4;8877967cbf1bd05c5e9cfc4db9f6a7e829f1d34d91ca5a8901f048fc774b72c8
neo6.4;8877967cbf1bd05c5e9cfc4db9f6a7e829f1d34d91ca5a8901f048fc774b72c8
336.5;7e1909ec6b6f5edea89bbb3fb37e68a7bc9e9d8c4b8dcad4a020905d172ee6eb
336.5.x;3366a7f3630b9c133eebb2f455923b27bfb4a4145eeb563026d4043a1bb542d0
neo6.5;8c2a040daa692e81ad6c75f23c2fb07bc2962480a5c4455ea8ceeb1a1db07302
neo6.5.x;1e8b45d25577a633e19621707eca2ac2da1bf5f8f251d1a1b4883302b0fbd54c
336.6;deec855539f090ff63685b7c48722a88798513c5da945efa5c45c8de0aa0414b
336.6.x;bbde8ee98e5aed316a763df3eac3978578bd0a2fb6c5ce0976bbcd3badc37964
neo6.6;79fd681e4b672fcc7c1f5ef5e10a3036dcfdb251ab1f0b099cc4fa76bc35234f
neo6.6.x;fda5ea4a9e0c5969993eaba9131caa97a587c0041bb75622aa48dd762b1be280
336.7;0e764f187d67d969c204d400f197cd25db1d8263832d0f63b347e0d6074257a4
336.7.x;b7a93df28a738756ba5fd7fa290bcb09f14dd9d953ea00ae716bf2851e1b7d4c
neo6.7;00da895e13aa9c78f75d142c15436b023358ace9c3fbb88ee7de60f136e984b1
neo6.7.x;2e714757a86c5581272bf2579a97266de79b3ad01cf85f3ea1181d5382828e5e
336.8;12ca5ec86d05322d206c2e9a6b05707644c731992b6e15ccb88fbed819c40ddb
336.8.x;b55b2d8d5e40c85a9d25b94ca48f3193cb17b18a9df0b44dd8f063f7580e4dce
neo6.8;42b6ceca45b3fd00f7b32270a8e2a900b146da5110933e4cade3957d37f0d4f9
neo6.8.x;3f363efbcdc455d2968c3dbb33564a16016be4f591140c67d022aebe51d606da
336.9;eaecae77da048fd936a72e27c00594042ee43f878f77d6085efc63d047954640
neo6.9;96b8a6bf6b68e96ddf4f695b3101b743e11d43471a00da5bd558c381618e06e4
```

Check, from a tree that has `raw/` and its pepper (`<file>;<HMAC hex>`, in report order):

```
python3 tools/gnss_bench_report.py --provenance doc/benches/gnss-20261004/raw
```
