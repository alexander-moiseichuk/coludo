# 3D model data after printing TMS-7

## Geometry of Coludo 

Measured on printed model without holder.

|Component|Measured Size in mm|
| ------------------- | --- | 
|Booster Body Length  |  290  | 
|Glider Body Lnegth   |  390  |
|Full length of Coludo|  680  |
|Wing Span Extended   |  598  |
|Wing Span Closed Near Holder  |  76  |
|Wing Span Closed Near Locker  |  108  |
|Fin Span             |  198  |

These numbers are coming from the printed TMS-7 models.
Coludo consists of an upper part that is a glider and the lower part that is a booster. 
Until separation wings are in a folded state. Near the holder of the glider's side is less
 then the wing span than the locker on the Booster side. Meaning the wings are closed 
 but they are still slightly spread out. 

## Booster

Booster is the lower part of Coludo which may have E16 or F15 solid rocket propellant engines. 

|Component|Count|Weight [g]|
| ------------------- | --- | ---- |
|Body Tube  |  1  |  77.7  |
|Holder and Cap|  1  |  35  |
|Parachute and cord|  1  |   5  |
|E16-4 Engine         |  1  | 82.5 |
|F15-4 Engine         |  1  | 98.9 |
| ------------------- | --- | ---- |
|Full Booster With E16-4 Engine|  1  | 200 |
|Full Booster With F15-4 Engine|  1  | 217 |

## Glider

The Glider is seated on top of the Booster and its wings are in a folded state until separation. 

|Component|Count|Weight [g]|
| ------------------- | --- | ---- |
|Back Upper Body     |  1  |  2.0 |
|Front Lower Body    |  3  |   8.0|
|Front Upper Body    |  1  | 16.1 |
|Back Lower Body     |  1  | 30.3 |
|Left Fin            |  1  | 8.0  |
|Right fin           |  1  | 8.0  |
|Top Fin             |  1  | 8.0  |
|Left Wing           |  1  | 35.0 |
|Right Wing          |  1  | 35.0 |
| ------------------- | --- | ---- |
|Glider Construction with all of the above | 1 | 150.4 |

## Coludo assembled

Weight with whole assembly having around 100-150 grams of electronics on board. 

|Component|Weight [g]|
| ------------------- | ---- |
|Coludo 100 g electronics E16-4 engine |  451 |
|Coludo 100 g electronics F15-4 engine |  468 |
|Coludo 150 g electronics E16-4 engine |  501 |
|Coludo 150 g electronics F15-4 engine |  518 |

## Derived geometry (from STL analysis)

Wing and fin planform areas measured from the STL meshes (see `glider/` and `booster/`). The
STL coordinates are in mm; planform = one-side projected area.

| Component | Span [mm] | Root chord [mm] | Tip chord [mm] | Planform [cm²] | Notes |
|---|---|---|---|---|---|
| Left Wing | 299 (semi) | 79 | 20 | 148 | trapezoidal, nearly unswept |
| Right Wing | 299 (semi) | 79 | 20 | 148 | mirror of left |
| **Total wing** | **598** | — | — | **296** | **AR 12.1** |
| Left Fin | 77 | 69 | ~15 | 26 | vertical surface |
| Right Fin | 77 | 69 | ~15 | 21 | |
| Top Fin | 77 | 69 | ~15 | 24 | |
| **Total fins (3)** | — | — | — | **71** | |

Wing thickness is 2.4 mm (from Y-span of STL bbox), giving a thickness/chord ratio of ~3 %
at root and ~12 % at tip. Aspect ratio (AR) = b²/S = 0.598² / 0.0296 = 12.1 — high,
favourable for induced-drag efficiency. The fins are roughly trapezoidal with ~69 mm
chord along the body; their combined area is ~24 % of the wing, adequate for directional
authority.

## Performance estimates

Basic-fidelity estimates using the measured masses above, motor data from Estes
specifications (E16-4: 28 N·s / 1.8 s burn / 33 N peak; F15-4: 50 N·s / 3.5 s burn /
25 N peak), and aerodynamic parameters derived from the STL geometry. Assumes: sea-level
ρ = 1.225 kg/m³; Cd = 0.6 for the boost configuration (frontal area ~17 cm² = glider body
~46 mm effective diameter); no wind; vertical launch.

### Boost phase

| Parameter | E16-4 | F15-4 |
|---|---|---|
| Liftoff mass (100g electronics) | 451 g | 468 g |
| Liftoff mass (150g electronics) | 501 g | 518 g |
| Peak thrust / weight ratio | 33 N / 4.42 N = 7.5 | 25 N / 4.59 N = 5.4 |
| Peak specific force (accelerometer reads) | **~7.5 g** | **~5.4 g** |
| Ideal Δv (impulse / avg mass) | 65.9 m/s | 115.2 m/s |
| Gravity loss (g × t_burn) | −17.7 m/s | −34.3 m/s |
| Drag loss (approx) | −4 m/s | −20 m/s |
| **Burnout speed (vertical)** | **~44 m/s (158 km/h)** | **~61 m/s (220 km/h)** |
| Coast to apogee | ~98 m | ~188 m |
| **Apogee** | **~140 m** | **~290 m** |
| Time to apogee | ~7 s | ~11 s |

For the F15, the longer 3.5 s burn means drag accumulates more, narrowing the gap to the
E16 vs. the ideal-impulse-only prediction. The E16's higher peak thrust (33 N vs 25 N)
gives it better early acceleration despite lower total impulse.

### Glide phase (post-separation)

| Parameter | 100g electronics | 150g electronics |
|---|---|---|
| Glider mass (structure + elec) | 250.4 g | 300.4 g |
| Wing loading (W/S) | 8.3 kg/m² | 9.9 kg/m² |
| Stall speed (CL_max 0.7) | ~14 m/s (~50 km/h) | ~15 m/s (~54 km/h) |
| Best glide speed (CL 0.4) | ~17 m/s (~61 km/h) | ~19 m/s (~68 km/h) |
| Estimated L/D (flat plate, low Re) | **5–10** | 5–10 |
| Sink rate at best glide | 1.7–3.4 m/s | 1.9–3.8 m/s |

### Mission summary (100g electronics, mid L/D ≈ 6)

| | E16-4 | F15-4 |
|---|---|---|
| Apogee | ~140 m | ~290 m |
| Glide range (L/D 6) | ~840 m | ~1740 m |
| Glide time | ~50 s | ~100 s |
| Total flight time | ~55 s | ~110 s |

### Key differences from earlier estimates

The original coludo.md assumed a much smaller wing (124 cm² total vs 296 cm² now) and
correspondingly higher wing loading (19 vs 8.3 kg/m²). The larger wings are the single
biggest improvement:

- **Stall speed dropped from ~19 m/s to ~14 m/s** — the glider is now controllable
  instead of always flying at the edge of stall.
- **Wing loading halved** — a gentler glide, lower sink rate, and more margin for turns.
- **Boost apogee is lower** than originally estimated (~140 vs 180 m for E16, ~290 vs
  360 m for F15) because the actual printed glider structure (150.4 g) is heavier than
  the earlier ~113 g airframe estimate. The stack is ~15 % heavier, reducing burnout
  speed and apogee.

The net effect is a vehicle that climbs less high but glides much better once separated
— trading peak altitude for flyability, which is the right trade for proving active
control.

## Speed envelope

### Speeds at a glance

All values for the 250.4 g glider (100 g electronics) unless noted.

| Speed | Value | Notes |
|---|---|---|
| Stall speed (Vs) | **~14 m/s** (~50 km/h) | CL_max 0.7, at 8.3 kg/m² wing loading |
| Best L/D speed | **~17 m/s** (~61 km/h) | CL 0.4, mid-range of L/D polar |
| Manoeuvring speed (Va) | **~31 m/s** (~112 km/h) | Full deflection × limit load (5 g); above this, full throw overstresses the wing |
| Maximum level speed | **~30–37 m/s** (108–133 km/h) | At minimum usable CL ≈ 0.1–0.2 in a glide |
| Maximum dive speed (E16 launch) | **~38 m/s** (137 km/h) | Achievable from 140 m apogee in a steep dive (70 % of Vt) |
| Maximum dive speed (F15 launch) | **~45 m/s** (162 km/h) | Achievable from 290 m apogee (approaches Vt) |
| Terminal velocity (Vt, vertical) | **~49 m/s** (177 km/h) | Limit of drag = weight; glider cannot exceed this regardless of starting height |
| Never-exceed speed (Vne) | **~47 m/s** (~170 km/h) | 1.5× Va; above this, a gust or full pull-up may exceed 5 g |
| Fin full-authority ceiling | ≤16 m/s | ±45° available |
| Fin reduced-authority floor | ≥50 m/s | ±5° only; see schedule in coludo.md |

### How each limit is derived

**Stall speed** — from the lift equation:

```
Vs = sqrt(2 × W / (ρ × S × CL_max))
   = sqrt(2 × 0.250 × 9.81 / (1.225 × 0.0296 × 0.7))
   = 13.8 m/s  → ~14 m/s
```

CL_max = 0.7 is conservative for a thin flat-plate wing at Re ≈ 50 000–100 000.
A smoother carbon surface may raise it to 0.8–0.9, lowering Vs to ~12–13 m/s.

**Terminal velocity** — drag = weight in a vertical dive. Total effective drag area
CdA ≈ 0.00165 m² is the sum of fuselage (Cd 0.4 × 0.0023 m²), wing profile (CD0 0.02 ×
0.0296 m²), and fin profile (CD0 0.02 × 0.0071 m²), all referenced to 1.225 kg/m³:

```
Vt = sqrt(2 × W / (ρ × CdA))
   = sqrt(2 × 0.250 × 9.81 / (1.225 × 0.00165))
   = 49.3 m/s  → ~49 m/s
```

For the heavier 300 g glider: Vt ≈ 54 m/s (194 km/h).

**Manoeuvring speed Va** — the speed at which the wing reaches CL_max = 0.7 at the
limit load factor n_lim = 5 g (typical for small RC gliders; the actual structural
limit should be ground-verified by static wing-load test before flight):

```
Va = sqrt(2 × n_lim × W / (ρ × S × CL_max))
   = sqrt(2 × 5 × 0.250 × 9.81 / (1.225 × 0.0296 × 0.7))
   = 31.1 m/s  → ~31 m/s
```

**Maximum dive speed** — limited by available potential energy (apogee) and terminal
velocity. In a vertical dive with quadratic drag, the fraction of Vt reached after
falling a height h is:

```
v / Vt = sqrt(1 − exp(−2 × g × h / Vt²))
```

| Apogee | h (fall distance) | v / Vt | Max dive speed |
|---|---|---|---|
| E16: 140 m | 100 m | 0.70 | 0.70 × 49 = **34 m/s** |
| E16: 140 m | 130 m | 0.76 | 0.76 × 49 = **37 m/s** |
| F15: 290 m | 250 m | 0.86 | 0.86 × 49 = **42 m/s** |
| F15: 290 m | 280 m | 0.88 | 0.88 × 49 = **43 m/s** |

But the dive is not perfectly vertical in practice — the glider must also navigate.
Maximum speed reached in a steep (60°) dive is ~38 m/s (E16) or ~45 m/s (F15)
at the heavier (300 g) configuration.

## Wind speed limitations

### Crosswind component

The glider's three all-moving fins (total area 0.0071 m²) can deflect ±45° at low
speed, providing yaw authority to crab into a crosswind. The practical crosswind
limit is set by landing-phase considerations (approach speed 1.3 × Vs ≈ 18 m/s),
not by fin stall:

| Parameter | Value | Notes |
|---|---|---|
| Maximum crosswind (fin authority) | ~11 m/s (40 km/h) | Fin 45°, yaw stability ratio Cnβ/Cnδ ≈ 1.3 |
| Recommended crosswind limit (first flights) | **≤5 m/s** (18 km/h) | Pilot/autopilot skill, gust margin, wingtip clearance |
| Crosswind at 25 m/s cruise (scheduled 30° deflection) | ~10 m/s (36 km/h) | Fin still has margin; enters <40 % of authority |
| Crosswind at 50 m/s (scheduled 5° deflection) | ~3 m/s (11 km/h) | Fin limited; but 50 m/s happens only in boost or a deliberate dive |

Method for the authority-based limit:

```
β_max = (Cnδ × δ_max) / Cnβ      (sideslip angle at full fin)
Cnβ ≈ 0.002/°                     (yaw stability, small glider)
Cnδ ≈ 0.0015/°                    (fin effectiveness)
δ_max = 45° at V ≤ 16 m/s

β_max = (45 × 0.0015) / 0.002 ≈ 34°
W_max = V × tan(β_max) = 17 × tan(34°) ≈ 11.5 m/s
```

### Gust response

A vertical gust adds a transient load factor. With gust-alleviation factor
Kg ≈ 0.80 for this glider:

```
Δn = Kg × ρ × V × a × S × U_g / (2 × W)
```

| Airspeed | Gust (U_g) | Δn | Total load factor (incl. 1 g) |
|---|---|---|---|
| 14 m/s (stall) | 5 m/s | +2.2 g | 3.2 g |
| 17 m/s (best glide) | 5 m/s | +2.7 g | 3.7 g |
| 17 m/s (best glide) | 7.5 m/s | +4.0 g | 5.0 g — **limit** |
| 25 m/s (cruise) | 5 m/s | +3.9 g | 4.9 g — near limit |
| 31 m/s (Va) | 5 m/s | +4.9 g | exceeds 5 g |

### Recommended wind limits for flight

| Condition | Limit | Rationale |
|---|---|---|
| Surface wind at launch | **<5 m/s** (<18 km/h) | Boost trajectory; rail clearance, parachute drift |
| Maximum steady wind for gliding | **<8 m/s** (<29 km/h) | Glide slope management, ground-speed control |
| Maximum gust | **<5 m/s** (<18 km/h) Gust | Structure + fin authority margin at cruise |
| Crosswind component at landing | **<5 m/s** (<18 km/h) | Wingtip clearance, flare margin, autopilot authority |

The limiting case is a gust at best-glide speed: a 7.5 m/s vertical gust reaches
the 5 g limit load. A 5 m/s gust keeps the total under 4 g with comfortable margin.
Above 8 m/s steady wind, the sink rate relative to ground increases beyond the
glider's ~3 m/s sink, making a controlled landing difficult.

### Fin deflection schedule vs wind

The speed-governed fin deflection schedule (from coludo.md) interacts with
crosswind capability:

- **≤16 m/s (full 45°):** all the yaw authority above is available — crosswind
  limit is ~11 m/s. This covers the approach and landing phase.
- **25 m/s (scheduled ~30°):** crosswind limit drops to ~10 m/s — still adequate.
- **50 m/s (scheduled 5°):** crosswind limit is ~3 m/s — but at 50 m/s the glider
  is usually boost-climbing vertically, not fighting crosswinds.
- **Below stall (<14 m/s):** the fin is aerodynamically soft (low q). These
  speeds occur only during the landing flare, where crosswind correction uses
  brief full-deflection pulses just before touchdown. Yaw authority degrades as
  v² — at 10 m/s a 45° fin generates only 35 % of the force it would at 17 m/s,
   so crosswind correction must be initiated early, before the flare.

## Design improvement notes

Recommendations arising from the STL analysis and performance estimates above.
Each item states what changes, why, and what it costs.

### 1. Wing airfoil — flat plate → cambered section

**What:** Replace the 2.4 mm flat-plate wing with a thin cambered airfoil (e.g. a
modified Clark-Y or a 4–6 % thick undercambered section). The wing is already 3 % thick
at the root — that profile could be kept but with the camber line curved.

**Why:** This is the single biggest aerodynamic gain available. Flat-plate CL_max ≈ 0.7
and L/D ≈ 5–10. A cambered section at the same Re (50 000–100 000) raises CL_max to
1.0–1.2 and L/D to 12–18. For the current 8.3 kg/m² wing loading, this means:

| Parameter | Flat plate | Cambered | Benefit |
|---|---|---|---|
| Stall speed | 14 m/s | **10–11 m/s** | Slower, safer landing |
| Best glide speed | 17 m/s | **13–14 m/s** | Stays well under 20 m/s |
| Best L/D | 5–10 | **12–18** | 2× the glide range |
| Sink rate | 1.7–3.4 m/s | **<1.5 m/s** | Softer descent |

A cambered section also delays tip stall (the current thin tip chord stalls very early),
improving roll control near the ground.

**Cost:** Lives entirely inside the existing 2.4 mm envelope if undercambered (bottom
surface curves up, top stays flat — simple to print or lay up). Weight increase is
zero (same bounding box, same shell). The wing's external planform and span stay the
same, so the folded-wing fit inside the booster is unaffected.

**Practical note:** At Re ≈ 70 000 a cambered plate outperforms a symmetric one but
is still far from a "proper" airfoil at Re > 200 000. The priority is to move the
mean camber line up ~1–1.5 mm at 30–40 % chord — a detail easily added to the STL
without redoing the wing outline.

### 2. Wing structure — flutter resistance

**What:** Add a uni-carbon spar (0.5 × 3 mm strip) along the maximum-thickness line,
or increase tip thickness from 2.4 mm to 3–4 mm.

**Why:** The wing has AR 12.1 and is 2.4 mm thick over a 598 mm span — a very flexible
structure. The flutter speed for a flat plate is approximately:

```
V_flutter ≈ k × (t/b) × sqrt(G/ρ) × sqrt(AR)
```

where t = thickness, b = span, G = shear modulus. For a 2.4 mm carbon/PLA wing
at 598 mm span, V_flutter is in the **35–50 m/s range** — right inside the dive
speed envelope. A stiffening spar raises this above Vne (47 m/s).

**Cost:** +1–2 g per wing for the strip. The tip thickness increase is also +1–2 g.
Either is negligible against the 35 g wing mass.

### 3. Wing planform — root chord and taper

**What:** Increase root chord, reduce tip chord taper ratio, or add a modest (~3–5°)
washout (twist) so the root stalls before the tip.

**Why:** The current 79 → 20 mm taper (taper ratio 0.25) with washout is acceptable,
but at AR 12.1 the tip chord of 20 mm at 2.4 mm thickness gives a very low local Re
(≈ 15 000 at best glide). That tip is essentially stalled all the time, wasting
span and reducing effective AR. A 30–35 mm tip chord (taper ratio 0.38–0.44) would
double the tip Re, improve aileron effectiveness, and only cost ~15 cm² of extra
wing area (+5 %), lowering stall speed by another 0.3 m/s.

**Cost:** +3–5 g per wing from the extra area. The slightly wider folded wing might
require the booster fairing internal clearance checked — the folded span at the tip
should stay within the 76 mm (holder) / 108 mm (locker) envelope.

**If the target is flight speed < 20 m/s,** the current wing is already adequate —
stall is at 14 m/s and best glide at 17 m/s. The cambered section (item 1) is the
only change needed to comfortably cruise at 13–14 m/s. The planform change (item 3)
is a refinement for tip-stall margin, not a requirement.

### 4. Triangular (keel-shaped) fuselage

**What:** Change the body cross-section from rectangular (46 × 50 mm) to a triangular
keel shape — wider at the bottom, narrowing toward the top.

**Why:** Three benefits:

- **Landing stability.** A flat or slightly crowned bottom is aerodynamically clean
  but tips over easily on uneven ground. A keel-shaped underside (wider base,
  ~60 mm at the bottom, ~30 mm at the top) gives a natural self-righting moment and
  protects the wings from ground impact on touchdown.

- **CG placement.** The heaviest components (battery, servos) sit in the lower,
  wider part — a keel section naturally lowers the CG, improving pendulum stability
  (equivalent to ~3–5° of dihedral effect) without adding tip weight.

- **Boost aerodynamics.** The wedge shape aligns with the airflow over the folded
  wings inside the fairing; the flat-bottomed body produces less base drag than an
  equal-volume rectangular section.

**Cost:** +3–5 g of printed structure (the wider base adds perimeter). The internal
volume is slightly larger, which helps electronics packaging. The folded wing
clearance is unaffected because the triangular section lives inside the same 46 mm
max width envelope — only the lower corner widens.

**Not recommended** as a first change — the rectangular body is light and adequate
for proving flight. Add the keel on a v2 airframe once the glide characteristics
are understood from telemetry.

### 5. Dihedral or polyhedral wing

**What:** Add 3–5° of dihedral (each wing tips up) or a polyhedral break at ~60 %
span.

**Why:** The current wing is flat (0° dihedral from the STL). A rocket-deployed
glider that separates from the booster with wings unfolding needs inherent roll
stability — especially in the first seconds after deployment when the controller
may not have a reliable attitude estimate. Dihedral provides a self-righting roll
moment in response to sideslip:

```
C_roll_dihedral ≈ −0.05 × Γ / (57.3 × AR)     (per degree of sideslip)
```

At 3° dihedral, a 5° sideslip produces ≈ −0.00022 C_roll — enough to return the
wings toward level in under 2 s at 17 m/s without the controller doing anything.

**Cost:** +0 g (purely a geometry change in the STL). The wing root must remain at
the original attachment angle so the folded-wing alignment inside the booster is
unchanged; the dihedral starts just outboard of the body.

**One caveat:** Dihedral reduces crosswind landing performance slightly (the
effective fin area in a crab is smaller). At the recommended 5 m/s crosswind limit
this is negligible.

### 6. Boost stability — fin contribution with folded wings

**What:** Verify the combined glider+booster CP/CG margin in the STL assembly, and
consider a temporary boost fin if the margin is < 1 caliber.

**Why:** During boost the wings are folded inside the fairing. The only stabilizing
surfaces are the glider's tail fins (198 mm total span) protruding from the stack.
The coludo.md original analysis flagged this as unverified. With the newer, heavier
glider (150.4 g structure), the CG may have moved, affecting stability.

The fin moment arm is the distance from the glider fins to the CG of the full stack
— roughly 290–390 mm depending on booster CG. At the measured mass distribution,
the combined CP should sit ≥ 1 caliber (40 mm) behind the combined CG for stable
boost. This can only be confirmed by balancing the physical assembly or computing
from the STL.

**Recommendation:** Before the first active-control flight, tape the glider to the
booster, find the balance point, and compare to the fin centre-of-pressure
(≈ 38–46 mm behind the body junction). If the margin is tight, add a small
expendable boost fin (thin printed PLA, 0.5 g) to the body tube.

### 7. Wing deployment — reliability margin

**What:** Add a redundant deployment mechanism — a second rubber band or a small
torsion spring at the hinge — so the wings are guaranteed to snap open even if one
band snags.

**Why:** A rocket-deployed glider has exactly one chance to deploy its wings. If a
band breaks, snags on the ejection charge wadding, or is weakened by temperature,
the glider falls ballistic. The current design relies on rubber bands anchored at
the front. Adding a torsion spring at the hinge (a 0.3 mm music-wire spring,
≈ 0.3 g) provides a mechanical backup that does not depend on the band's elastic
memory after months in the folded position.

**Cost:** ~0.3–0.5 g per hinge. Minimal impact on folded-wing stowage if the
spring is embedded in the hinge joint.

### Priority summary for a v2 iteration

| Priority | Change | Impact | Difficulty |
|---|---|---|---|
| **P0** | Cambered wing section | 2× L/D, −4 m/s stall, high confidence | Easy (STL edit) |
| **P1** | Flutter spar | Prevents wing failure in dive | Easy (adds 1–2 g) |
| **P1** | Dihedral (3–5°) | Passive roll stability at deployment | Easy (STL edit) |
| **P2** | Boost CG/CP check | Prevents tumble on ascent | Measurement, not build |
| **P2** | Tip chord increase | Better aileron authority | Medium (area +5 %) |
| **P3** | Keel fuselage | Landing stability | Medium (v2 airframe) |
| **P3** | Torsion-spring hinge | Deployment redundancy | Medium (mechanical detail) |
