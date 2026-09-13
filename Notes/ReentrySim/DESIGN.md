# Editor-side reentry survivability analysis

An investigation into answering, from the VAB/SPH, *"will this vehicle survive the heat of a
return from a given orbit, and at what angle of attack and periapsis?"* — plus the flight-side
reentry mode that lets the answer be checked against what the game actually does.

Status: **both halves implemented, awaiting in-game testing.** The flight side is a `Reentry`
mode in the simulation config (`SimReentryUtils`, `SetReentrySimOrbit`) plus
`ReentryTelemetryLogger`. The editor side is `Source/RP0/Reentry/` plus the `Reentry` tab
(`ReentryGUI`). `reentry_ref.py` is the earlier Python reference and is now superseded on the
thermal side by the decompiled formulae below.

> **Revision note.** Sections 3 and 7 were first written without access to the KSP assembly.
> With `ilspycmd` against `Assembly-CSharp.dll` the actual `FlightIntegrator`, `ModuleAblator`,
> `DragCubeList`, `OcclusionData` and `OcclusionCone` bodies were read, and three things changed:
> the convective-coefficient "gap" in §3.5 is closed (no calibration constant remains);
> `externalTemperature` is `max(ambient, shock)`, not `ambient + shock`; and the stock
> occlusion model turned out to be public, purely geometric classes that run fine in the editor,
> so the C# prototype uses the real thing rather than the approximation proposed in §2.

---

## 1. The question, and why it is not easy

A player finishing a lunar program wants to know, before spending a year of build time, whether
the capsule they just designed survives an 11 km/s return, and what corridor they have to fly.
Today the only way to find out is to fly it — a simulation costs money and real time, and a
failed entry teaches you only that *that* pair of (AoA, periapsis) did not work.

Three things make the question awkward to answer statically:

1. **Heating is path-dependent.** Peak skin temperature is not a function of entry velocity. It
   is the outcome of integrating flux against thermal mass down a trajectory that is itself
   determined by the vehicle's drag, its lift, and the periapsis it is aimed at. There is no
   closed form; something has to be integrated.
2. **The corridor is two-sided and narrow.** Too steep and the vehicle burns or pulls lethal g;
   too shallow and it skips back out. The width of that corridor is the actual answer the player
   wants, and it moves with angle of attack, because AoA buys L/D which buys corridor.
3. **The thermal model is the game's, not physics'.** A physically correct Sutton–Graves
   calculation would give the wrong answer, because what kills the part is KSP's
   `FlightIntegrator` arithmetic against `skinMaxTemp`, tuned by RealismOverhaul's
   `PHYSICSGLOBALS`. The analysis has to predict *the game*.

Point 3 is what makes the flight-side reentry mode the other half of this task rather than a
side quest: a reimplementation of someone else's thermal model is worth nothing until it has
been held up against the original.

---

## 2. What the editor actually gives us

RP-1 already does editor-side vehicle analysis (`AvionicsGUI`, `ControlLockerUtils`,
`CostBreakdownGUI`), so the plumbing pattern exists. The question is which *physics* inputs are
reachable outside the flight scene.

### Available, directly

| Need | Source | Notes |
|---|---|---|
| Pressure, temperature, density, speed of sound vs altitude | `CelestialBody.GetPressure/GetTemperature/GetDensity/GetSpeedOfSound` | Public, scene-independent. Works for RSS or any body, so no atmosphere model needs writing. |
| Gravity, radius, atmosphere depth, SOI | `CelestialBody.gravParameter/Radius/atmosphereDepth/sphereOfInfluence` | |
| All heating constants | `PhysicsGlobals.*` | RO overrides these in `RO_Physics.cfg`; reading them at runtime means RO retuning flows through for free. |
| Per-part thermal properties | `Part.skinMaxTemp`, `maxTemp`, `skinMassPerArea`, `skinThermalMassModifier`, `thermalMassModifier`, `skinInternalConductionMult`, `emissiveConstant`, `heatConvectiveConstant`, `mass`, `GetResourceMass()` | All present on the editor's `Part` instances. |
| Ablator behaviour | `ModuleAblator.lossConst`, `lossExp`, `pyrolysisLossFactor`, `ablationTempThresh`, `reentryConductivity`, plus the `Ablator` resource amount | RO configures every heat shield through stock `ModuleAblator` (`RO_Heatshields.cfg` renames `ModuleHeatShield` to it precisely so there is one code path). |
| Stock drag/exposed areas | `Part.DragCubes.SetDrag(dir, mach)` then `.AreaDrag`, `.Area`, `.ExposedArea` | Drag cubes are built in the editor. |

### Available, but only through reflection

FAR is guaranteed present — RP-1 depends on RealismOverhaul, whose `ferramGraph.dll` RP-1
already references directly for the program funding graph. FAR's editor aerodynamics is exactly
what an AoA sweep needs, and it is already doing AoA sweeps for its own static analysis graphs:

- `FerramAerospaceResearch.FARGUI.FAREditorGUI.EditorGUI.Instance` — **public static**
- private field `_instantSim` of type `InstantConditionSim` (`internal`)
- `InstantConditionSim.GetClCdCmSteady(InstantConditionSimInput, out InstantConditionSimOutput, bool clear, bool reset_stall)` — public method on an internal type
- `InstantConditionSimInput` carries `alpha`, `beta`, `machNumber`, `flaps`, `spoilers`;
  `InstantConditionSimOutput` returns `Cl`, `Cd`, `Cm`, referenced to wing area, or to
  `_maxCrossSectionFromBody` for a wingless vehicle such as a capsule.

So: one reflection handle to `EditorGUI.Instance`, one to `_instantSim`, one `MethodInfo`, and a
pair of accessors over the input/output types. This is the same "grab the other mod's editor
sim" pattern RP-1 already uses for `ModUtils`/`TFInterop`, and it degrades cleanly: if any handle
is missing, fall back to drag cubes and say so in the UI.

FAR's own sweep code (`SweepSim.AngleOfAttackSweep`) is the worked example of how to drive it,
including `FARAeroUtil.UpdateCurrentActiveBody(body)` and `FARAeroUtil.ResetEditorParts()` before
a batch.

One calibration detail to confirm in-game: the editor sim computes forces at `atmDensity = 2`
with a unit velocity vector and then divides by reference area, so the returned `Cd` should be a
plain dimensionless coefficient and `drag_kN = Cd * area * 0.0005 * rho * v^2`. That relation
should be checked against `FARAPI.VesselDragCoeff` on a flown vessel once, not assumed.

### Not available, and must be approximated

| Missing | Why | Proposed handling |
|---|---|---|
| `FlightIntegrator` itself | It is a `VesselModule` on a live `Vessel`. Its thermal methods are largely `public virtual`, but driving an instance without a vessel, rigidbodies and Krakensbane is not worth the fragility. | Reimplement the chain (section 3). |
| Occlusion (`PartThermalData.convectionTempMultiplier` / `convectionCoeffMultiplier` / `convectionAreaMultiplier`) | ~~Computed from raycasts against a live vessel each tick.~~ **Corrected:** `FlightIntegrator.UpdateOcclusionConvection` is a geometric shock-cone model over `OcclusionData` (built from `part.partTransform` and drag-cube bounds) and `OcclusionCone`. Both classes and `PartThermalData` are public with public constructors. | **Run the real thing.** `ReentryVesselBuilder.ComputeOcclusion` is a transcription of the stock loop against the editor parts: sort by `maximumDot`, lead part gets the first cone, each following part accumulates `GetShockStats` from every cone ahead of it. Note the lead part behind a *detached* shock gets `convectionTempMultiplier = 0.5` (`OcclusionCone.detachedShockHeatMult`), which is why a bare stagnation-temperature model overheats blunt capsules. |
| FAR voxel `radiativeArea` / `exposedArea` | FAR feeds these in from `FARAeroPartModule.ProjectedAreas` in flight, which is voxelised per-vessel. | Use drag cube areas in the editor, and note the discrepancy in the calibration run. |
| Control authority / trim | Whether the vehicle can actually *hold* the AoA the sweep assumes. | Out of scope; report the AoA and let the player check trim in FAR's own stability tab. Flag it in the UI so the number is not read as a promise. |

---

## 3. The thermal model to reproduce

FAR does **not** replace the heating maths — its `ModularFlightIntegratorRegisterer` overrides
only `UpdateAerodynamics` and the *areas* (`CalculateAreaExposed`, `CalculateAreaRadiative`,
sun/body area). The flux formulae stay stock, tuned by RO's `PHYSICSGLOBALS`. So the target is
stock `FlightIntegrator` + RO constants.

### 3.1 Convection — high confidence

This one is not guesswork: RP-1's own `ModuleNonReentryRated` states how stock assembles it.

```csharp
ptd.postShockExtTemp = UtilMath.LerpUnclamped(vessel.atmosphericTemperature,
                                             vessel.externalTemperature,
                                             ptd.convectionTempMultiplier);
ptd.finalCoeff = vessel.convectiveCoefficient * ptd.convectionArea * 0.001d
                 * part.heatConvectiveConstant * ptd.convectionCoeffMultiplier;
```

and the flux into the skin is `finalCoeff * (postShockExtTemp - skinTemperature)` kW. That gives
the whole per-part half of the model for free.

### 3.2 Shock temperature — confirmed by decompile

`externalTemperature = max(atmosphericTemperature, shock)` — **max, not sum** (the first draft
of this document had it as a sum). `shock` blends a low-speed and a hypersonic branch with
`convectiveMachLerp = clamp01((mach - start) / (end - start)) ^ newtonianMachTempLerpExponent`,
and is then multiplied by the difficulty `ReentryHeatScale` and the body's
`shockTemperatureMultiplier`:

- low speed: `spd * newtonianTemperatureFactor`
- hypersonic: `machTemperatureScalar * spd^machTemperatureVelocityExponent`

RO's own comment confirms the hypersonic branch exactly:
`machTemperatureScalar = 7.5 // ~6000 at 7.3km/sec`, and `7.5 * 7300^0.75 = 5922`. That is a
direct check of the formula, not an inference.

### 3.3 Density thermal lerp and background radiation — high confidence

Recovered verbatim from KSP-RO's own `RealHeat`, which reimplements these two:

```
densityThermalLerp(rho) = 1 - rho^0.25          for rho < 0.0625
                        = 0.75 - sqrt(rho)      for rho < 0.25
                        = 0.0625 / rho          otherwise

backgroundRadiationTemp = Lerp(ambientTemp, PhysicsGlobals.SpaceTemperature, densityThermalLerp)
```

Skin re-radiation is then
`emissiveConstant * sigma * radiativeArea * (T_skin^4 - T_bg^4) * 0.001 * radiationFactor` kW,
with `partEmissivityExponent = 4` and `radiationFactor = 1` under RO.

### 3.4 Ablation — high confidence on shape, needs a units check

Stock `ModuleAblator`, with RO's Lunar-shield values (`lossConst = 150`, `lossExp = -25000`,
`pyrolysisLossFactor = 145833`, `ablationTempThresh = 1250`):

```
rate (units/s)   = lossConst * exp(lossExp / skinTemperature) * ablativeMaxAmount     (above the threshold)
mass loss (t/s)  = rate * resource.density
heat removed (kW)= pyrolysisLossFactor * resource.specificHeatCapacity * mass loss    (as exposed skin flux)
part.heatConductivity = reentryConductivity while the shield face is not occluded, else the original
```

Decompiled from `ModuleAblator.FixedUpdate`, which settled the open questions: the rate scales
with the tank's **max** amount (so a bigger shield ablates faster in absolute terms), and what
`reentryConductivity` replaces is `part.heatConductivity` — which feeds `conductionMult` for
skin-to-internal *and* part-to-part conduction — not `skinInternalConductionMult`. The
`exp(-25000/T)` thermostat is as sharp as expected (the solver sub-steps against its slope).

### 3.5 Convective coefficient — closed

**Superseded.** The decompiled `CalculateConvectiveCoefficientMach` is simply

```
coeff = 1E-07 * machConvectionFactor * rho^machConvectionDensityExponent * spd^machConvectionVelocityExponent
```

a direct W/(m²K) coefficient with a fixed `1e-7`, blended with the Newtonian branch by
`convectiveMachLerp` and multiplied by the body's `convectionMultiplier`. (For `density > 1`
both branches use `density` unexponentiated.) There is no flux-over-ΔT step and nothing left
to calibrate; the telemetry logger's job is now verification, not fitting. The reasoning below
is kept for the record.

The constants are all named and their RO values known:

```
newtonianConvectionFactorBase   4.0      machConvectionFactor            1.05
newtonianConvectionFactorTotal  1.5873   machConvectionDensityExponent   0.5
newtonianDensityExponent        0.5      machConvectionVelocityExponent  3.0
newtonianVelocityExponent       1.0
```

The Newtonian branch is unambiguous from the names and RO's comments:
`rho^0.5 * (4 + v) * 1.5873`, which gives ~180 W/m²K at sea level and 100 m/s — a sane forced-
convection number.

The Mach branch's *shape* is equally clear — `machConvectionFactor * rho^0.5 * v^3` is the
classic Sutton–Graves stagnation form, and stock stores it in a field literally named
`convectiveMachFlux`, i.e. a flux, which then has to be divided by a temperature difference to
become the W/(m²K) coefficient the convection formula consumes. **What could not be recovered
from any public source is the internal scale factor stock applies to that product.** The
decompiled body is not published; the KSP API docs list the signature
(`CalculateConvectiveCoefficientMach()`) but no implementation, and
`kspmoddinglibs.github.io` is unreachable from this environment.

The prototype therefore carries it as one named constant,
`PhysicsConstants.machConvectionFluxScalar`, defaulted to `1e-4` because that is what makes a
capsule behave the way RP-1 capsules are known to behave (comfortable LEO return, marginal
lunar return — see section 6). **This is the single number the calibration run exists to pin
down**, and because it is one multiplicative scalar on one branch, a single logged entry fixes
it exactly.

---

## 4. Solver design

Deliberately structured so the physics core has no KSP types in it and can be unit tested; only
thin adapters touch `Part`, `CelestialBody` and FAR. `reentry_ref.py` is written in this shape.

### 4.1 Trajectory

Planar 3-DOF over a spherical rotating body:

```
dv/dt     = -D/m - g sin(gamma)
dgamma/dt = (L cos(bank)) / (m v) + (v/r - g/v) cos(gamma)
dr/dt     = v sin(gamma)
dtheta/dt = v cos(gamma) / r
```

3-DOF is the right altitude of modelling here. 6-DOF would demand trim solutions, control
authority and attitude dynamics — all of which belong to FAR's stability tab, not to a
heat-margin question. The assumption being made (the vehicle holds the commanded AoA and bank)
must be stated in the UI.

### 4.2 Thermal

Two nodes per part — skin and internal — matching stock, integrated alongside the trajectory:

- skin: `+convection −reradiation −ablation −conduction_to_internal`
- internal: `+conduction_from_skin`
- thermal masses from `skinMassPerArea * area * skinThermalMassModifier * standardSpecificHeatCapacity`
  and `mass * thermalMassModifier * standardSpecificHeatCapacity`

Most vessels do not need every part simulated. Group parts into thermal classes and simulate
representatives: the windward heat shield, the hottest non-reentry-rated part, and anything
whose `skinMaxTemp` is within a factor of the peak. This keeps a 30-part capsule stack at a
handful of nodes.

### 4.3 Stepping

Explicit Euler with an adaptive step keyed to deceleration and heating rate is enough and is
what the prototype uses: ~1200 steps for a full lunar entry, from interface to subsonic. The
stiff term is ablation, which is naturally self-limiting; if it misbehaves in C#, clamp the step
so no node moves more than ~25 K per step rather than reaching for an implicit integrator.

### 4.4 Cost

CPython does one full lunar entry with two thermal nodes in **9 ms**. C# with, say, six nodes
should land in the same order or better. A 8 × 5 (Pe × AoA) sweep is therefore comfortably
sub-second — fast enough to run on demand from a button, and easily fast enough to run
incrementally across frames in a coroutine so the editor never stutters. No threading needed,
which matters because the FAR aero sample must happen on the main thread anyway.

Sample the FAR aero table **once** — Cl/Cd over the (AoA, Mach) grid — and reuse it across every
trajectory in the sweep. That is the expensive call, and it does not depend on the trajectory.

---

## 5. The sweep and what it reports

For each (AoA, periapsis) pair: derive the entry state from the arriving orbit, integrate, score.

Entry state from the orbit is the same maths the flight-side reentry mode now uses
(`SimReentryUtils.TryComputeEntry`): given periapsis plus either an apoapsis or a hyperbolic
excess speed, the speed and flight path angle at the interface follow from energy and angular
momentum. The two halves should share that code.

A pair is **acceptable** when the vehicle captures (does not skip out), no part exceeds
`skinMaxTemp` or `maxTemp`, no ablator is exhausted, and peak g stays under a limit — crewed
vs. uncrewed, which RP-1 already knows from the vessel.

The headline output is not a single number, it is the **corridor**: for each AoA, the band of
periapsides that work. That is the answer to "can this thing come home", and its width is the
answer to "how precisely do I have to fly it". Report alongside it the limiting part by name
(the thing that burns first is usually not the heat shield — in the prototype it is the hull),
peak skin temperature as a fraction of limit, peak g, integrated heat load, and ablator consumed.

---

## 6. Reference implementation results

`reentry_ref.py` — dependency-free, runs anywhere. Full output in `sample_output.txt`.

A Mercury-class capsule (1.4 t, 2.81 m², RO Lunar-tag shield, L/D ~0.3 at 30° AoA) returning to
RSS Earth:

**LEO return, Ap 250 km** — every pair in the grid survives, peak skin 1770–2140 K against a
3600 K limit, 2.2–9.0 g. Correct: a LEO return is not what kills capsules.

**Lunar return, v∞ 1100 m/s (hyperbolic, 11.12 km/s at interface)** — the corridor appears, and
it is narrow and AoA-dependent:

```
  Pe km   AoA  ok   skin K  skin%   int%      g   outcome
   -200     0   n     3279   91.1   33.4   67.7  (survives, but 68 g)
      0    20   n     2963   82.3   33.5   14.6  (survives, but 15 g)
     20    20   Y     2897   80.5   33.6   10.9  slowed to subsonic descent
     40    10   Y     2835   79.5   33.5    8.4  slowed to subsonic descent
     40    20   n     2794   77.9   33.4    6.9  skipped out of the atmosphere
     60     0   Y     2694   76.7   33.5    7.0  slowed to subsonic descent
     60    10   n     2631   74.1   33.4    3.3  skipped out of the atmosphere

  entry corridor (12 g limit):
    AoA   0 deg: Pe  60 km
    AoA  10 deg: Pe  40 km
    AoA  20 deg: Pe  20 km
```

Both failure modes are present and on the right sides — steep entries pull lethal g, shallow
lifting entries skip out, and lifting up moves the whole usable band deeper. The limiting part
is the capsule hull, not the shield, which is the realistic answer and exactly the kind of thing
a player cannot see today.

This is qualitative validation of the *method*. The absolute temperatures are only as good as
§3.5's scalar, which is what the next section fixes.

---

## 7. Calibration and validation — the flight-side half

Implemented in this branch.

**Reentry mode** (`Reentry` toggle in the simulation config, `SimReentryUtils`): describes the
arriving trajectory rather than a parking orbit, takes periapsis as typed including inside the
atmosphere, accepts hyperbolic arrivals by excess speed, and places the vessel on the inbound
leg at the entry interface with a lead time for attitude. The previous code clamped apoapsis to
the SOI and periapsis to at least `atmosphereDepth`, so none of this was reachable. The panel
previews the entry speed, flight path angle and spawn altitude before committing.

**Telemetry** (`ReentryTelemetryLogger`, opt-in checkbox): writes a CSV per simulated entry to
`saves/<save>/RP-1_ReentryLogs/`, sampling every 0.5 s inside the atmosphere:

```
ut, missionTime, altitude, srfSpeed, mach, atmDensity, dynPreskPa, geeForce,
atmosphericTemperature, externalTemperature, convectiveCoefficient,
part, skinTemperature, skinMaxTemp, temperature, maxTemp,
exposedArea, radiativeArea, convectionFlux, radiationFlux, skinToInternalFlux
```

The three middle columns are the point. `externalTemperature` and `convectiveCoefficient` are
the live outputs of the two formulae in §3.2 and §3.5, logged next to the `atmDensity`, `mach`
and `srfSpeed` that produced them. Fitting the predicted coefficient against the logged one over
a single entry determines `machConvectionFluxScalar` to whatever precision anyone cares about,
and simultaneously confirms — or refutes — the assumed shape of the Mach branch. If the residual
is not a flat multiplicative offset, the shape is wrong and the log says so immediately.

`exposedArea` / `radiativeArea` against the editor's drag-cube estimate quantifies the
FAR-voxelisation gap from §2. `convectionFlux` against predicted flux closes the loop on the
occlusion multipliers.

**Suggested calibration procedure**

1. Build a bare capsule + heat shield. Simulate: reentry mode, Pe 40 km, hyperbolic, v∞ 1100,
   telemetry on.
2. Hold retrograde through peak heating.
3. Fit `machConvectionFluxScalar` from `convectiveCoefficient` vs. (`atmDensity`, `srfSpeed`,
   `mach`); check the residual is flat.
4. Re-run the prototype with the fitted value and compare the predicted `skinTemperature`
   history to the logged one.
5. Repeat once from LEO to confirm the Newtonian→Mach blend, and once with a non-reentry-rated
   part attached to exercise the occlusion multipliers.

### 7.1 What the flights showed (Mk2 SSTO / Mk2 SSTO Big, RP-1 4.7 + RealHeat + FAR fork, Sept 2026)

Compared by altitude, prediction vs the flown reentry at the same setup (Pe/Ap from the sim
panel, AoA 42 held, nose first):

- **Heating inputs match.** `externalTemperature` and `convectiveCoefficient` agree with the
  flown values to 0.1 % at equal airspeed once RealHeat's curve replaces the stock shock
  temperature and the airspeed is inertial minus ground rotation.
- **Areas match once FAR's voxel areas replace the drag cubes** (tank: 182 vs 176 m2 exposed,
  63.3 vs 62.6 radiative). Flight does not cap `exposedArea` at `radiativeArea`, so neither
  does the builder.
- **Engines run hot in the prediction** (RD-275 966 vs 715 K at 80 km): flight shows their
  occlusion area multiplier falling from 0.17 to 0.02 as the elevons deflect to hold the AoA,
  and the editor keeps control surfaces neutral. Not modelled.
- **Tank interior.** With the prefab `heatConductivity` the interior tracked the skin; the
  RealFuels MLI bridge gives a conductance that matches the flown one at like temperatures
  (~3 kW/K at 1750 K mean, ~1.6 kW/K at 1400 K, the same T^4.67 law). The internal thermal
  mass was double the flown value until the skin share was subtracted from the total
  (`UpdateMassStats` does; the builder now does).
- **The vehicle is lost to the fuselage tank's internal limit, not its skin.** The flown tank
  reached `maxTemp` 1473 K at 77 km with its skin at 79 % of `skinMaxTemp`; its interior
  started the entry at 100 K (cryogenic contents), which the editor cannot know - the
  prediction starts every part at 293 K.
- **Lift is the open item.** FAR's editor sim at Mach 25 gives Cl 1.5 at 42 deg; the flown FAR
  readout is 0.70. Part of that is definition (FAR's flight Cl is the roof-axis component,
  cos(AoA) short of the perpendicular lift), the rest is unexplained. The prediction spent
  ~700 s longer between 96 and 98 km than the flight because of it, and soaked the tank
  accordingly. The flight log now carries the total and wing-model force vectors resolved
  in the editor sim's frame, and the prediction its own, to settle this directly.
- Flight also scales each part's body drag by max(`pseudoReDragMult`, 1) (FAR's
  `CalculateLocalDynPresAndAngularDrag`), which the editor sim does not; the FAR aero model
  applies it to the body share. This is a 2-3x drag boost above 95 km.

Later the same day, with the KSPUITest harness flying the entry (SmartASS SVEL+ pitch offset =
AoA, 4x physics warp, `Reentry_Predict` + `Reentry_Fly`, see `KSPUITest/Source/Tests.RP1`):

- **The lift gap was the frame.** FAR's editor sim fixes nose = build axis and roof = toward the
  facility's back wall; this craft was rotated about the build axis in the VAB, so the sim flowed
  it through its yaw plane (per-wing log: main wings Cl 0, vertical fins Cl 0.6). The builder now
  takes nose/roof from the command part's transform (flight's `ReferenceTransform`) and rolls
  FAR's sim to match through its `phi` input. Lift, drag and the wing/body split then agree with
  flight within 10 % from 100 km down.
- **Convection is scaled by `thermalConvergenceFactor` (0.63) in flight** -
  `thermalConvectionFlux = convectionFlux * cf` is what reaches the skin; radiation is not scaled.
  Backed out of the flown tank's flux to 0.5 %. The solver applied the raw product. Fixed; the
  fuselage tank's skin now tracks flight within 3 % (1955 vs 1888 K at 82 km).
- **Remaining:** the interior still leads flight by ~3 km (predicted limit at ~80 km vs flown
  77 km), which is the 293 K start against a cryogenic 99 K; the engines are still over-predicted
  (elevons deflected to the stop in flight shade them); part-to-part conduction is absent.
- **FAR quirk found on the way:** a vessel re-voxelized (staging) while RCS is slewing it gets
  every part's projected area scaled by ~sqrt(2) for the rest of the flight - the voxel frame and
  the meshes are sampled at different attitudes. Stage, let it settle, then manoeuvre.

---

## 8. UI

A new `Reentry` tab alongside `Avionics` in `TopWindow`, editor-only (`ShouldShowTab`), following
`AvionicsGUI`'s update-on-interval pattern:

- **Return from**: LEO / Highly elliptical (apoapsis) / Hyperbolic (v∞) — the same three cases
  the flight-side reentry mode takes, so the numbers can be copied straight across into a
  simulation to verify.
- **Run analysis** button (not continuous — it costs real milliseconds and the ship changes
  constantly while editing).
- **Result**: the corridor per AoA, the best pair, the limiting part, peak skin % of limit, peak
  g, ablator consumed.
- **Caveats line**: whether FAR aero or drag cubes were used, and that holding the AoA is
  assumed, not verified.

A `DesignConcerns` entry ("this vehicle cannot survive a return from its own mission profile")
is tempting but should wait until after calibration — a wrong concern is worse than no concern.

---

## 9. Risks and open questions

| Risk | Mitigation |
|---|---|
| `machConvectionFluxScalar` (§3.5) is not just a scalar — the Mach branch shape is wrong | The calibration log shows this immediately as a non-flat residual. Cheap to detect, and the fix is confined to one method. |
| FAR reflection breaks on a FAR update (`_instantSim` is a private field of an internal type) | Resolve handles once, cache, and fall back to drag cubes with a UI note. Never throw into the editor GUI. |
| Editor drag-cube areas differ materially from FAR's voxel areas in flight | Quantified by the telemetry log's `exposedArea`/`radiativeArea` columns. If large, apply a per-vessel correction sampled from one flight. |
| Occlusion approximation is too crude for a multi-part stack | Restrict v1 to reporting the *windward* part and any non-reentry-rated part, which is where the multipliers are least ambiguous. |
| 3-DOF assumes attitude is held | State it in the UI. Do not bury it. |
| Players read the corridor as a guarantee | Report a margin, not a verdict; show the limiting part so the answer is inspectable. |

Open questions for whoever picks this up:

- Does `SetShipOrbit` accept `ecc > 1` cleanly in all cases? The reentry-mode code assumes it
  does (KSP models hyperbolic orbits, and HyperEdit-style mods set them), but it has not been
  run in-game in this branch.
- Should the corridor account for a bank-angle roll program, as real capsules fly? The solver
  already takes a bank angle; a fixed bank is one more sweep axis and probably the cheapest
  realism win after this lands.
- Should Principia change any of this? Reentry mode refuses backwards time travel the same way
  the existing simulation code does, but a hyperbolic spawn under Principia is untested.

---

## 10. Effort and phasing

| Phase | Work | Estimate |
|---|---|---|
| 0 — done | Reentry mode, entry-interface placement, telemetry logger, reference solver | — |
| 1 | Calibrate §3.5 from one logged entry; fix the constant in code | Half a day, mostly flying |
| 2 | Port the solver core to C# with no KSP types (`ReentrySolver`, `ThermalNode`, `AtmosphereAdapter`) | ~600 lines |
| 3 | Vessel adapter: part grouping, thermal properties, ablator, occlusion estimate | ~300 lines |
| 4 | FAR reflection adapter + drag-cube fallback | ~250 lines |
| 5 | `ReentryGUI` tab and sweep driver | ~300 lines |
| 6 | Validate against 3–4 flown entries across the envelope | Half a day |

Phases 2–5 are independently reviewable, and phase 2 is testable on its own against
`reentry_ref.py`.

---

## Sources

- `Source/RP0/PartModules/ModuleNonReentryRated.cs` — the convection flux assembly, in this repo
- KSP-RO/RealismOverhaul, `GameData/RealismOverhaul/RO_Physics.cfg` — the `PHYSICSGLOBALS` values
- KSP-RO/RealismOverhaul, `GameData/RealismOverhaul/RO_Heatshields.cfg` — `ModuleAblator` values
- KSP-RO/RealHeat, `Source/RealHeat.cs` — `CalculateDensityThermalLerp`, background radiation temp
- dkavolis/Ferram-Aerospace-Research — `ModularFlightIntegratorRegisterer.cs` (FAR overrides only
  areas, not flux), `InstantConditionSim.cs`, `SweepSim.cs`
- sarbian/ModularFlightIntegrator — the stock `FlightIntegrator` override surface
- KSPModdingLibs/KSPDocsSite — `FlightIntegrator`, `PartThermalData`, `Part`, `ModuleAblator`
  member lists (signatures and access levels; no bodies)
