# Reentry simulation notes

- **`DESIGN.md`** — the investigation: can the editor answer "will this survive a return from
  this orbit, at what angle of attack and periapsis", what data is reachable outside the flight
  scene, which parts of KSP's thermal model have to be reimplemented, how to calibrate them
  against real flight, and what building it would cost.
- **`reentry_ref.py`** — dependency-free reference implementation of the proposed solver
  (3-DOF trajectory + the stock two-node thermal model + an AoA x periapsis sweep). Run it with
  `python3 reentry_ref.py`. It exists so the numerics can be reviewed without a running copy of
  KSP, and so the eventual C# port has something to be checked against.
- **`sample_output.txt`** — its output for a Mercury-class capsule returning from LEO and from a
  lunar-return hyperbola.

The flight-side half of the investigation is implemented rather than described: the simulation
config has a `Reentry` mode (`Source/RP0/SpaceCenter/SimReentryUtils.cs`) that places a vessel on
an arriving trajectory at the entry interface, and an opt-in telemetry logger
(`Source/RP0/SpaceCenter/ReentryTelemetryLogger.cs`) that records what the game's own heating
model did, which is what the editor-side prediction has to be calibrated against.
