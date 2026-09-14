using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using UnityEngine;
using RP0.Reentry;

namespace RP0
{
    /// <summary>
    /// Editor tab: will this vehicle survive a return from a given orbit, and at what angle of attack and
    /// periapsis? Sweeps AoA x periapsis through the editor-side reentry solver and reports the corridor.
    /// </summary>
    public class ReentryGUI : UIBase
    {
        private static readonly string[] _arrivalLabels = { "Apoapsis", "Entry speed" };
        private static readonly string[] _attitudeLabels = { "Heat shield first", "Nose first" };

        private int _arrival = 0;
        private string _sAp = "250", _sEntrySpeed = "11000";
        private string _sPeMin = "-50", _sPeMax = "80", _sPeSteps = "6";
        private string _sAoAMax = "30", _sAoASteps = "4";
        private string _sGLimit = "";
        private string _sPredictAoA = "0";
        private string _sPropellantPct = "";
        private int _attitude = -1;
        private bool _bothLiftDirections = true;

        private List<ReentryResult> _results;
        private double[] _peList, _aoaList;
        private string _summary = "";

        /// <summary>Airspeed below which the touchdown prediction flies at 0 deg AoA: the final approach, not the entry.</summary>
        public const double ApproachSpeed = 1000.0;

        /// <summary>
        /// Fly the simulation's Reentry setup through the solver at the given attitude and report where it touches
        /// down: the in-plane angle past periapsis and the time after the starting altitude. Below ApproachSpeed the
        /// AoA drops to 0 for the approach. False (with a reason) when the analysis cannot run; the aim then falls
        /// back to the periapsis.
        /// </summary>
        public static bool TryPredictTouchdown(SimulationParams sp, double aoaDeg, bool shieldFirst, out double angleFromPeDeg, out double secondsFromInterface, out double downrangeKm, out string error)
        {
            angleFromPeDeg = secondsFromInterface = downrangeKm = double.NaN;
            error = null;
            try
            {
                List<Part> parts = EditorLogic.fetch?.ship?.Parts;
                CelestialBody body = sp.SimulationBody ?? Planetarium.fetch.Home;
                if (parts == null || parts.Count == 0) { error = "no vessel"; return false; }
                if (FarEditorAero.Available && !FarEditorAero.Ready) { error = "FAR is still voxelizing the craft"; return false; }
                var k = ReentryConstants.FromGame(body);
                var solver = new ReentrySolver(body, k);
                ReentryAttitude att = ReentryVesselBuilder.Build(parts, body, k, aoaDeg, shieldFirst, new DefaultAeroFactory());
                if (aoaDeg != 0.0)
                {
                    solver.LowSpeedCutoff = ApproachSpeed;
                    solver.LowSpeedAero = ReentryVesselBuilder.Build(parts, body, k, 0.0, shieldFirst, new DefaultAeroFactory()).Aero;
                }
                if (!solver.TryEntryState(sp.SimOrbitPe, sp.SimReentryArrival, sp.SimOrbitAp, sp.SimEntrySpeed, sp.SimEntryAltitude,
                                          out double v0, out double fpa, out error))
                    return false;
                ReentryResult r = solver.Simulate(att, sp.SimEntryAltitude, v0, fpa, liftDown: false);
                if (r.Outcome == ReentryOutcome.SkippedOut || r.Outcome == ReentryOutcome.TimedOut) { error = "the vehicle does not come down on this pass"; return false; }
                // True anomaly at the interface (inbound, so negative) from the conic; downrange is the inertial angle flown after it.
                if (!SimReentryUtils.TryComputeEntry(body, sp.SimOrbitPe, sp.SimReentryArrival, sp.SimOrbitAp, sp.SimEntrySpeed, sp.SimEntryAltitude,
                                                     0d, out SimReentryUtils.EntryState entry, out error))
                    return false;
                double rEntry = body.Radius + sp.SimEntryAltitude;
                double cosNu = (entry.SMA * (1d - entry.Ecc * entry.Ecc) / rEntry - 1d) / entry.Ecc;
                double nuEntry = -Math.Acos(Math.Max(-1d, Math.Min(1d, cosNu)));
                angleFromPeDeg = (nuEntry + r.Downrange / body.Radius) * 180d / Math.PI;
                secondsFromInterface = r.Duration;
                downrangeKm = r.Downrange / 1000d;
                return true;
            }
            catch (Exception ex)
            {
                error = ex.Message;
                RP0Debug.LogWarning($"Touchdown prediction failed: {ex}");
                return false;
            }
        }

        /// <summary>The last analysis/prediction summary and the last predicted trace's path, for automation.</summary>
        public static string LastSummary { get; private set; } = "";
        public static string LastPredictionPath { get; private set; } = "";
        private string _aeroSource = "";
        private double _elapsedMs;
        private double _gLimitUsed;
        private ReentryResult _best;
        private Vector2 _scroll;

        public void RenderReentryTab()
        {
            List<Part> parts = EditorLogic.fetch?.ship?.Parts;
            if (parts == null || parts.Count == 0)
            {
                GUILayout.Label("No vessel.");
                return;
            }
            CelestialBody body = Planetarium.fetch.Home;
            if (_attitude < 0) _attitude = DefaultAttitude(parts);
            if (EditorLogic.fetch.ship.shipFacility != EditorDriver.editorFacility)
            {
                // FAR's editor sim and this analysis both take "forward" from the facility, so a craft
                // opened in the other editor has its nose on the wrong axis and every angle is off.
                GUILayout.Label($"This is a {EditorLogic.fetch.ship.shipFacility} craft opened in the {EditorDriver.editorFacility}; open it in the {EditorLogic.fetch.ship.shipFacility} to analyse it.", BoldLabel);
                return;
            }

            GUILayout.Label($"Return to {body.bodyName}", BoldLabel);

            int arrival = GUILayout.SelectionGrid(_arrival, _arrivalLabels, 2);
            if (arrival != _arrival) { _arrival = arrival; TopWindow.ResetSize(); }
            GUILayout.BeginHorizontal();
            if (_arrival == 0)
            {
                GUILayout.Label(new GUIContent("Apoapsis (km):", "Apoapsis of the orbit being returned from."), GUILayout.Width(150));
                _sAp = GUILayout.TextField(_sAp, GUILayout.Width(70));
            }
            else
            {
                GUILayout.Label(new GUIContent("Entry speed (m/s):", "Inertial speed at the top of the atmosphere. A lunar return is roughly 11,000 m/s."), GUILayout.Width(150));
                _sEntrySpeed = GUILayout.TextField(_sEntrySpeed, GUILayout.Width(70));
            }
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label(new GUIContent("Periapsis (km) from / to / steps:", "Range of target periapsides to try. Negative and sub-atmospheric values are the interesting ones."), GUILayout.Width(200));
            _sPeMin = GUILayout.TextField(_sPeMin, GUILayout.Width(45));
            _sPeMax = GUILayout.TextField(_sPeMax, GUILayout.Width(45));
            _sPeSteps = GUILayout.TextField(_sPeSteps, GUILayout.Width(30));
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label(new GUIContent("AoA (deg) up to / steps:", "Angles of attack to try, from 0 up to this. The vehicle is assumed to hold the angle; check trim in FAR."), GUILayout.Width(200));
            _sAoAMax = GUILayout.TextField(_sAoAMax, GUILayout.Width(45));
            _sAoASteps = GUILayout.TextField(_sAoASteps, GUILayout.Width(30));
            GUILayout.EndHorizontal();

            _attitude = GUILayout.SelectionGrid(_attitude, _attitudeLabels, 2);

            GUILayout.BeginHorizontal();
            GUILayout.Label(new GUIContent("g limit:", "Peak deceleration allowed. Blank = 12 g crewed, 30 g uncrewed."), GUILayout.Width(150));
            _sGLimit = GUILayout.TextField(_sGLimit, GUILayout.Width(70));
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            GUILayout.Label(new GUIContent("Propellant remaining (%):", "Before analysing, set every engine propellant on the craft to this share of its tanks (the simulation's Reentry mode takes the same number). Blank = as built."), GUILayout.Width(150));
            _sPropellantPct = GUILayout.TextField(_sPropellantPct, GUILayout.Width(70));
            GUILayout.EndHorizontal();
            _bothLiftDirections = GUILayout.Toggle(_bothLiftDirections, new GUIContent(" Try lift up and lift down",
                "Fly each pair both lift-up and lift-down and keep the better. Off = lift up only."));

            GUILayout.Space(4);
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Analyse", GUILayout.Width(120)))
                Run(parts, body);
            if (_best != null && GUILayout.Button(new GUIContent("Set up simulation", "Fill the simulation's Reentry mode with the best pair, to fly it for real."), GUILayout.Width(150)))
                KCT_GUI.PrefillReentrySim(_best.PeAltitude / 1000.0, _arrival == 0 ? SimReentryArrival.Apoapsis : SimReentryArrival.EntrySpeed,
                                          ParseOr(_sAp, 250.0), ParseOr(_sEntrySpeed, 11000.0), _attitude == 0);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            if (GUILayout.Button(new GUIContent("Predict sim setup", "Run one entry with the simulation config's Reentry-mode inputs and write the predicted trace as a CSV next to the flown telemetry logs, for comparison."), GUILayout.Width(150)))
                PredictSimSetup(parts, body);
            GUILayout.Label("at AoA", GUILayout.Width(50));
            _sPredictAoA = GUILayout.TextField(_sPredictAoA, GUILayout.Width(40));
            if (ReentryHeatHighlight.Active && GUILayout.Button(new GUIContent("Clear tint", "Remove the temperature tint from the parts."), GUILayout.Width(80)))
                ReentryHeatHighlight.Clear();
            GUILayout.EndHorizontal();

            if (!string.IsNullOrEmpty(_summary))
            {
                GUILayout.Space(4);
                DrawHorizontalSeparator();
                GUILayout.Label(_summary);
                if (_results != null) DrawGrid();
                GUILayout.Label($"Aero: {_aeroSource}. {_elapsedMs:N0} ms.", HighLogic.Skin.label);
            }
        }

        private void DrawGrid()
        {
            _scroll = GUILayout.BeginScrollView(_scroll, GUILayout.Height(Math.Min(260, 24 * (_peList.Length + 2))));
            GUILayout.BeginHorizontal();
            GUILayout.Label("Pe km \\ AoA", BoldLabel, GUILayout.Width(80));
            foreach (double a in _aoaList) GUILayout.Label($"{a:N0}°", BoldRightLabel, GUILayout.Width(78));
            GUILayout.EndHorizontal();
            int idx = 0;
            for (int pi = 0; pi < _peList.Length; ++pi)
            {
                GUILayout.BeginHorizontal();
                GUILayout.Label($"{_peList[pi] / 1000.0:N0}", RightLabel, GUILayout.Width(80));
                for (int ai = 0; ai < _aoaList.Length; ++ai, ++idx)
                {
                    ReentryResult r = _results[idx];
                    string cell = Cell(r);
                    GUILayout.Label(new GUIContent(cell, Tooltip(r)), RightLabel, GUILayout.Width(78));
                }
                GUILayout.EndHorizontal();
            }
            GUILayout.EndScrollView();
        }

        private string Cell(ReentryResult r)
        {
            bool ok = r.Acceptable(_gLimitUsed);
            string mark = ok ? "OK" : (r.Outcome == ReentryOutcome.SkippedOut ? "skip" : r.Survived ? $"{r.PeakG:N0}g" : "burn");
            return $"{mark} {r.PeakSkinFrac * 100:N0}%";
        }

        private static string Tooltip(ReentryResult r)
        {
            return $"{Describe(r.Outcome)}\nPeak skin {r.PeakSkin:N0} K ({r.PeakSkinFrac * 100:N0}% of limit) on {r.LimitingPart}\n" +
                   $"Internal {r.PeakInternalFrac * 100:N0}% of limit, {r.PeakG:N1} g, {r.HeatLoadKJ / 1000:N0} MJ, ablator used {r.AblatorUsed:N1}\n" +
                   $"{(r.LiftDown ? "lift down" : "lift up")}, peak heating at {r.PeakHeatingAltitude / 1000:N0} km / {r.PeakHeatingSpeed:N0} m/s";
        }

        private static string Describe(ReentryOutcome o)
        {
            switch (o)
            {
                case ReentryOutcome.Landed: return "Captured and slowed";
                case ReentryOutcome.SkippedOut: return "Skipped back out of the atmosphere";
                case ReentryOutcome.SkinOverheat: return "Skin over its maximum temperature";
                case ReentryOutcome.InternalOverheat: return "Internal over its maximum temperature";
                case ReentryOutcome.AblatorExhausted: return "Ablator exhausted";
                default: return "Did not finish";
            }
        }

        // Which tanks start cold, so the reader knows the interior margins assume a fuelled-on-the-pad start.
        private static string CryoNote(ReentryAttitude att)
        {
            int n = 0; double coldest = double.MaxValue;
            foreach (ReentryNode node in att.Nodes)
                if (!double.IsNaN(node.InternalStartTemperature)) { ++n; coldest = Math.Min(coldest, node.InternalStartTemperature); }
            string cryo = n > 0 ? $"; {n} cryogenic tank{(n == 1 ? "" : "s")} start at {coldest:N0} K" : "";
            return cryo + $"; {att.ControlTorqueKNm:N0} kN m of wheel/RCS pitch torque";
        }

        // Which parts went over a limit, and which came close, by part type: the tint cannot show a hidden
        // engine or tell a 96 % tank from a lost one, and the loss altitude is what a pilot wants to know.
        private static string PartLimitsReport(ReentryAttitude att)
        {
            var groups = new Dictionary<string, List<ReentryNode>>();
            foreach (ReentryNode n in att.Nodes)
            {
                if (n.WorstFrac < 0.9) continue;
                if (!groups.TryGetValue(n.Name, out List<ReentryNode> list)) groups[n.Name] = list = new List<ReentryNode>();
                list.Add(n);
            }
            if (groups.Count == 0) return "\nNo part came within 10% of a limit.";
            var lines = new List<KeyValuePair<double, string>>();
            foreach (KeyValuePair<string, List<ReentryNode>> g in groups)
            {
                double worst = 0.0, skin = 0.0, interior = 0.0, lossAlt = double.MaxValue;
                int lost = 0;
                foreach (ReentryNode n in g.Value)
                {
                    worst = Math.Max(worst, n.WorstFrac);
                    skin = Math.Max(skin, n.SkinFrac);
                    interior = Math.Max(interior, n.InternalFrac);
                    if (!double.IsNaN(n.LimitAltitude)) { ++lost; lossAlt = Math.Min(lossAlt, n.LimitAltitude); }
                }
                string where = skin >= interior ? $"skin {skin * 100:N0}%" : $"interior {interior * 100:N0}%";
                string count = g.Value.Count > 1 ? $" x{g.Value.Count}" : "";
                string text = lost > 0
                    ? $"  LOST: {g.Key}{count} - {where}{(lost < g.Value.Count ? $" ({lost} of {g.Value.Count})" : "")}, first at {lossAlt / 1000:N0} km"
                    : $"  near: {g.Key}{count} - {where}";
                lines.Add(new KeyValuePair<double, string>(worst, text));
            }
            lines.Sort((a, b) => b.Key.CompareTo(a.Key));
            var sb = new StringBuilder("\nParts over or within 10% of a limit (skin or interior, whichever is worse):");
            foreach (KeyValuePair<double, string> l in lines) sb.Append('\n').Append(l.Value);
            return sb.ToString();
        }

        // The propellant-remaining box, applied to the editor ship so FAR's CoM and this analysis agree.
        private void ApplyPropellantLoad(List<Part> parts)
        {
            if (!double.TryParse(_sPropellantPct, out double pct) || pct < 0.0) return;
            int n = PropellantLoad.Apply(parts, pct);
            if (n > 0 && EditorLogic.fetch?.ship != null) GameEvents.onEditorShipModified.Fire(EditorLogic.fetch.ship);
        }

        // FAR voxelizes a craft a moment after it loads or changes; running before that would silently fall
        // back to drag cubes, which are not what the vessel flies with.
        private bool FarReadyOrExplain()
        {
            if (!FarEditorAero.Available || FarEditorAero.Ready) return true;
            _summary = "FAR is still voxelizing this craft; try again in a moment.";
            return false;
        }

        private void Run(List<Part> parts, CelestialBody body)
        {
            var sw = Stopwatch.StartNew();
            _results = null; _best = null; _summary = "";
            try
            {
                if (!FarReadyOrExplain()) return;
                ApplyPropellantLoad(parts);
                double peMin = ParseOr(_sPeMin, -50) * 1000.0, peMax = ParseOr(_sPeMax, 80) * 1000.0;
                int peSteps = Math.Max(1, (int)ParseOr(_sPeSteps, 6));
                double aoaMax = ParseOr(_sAoAMax, 30);
                int aoaSteps = Math.Max(1, (int)ParseOr(_sAoASteps, 4));
                bool crewed = false;
                foreach (Part p in parts) if (p.CrewCapacity > 0) { crewed = true; break; }
                _gLimitUsed = string.IsNullOrWhiteSpace(_sGLimit) ? (crewed ? 12.0 : 30.0) : ParseOr(_sGLimit, 12.0);

                _peList = new double[peSteps];
                for (int i = 0; i < peSteps; ++i) _peList[i] = peSteps == 1 ? peMin : peMin + (peMax - peMin) * i / (peSteps - 1);
                _aoaList = new double[aoaSteps];
                for (int i = 0; i < aoaSteps; ++i) _aoaList[i] = aoaSteps == 1 ? 0.0 : aoaMax * i / (aoaSteps - 1);

                var k = ReentryConstants.FromGame(body);
                var solver = new ReentrySolver(body, k);
                var factory = new DefaultAeroFactory();
                bool shieldFirst = _attitude == 0;
                SimReentryArrival arrival = _arrival == 0 ? SimReentryArrival.Apoapsis : SimReentryArrival.EntrySpeed;
                double apAlt = ParseOr(_sAp, 250) * 1000.0, entrySpeed = ParseOr(_sEntrySpeed, 11000);

                var attitudes = new ReentryAttitude[aoaSteps];
                for (int ai = 0; ai < aoaSteps; ++ai)
                    attitudes[ai] = ReentryVesselBuilder.Build(parts, body, k, _aoaList[ai], shieldFirst, factory);
                _aeroSource = $"{factory.LastSource}, {(attitudes[0].VoxelAreas ? "FAR voxel" : "drag cube")} areas, {solver.ThermalModelDescription} heating" + (factory.TrimSummary.Length > 0 ? $"; {factory.TrimSummary}" : "") + CryoNote(attitudes[0]);

                _results = new List<ReentryResult>(peSteps * aoaSteps);
                var corridor = new Dictionary<double, KeyValuePair<double, double>>();
                string firstError = null;
                for (int pi = 0; pi < peSteps; ++pi)
                {
                    double pe = _peList[pi];
                    bool haveEntry = solver.TryEntryState(pe, arrival, apAlt, entrySpeed, body.atmosphereDepth, out double v0, out double fpa, out string err);
                    if (!haveEntry && firstError == null) firstError = err;
                    for (int ai = 0; ai < aoaSteps; ++ai)
                    {
                        ReentryResult r;
                        if (!haveEntry)
                        {
                            r = new ReentryResult { AoADeg = _aoaList[ai], PeAltitude = pe, Outcome = ReentryOutcome.TimedOut, LimitingPart = err };
                        }
                        else
                        {
                            r = solver.Simulate(attitudes[ai], body.atmosphereDepth, v0, fpa, liftDown: false);
                            if (_bothLiftDirections)
                            {
                                ReentryResult down = solver.Simulate(attitudes[ai], body.atmosphereDepth, v0, fpa, liftDown: true);
                                if (Score(down) < Score(r)) r = down;
                            }
                            r.PeAltitude = pe;
                        }
                        _results.Add(r);
                        if (r.Acceptable(_gLimitUsed))
                        {
                            if (!corridor.TryGetValue(r.AoADeg, out var band)) band = new KeyValuePair<double, double>(pe, pe);
                            corridor[r.AoADeg] = new KeyValuePair<double, double>(Math.Min(band.Key, pe), Math.Max(band.Value, pe));
                            if (_best == null || Score(r) < Score(_best)) _best = r;
                        }
                    }
                }

                var sb = new StringBuilder();
                if (_best != null)
                {
                    sb.AppendLine($"Best: Pe {_best.PeAltitude / 1000:N0} km at {_best.AoADeg:N0}° AoA ({(_best.LiftDown ? "lift down" : "lift up")}) - " +
                                  $"peak skin {_best.PeakSkin:N0} K, {_best.PeakSkinFrac * 100:N0}% of limit on {_best.LimitingPart}, {_best.PeakG:N1} g");
                    sb.AppendLine($"Corridor (under {_gLimitUsed:N0} g):");
                    foreach (var kv in corridor)
                        sb.AppendLine($"  AoA {kv.Key,3:N0}°: Pe {kv.Value.Key / 1000,5:N0} .. {kv.Value.Value / 1000,5:N0} km");
                }
                else
                {
                    sb.AppendLine(firstError != null ? $"No entry possible: {firstError}" : $"No surviving pair under {_gLimitUsed:N0} g in this grid.");
                    ReentryResult least = null;
                    foreach (ReentryResult r in _results) if (least == null || Score(r) < Score(least)) least = r;
                    if (least != null && least.LimitingPart != null)
                        sb.AppendLine($"Closest: Pe {least.PeAltitude / 1000:N0} km at {least.AoADeg:N0}° - {Describe(least.Outcome)}, {least.PeakSkinFrac * 100:N0}% skin on {least.LimitingPart}, {least.PeakG:N1} g");
                }
                _summary = sb.ToString().TrimEnd();
            }
            catch (Exception ex)
            {
                _summary = $"Analysis failed: {ex.Message}";
                RP0Debug.LogError($"Reentry analysis failed: {ex}");
            }
            sw.Stop();
            _elapsedMs = sw.Elapsed.TotalMilliseconds;
            LastSummary = _summary;
            TopWindow.ResetSize();
        }

        // One entry with exactly the simulation's Reentry-mode inputs, written out row-for-row in the
        // telemetry logger's format so a flown entry and its prediction can be diffed directly.
        private void PredictSimSetup(List<Part> parts, CelestialBody body)
        {
            var sw = Stopwatch.StartNew();
            _results = null; _best = null;
            try
            {
                if (!FarReadyOrExplain()) return;
                ApplyPropellantLoad(parts);
                SimulationParams sp = SpaceCenterManagement.Instance?.SimulationParams;
                if (sp == null || sp.SimOrbitMode != SimOrbitMode.Reentry)
                {
                    _summary = "Set the simulation config to Reentry mode first (its periapsis, arrival and entry interface are what gets predicted).";
                    return;
                }
                double aoa = ParseOr(_sPredictAoA, 0.0);
                var k = ReentryConstants.FromGame(body);
                var solver = new ReentrySolver(body, k);
                var factory = new DefaultAeroFactory();
                if (FarEditorAero.Ready)
                {
                    ReentryVesselBuilder.CraftFrame(parts, out Vector3 cf, out Vector3 cu, out Vector3 cr);
                    double phi = FarEditorAero.FacilityRoll(cu);
                    RP0Debug.Log($"[ReentryAero] craft frame nose {cf}, roof {cu}; FAR roll {phi:N1} deg", true);
                    FarEditorAero.LogAlphaSweep(body, 25.0, 80000.0, phi);
                    // per-wing numbers at the flown condition, untrimmed and at full pitch-up, for the flight log's [ReentryWings]
                    double alpha = _attitude == 0 ? 180.0 - aoa : aoa;
                    FarEditorAero.LogWingBreakdown(body, 25.0, 80000.0, alpha, 0.0, phi);
                    FarEditorAero.LogWingBreakdown(body, 25.0, 80000.0, alpha, 1.0, phi);
                }
                ReentryVesselBuilder.DebugOcclusionMach = 25.0;   // log the engines' occluder chain for this one build
                ReentryAttitude att;
                try { att = ReentryVesselBuilder.Build(parts, body, k, aoa, _attitude == 0, factory); }
                finally { ReentryVesselBuilder.DebugOcclusionMach = -1.0; }
                _aeroSource = $"{factory.LastSource}, {(att.VoxelAreas ? "FAR voxel" : "drag cube")} areas, {solver.ThermalModelDescription} heating" + CryoNote(att);
                if (!solver.TryEntryState(sp.SimOrbitPe, sp.SimReentryArrival, sp.SimOrbitAp, sp.SimEntrySpeed, sp.SimEntryAltitude,
                                          out double v0, out double fpa, out string err))
                {
                    _summary = $"No entry possible: {err}";
                    return;
                }
                var trace = new List<ReentryTraceSample>();
                var partsTrace = new List<ReentryPartSample>();
                ReentryResult r = solver.Simulate(att, sp.SimEntryAltitude, v0, fpa, liftDown: false, trace: trace, partsTrace: partsTrace);

                string dir = $"{KSPUtil.ApplicationRootPath}saves/{HighLogic.SaveFolder}/RP-1_ReentryLogs";
                System.IO.Directory.CreateDirectory(dir);
                string ship = EditorLogic.fetch.ship.shipName ?? "vessel";
                string safe = string.Join("_", ship.Split(System.IO.Path.GetInvalidFileNameChars()));
                string path = System.IO.Path.Combine(dir, $"predicted_{safe}_{DateTime.Now:yyyyMMdd_HHmmss}.csv");
                LastPredictionPath = path;
                var inv = System.Globalization.CultureInfo.InvariantCulture;
                using (var w = new System.IO.StreamWriter(path, false))
                {
                    w.WriteLine(ReentryTelemetryLogger.HeaderLine);
                    foreach (ReentryTraceSample s in trace)
                    {
                        w.WriteLine(string.Join(",", new[] {
                            s.Time.ToString("G9", inv), s.Time.ToString("G9", inv), s.Altitude.ToString("G9", inv), s.Airspeed.ToString("G9", inv),
                            s.Mach.ToString("G9", inv), s.Density.ToString("G9", inv), s.DynPresKPa.ToString("G9", inv), s.GeeForce.ToString("G9", inv),
                            s.AtmosphericTemperature.ToString("G9", inv), s.ExternalTemperature.ToString("G9", inv), s.ConvectiveCoefficient.ToString("G9", inv),
                            s.Part, s.SkinTemperature.ToString("G9", inv), s.SkinMaxTemp.ToString("G9", inv), s.Temperature.ToString("G9", inv), s.MaxTemp.ToString("G9", inv),
                            s.ExposedArea.ToString("G9", inv), s.RadiativeArea.ToString("G9", inv), s.ConvectionFlux.ToString("G9", inv),
                            s.RadiationFlux.ToString("G9", inv), s.SkinToInternalFlux.ToString("G9", inv),
                            s.AoA.ToString("G9", inv), s.Cl.ToString("G9", inv), s.Cd.ToString("G9", inv), s.RefArea.ToString("G9", inv),
                            s.BackgroundTempExposed.ToString("G9", inv), s.TrimPitch.ToString("G9", inv),
                            s.Sideslip.ToString("G9", inv), s.LiftKN.ToString("G9", inv), s.DragKN.ToString("G9", inv), s.SideKN.ToString("G9", inv),
                            s.WingLiftKN.ToString("G9", inv), s.WingDragKN.ToString("G9", inv), "0", "0" }));
                    }
                }
                using (var w = new System.IO.StreamWriter(path.Replace(".csv", "") + ".parts.csv", false))
                {
                    w.WriteLine(ReentryPartSample.Header);
                    foreach (ReentryPartSample s in partsTrace)
                    {
                        w.WriteLine(string.Join(",", new[] {
                            s.Time.ToString("G9", inv), s.Time.ToString("G9", inv), s.Altitude.ToString("G9", inv), s.Part,
                            s.SkinTemperature.ToString("G9", inv), s.SkinMaxTemp.ToString("G9", inv), s.Temperature.ToString("G9", inv), s.MaxTemp.ToString("G9", inv),
                            s.SkinUnexposedTemperature.ToString("G9", inv),
                            s.ExposedArea.ToString("G9", inv), s.RadiativeArea.ToString("G9", inv), s.SkinExposedAreaFrac.ToString("G9", inv),
                            s.TempMult.ToString("G9", inv), s.CoeffMult.ToString("G9", inv), s.AreaMult.ToString("G9", inv),
                            s.ConvectionFlux.ToString("G9", inv), s.RadiationFlux.ToString("G9", inv), s.SkinToInternalFlux.ToString("G9", inv),
                            s.SkinThermalMass.ToString("G9", inv), s.ThermalMass.ToString("G9", inv),
                            s.HeatConvectiveConstant.ToString("G9", inv), s.EmissiveConstant.ToString("G9", inv),
                            s.Mass.ToString("G9", inv), s.ResourceMass.ToString("G9", inv) }));
                    }
                }
                _summary = $"Predicted (Pe {sp.SimOrbitPe / 1000:N0} km, entry {v0:N0} m/s at {fpa:N2}°, AoA {aoa:N0}° {(_attitude == 0 ? "shield first" : "nose first")}): {Describe(r.Outcome)} - " +
                           $"peak skin {r.PeakSkin:N0} K ({r.PeakSkinFrac * 100:N0}%) on {r.LimitingPart}, {r.PeakG:N1} g, {r.Duration:N0} s.\n" +
                           $"Wrote {trace.Count} rows to {System.IO.Path.GetFileName(path)} (+ .parts.csv)" + (factory.TrimSummary.Length > 0 ? $"\n{factory.TrimSummary}" : "") +
                           "\nParts carry the flight thermal highlight at their peak (red from 40% of the limit, orange at it); parts that went over it pulse red, shown through whatever hides them. Hover a part for its numbers." +
                           PartLimitsReport(att);
                ReentryHeatHighlight.Apply(att);
            }
            catch (Exception ex)
            {
                _summary = $"Prediction failed: {ex.Message}";
                RP0Debug.LogError($"Reentry prediction failed: {ex}");
            }
            sw.Stop();
            _elapsedMs = sw.Elapsed.TotalMilliseconds;
            LastSummary = _summary;
            TopWindow.ResetSize();
        }

        // Which way round the vehicle enters is a property of the vehicle, not of the building it was
        // built in: anything with wings or control surfaces flies nose first; a heat shield with no
        // wings goes shield first. Only fall back to the facility when neither says anything.
        private static int DefaultAttitude(List<Part> parts)
        {
            bool wings = false, shield = false;
            foreach (Part p in parts)
            {
                foreach (PartModule m in p.Modules)
                {
                    string n = m.moduleName;
                    if (m is ModuleLiftingSurface || m is ModuleControlSurface || n == "FARWingAerodynamicModel" || n == "FARControllableSurface")
                        wings = true;
                    else if (m is ModuleAblator)
                        shield = true;
                }
            }
            if (wings) return 1;
            if (shield) return 0;
            return EditorDriver.editorFacility == EditorFacility.VAB ? 0 : 1;
        }

        // Lower is better: survival first, then thermal margin, then gentleness.
        private double Score(ReentryResult r)
        {
            double frac = Math.Max(r.PeakSkinFrac, r.PeakInternalFrac);
            if (!r.Acceptable(_gLimitUsed))
                return 1e6 + (r.Outcome == ReentryOutcome.SkippedOut ? 500.0 : 0.0) + frac * 1000.0 + Math.Max(0.0, r.PeakG - _gLimitUsed) * 10.0;
            return frac * 100.0 + r.PeakG / Math.Max(_gLimitUsed, 1.0) * 25.0;
        }

        private static double ParseOr(string s, double fallback)
        {
            return double.TryParse(s, out double v) ? v : fallback;
        }
    }
}
