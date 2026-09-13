using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

namespace RP0
{
    /// <summary>
    /// Dumps the heating state of a simulated reentry to a CSV, one row per sample.
    /// <para>
    /// This exists to check a predicted entry against a flown one. The editor-side reentry
    /// analysis has to reimplement the stock <c>FlightIntegrator</c> thermal chain, and a
    /// reimplementation is only worth anything if someone can hold it up against what the game
    /// actually does. Flying the same entry in reentry mode with this switched on produces the
    /// reference curve: freestream conditions, the shock temperature and convective coefficient
    /// the game derived from them, and the resulting flux and temperature on the hottest part.
    /// </para>
    /// <para>Only runs during a simulated flight that asked for it, and only inside the atmosphere.</para>
    /// </summary>
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class ReentryTelemetryLogger : MonoBehaviour
    {
        private const double SampleInterval = 0.5d;
        public const string HeaderLine =
            "ut,missionTime,altitude,srfSpeed,mach,atmDensity,dynPreskPa,geeForce," +
            "atmosphericTemperature,externalTemperature,convectiveCoefficient," +
            "part,skinTemperature,skinMaxTemp,temperature,maxTemp," +
            "exposedArea,radiativeArea,convectionFlux,radiationFlux,skinToInternalFlux," +
            "aoa,cl,cd,refArea,brtExposed,trimPitch," +
            "sideslip,liftKN,dragKN,sideKN,wingLiftKN,wingDragKN";

        // FAR's flight readouts, by reflection (FARAPI is public static). Optional: blank columns without FAR.
        private static bool _farResolved;
        private static System.Reflection.MethodInfo _farAoA, _farCl, _farCd, _farRefArea, _farSideslip;
        // Per-part aero forces: the voxel body's (FARAeroPartModule.worldSpaceAeroForce) and the wing model's
        // (FARWingAerodynamicModel.worldSpaceForce), so lift can be split the way the editor sim reports it.
        private static Type _farAeroModuleType, _farWingType;
        private static System.Reflection.FieldInfo _farBodyForce, _farWingForce;

        private StreamWriter _writer;
        private string _path;
        private double _nextSampleUT;
        private bool _active;
        private bool _wroteAnything;

        public void Start()
        {
            SpaceCenterManagement scm = SpaceCenterManagement.Instance;
            SimulationParams simParams = scm?.SimulationParams;
            _active = scm != null && scm.IsSimulatedFlight && simParams != null
                      && simParams.SimOrbitMode == SimOrbitMode.Reentry && simParams.LogReentryTelemetry;
            if (!_active)
                enabled = false;
        }

        public void FixedUpdate()
        {
            if (!_active) return;

            Vessel v = FlightGlobals.ActiveVessel;
            if (v == null || !v.loaded || v.packed) return;

            // Nothing interesting happens outside the atmosphere, and logging a coast to the
            // interface would bury the part of the trace that matters.
            if (v.atmDensity <= 0d) return;
            // Not while still sitting on the pad waiting to be moved onto the entry trajectory.
            if (!SpaceCenterManagement.Instance.SimulationParams.IsVesselMoved) return;

            double ut = Planetarium.GetUniversalTime();
            if (ut < _nextSampleUT) return;
            _nextSampleUT = ut + SampleInterval;

            Part hottest = FindHottestPart(v.parts);
            if (hottest == null) return;

            if (_writer == null && !TryOpen(v))
            {
                _active = false;
                enabled = false;
                return;
            }

            // Prefer the integrator's own values: mods hooked through ModularFlightIntegrator (RealHeat) rewrite
            // the FI fields after the vessel copies were taken, so the vessel's are the pre-hook numbers.
            FlightIntegrator fi = FlightIntegrator.ActiveVesselFI;
            bool haveFI = fi != null && fi.Vessel == v;
            double extT = haveFI ? fi.externalTemperature : v.externalTemperature;
            double convCoeff = haveFI ? fi.convectiveCoefficient : v.convectiveCoefficient;
            double brtExposed = haveFI ? fi.backgroundRadiationTempExposed : double.NaN;

            var sb = new StringBuilder(320);
            sb.Append(ut.ToString("R", CultureInfo.InvariantCulture)).Append(',');   // UT needs more than 9 digits
            Append(sb, v.missionTime);
            Append(sb, v.altitude);
            Append(sb, v.srfSpeed);
            Append(sb, v.mach);
            Append(sb, v.atmDensity);
            Append(sb, v.dynamicPressurekPa);
            Append(sb, v.geeForce);
            Append(sb, v.atmosphericTemperature);
            Append(sb, extT);
            Append(sb, convCoeff);
            sb.Append(hottest.partInfo != null ? hottest.partInfo.name : hottest.name).Append(',');
            Append(sb, hottest.skinTemperature);
            Append(sb, hottest.skinMaxTemp);
            Append(sb, hottest.temperature);
            Append(sb, hottest.maxTemp);
            Append(sb, hottest.exposedArea);
            Append(sb, hottest.radiativeArea);
            Append(sb, hottest.thermalConvectionFlux);
            Append(sb, hottest.thermalRadiationFlux);
            Append(sb, hottest.skinToInternalFlux);
            Append(sb, FarValue(_farAoA, v));
            Append(sb, FarValue(_farCl, v));
            Append(sb, FarValue(_farCd, v));
            Append(sb, FarValue(_farRefArea, v));
            Append(sb, brtExposed);
            Append(sb, v.ctrlState != null ? v.ctrlState.pitch : double.NaN);   // pitch input incl. SAS, for trim comparison
            Append(sb, FarValue(_farSideslip, v));
            AeroForceBreakdown(v, out double lift, out double drag, out double side, out double wingLift, out double wingDrag);
            Append(sb, lift); Append(sb, drag); Append(sb, side); Append(sb, wingLift); Append(sb, wingDrag, last: true);

            try
            {
                _writer.WriteLine(sb.ToString());
                _wroteAnything = true;
                WriteParts(v, ut, haveFI);
                if (v.missionTime >= _nextWingLog)
                {
                    _nextWingLog = v.missionTime + 30.0;
                    LogWings(v);
                }
            }
            catch (Exception ex)
            {
                RP0Debug.LogError($"Failed writing reentry telemetry: {ex.Message}");
                _active = false;
                enabled = false;
            }
        }

        public void OnDestroy()
        {
            if (_partsWriter != null)
            {
                try { _partsWriter.Dispose(); } catch (Exception) { }
                _partsWriter = null;
            }
            if (_writer == null) return;
            try
            {
                _writer.Flush();
                _writer.Dispose();
                if (_wroteAnything)
                    RP0Debug.Log($"Wrote reentry telemetry to {_path}");
            }
            catch (Exception ex)
            {
                RP0Debug.LogError($"Failed closing reentry telemetry: {ex.Message}");
            }
            _writer = null;
        }

        // Per-part companion log: the same thermal inputs the editor analysis has to reconstruct (occlusion
        // multipliers, exposed/radiative areas, skin fraction) for the hottest few parts plus every engine,
        // so a per-part disagreement can be traced to a specific input rather than guessed at.
        private const int PartsHottest = 4;
        private StreamWriter _partsWriter;
        private readonly List<Part> _partsPick = new List<Part>();

        private void WriteParts(Vessel v, double ut, bool haveFI)
        {
            if (_partsWriter == null)
            {
                try
                {
                    _partsWriter = new StreamWriter(_path.Replace(".csv", "") + ".parts.csv", false) { AutoFlush = true };
                    _partsWriter.WriteLine(Reentry.ReentryPartSample.Header);
                }
                catch (Exception ex)
                {
                    RP0Debug.LogError($"Could not open reentry parts log: {ex.Message}");
                    _partsWriter = null;
                    return;
                }
            }

            _partsPick.Clear();
            var sorted = new List<Part>(v.parts);
            sorted.Sort((a, b) => Frac(b).CompareTo(Frac(a)));
            for (int i = 0; i < sorted.Count && _partsPick.Count < PartsHottest; ++i) _partsPick.Add(sorted[i]);
            foreach (Part p in v.parts)
                if (!_partsPick.Contains(p) && p.FindModuleImplementing<ModuleEngines>() != null) _partsPick.Add(p);

            var inv = CultureInfo.InvariantCulture;
            foreach (Part p in _partsPick)
            {
                PartThermalData ptd = p.ptd;
                var sb = new StringBuilder(320);
                sb.Append(ut.ToString("R", inv)).Append(',');
                Append(sb, v.missionTime);
                Append(sb, v.altitude);
                sb.Append(p.partInfo != null ? p.partInfo.name : p.name).Append(',');
                Append(sb, p.skinTemperature); Append(sb, p.skinMaxTemp); Append(sb, p.temperature); Append(sb, p.maxTemp);
                Append(sb, p.skinUnexposedTemperature);
                Append(sb, p.exposedArea); Append(sb, p.radiativeArea); Append(sb, p.skinExposedAreaFrac);
                Append(sb, ptd != null ? ptd.convectionTempMultiplier : double.NaN);
                Append(sb, ptd != null ? ptd.convectionCoeffMultiplier : double.NaN);
                Append(sb, ptd != null ? ptd.convectionAreaMultiplier : double.NaN);
                Append(sb, p.thermalConvectionFlux); Append(sb, p.thermalRadiationFlux); Append(sb, p.skinToInternalFlux);
                Append(sb, p.skinThermalMass); Append(sb, p.thermalMass);
                Append(sb, p.heatConvectiveConstant); Append(sb, p.emissiveConstant, last: true);
                _partsWriter.WriteLine(sb.ToString());
            }
        }

        private static double Frac(Part p)
        {
            double f = p.skinMaxTemp > 0d ? p.skinTemperature / p.skinMaxTemp : 0d;
            return p.maxTemp > 0d ? Math.Max(f, p.temperature / p.maxTemp) : f;
        }

        /// <summary>The part closest to its own skin limit - the one that decides whether the entry is survivable.</summary>
        private static Part FindHottestPart(List<Part> parts)
        {
            if (parts == null) return null;
            Part best = null;
            double bestFrac = double.NegativeInfinity;
            for (int i = 0; i < parts.Count; ++i)
            {
                Part p = parts[i];
                if (p == null || p.skinMaxTemp <= 0d) continue;
                double frac = p.skinTemperature / p.skinMaxTemp;
                if (p.maxTemp > 0d)
                    frac = Math.Max(frac, p.temperature / p.maxTemp);
                if (frac > bestFrac)
                {
                    bestFrac = frac;
                    best = p;
                }
            }
            return best;
        }

        private bool TryOpen(Vessel v)
        {
            try
            {
                string dir = $"{KSPUtil.ApplicationRootPath}saves/{HighLogic.SaveFolder}/RP-1_ReentryLogs";
                Directory.CreateDirectory(dir);
                string safeName = string.Join("_", (v.vesselName ?? "vessel").Split(Path.GetInvalidFileNameChars()));
                _path = Path.Combine(dir, $"{safeName}_{DateTime.Now:yyyyMMdd_HHmmss}.csv");
                _writer = new StreamWriter(_path, false);
                _writer.AutoFlush = true;
                _writer.WriteLine(HeaderLine);
                return true;
            }
            catch (Exception ex)
            {
                RP0Debug.LogError($"Could not open reentry telemetry log: {ex.Message}");
                _writer = null;
                return false;
            }
        }

        private static void Append(StringBuilder sb, double value, bool last = false)
        {
            if (!double.IsNaN(value)) sb.Append(value.ToString("G9", CultureInfo.InvariantCulture));
            if (!last) sb.Append(',');
        }

        private static double FarValue(System.Reflection.MethodInfo m, Vessel v)
        {
            if (!_farResolved) ResolveFar();
            if (m == null) return double.NaN;
            try { return Convert.ToDouble(m.Invoke(null, new object[] { v })); }
            catch { return double.NaN; }
        }

        // Per-wing breakdown (FARWingAerodynamicModel: area, MAC, Cl, Cd, stall fraction, force) every 30 s of
        // mission time, to hold against FarEditorAero.LogWingBreakdown for the same craft and condition.
        private double _nextWingLog;

        private void LogWings(Vessel v)
        {
            if (_farWingType == null) return;
            try
            {
                Vector3 velNorm = v.srf_velocity.normalized;
                Vector3 liftDir = Vector3.ProjectOnPlane(-v.ReferenceTransform.forward, velNorm).normalized;
                var sb = new StringBuilder(512);
                sb.Append($"[ReentryWings] flight alt {v.altitude / 1000:N1} km, Mach {v.mach:N1}, q {v.dynamicPressurekPa:N3} kPa, AoA {FarValue(_farAoA, v):N1}:");
                foreach (Part p in v.parts)
                    foreach (PartModule m in p.Modules)
                    {
                        if (!_farWingType.IsInstanceOfType(m)) continue;
                        Vector3 f = (Vector3)_farWingForce.GetValue(m);
                        sb.Append($"\n  {p.partInfo.name}: S {WingNum(m, "S"):N2} MAC {WingNum(m, "GetMAC"):N2} Cl {WingNum(m, "GetCl"):N3} Cd {WingNum(m, "GetCd"):N3} stall {WingNum(m, "GetStall"):N2}" +
                                  $" lift {Vector3.Dot(f, liftDir):N2} drag {-Vector3.Dot(f, velNorm):N2} kN");
                    }
                RP0Debug.Log(sb.ToString(), true);
            }
            catch (Exception ex)
            {
                RP0Debug.LogWarning($"wing breakdown failed: {ex.Message}");
                _nextWingLog = double.MaxValue;
            }
        }

        private static double WingNum(PartModule m, string member)
        {
            const System.Reflection.BindingFlags any = System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance;
            Type t = m.GetType();
            var f = t.GetField(member, any);
            if (f != null) return Convert.ToDouble(f.GetValue(m));
            var mi = t.GetMethod(member, any, null, Type.EmptyTypes, null);
            return mi != null ? Convert.ToDouble(mi.Invoke(m, null)) : double.NaN;
        }

        // Sum FAR's per-part forces (kN) and resolve them the way the editor sim does: drag along the
        // airspeed, lift perpendicular to it in the symmetry plane (the roof side positive), side out of
        // that plane. FAR's own flight Cl is the roof-axis component instead, cos(AoA) short of this.
        private static void AeroForceBreakdown(Vessel v, out double lift, out double drag, out double side, out double wingLift, out double wingDrag)
        {
            lift = drag = side = wingLift = wingDrag = double.NaN;
            if (!_farResolved) ResolveFar();
            if (_farBodyForce == null || v.ReferenceTransform == null) return;
            Vector3 vel = v.srf_velocity;
            if (vel.sqrMagnitude < 1e-6) return;
            Vector3 velNorm = vel.normalized;
            Vector3 liftDir = Vector3.ProjectOnPlane(-v.ReferenceTransform.forward, velNorm);
            if (liftDir.sqrMagnitude < 1e-6) liftDir = Vector3.ProjectOnPlane(v.ReferenceTransform.up, velNorm);
            liftDir.Normalize();
            Vector3 sideDir = Vector3.Cross(velNorm, liftDir);

            Vector3 body = Vector3.zero, wing = Vector3.zero;
            try
            {
                foreach (Part p in v.parts)
                    foreach (PartModule m in p.Modules)
                    {
                        if (_farAeroModuleType.IsInstanceOfType(m)) body += (Vector3)_farBodyForce.GetValue(m);
                        else if (_farWingType != null && _farWingType.IsInstanceOfType(m)) wing += (Vector3)_farWingForce.GetValue(m);
                    }
            }
            catch { return; }
            Vector3 total = body + wing;
            lift = Vector3.Dot(total, liftDir);
            drag = -Vector3.Dot(total, velNorm);
            side = Vector3.Dot(total, sideDir);
            wingLift = Vector3.Dot(wing, liftDir);
            wingDrag = -Vector3.Dot(wing, velNorm);
        }

        private static void ResolveFar()
        {
            _farResolved = true;
            try
            {
                foreach (AssemblyLoader.LoadedAssembly la in AssemblyLoader.loadedAssemblies)
                {
                    if (la.name != "FerramAerospaceResearch") continue;
                    Type api = la.assembly.GetType("FerramAerospaceResearch.FARAPI");
                    if (api == null) break;
                    var flags = System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Static;
                    var sig = new[] { typeof(Vessel) };
                    _farAoA = api.GetMethod("VesselAoA", flags, null, sig, null);
                    _farCl = api.GetMethod("VesselLiftCoeff", flags, null, sig, null);
                    _farCd = api.GetMethod("VesselDragCoeff", flags, null, sig, null);
                    _farRefArea = api.GetMethod("VesselRefArea", flags, null, sig, null);
                    _farSideslip = api.GetMethod("VesselSideslip", flags, null, sig, null);
                    var pub = System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Instance;
                    _farAeroModuleType = la.assembly.GetType("FerramAerospaceResearch.FARAeroComponents.FARAeroPartModule");
                    _farBodyForce = _farAeroModuleType?.GetField("worldSpaceAeroForce", pub);
                    _farWingType = la.assembly.GetType("ferram4.FARWingAerodynamicModel");
                    _farWingForce = _farWingType?.GetField("worldSpaceForce", pub);
                    if (_farBodyForce == null) _farAeroModuleType = null;
                    if (_farWingForce == null) _farWingType = null;
                    break;
                }
            }
            catch (Exception ex)
            {
                RP0Debug.LogWarning($"FARAPI unavailable to the reentry telemetry: {ex.Message}");
            }
        }
    }
}
