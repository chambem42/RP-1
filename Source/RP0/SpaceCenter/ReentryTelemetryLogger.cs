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
            "aoa,cl,cd,refArea,brtExposed";

        // FAR's flight readouts, by reflection (FARAPI is public static). Optional: blank columns without FAR.
        private static bool _farResolved;
        private static System.Reflection.MethodInfo _farAoA, _farCl, _farCd, _farRefArea;

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
            Append(sb, ut);
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
            Append(sb, brtExposed, last: true);

            try
            {
                _writer.WriteLine(sb.ToString());
                _wroteAnything = true;
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
