using System;
using System.Collections.Generic;
using System.Reflection;

namespace RP0
{
    /// <summary>
    /// FAR's reentry analysis (FerramAerospaceResearch.Reentry.FARReentryAPI, chambm's fork), reached by
    /// reflection so RP-1 keeps no hard reference to FAR: the touchdown prediction that aims a simulated
    /// entry at a landing site, the propellant load that puts the simulated vessel at the analysed weight,
    /// and the hook the editor tab's "Set up simulation" button calls into KCT. Everything degrades to a
    /// no-op with a logged reason when that FAR is not installed.
    /// </summary>
    public static class FARReentryInterop
    {
        private static bool _resolved;
        private static Type _api;
        private static MethodInfo _applyPropellant, _predictTouchdown, _predictFromTop;
        private static PropertyInfo _ready, _approachSpeed, _inclination;
        private static bool _predictTakesInclination;
        private static FieldInfo _setupSimulation;

        public static bool Available
        {
            get
            {
                if (!_resolved) Resolve();
                return _api != null;
            }
        }

        private static void Resolve()
        {
            _resolved = true;
            try
            {
                foreach (AssemblyLoader.LoadedAssembly la in AssemblyLoader.loadedAssemblies)
                {
                    if (la.name != "FerramAerospaceResearch") continue;
                    Type api = la.assembly.GetType("FerramAerospaceResearch.Reentry.FARReentryAPI");
                    if (api == null) break;
                    const BindingFlags any = BindingFlags.Public | BindingFlags.Static;
                    _applyPropellant = api.GetMethod("ApplyPropellantLoad", any);
                    _predictTouchdown = api.GetMethod("TryPredictTouchdown", any);
                    // Newer builds take the orbit's inclination after the entry altitude (13 parameters, 12 before).
                    _predictTakesInclination = _predictTouchdown != null && _predictTouchdown.GetParameters().Length == 13;
                    _predictFromTop = api.GetMethod("TryPredictFromTop", any);
                    _ready = api.GetProperty("Ready", any);
                    _approachSpeed = api.GetProperty("ApproachSpeed", any);
                    _inclination = api.GetProperty("InclinationDeg", any);
                    _setupSimulation = api.GetField("SetupSimulation", any);
                    if (_applyPropellant != null && _predictTouchdown != null) _api = api;
                    break;
                }
            }
            catch (Exception ex)
            {
                RP0Debug.LogWarning($"FAR reentry API unavailable: {ex.Message}");
                _api = null;
            }
            if (_api == null) RP0Debug.Log("FAR reentry API not found; simulated entries aim their periapsis and keep the craft's propellant load", true);
        }

        /// <summary>True in the editor once FAR has a voxelization to analyse.</summary>
        public static bool Ready => Available && _ready != null && _ready.GetValue(null, null) is bool b && b;

        /// <summary>The inclination (deg) entered on FAR's Reentry tab; NaN when FAR does not report one.</summary>
        public static double InclinationDeg => Available && _inclination != null ? Convert.ToDouble(_inclination.GetValue(null, null)) : double.NaN;

        /// <summary>Airspeed below which a nose-first touchdown prediction flies at 0 deg AoA (1000 m/s without FAR to ask).</summary>
        public static double ApproachSpeed => Available && _approachSpeed != null ? Convert.ToDouble(_approachSpeed.GetValue(null, null)) : 1000.0;

        /// <summary>Sets each engine propellant on the parts to percent of capacity; returns the number changed (0 without FAR).</summary>
        public static int ApplyPropellantLoad(IEnumerable<Part> parts, double percent)
        {
            if (!Available) return 0;
            try { return Convert.ToInt32(_applyPropellant.Invoke(null, new object[] { parts, percent })); }
            catch (Exception ex)
            {
                RP0Debug.LogWarning($"FAR propellant load failed: {ex.InnerException?.Message ?? ex.Message}");
                return 0;
            }
        }

        /// <summary>
        /// Where the editor craft's entry touches down: the in-plane angle past periapsis and the seconds after the
        /// starting altitude. The inclination (deg) sets how much of the ground's rotation the airspeed sees (a FAR
        /// build without that parameter assumes a prograde equatorial entry). False (with a reason) when FAR cannot
        /// predict it; the caller aims the periapsis then.
        /// </summary>
        public static bool TryPredictTouchdown(CelestialBody body, double peAlt, bool arrivalIsApoapsis, double apAlt, double entrySpeed, double entryAlt,
                                               double inclinationDeg, double aoaDeg, bool shieldFirst,
                                               out double angleFromPeDeg, out double secondsFromInterface, out double downrangeKm, out string error)
        {
            angleFromPeDeg = secondsFromInterface = downrangeKm = double.NaN;
            error = null;
            if (!Available) { error = "FAR's reentry analysis is not installed"; return false; }
            try
            {
                object[] args = _predictTakesInclination
                    ? new object[] { body, peAlt, arrivalIsApoapsis, apAlt, entrySpeed, entryAlt, inclinationDeg, aoaDeg, shieldFirst, null, null, null, null }
                    : new object[] { body, peAlt, arrivalIsApoapsis, apAlt, entrySpeed, entryAlt, aoaDeg, shieldFirst, null, null, null, null };
                bool ok = (bool)_predictTouchdown.Invoke(null, args);
                int o = args.Length - 4;
                angleFromPeDeg = Convert.ToDouble(args[o]);
                secondsFromInterface = Convert.ToDouble(args[o + 1]);
                downrangeKm = Convert.ToDouble(args[o + 2]);
                error = args[o + 3] as string;
                return ok;
            }
            catch (Exception ex)
            {
                error = ex.InnerException?.Message ?? ex.Message;
                RP0Debug.LogWarning($"FAR touchdown prediction failed: {error}");
                return false;
            }
        }

        /// <summary>
        /// The entry flown from the top of the atmosphere: the inertial speed and flight path angle where it crosses
        /// startAlt, and the touchdown from that crossing (angle flown, seconds; NaN when it does not come down).
        /// False when FAR cannot predict it (or is too old to have this).
        /// </summary>
        public static bool TryPredictFromTop(CelestialBody body, double peAlt, bool arrivalIsApoapsis, double apAlt, double entrySpeed,
                                             double inclinationDeg, double aoaDeg, bool shieldFirst, double startAlt,
                                             out double speedAtStart, out double fpaAtStartDeg, out double angleStartToTouchdownDeg, out double secondsStartToTouchdown, out string error)
        {
            speedAtStart = fpaAtStartDeg = angleStartToTouchdownDeg = secondsStartToTouchdown = double.NaN;
            error = null;
            if (!Available || _predictFromTop == null) { error = "FAR's reentry analysis is not installed (or predates the start-state prediction)"; return false; }
            try
            {
                object[] args = { body, peAlt, arrivalIsApoapsis, apAlt, entrySpeed, inclinationDeg, aoaDeg, shieldFirst, startAlt, null, null, null, null, null };
                bool ok = (bool)_predictFromTop.Invoke(null, args);
                speedAtStart = Convert.ToDouble(args[9]);
                fpaAtStartDeg = Convert.ToDouble(args[10]);
                angleStartToTouchdownDeg = Convert.ToDouble(args[11]);
                secondsStartToTouchdown = Convert.ToDouble(args[12]);
                error = args[13] as string;
                return ok;
            }
            catch (Exception ex)
            {
                error = ex.InnerException?.Message ?? ex.Message;
                RP0Debug.LogWarning($"FAR start-state prediction failed: {error}");
                return false;
            }
        }

        /// <summary>
        /// Registers the handler FAR's Reentry tab calls from its "Set up simulation" button:
        /// (periapsis m, arrival is apoapsis, apoapsis m, entry speed m/s, shield first, AoA deg, propellant percent or -1).
        /// </summary>
        public static void RegisterSimulationSetup(Action<double, bool, double, double, bool, double, double> handler)
        {
            if (!Available || _setupSimulation == null) return;
            try { _setupSimulation.SetValue(null, handler); }
            catch (Exception ex) { RP0Debug.LogWarning($"FAR reentry setup hook failed: {ex.Message}"); }
        }
    }
}
