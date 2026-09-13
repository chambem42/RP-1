using System;
using System.Collections.Generic;
using System.Reflection;
using UnityEngine;

namespace RP0.Reentry
{
    /// <summary>
    /// Stock aerodynamics from the parts' drag cubes, sampled at the builder's Mach grid for the attitude.
    /// Drag as FlightIntegrator.CalculateDragValue (AreaDrag * DragCubeMultiplier), body lift as ApplyAeroLift.
    /// Wing lift (ModuleLiftingSurface) is not modelled; RP-1 flies FAR, where this is only the fallback.
    /// </summary>
    public class DragCubeAero : IReentryAero
    {
        private readonly double[] _mach, _dragArea, _liftArea;
        private readonly double _dragMult, _bodyLiftMult;

        public string Description => "stock drag cubes";
        public double ReferenceArea => 1.0;
        public double LastTrimPitch => double.NaN;
        public double LastWingLiftKN => double.NaN;
        public double LastWingDragKN => double.NaN;
        public double LastAoA => double.NaN;

        public DragCubeAero(List<ReentryNode> nodes, Vector3 velDir, Vector3 liftAxis, ReentryConstants k)
        {
            _mach = ReentryVesselBuilder.MachSamples;
            _dragArea = new double[_mach.Length];
            _liftArea = new double[_mach.Length];
            _dragMult = k.DragCubeMultiplier * k.DragMultiplier;
            _bodyLiftMult = k.BodyLiftMultiplier;

            for (int m = 0; m < _mach.Length; ++m)
            {
                float mach = (float)_mach[m];
                double drag = 0.0, lift = 0.0;
                foreach (ReentryNode n in nodes)
                {
                    Part p = n.Part;
                    if (p.ShieldedFromAirstream) continue;
                    Vector3 dirLocal = -p.partTransform.InverseTransformDirection(velDir);
                    p.DragCubes.SetDrag(dirLocal, mach);
                    drag += p.DragCubes.AreaDrag;
                    if (!p.hasLiftModule)
                    {
                        Vector3 lf = p.transform.rotation * (p.DragCubes.LiftForce * p.bodyLiftMultiplier);
                        lf = Vector3.ProjectOnPlane(lf, velDir);
                        lift += Vector3.Dot(lf, liftAxis);
                    }
                }
                _dragArea[m] = drag;
                _liftArea[m] = lift;
            }
        }

        public void Forces(double dynPresKPa, double mach, double altitude, double pseudoReDragMult, out double dragKN, out double liftKN)
        {
            int i = Index(mach, out double f);
            double dA = _dragArea[i] + (i + 1 < _mach.Length ? (_dragArea[i + 1] - _dragArea[i]) * f : 0.0);
            double lA = _liftArea[i] + (i + 1 < _mach.Length ? (_liftArea[i + 1] - _liftArea[i]) * f : 0.0);
            dragKN = dynPresKPa * dA * _dragMult * pseudoReDragMult;
            liftKN = dynPresKPa * lA * _bodyLiftMult * PhysicsGlobals.BodyLiftCurve.liftMachCurve.Evaluate((float)mach);
        }

        private int Index(double mach, out double f)
        {
            if (mach <= _mach[0]) { f = 0.0; return 0; }
            int last = _mach.Length - 1;
            if (mach >= _mach[last]) { f = 0.0; return last; }
            int i = 0;
            while (i < last - 1 && _mach[i + 1] < mach) ++i;
            f = (mach - _mach[i]) / (_mach[i + 1] - _mach[i]);
            return i;
        }
    }

    /// <summary>
    /// FAR's editor aerodynamics, reached by reflection so RP-1 keeps no hard reference to FAR:
    /// FerramAerospaceResearch.FARGUI.FAREditorGUI.EditorGUI.Instance -> _instantSim (InstantConditionSim)
    /// -> GetClCdCmSteady(InstantConditionSimInput, out InstantConditionSimOutput, bool, bool). Cl/Cd come
    /// back referenced to the wing area, or to the body's maximum cross-section for a wingless vehicle.
    /// <para>
    /// Stock FAR runs that sim in continuum flow (pseudo-Knudsen 0, a fixed Reynolds), which overstates
    /// lift in the 70-95 km band where the heating happens. A FAR build with InstantConditionSim.AeroCondition
    /// and SimAeroCondition.ForFlightPoint (chambm's fork) lets each sample be taken at the flight-point
    /// Reynolds/Knudsen; when that is present, Cl/Cd are sampled over Mach x altitude instead of Mach alone.
    /// </para>
    /// </summary>
    public class FarEditorAero : IReentryAero
    {
        private static bool _resolved, _available;
        private static Type _editorGuiType, _inputType, _outputType, _simType;
        private static PropertyInfo _instanceProp, _readyProp, _aeroConditionProp;
        private static FieldInfo _instantSimField, _wingListField, _maxCrossSectionField, _bodyLengthField;
        private static FieldInfo _inAlpha, _inPhi, _inMach, _inPitch, _outCl, _outCd, _outCm, _outWingCl, _outWingCd;
        private static MethodInfo _getClCdCm, _forFlightPoint;
        private static FieldInfo _wingAreaField;

        /// <summary>Altitudes (m) the flight-condition sampling is taken at, when supported.</summary>
        public static readonly double[] AltitudeSamples = { 20000, 40000, 55000, 65000, 75000, 85000, 95000, 110000, 140000 };

        private readonly double[] _mach, _alt;
        private readonly double[,] _cl, _cd, _trimPitch;   // [alt, mach]; a single altitude row when flight conditions are unsupported
        private readonly double[,] _wingCl, _wingCd;       // the wing model's share of _cl/_cd (fork output), else NaN
        private readonly double[,] _alpha;                 // FAR alpha actually held at each point (trim-limited where the elevator saturates)
        private readonly double _refArea;
        private readonly double _liftSign;
        private readonly bool _flightConditions, _shieldFirst;
        private int _untrimmable;
        private double _maxTrimPitch;
        private double _lastWingCl, _lastWingCd;           // from the most recent EvalCm

        public string Description => (_flightConditions ? "FAR editor aero at flight Re/Kn" : "FAR editor aero (continuum)") + ", trimmed";

        /// <summary>Largest pitch control input (0..1 of full deflection) the trim needed anywhere on the grid.</summary>
        public double MaxTrimPitch => _maxTrimPitch;
        /// <summary>Number of grid points where full control deflection could not zero the pitching moment.</summary>
        public int UntrimmableSamples => _untrimmable;
        public int TotalSamples => _alt.Length * _mach.Length;
        public double LastTrimPitch { get; private set; }
        public double LastWingLiftKN { get; private set; } = double.NaN;
        public double LastWingDragKN { get; private set; } = double.NaN;
        public double LastAoA { get; private set; } = double.NaN;

        // Trim: find the pitch control input that zeroes Cm at this alpha/Mach/altitude and take Cl/Cd
        // there. Holding an angle of attack costs control-surface deflection, and on a lifting body
        // the deflected surfaces take a large bite out of the lift the untrimmed sim reports. Cm is
        // close to linear in the input, so two secant steps are enough; if the root falls outside
        // full deflection the point is flagged and the nearest endpoint is used.
        private void SampleTrimmed(object sim, object input, bool clear, out double cl, out double cd, out double trim, out bool trimmed)
        {
            double p0 = 0.0, c0 = EvalCm(sim, input, p0, clear, out cl, out cd);
            double cmUntrimmed = Math.Abs(c0);
            trim = 0.0; trimmed = true;
            if (_inPitch == null || _outCm == null || cmUntrimmed < 1e-6) return;

            double p1 = c0 > 0.0 ? -0.5 : 0.5;
            double c1 = EvalCm(sim, input, p1, false, out double cl1, out double cd1);
            for (int i = 0; i < 3; ++i)
            {
                if (Math.Abs(c1 - c0) < 1e-9) break;
                double p2 = p1 - c1 * (p1 - p0) / (c1 - c0);
                if (p2 > 1.0) p2 = 1.0; else if (p2 < -1.0) p2 = -1.0;
                double c2 = EvalCm(sim, input, p2, false, out cl1, out cd1);
                p0 = p1; c0 = c1; p1 = p2; c1 = c2;
                if (Math.Abs(c1) < 1e-4 || (Math.Abs(p1) >= 1.0 && i > 0)) break;
            }
            trim = p1; cl = cl1; cd = cd1;
            // Untrimmable when the best we could do sits at full deflection with the moment still well off zero.
            trimmed = !(Math.Abs(p1) >= 1.0 && Math.Abs(c1) > 0.05 * Math.Max(cmUntrimmed, 1e-3));
            _inPitch.SetValue(input, 0.0);
        }

        // When full deflection cannot hold the requested angle, the vehicle does not hold it: it settles
        // where the moment at the stop is zero. Flight shows exactly that (AoA sagging from 42 to 39 deg
        // with the elevator pinned), so find that angle by secant on alpha at the stop and take its Cl/Cd.
        // Returns the achieved alpha, or the requested one if no root is found within reach.
        private double SolveAlphaAtStop(object sim, object input, double alphaReq, double stop, out double cl, out double cd)
        {
            _inPitch.SetValue(input, stop);
            double a0 = alphaReq, c0 = EvalCmAtAlpha(sim, input, a0, out cl, out cd);
            double a1 = alphaReq - Math.Sign(alphaReq == 0.0 ? 1.0 : alphaReq) * 4.0;
            double c1 = EvalCmAtAlpha(sim, input, a1, out double cl1, out double cd1);
            double best = alphaReq;
            for (int i = 0; i < 6; ++i)
            {
                if (Math.Abs(c1 - c0) < 1e-9) break;
                double a2 = a1 - c1 * (a1 - a0) / (c1 - c0);
                if (Math.Abs(a2 - alphaReq) > 25.0) break;           // the root is not near: give up
                double c2 = EvalCmAtAlpha(sim, input, a2, out cl1, out cd1);
                a0 = a1; c0 = c1; a1 = a2; c1 = c2;
                if (Math.Abs(c1) < 1e-4) { best = a1; cl = cl1; cd = cd1; break; }
                best = a1; cl = cl1; cd = cd1;
            }
            _inAlpha.SetValue(input, alphaReq);
            _inPitch.SetValue(input, 0.0);
            return best;
        }

        private double EvalCmAtAlpha(object sim, object input, double alpha, out double cl, out double cd)
        {
            _inAlpha.SetValue(input, alpha);
            object[] args = { input, null, false, false };
            _getClCdCm.Invoke(sim, args);
            cl = Convert.ToDouble(_outCl.GetValue(args[1]));
            cd = Convert.ToDouble(_outCd.GetValue(args[1]));
            _lastWingCl = _outWingCl != null ? Convert.ToDouble(_outWingCl.GetValue(args[1])) : double.NaN;
            _lastWingCd = _outWingCd != null ? Convert.ToDouble(_outWingCd.GetValue(args[1])) : double.NaN;
            return _outCm != null ? Convert.ToDouble(_outCm.GetValue(args[1])) : 0.0;
        }

        private double EvalCm(object sim, object input, double pitch, bool clear, out double cl, out double cd)
        {
            _inPitch?.SetValue(input, pitch);
            object[] args = { input, null, clear, false };
            _getClCdCm.Invoke(sim, args);
            cl = Convert.ToDouble(_outCl.GetValue(args[1]));
            cd = Convert.ToDouble(_outCd.GetValue(args[1]));
            _lastWingCl = _outWingCl != null ? Convert.ToDouble(_outWingCl.GetValue(args[1])) : double.NaN;
            _lastWingCd = _outWingCd != null ? Convert.ToDouble(_outWingCd.GetValue(args[1])) : double.NaN;
            return _outCm != null ? Convert.ToDouble(_outCm.GetValue(args[1])) : 0.0;
        }

        public static bool Available
        {
            get
            {
                if (!_resolved) Resolve();
                return _available;
            }
        }

        /// <summary>True when the loaded FAR can run its editor sim at a given flight point's Reynolds / Knudsen.</summary>
        public static bool SupportsFlightConditions => Available && _aeroConditionProp != null && _forFlightPoint != null && _bodyLengthField != null;

        /// <summary>True when FAR has an editor sim with a valid voxelization to query.</summary>
        public static bool Ready
        {
            get
            {
                if (!Available) return false;
                object sim = InstantSim();
                if (sim == null) return false;
                object ready = _readyProp != null ? _readyProp.GetValue(sim, null) : true;
                return ready is bool b && b;
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
                    Assembly asm = la.assembly;
                    _editorGuiType = asm.GetType("FerramAerospaceResearch.FARGUI.FAREditorGUI.EditorGUI");
                    _simType = asm.GetType("FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation.InstantConditionSim");
                    _inputType = asm.GetType("FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation.InstantConditionSimInput");
                    _outputType = asm.GetType("FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation.InstantConditionSimOutput");
                    Type wingType = asm.GetType("FerramAerospaceResearch.FARAeroComponents.FARWingAerodynamicModel")
                                    ?? asm.GetType("ferram4.FARWingAerodynamicModel");
                    Type condType = asm.GetType("FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation.SimAeroCondition");
                    if (_editorGuiType == null || _simType == null || _inputType == null || _outputType == null) break;

                    const BindingFlags any = BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.Static;
                    _instanceProp = _editorGuiType.GetProperty("Instance", any);
                    _instantSimField = _editorGuiType.GetField("_instantSim", any);
                    _readyProp = _simType.GetProperty("Ready", any);
                    _wingListField = _simType.GetField("_wingAerodynamicModel", any);
                    _maxCrossSectionField = _simType.GetField("_maxCrossSectionFromBody", any);
                    _bodyLengthField = _simType.GetField("_bodyLength", any);
                    _getClCdCm = _simType.GetMethod("GetClCdCmSteady", any);
                    _inAlpha = _inputType.GetField("alpha", any);
                    _inPhi = _inputType.GetField("phi", any);
                    _inMach = _inputType.GetField("machNumber", any);
                    _outCl = _outputType.GetField("Cl", any);
                    _outCd = _outputType.GetField("Cd", any);
                    _inPitch = _inputType.GetField("pitchValue", any);
                    _outCm = _outputType.GetField("Cm", any);
                    _outWingCl = _outputType.GetField("WingCl", any);   // fork-only wing/body split
                    _outWingCd = _outputType.GetField("WingCd", any);
                    if (wingType != null) _wingAreaField = wingType.GetField("S", any);
                    // Optional, fork-only: per-sample flight conditions.
                    _aeroConditionProp = _simType.GetProperty("AeroCondition", any);
                    if (condType != null)
                        _forFlightPoint = condType.GetMethod("ForFlightPoint", any, null,
                                                             new[] { typeof(CelestialBody), typeof(double), typeof(double), typeof(double) }, null);

                    _available = _instanceProp != null && _instantSimField != null && _getClCdCm != null
                                 && _inAlpha != null && _inMach != null && _outCl != null && _outCd != null;
                    break;
                }
            }
            catch (Exception ex)
            {
                RP0Debug.LogWarning($"FAR editor aero unavailable: {ex.Message}");
                _available = false;
            }
        }

        /// <summary>
        /// FAR's editor sim fixes its frame from the facility (nose along the build axis, "up" toward the back
        /// wall) and takes the flow in the plane of that up. Its roll input phi turns that plane: with
        /// liftDown = -forward*sin(a) + right*sin(phi)*cos(a) - up*cos(phi)*cos(a), phi puts the lift axis on
        /// up*cos(phi) - right*sin(phi). Solve that for the craft's actual roof, so a craft rotated about its
        /// build axis in the VAB is flown through its pitch plane and not across it.
        /// </summary>
        public static double FacilityRoll(Vector3 craftUp)
        {
            ReentryVesselBuilder.FacilityFrame(out _, out Vector3 up, out Vector3 right);
            return Math.Atan2(-Vector3.Dot(craftUp, right), Vector3.Dot(craftUp, up)) * 180.0 / Math.PI;
        }

        private static object InstantSim()
        {
            object gui = _instanceProp.GetValue(null, null);
            return gui == null ? null : _instantSimField.GetValue(gui);
        }

        public FarEditorAero(CelestialBody body, double aoaDeg, bool shieldFirst, Vector3 forward, Vector3 up, Vector3 liftAxis)
        {
            object sim = InstantSim();
            if (sim == null) throw new InvalidOperationException("FAR editor sim not running");
            _flightConditions = SupportsFlightConditions && body != null;
            _shieldFirst = shieldFirst;

            _mach = ReentryVesselBuilder.MachSamples;
            if (_flightConditions)
            {
                var alts = new List<double>();
                foreach (double a in AltitudeSamples) if (a < body.atmosphereDepth) alts.Add(a);
                alts.Add(body.atmosphereDepth);
                _alt = alts.ToArray();
            }
            else
            {
                _alt = new[] { 0.0 };
            }
            _cl = new double[_alt.Length, _mach.Length];
            _cd = new double[_alt.Length, _mach.Length];
            _wingCl = new double[_alt.Length, _mach.Length];
            _wingCd = new double[_alt.Length, _mach.Length];
            _alpha = new double[_alt.Length, _mach.Length];
            _trimPitch = new double[_alt.Length, _mach.Length];

            // FAR's alpha is measured from +forward; a shield-first entry is alpha = 180 - AoA.
            double alpha = shieldFirst ? 180.0 - aoaDeg : aoaDeg;
            double a0 = alpha * Math.PI / 180.0;
            // FAR reports Cl along -liftDown = forward*sin(a) + up*cos(a) in its own frame; with the roll input
            // below, that "up" is the craft's roof, so the lift lands on our lift axis.
            Vector3 liftDir = (forward * (float)Math.Sin(a0) + up * (float)Math.Cos(a0)).normalized;
            _liftSign = Vector3.Dot(liftDir, liftAxis);

            object input = Activator.CreateInstance(_inputType);
            _inAlpha.SetValue(input, alpha);
            _inPhi?.SetValue(input, FacilityRoll(up));
            double lengthScale = _flightConditions ? Math.Max(1.0, Convert.ToDouble(_bodyLengthField.GetValue(sim))) : 1.0;
            try
            {
                bool first = true;
                for (int ai = 0; ai < _alt.Length; ++ai)
                {
                    for (int m = 0; m < _mach.Length; ++m)
                    {
                        if (_flightConditions)
                        {
                            object cond = _forFlightPoint.Invoke(null, new object[] { body, _alt[ai], _mach[m], lengthScale });
                            _aeroConditionProp.SetValue(sim, cond, null);
                        }
                        _inMach.SetValue(input, _mach[m]);
                        SampleTrimmed(sim, input, first, out _cl[ai, m], out _cd[ai, m], out _trimPitch[ai, m], out bool trimmed);
                        _alpha[ai, m] = alpha;
                        if (!trimmed)
                        {
                            _untrimmable++;
                            _alpha[ai, m] = SolveAlphaAtStop(sim, input, alpha, _trimPitch[ai, m], out _cl[ai, m], out _cd[ai, m]);
                        }
                        _wingCl[ai, m] = _lastWingCl;
                        _wingCd[ai, m] = _lastWingCd;
                        first = false;
                        if (Math.Abs(_trimPitch[ai, m]) > _maxTrimPitch) _maxTrimPitch = Math.Abs(_trimPitch[ai, m]);
                    }
                }
            }
            finally
            {
                // Leave FAR's own static-analysis tab as we found it.
                if (_flightConditions) _aeroConditionProp.SetValue(sim, null, null);
            }

            _refArea = 0.0;
            if (_wingListField != null && _wingAreaField != null && _wingListField.GetValue(sim) is System.Collections.IEnumerable wings)
                foreach (object w in wings)
                    if (w != null) _refArea += Convert.ToDouble(_wingAreaField.GetValue(w));
            if (_refArea <= 1e-6 && _maxCrossSectionField != null)
                _refArea = Convert.ToDouble(_maxCrossSectionField.GetValue(sim));
            if (_refArea <= 1e-6) throw new InvalidOperationException("FAR reference area is zero");
        }

        public double ReferenceArea => _refArea;

        /// <summary>
        /// Diagnostic: log FAR's raw Cl/Cd over an alpha sweep at one Mach, in continuum flow and (when the
        /// build supports it) at a flight point, so a sign or magnitude disagreement with flight can be
        /// pinned to FAR's editor path rather than to this adapter.
        /// </summary>
        public static void LogAlphaSweep(CelestialBody body, double mach, double altitude, double phi)
        {
            if (!Ready) { RP0Debug.Log("[ReentryAero] FAR editor sim not ready for the alpha sweep", true); return; }
            object sim = InstantSim();
            object input = Activator.CreateInstance(_inputType);
            _inMach.SetValue(input, mach);
            _inPhi?.SetValue(input, phi);
            bool flight = SupportsFlightConditions && body != null;
            double lengthScale = flight ? Math.Max(1.0, Convert.ToDouble(_bodyLengthField.GetValue(sim))) : 1.0;
            var sb = new System.Text.StringBuilder();
            sb.Append($"[ReentryAero] FAR alpha sweep at Mach {mach:N0}, facility {EditorDriver.editorFacility}, roll {phi:N0}:");
            try
            {
                for (int pass = 0; pass < (flight ? 2 : 1); ++pass)
                {
                    if (flight)
                        _aeroConditionProp.SetValue(sim, pass == 1 ? _forFlightPoint.Invoke(null, new object[] { body, altitude, mach, lengthScale }) : null, null);
                    sb.Append(pass == 1 ? $"\n  at {altitude / 1000:N0} km flight conditions:" : "\n  continuum (stock path):");
                    // Fork-only split of the total into wing and body contributions, when the output carries it.
                    FieldInfo wingCl = _outputType.GetField("WingCl"), bodyCl = _outputType.GetField("BodyCl");
                    foreach (double alpha in new[] { -20.0, -10.0, 0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0 })
                    {
                        _inAlpha.SetValue(input, alpha);
                        object[] args = { input, null, true, true };
                        _getClCdCm.Invoke(sim, args);
                        sb.Append($" a={alpha:N0}: Cl={Convert.ToDouble(_outCl.GetValue(args[1])):N3} Cd={Convert.ToDouble(_outCd.GetValue(args[1])):N3}");
                        if (wingCl != null && bodyCl != null)
                            sb.Append($" (wing {Convert.ToDouble(wingCl.GetValue(args[1])):N3}, body {Convert.ToDouble(bodyCl.GetValue(args[1])):N3})");
                        sb.Append(';');
                    }
                }
            }
            finally
            {
                if (flight) _aeroConditionProp.SetValue(sim, null, null);
            }
            RP0Debug.Log(sb.ToString(), true);
        }

        /// <summary>
        /// Diagnostic: evaluate the editor sim once at (alpha, Mach, altitude, trimmed with pitch input) and
        /// log each wing part's area, MAC, Cl, Cd and stall fraction, in the same shape as the flight logger's
        /// [ReentryWings] lines, so the two wing models can be compared part by part.
        /// </summary>
        public static void LogWingBreakdown(CelestialBody body, double mach, double altitude, double alpha, double pitch, double phi)
        {
            if (!Ready) return;
            object sim = InstantSim();
            object input = Activator.CreateInstance(_inputType);
            _inMach.SetValue(input, mach);
            _inPhi?.SetValue(input, phi);
            _inAlpha.SetValue(input, alpha);
            _inPitch?.SetValue(input, pitch);
            bool flight = SupportsFlightConditions && body != null;
            double lengthScale = flight ? Math.Max(1.0, Convert.ToDouble(_bodyLengthField.GetValue(sim))) : 1.0;
            var sb = new System.Text.StringBuilder();
            try
            {
                if (flight) _aeroConditionProp.SetValue(sim, _forFlightPoint.Invoke(null, new object[] { body, altitude, mach, lengthScale }), null);
                object[] args = { input, null, true, true };
                _getClCdCm.Invoke(sim, args);
                sb.Append($"[ReentryWings] editor alt {altitude / 1000:N0} km, Mach {mach:N0}, alpha {alpha:N1}, pitch {pitch:N2}: " +
                          $"Cl {Convert.ToDouble(_outCl.GetValue(args[1])):N3} Cd {Convert.ToDouble(_outCd.GetValue(args[1])):N3}");
                if (_wingListField?.GetValue(sim) is System.Collections.IEnumerable wings)
                    foreach (object w in wings)
                    {
                        if (!(w is PartModule m) || m.part == null) continue;
                        sb.Append($"\n  {m.part.partInfo.name}: S {Num(m, "S"):N2} MAC {Num(m, "GetMAC"):N2} Cl {Num(m, "GetCl"):N3} Cd {Num(m, "GetCd"):N3} stall {Num(m, "GetStall"):N2}");
                    }
            }
            catch (Exception ex) { sb.Append($"\n  failed: {ex.Message}"); }
            finally
            {
                _inPitch?.SetValue(input, 0.0);
                if (flight) _aeroConditionProp.SetValue(sim, null, null);
            }
            RP0Debug.Log(sb.ToString(), true);
        }

        private static double Num(object o, string member)
        {
            const BindingFlags any = BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance;
            Type t = o.GetType();
            FieldInfo f = t.GetField(member, any);
            if (f != null) return Convert.ToDouble(f.GetValue(o));
            MethodInfo mi = t.GetMethod(member, any, null, Type.EmptyTypes, null);
            return mi != null ? Convert.ToDouble(mi.Invoke(o, null)) : double.NaN;
        }

        public void Forces(double dynPresKPa, double mach, double altitude, double pseudoReDragMult, out double dragKN, out double liftKN)
        {
            Index(_mach, mach, out int mi, out double mf);
            Index(_alt, altitude, out int ai, out double af);
            double cl = Bilinear(_cl, ai, af, mi, mf);
            double cd = Bilinear(_cd, ai, af, mi, mf);
            double wingCl = Bilinear(_wingCl, ai, af, mi, mf);
            double wingCd = Bilinear(_wingCd, ai, af, mi, mf);
            LastTrimPitch = Bilinear(_trimPitch, ai, af, mi, mf);
            double alphaHeld = Bilinear(_alpha, ai, af, mi, mf);
            LastAoA = _shieldFirst ? 180.0 - alphaHeld : alphaHeld;
            // In flight FAR scales each part's along-velocity body force by the stock pseudo-Reynolds drag
            // curve (never below 1), a large boost in the rarefied band. The editor sim runs at density 2
            // and unit speed, where that curve reads 1, so it has to go back on here. Wing-model forces are
            // applied directly and do not get it.
            double bodyCd = double.IsNaN(wingCd) ? cd : cd - wingCd;
            double wingCdOnly = double.IsNaN(wingCd) ? 0.0 : wingCd;
            double dragMult = Math.Max(pseudoReDragMult, 1.0);
            dragKN = dynPresKPa * _refArea * (bodyCd * dragMult + wingCdOnly);
            liftKN = dynPresKPa * cl * _refArea * _liftSign;
            LastWingLiftKN = double.IsNaN(wingCl) ? double.NaN : dynPresKPa * wingCl * _refArea * _liftSign;
            LastWingDragKN = double.IsNaN(wingCd) ? double.NaN : dynPresKPa * wingCd * _refArea;
        }

        private static void Index(double[] axis, double x, out int i, out double f)
        {
            int last = axis.Length - 1;
            if (last == 0 || x <= axis[0]) { i = 0; f = 0.0; return; }
            if (x >= axis[last]) { i = last; f = 0.0; return; }
            i = 0;
            while (i < last - 1 && axis[i + 1] < x) ++i;
            f = (x - axis[i]) / (axis[i + 1] - axis[i]);
        }

        private static double Bilinear(double[,] t, int ai, double af, int mi, double mf)
        {
            int aj = af > 0.0 && ai + 1 < t.GetLength(0) ? ai + 1 : ai;
            int mj = mf > 0.0 && mi + 1 < t.GetLength(1) ? mi + 1 : mi;
            double r0 = t[ai, mi] + (t[ai, mj] - t[ai, mi]) * mf;
            double r1 = t[aj, mi] + (t[aj, mj] - t[aj, mi]) * mf;
            return r0 + (r1 - r0) * af;
        }
    }
    /// <summary>
    /// FAR's per-part voxel projected areas (FARAeroPartModule.ProjectedAreas), applied to the part's drag
    /// cube the way FAR's flight integrator hook does, so the editor's occlusion geometry and areas match
    /// what the game uses in flight.
    /// </summary>
    public static class FarPartAreas
    {
        private static bool _resolved;
        private static PropertyInfo _areasProp;
        private static FieldInfo[] _faceFields;
        private static FieldInfo _totalField;
        private static int[] _faceMap;

        private static void Resolve()
        {
            _resolved = true;
            try
            {
                foreach (AssemblyLoader.LoadedAssembly la in AssemblyLoader.loadedAssemblies)
                {
                    if (la.name != "FerramAerospaceResearch") continue;
                    Type module = la.assembly.GetType("FerramAerospaceResearch.FARAeroComponents.FARAeroPartModule");
                    if (module == null) break;
                    const BindingFlags any = BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.Static;
                    _areasProp = module.GetProperty("ProjectedAreas", any);
                    Type area = _areasProp?.PropertyType;
                    if (area == null) break;
                    // Field order matches the struct's indexer and FaceMap: iP, iN, jP, jN, kP, kN.
                    string[] names = { "iP", "iN", "jP", "jN", "kP", "kN" };
                    _faceFields = new FieldInfo[6];
                    for (int i = 0; i < 6; ++i) _faceFields[i] = area.GetField(names[i], any);
                    _totalField = area.GetField("totalArea", any);
                    FieldInfo map = area.GetField("FaceMap", any);
                    _faceMap = map != null ? map.GetValue(null) as int[] : null;
                    if (_faceMap == null || _faceMap.Length != 6)
                        _faceMap = new[] { (int)DragCube.DragFace.XP, (int)DragCube.DragFace.XN, (int)DragCube.DragFace.YP,
                                           (int)DragCube.DragFace.YN, (int)DragCube.DragFace.ZP, (int)DragCube.DragFace.ZN };
                    break;
                }
            }
            catch (Exception ex)
            {
                RP0Debug.LogWarning($"FAR part areas unavailable: {ex.Message}");
                _areasProp = null;
            }
        }

        /// <summary>
        /// Copies the part's voxel face areas into its drag cube's occluded-area array, as FAR does in flight.
        /// Returns false (and leaves the cube alone) when FAR is absent or this part has no voxelization yet.
        /// </summary>
        public static bool TryApply(Part part, out double totalArea)
        {
            totalArea = 0.0;
            if (!_resolved) Resolve();
            if (_areasProp == null || part == null || part.DragCubes == null || part.DragCubes.None) return false;
            PartModule module = null;
            foreach (PartModule m in part.Modules)
                if (m.GetType().Name == "FARAeroPartModule") { module = m; break; }
            if (module == null) return false;
            try
            {
                object areas = _areasProp.GetValue(module, null);
                totalArea = _totalField != null ? Convert.ToDouble(_totalField.GetValue(areas)) : 0.0;
                if (!(totalArea > 0.0)) return false;
                float[] occluded = part.DragCubes.AreaOccluded;
                for (int j = 0; j < 6; ++j)
                    if (_faceFields[j] != null) occluded[_faceMap[j]] = (float)Convert.ToDouble(_faceFields[j].GetValue(areas));
                return true;
            }
            catch (Exception)
            {
                totalArea = 0.0;
                return false;
            }
        }
    }

    /// <summary>Picks FAR when it is present and voxelized, otherwise stock drag cubes.</summary>
    public class DefaultAeroFactory : IReentryAeroFactory
    {
        public bool PreferFar = true;
        public string LastSource = "";
        // Trim bookkeeping across every attitude built through this factory.
        public double MaxTrimPitch;
        public int Untrimmable, TotalSamples;

        /// <summary>One line on what holding the analysed attitudes costs in pitch control.</summary>
        public string TrimSummary => TotalSamples == 0 ? ""
            : Untrimmable > 0 ? $"full elevator cannot hold the AoA at {Untrimmable} of {TotalSamples} flight points; held at the angle it can (see the trace)"
            : $"trim needs up to {MaxTrimPitch:P0} elevator";

        public IReentryAero Create(CelestialBody body, List<ReentryNode> nodes, Vector3 velDir, Vector3 liftAxis, Vector3 forward, Vector3 up,
                                   double aoaDeg, bool shieldFirst, ReentryConstants k)
        {
            if (PreferFar && FarEditorAero.Ready)
            {
                try
                {
                    var far = new FarEditorAero(body, aoaDeg, shieldFirst, forward, up, liftAxis);
                    if (far.MaxTrimPitch > MaxTrimPitch) MaxTrimPitch = far.MaxTrimPitch;
                    Untrimmable += far.UntrimmableSamples;
                    TotalSamples += far.TotalSamples;
                    LastSource = far.Description;
                    return far;
                }
                catch (Exception ex)
                {
                    RP0Debug.LogWarning($"FAR editor aero failed, falling back to drag cubes: {ex.Message}");
                }
            }
            var cubes = new DragCubeAero(nodes, velDir, liftAxis, k);
            LastSource = cubes.Description;
            return cubes;
        }
    }
}
