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
        private static FieldInfo _inAlpha, _inMach, _outCl, _outCd;
        private static MethodInfo _getClCdCm, _forFlightPoint;
        private static FieldInfo _wingAreaField;

        /// <summary>Altitudes (m) the flight-condition sampling is taken at, when supported.</summary>
        public static readonly double[] AltitudeSamples = { 20000, 40000, 55000, 65000, 75000, 85000, 95000, 110000, 140000 };

        private readonly double[] _mach, _alt;
        private readonly double[,] _cl, _cd;     // [alt, mach]; a single altitude row when flight conditions are unsupported
        private readonly double _refArea;
        private readonly double _liftSign;
        private readonly bool _flightConditions;

        public string Description => _flightConditions ? "FAR editor aero at flight Re/Kn" : "FAR editor aero (continuum)";

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
                    _inMach = _inputType.GetField("machNumber", any);
                    _outCl = _outputType.GetField("Cl", any);
                    _outCd = _outputType.GetField("Cd", any);
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

            // FAR's alpha is measured from +forward; a shield-first entry is alpha = 180 - AoA.
            double alpha = shieldFirst ? 180.0 - aoaDeg : aoaDeg;
            double a0 = alpha * Math.PI / 180.0;
            // FAR reports Cl along -liftDown, liftDown = -forward*sin(a) - up*cos(a); express that on our lift axis.
            Vector3 liftDir = (forward * (float)Math.Sin(a0) + up * (float)Math.Cos(a0)).normalized;
            _liftSign = Vector3.Dot(liftDir, liftAxis);

            object input = Activator.CreateInstance(_inputType);
            _inAlpha.SetValue(input, alpha);
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
                        object[] args = { input, null, first, false };
                        first = false;
                        _getClCdCm.Invoke(sim, args);
                        object output = args[1];
                        _cl[ai, m] = Convert.ToDouble(_outCl.GetValue(output));
                        _cd[ai, m] = Convert.ToDouble(_outCd.GetValue(output));
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

        public void Forces(double dynPresKPa, double mach, double altitude, double pseudoReDragMult, out double dragKN, out double liftKN)
        {
            Index(_mach, mach, out int mi, out double mf);
            Index(_alt, altitude, out int ai, out double af);
            double cl = Bilinear(_cl, ai, af, mi, mf);
            double cd = Bilinear(_cd, ai, af, mi, mf);
            dragKN = dynPresKPa * cd * _refArea;
            liftKN = dynPresKPa * cl * _refArea * _liftSign;
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
    /// <summary>Picks FAR when it is present and voxelized, otherwise stock drag cubes.</summary>
    public class DefaultAeroFactory : IReentryAeroFactory
    {
        public bool PreferFar = true;
        public string LastSource = "";

        public IReentryAero Create(CelestialBody body, List<ReentryNode> nodes, Vector3 velDir, Vector3 liftAxis, Vector3 forward, Vector3 up,
                                   double aoaDeg, bool shieldFirst, ReentryConstants k)
        {
            if (PreferFar && FarEditorAero.Ready)
            {
                try
                {
                    var far = new FarEditorAero(body, aoaDeg, shieldFirst, forward, up, liftAxis);
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
