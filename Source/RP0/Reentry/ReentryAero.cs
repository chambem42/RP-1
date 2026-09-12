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

        public void Forces(double dynPresKPa, double mach, double pseudoReDragMult, out double dragKN, out double liftKN)
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
    /// </summary>
    public class FarEditorAero : IReentryAero
    {
        private static bool _resolved, _available;
        private static Type _editorGuiType, _inputType, _outputType, _simType;
        private static PropertyInfo _instanceProp, _readyProp;
        private static FieldInfo _instantSimField, _wingListField, _maxCrossSectionField;
        private static FieldInfo _inAlpha, _inMach, _outCl, _outCd;
        private static MethodInfo _getClCdCm;
        private static FieldInfo _wingAreaField;

        private readonly double[] _mach, _cl, _cd;
        private readonly double _refArea;
        private readonly double _liftSign;

        public string Description => "FAR editor aero";

        public static bool Available
        {
            get
            {
                if (!_resolved) Resolve();
                return _available;
            }
        }

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
                    if (_editorGuiType == null || _simType == null || _inputType == null || _outputType == null) break;

                    const BindingFlags any = BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.Static;
                    _instanceProp = _editorGuiType.GetProperty("Instance", any);
                    _instantSimField = _editorGuiType.GetField("_instantSim", any);
                    _readyProp = _simType.GetProperty("Ready", any);
                    _wingListField = _simType.GetField("_wingAerodynamicModel", any);
                    _maxCrossSectionField = _simType.GetField("_maxCrossSectionFromBody", any);
                    _getClCdCm = _simType.GetMethod("GetClCdCmSteady", any);
                    _inAlpha = _inputType.GetField("alpha", any);
                    _inMach = _inputType.GetField("machNumber", any);
                    _outCl = _outputType.GetField("Cl", any);
                    _outCd = _outputType.GetField("Cd", any);
                    if (wingType != null) _wingAreaField = wingType.GetField("S", any);

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

        public FarEditorAero(double aoaDeg, bool shieldFirst, Vector3 forward, Vector3 up, Vector3 liftAxis)
        {
            _mach = ReentryVesselBuilder.MachSamples;
            _cl = new double[_mach.Length];
            _cd = new double[_mach.Length];
            object sim = InstantSim();
            if (sim == null) throw new InvalidOperationException("FAR editor sim not running");

            // FAR's alpha is measured from +forward; a shield-first entry is alpha = 180 - AoA.
            double alpha = shieldFirst ? 180.0 - aoaDeg : aoaDeg;
            double a = alpha * Math.PI / 180.0;
            // FAR reports Cl along -liftDown, liftDown = -forward*sin(a) - up*cos(a); express that on our lift axis.
            Vector3 liftDir = (forward * (float)Math.Sin(a) + up * (float)Math.Cos(a)).normalized;
            _liftSign = Vector3.Dot(liftDir, liftAxis);

            object input = Activator.CreateInstance(_inputType);
            _inAlpha.SetValue(input, alpha);
            for (int m = 0; m < _mach.Length; ++m)
            {
                _inMach.SetValue(input, _mach[m]);
                object[] args = { input, null, m == 0, false };
                _getClCdCm.Invoke(sim, args);
                object output = args[1];
                _cl[m] = Convert.ToDouble(_outCl.GetValue(output));
                _cd[m] = Convert.ToDouble(_outCd.GetValue(output));
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

        public void Forces(double dynPresKPa, double mach, double pseudoReDragMult, out double dragKN, out double liftKN)
        {
            int last = _mach.Length - 1;
            double cl, cd;
            if (mach <= _mach[0]) { cl = _cl[0]; cd = _cd[0]; }
            else if (mach >= _mach[last]) { cl = _cl[last]; cd = _cd[last]; }
            else
            {
                int i = 0;
                while (i < last - 1 && _mach[i + 1] < mach) ++i;
                double f = (mach - _mach[i]) / (_mach[i + 1] - _mach[i]);
                cl = _cl[i] + (_cl[i + 1] - _cl[i]) * f;
                cd = _cd[i] + (_cd[i + 1] - _cd[i]) * f;
            }
            dragKN = dynPresKPa * cd * _refArea;
            liftKN = dynPresKPa * cl * _refArea * _liftSign;
        }
    }

    /// <summary>Picks FAR when it is present and voxelized, otherwise stock drag cubes.</summary>
    public class DefaultAeroFactory : IReentryAeroFactory
    {
        public bool PreferFar = true;
        public string LastSource = "";

        public IReentryAero Create(List<ReentryNode> nodes, Vector3 velDir, Vector3 liftAxis, Vector3 forward, Vector3 up,
                                   double aoaDeg, bool shieldFirst, ReentryConstants k)
        {
            if (PreferFar && FarEditorAero.Ready)
            {
                try
                {
                    var far = new FarEditorAero(aoaDeg, shieldFirst, forward, up, liftAxis);
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
