using System;
using System.Collections.Generic;
using System.Reflection;

namespace RP0.Reentry
{
    /// <summary>
    /// RealHeat (KSP-RO) replaces the stock shock temperature with a real-gas curve per body, changes the
    /// density thermal lerp to use the shock density with that gas's gamma, and scales the convective
    /// coefficient down at low dynamic pressure. It also retunes the stock OcclusionCone statics from its
    /// REALHEAT config node - but only from a flight-scene addon, so the editor must apply those itself.
    /// Everything here is reached by reflection so RP-1 keeps no reference to RealHeat.
    /// </summary>
    public class RealHeatBridge
    {
        private static bool _resolved, _available;
        private static Type _utilsType, _organizerType, _curveType, _containerType;
        private static FieldInfo _minMultField, _dynMultField, _atmospheresField, _gasConstField;
        private static MethodInfo _loadNodes, _calcCurve, _evalTempDiff, _evalCp;
        private static readonly Dictionary<string, RealHeatBridge> _cache = new Dictionary<string, RealHeatBridge>();

        private readonly object _curve;
        public readonly double MinConvectiveCoefficientMultLowQ, DynamicPressureMultiplier, SpecificGasConstant;

        public static bool Available
        {
            get
            {
                if (!_resolved) Resolve();
                return _available;
            }
        }

        private static void Resolve()
        {
            _resolved = true;
            try
            {
                foreach (AssemblyLoader.LoadedAssembly la in AssemblyLoader.loadedAssemblies)
                {
                    if (la.name != "RealHeat") continue;
                    Assembly asm = la.assembly;
                    _utilsType = asm.GetType("RealHeat.RealHeatUtils");
                    _organizerType = asm.GetType("RealHeat.AtmDataOrganizer");
                    _curveType = asm.GetType("RealHeat.AtmTempCurve");
                    _containerType = asm.GetType("RealHeat.tempCurveDataContainer");
                    if (_utilsType == null || _organizerType == null || _curveType == null || _containerType == null) break;

                    const BindingFlags any = BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.Static;
                    _minMultField = _utilsType.GetField("minConvectiveCoefficientMultLowQ", any);
                    _dynMultField = _utilsType.GetField("dynamicPressureMultiplier", any);
                    _atmospheresField = _organizerType.GetField("bodyOrganizedListOfAtmospheres", any);
                    _loadNodes = _organizerType.GetMethod("LoadConfigNodes", any);
                    _calcCurve = _organizerType.GetMethod("CalculateNewTemperatureCurve", any);
                    _evalTempDiff = _curveType.GetMethod("EvaluateTempDiffCurve", any);
                    _evalCp = _curveType.GetMethod("EvaluateVelCpCurve", any);
                    _gasConstField = _curveType.GetField("specificGasConstant", any);
                    _available = _minMultField != null && _dynMultField != null && _atmospheresField != null && _loadNodes != null
                                 && _calcCurve != null && _evalTempDiff != null && _evalCp != null && _gasConstField != null;
                    break;
                }
            }
            catch (Exception ex)
            {
                RP0Debug.LogWarning($"RealHeat bridge unavailable: {ex.Message}");
                _available = false;
            }
        }

        /// <summary>The bridge for a body, computing its temperature curve on first use. Null when RealHeat is absent or fails.</summary>
        public static RealHeatBridge For(CelestialBody body)
        {
            if (!Available || body == null) return null;
            if (_cache.TryGetValue(body.bodyName, out RealHeatBridge b)) return b;
            try
            {
                b = new RealHeatBridge(body);
                _cache[body.bodyName] = b;
                return b;
            }
            catch (Exception ex)
            {
                RP0Debug.LogWarning($"RealHeat temperature curve for {body.bodyName} failed: {ex.Message}");
                _cache[body.bodyName] = null;
                return null;
            }
        }

        private RealHeatBridge(CelestialBody body)
        {
            // The flight addon normally loads the gas data; in the editor it may not have run yet this session.
            if (_atmospheresField.GetValue(null) == null)
                _loadNodes.Invoke(null, null);

            // A private curve, computed synchronously, so nothing depends on RealHeat's threaded flight-scene state.
            _curve = Activator.CreateInstance(_curveType);
            object container = Activator.CreateInstance(_containerType, body, _curve, false);
            _calcCurve.Invoke(null, new[] { container });
            SpecificGasConstant = Convert.ToDouble(_gasConstField.GetValue(_curve));
            MinConvectiveCoefficientMultLowQ = Convert.ToDouble(_minMultField.GetValue(null));
            DynamicPressureMultiplier = Convert.ToDouble(_dynMultField.GetValue(null));

            // Force the curve objects to build now (they are built lazily on the first evaluate).
            TempRise(1000.0);
            SpecificHeat(1000.0);
        }

        /// <summary>Stagnation temperature rise above ambient for an airspeed (RealHeat replaces stock's shock temperature with this).</summary>
        public double TempRise(double airspeed)
        {
            return Convert.ToDouble(_evalTempDiff.Invoke(_curve, new object[] { (float)airspeed }));
        }

        /// <summary>Real-gas specific heat at constant pressure for an airspeed; gamma = Cp / (Cp - R).</summary>
        public double SpecificHeat(double airspeed)
        {
            return Convert.ToDouble(_evalCp.Invoke(_curve, new object[] { (float)airspeed }));
        }

        public double Gamma(double airspeed)
        {
            double cp = SpecificHeat(airspeed);
            return cp / (cp - SpecificGasConstant);
        }

        /// <summary>
        /// Apply the REALHEAT node's OcclusionCone overrides, exactly as RealHeatUtils.Start does in flight, so the
        /// editor's occlusion uses the same numbers whether or not a flight has happened this session.
        /// </summary>
        public static void ApplyOcclusionSettings()
        {
            if (!Available) return;
            foreach (ConfigNode node in GameDatabase.Instance.GetConfigNodes("REALHEAT"))
            {
                node.TryGetValue("detachedShockHeatMult", ref OcclusionCone.detachedShockHeatMult);
                node.TryGetValue("detachedShockCoeffMult", ref OcclusionCone.detachedShockCoeffMult);
                node.TryGetValue("detachedBehindShockHeatMult", ref OcclusionCone.detachedBehindShockHeatMult);
                node.TryGetValue("detachedBehindShockCoeffMult", ref OcclusionCone.detachedBehindShockCoeffMult);
                node.TryGetValue("detachedShockMachAngleMult", ref OcclusionCone.detachedShockMachAngleMult);
                node.TryGetValue("detachedShockStartAngle", ref OcclusionCone.detachedShockStartAngle);
                node.TryGetValue("detachedShockEndAngle", ref OcclusionCone.detachedShockEndAngle);
                node.TryGetValue("obliqueShockAngleMult", ref OcclusionCone.obliqueShockAngleMult);
                node.TryGetValue("obliqueShockPartAngleMult", ref OcclusionCone.obliqueShockPartAngleMult);
                node.TryGetValue("obliqueShockMinAngleMult", ref OcclusionCone.obliqueShockMinAngleMult);
                node.TryGetValue("obliqueShockConeHeatMult", ref OcclusionCone.obliqueShockConeHeatMult);
                node.TryGetValue("obliqueShockConeCoeffMult", ref OcclusionCone.obliqueShockConeCoeffMult);
                node.TryGetValue("obliqueShockCylHeatMult", ref OcclusionCone.obliqueShockCylHeatMult);
                node.TryGetValue("obliqueShockCylCoeffMult", ref OcclusionCone.obliqueShockCylCoeffMult);
                break;
            }
        }
    }
}
