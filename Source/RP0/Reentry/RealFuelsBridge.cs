using System;
using System.Reflection;

namespace RP0.Reentry
{
    /// <summary>
    /// RealFuels rewrites a tank's part.heatConductivity every fixed update from its multi-layer-insulation
    /// transfer rate (ModuleFuelTankRF.CalculateInsulation), which depends on the current skin and interior
    /// temperatures. In the editor the part still carries the prefab value, and with it the interior follows
    /// the skin within seconds instead of lagging it by a thousand kelvin. This reads the tank's MLI layer
    /// count and RF's gas-conduction coefficient by reflection and evaluates the same formula per step.
    /// </summary>
    public static class RealFuelsBridge
    {
        private static bool _resolved;
        private static PropertyInfo _layersProp;
        private static double _qvCoefficient;
        private static Type _tankType;

        private static void Resolve()
        {
            _resolved = true;
            try
            {
                foreach (AssemblyLoader.LoadedAssembly la in AssemblyLoader.loadedAssemblies)
                {
                    if (la.name != "RealFuels") continue;
                    Assembly asm = la.assembly;
                    _tankType = asm.GetType("RealFuels.Tanks.ModuleFuelTanks") ?? asm.GetType("RealFuels.Tanks.ModuleFuelTankRF");
                    const BindingFlags any = BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.Static;
                    _layersProp = _tankType?.GetProperty("totalMLILayers", any);
                    Type settings = asm.GetType("RealFuels.RFSettings");
                    PropertyInfo instance = settings?.GetProperty("Instance", any);
                    object inst = instance?.GetValue(null, null);
                    FieldInfo qv = settings?.GetField("QvCoefficient", any);
                    _qvCoefficient = inst != null && qv != null ? Convert.ToDouble(qv.GetValue(inst)) : 0.0;
                    break;
                }
            }
            catch (Exception ex)
            {
                RP0Debug.LogWarning($"RealFuels bridge unavailable: {ex.Message}");
                _layersProp = null;
            }
        }

        /// <summary>Number of MLI layers on this part's RealFuels tank, or 0 when it has none (or RF is absent).</summary>
        public static int MliLayers(Part part)
        {
            if (!_resolved) Resolve();
            if (_layersProp == null || part == null) return 0;
            foreach (PartModule m in part.Modules)
            {
                if (!_tankType.IsInstanceOfType(m)) continue;
                try { return Convert.ToInt32(_layersProp.GetValue(m, null)); }
                catch (Exception) { return 0; }
            }
            return 0;
        }

        /// <summary>
        /// ModuleFuelTankRF.CalculateInsulation: the effective part.heatConductivity for the current skin and
        /// interior temperatures. Constants and exponents are RealFuels' own (GetMLITransferRate).
        /// </summary>
        public static double HeatConductivity(int layers, double prefabSkinInternalConductionMult, double skinTemp, double internalTemp,
                                              double staticPressureKPa, double sicf, double conductionFactor, double convergenceFactor)
        {
            double dT = skinTemp - internalTemp;
            if (dT == 0.0) dT = 1e-11;
            double radiative = 4.944E-10 * 0.03 * (Math.Pow(skinTemp, 4.67) - Math.Pow(internalTemp, 4.67)) / layers;
            double conductive = 8.95E-08 * Math.Pow(10.055, 2.63) * ((skinTemp + internalTemp) / 2.0) / (layers + 1) * dT;
            double gas = _qvCoefficient * (staticPressureKPa * 7.500616851 * (Math.Pow(skinTemp, 0.52) - Math.Pow(internalTemp, 0.52))) / layers;
            double rate = radiative + conductive + gas;                       // W/m2
            double num3 = Math.Abs(rate / dT) * 0.001;                          // kW/(m2 K)
            double num4 = prefabSkinInternalConductionMult == 0.0 ? double.MaxValue : 1.0 / prefabSkinInternalConductionMult;
            double num = 1.0 / (sicf * conductionFactor * convergenceFactor * 10.0 * 0.5);
            return num / (1.0 / num3 + num4);
        }
    }
}
