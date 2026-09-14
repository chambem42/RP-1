using System;
using System.Collections.Generic;

namespace RP0.Reentry
{
    /// <summary>
    /// "Propellant remaining": sets every resource the vessel's engines can burn to a fraction of its tank
    /// capacity, on the editor ship before an analysis or on the simulated vessel before an entry, so both
    /// see the same landing-weight vehicle instead of whatever the craft file happened to hold.
    /// </summary>
    public static class PropellantLoad
    {
        /// <summary>Resource names consumed by any ModuleEngines on these parts.</summary>
        public static HashSet<string> EnginePropellants(IEnumerable<Part> parts)
        {
            var names = new HashSet<string>();
            foreach (Part p in parts)
            {
                if (p == null) continue;
                foreach (ModuleEngines e in p.FindModulesImplementing<ModuleEngines>())
                    if (e.propellants != null)
                        foreach (Propellant prop in e.propellants)
                            if (!string.IsNullOrEmpty(prop.name)) names.Add(prop.name);
            }
            return names;
        }

        /// <summary>
        /// Sets each engine propellant on every part to percent of its maximum. Returns the number of
        /// resources changed; does nothing (and returns 0) for a percent below zero.
        /// </summary>
        public static int Apply(IEnumerable<Part> parts, double percent)
        {
            if (percent < 0.0 || parts == null) return 0;
            double frac = Math.Min(percent, 100.0) / 100.0;
            HashSet<string> names = EnginePropellants(parts);
            if (names.Count == 0) return 0;
            int changed = 0;
            foreach (Part p in parts)
            {
                if (p == null || p.Resources == null) continue;
                foreach (PartResource r in p.Resources)
                {
                    if (!names.Contains(r.resourceName)) continue;
                    double want = r.maxAmount * frac;
                    if (Math.Abs(r.amount - want) < 1e-9) continue;
                    r.amount = want;
                    ++changed;
                }
            }
            return changed;
        }
    }
}
