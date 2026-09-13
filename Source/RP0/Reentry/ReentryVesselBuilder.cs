using System;
using System.Collections.Generic;
using UnityEngine;

namespace RP0.Reentry
{
    /// <summary>
    /// Turns the editor vessel into what the solver needs for one attitude: per-part thermal properties,
    /// per-Mach drag-cube areas, and the stock convection-occlusion multipliers, computed by running the
    /// game's own OcclusionData / OcclusionCone geometry against the editor parts.
    /// </summary>
    public static class ReentryVesselBuilder
    {
        /// <summary>Mach numbers the per-part aero is sampled at; the solver interpolates between them.</summary>
        public static readonly double[] MachSamples = { 0.5, 1.5, 3.0, 6.0, 12.0, 25.0, 40.0 };

        public static ReentryAttitude Build(List<Part> parts, CelestialBody body, ReentryConstants k, double aoaDeg,
                                            bool shieldFirst, IReentryAeroFactory aeroFactory)
        {
            var att = new ReentryAttitude { AoADeg = aoaDeg, ShieldFirst = shieldFirst, MachSamples = MachSamples };
            EditorFrame(out Vector3 forward, out Vector3 up, out Vector3 right);
            Vector3 velDir = VelocityDirection(forward, up, aoaDeg, shieldFirst);
            // Lift axis: the vessel's "up" made perpendicular to the velocity, so signed lift is well defined.
            Vector3 liftAxis = (up - Vector3.Dot(up, velDir) * velDir).normalized;

            double cp = k.StandardSpecificHeatCapacity;
            var nodes = new List<ReentryNode>();
            var ptds = new List<PartThermalData>();
            double massT = 0.0;

            foreach (Part p in parts)
            {
                if (p == null || p.physicalSignificance == Part.PhysicalSignificance.NONE && p.mass <= 0f) continue;
                double resMass = p.GetResourceMass(out double resThermal);
                double dry = p.mass;
                massT += dry + resMass;
                if (p.DragCubes == null || p.DragCubes.None) continue;

                var n = new ReentryNode
                {
                    Name = p.partInfo != null ? p.partInfo.title : p.name,
                    Part = p,
                    MaxTemp = p.maxTemp,
                    SkinMaxTemp = p.skinMaxTemp > 0.0 ? p.skinMaxTemp : p.maxTemp,
                    EmissiveConstant = p.emissiveConstant,
                    HeatConvectiveConstant = p.heatConvectiveConstant,
                    HeatConductivity = p.heatConductivity,
                    SkinInternalConductionMult = p.skinInternalConductionMult,
                    SkinSkinConductionMult = p.skinSkinConductionMult,
                    ThermalMass = Math.Max(0.1, dry * cp * p.thermalMassModifier + resThermal),
                    ExposedArea = new double[MachSamples.Length],
                    RadiativeArea = new double[MachSamples.Length],
                    ConvectionTempMult = new double[MachSamples.Length],
                    ConvectionCoeffMult = new double[MachSamples.Length],
                    ConvectionAreaMult = new double[MachSamples.Length],
                };
                ReadAblator(p, n);
                nodes.Add(n);
                ptds.Add(new PartThermalData(p));
            }

            // Drag cubes: weights first for every part, then occlusion (needs the neighbours' weights).
            foreach (ReentryNode n in nodes) n.Part.DragCubes.SetDragWeights();
            foreach (ReentryNode n in nodes) { n.Part.DragCubes.RequestOcclusionUpdate(); n.Part.DragCubes.SetPartOcclusion(); }

            // RealHeat retunes the OcclusionCone statics from a flight-scene addon; match that here.
            RealHeatBridge.ApplyOcclusionSettings();
            var occ = new List<OcclusionData>(nodes.Count);
            for (int i = 0; i < nodes.Count; ++i) occ.Add(ptds[i].convectionData);

            for (int m = 0; m < MachSamples.Length; ++m)
            {
                float mach = (float)MachSamples[m];
                for (int i = 0; i < nodes.Count; ++i)
                {
                    Part p = nodes[i].Part;
                    Vector3 dirLocal = -p.partTransform.InverseTransformDirection(velDir);
                    p.DragCubes.SetDrag(dirLocal, mach);
                    nodes[i].RadiativeArea[m] = p.DragCubes.PostOcclusionArea;
                    nodes[i].ExposedArea[m] = p.DragCubes.ExposedArea;
                    if (m == 0) nodes[i].DragCoeff = p.DragCubes.DragCoeff;
                }
                ComputeOcclusion(occ, velDir, MachSamples[m]);
                for (int i = 0; i < nodes.Count; ++i)
                {
                    PartThermalData ptd = ptds[i];
                    nodes[i].ConvectionTempMult[m] = ptd.convectionTempMultiplier;
                    nodes[i].ConvectionCoeffMult[m] = ptd.convectionCoeffMultiplier;
                    nodes[i].ConvectionAreaMult[m] = ptd.convectionAreaMultiplier;
                }
            }

            // Skin thermal mass uses the radiative area, which does not vary with Mach.
            foreach (ReentryNode n in nodes)
            {
                double radArea = n.RadiativeArea[0];
                double cpMod = cp * n.Part.thermalMassModifier;
                n.SkinThermalMass = Math.Max(0.1, Math.Min(0.001 * n.Part.skinMassPerArea * n.Part.skinThermalMassModifier * radArea * cpMod,
                                                            n.Part.mass * cpMod * 0.5));
                // A shield counts as exposed when its shielded face is not occluded by an attached part.
                n.AblatorExposed = n.AblatorMax > 0.0 && n.ExposedArea[MachSamples.Length - 1] > 0.1 * radArea;
            }

            att.Nodes = nodes;
            att.MassT = Math.Max(massT, 0.001);
            att.Aero = aeroFactory.Create(body, nodes, velDir, liftAxis, forward, up, aoaDeg, shieldFirst, k);
            return att;
        }

        // The editor's world frame for the vessel, matching FAR's InstantConditionSim conventions.
        public static void EditorFrame(out Vector3 forward, out Vector3 up, out Vector3 right)
        {
            if (EditorDriver.editorFacility == EditorFacility.VAB)
            {
                forward = Vector3.up;
                up = -Vector3.forward;
            }
            else
            {
                forward = Vector3.forward;
                up = Vector3.up;
            }
            right = Vector3.right;
        }

        /// <summary>Airflow direction (the vessel's velocity) for an angle of attack. Positive AoA pitches the nose above the velocity.</summary>
        public static Vector3 VelocityDirection(Vector3 forward, Vector3 up, double aoaDeg, bool shieldFirst)
        {
            double a = aoaDeg * Math.PI / 180.0;
            Vector3 axis = shieldFirst ? -forward : forward;
            return (axis * (float)Math.Cos(a) - up * (float)Math.Sin(a)).normalized;
        }

        // FlightIntegrator.UpdateOcclusionConvection, run against the editor parts.
        private static void ComputeOcclusion(List<OcclusionData> occ, Vector3 velDir, double mach)
        {
            int count = occ.Count;
            if (count == 0) return;
            if (mach <= 1.0)
            {
                foreach (OcclusionData o in occ)
                    o.ptd.convectionTempMultiplier = o.ptd.convectionAreaMultiplier = o.ptd.convectionCoeffMultiplier = 1.0;
                return;
            }
            foreach (OcclusionData o in occ) o.Update(velDir);
            occ.Sort();

            double sqrtMach = Math.Sqrt(mach);
            double machAngle = Math.Asin(1.0 / sqrtMach);
            double detachAngle = 0.7957 * (1.0 - 1.0 / (mach * sqrtMach));

            var cones = new List<OcclusionCone>(count);
            OcclusionData lead = occ[count - 1];
            lead.ptd.convectionTempMultiplier = lead.ptd.convectionAreaMultiplier = lead.ptd.convectionCoeffMultiplier = 1.0;
            lead.convCone.Setup(lead, sqrtMach, machAngle, detachAngle);
            cones.Add(lead.convCone);

            for (int i = count - 2; i >= 0; --i)
            {
                OcclusionData o = occ[i];
                o.ptd.convectionTempMultiplier = o.ptd.convectionAreaMultiplier = o.ptd.convectionCoeffMultiplier = 1.0;
                foreach (OcclusionCone cone in cones)
                {
                    double areaMult = o.GetShockStats(cone, ref o.ptd.convectionTempMultiplier, ref o.ptd.convectionCoeffMultiplier);
                    o.ptd.convectionAreaMultiplier *= areaMult;
                    if (o.ptd.convectionAreaMultiplier <= 0.001) { o.ptd.convectionAreaMultiplier = 0.0; break; }
                }
                if (o.ptd.convectionAreaMultiplier > 0.0)
                {
                    o.convCone.Setup(o, sqrtMach, machAngle, detachAngle);
                    cones.Add(o.convCone);
                }
            }
        }

        private static void ReadAblator(Part p, ReentryNode n)
        {
            ModuleAblator abl = p.FindModuleImplementing<ModuleAblator>();
            if (abl == null || string.IsNullOrEmpty(abl.ablativeResource)) return;
            PartResourceDefinition def = PartResourceLibrary.Instance.GetDefinition(abl.ablativeResource);
            PartResource res = p.Resources.Get(abl.ablativeResource);
            if (def == null || res == null || res.maxAmount <= 0.0) return;
            n.AblatorMax = res.amount;
            n.AblatorDensity = def.density;
            n.AblatorPyrolysisLoss = abl.pyrolysisLossFactor * def.specificHeatCapacity;
            n.AblationTempThresh = abl.ablationTempThresh;
            n.LossConst = abl.lossConst;
            n.LossExp = abl.lossExp;
            n.ReentryConductivity = abl.reentryConductivity;
            n.AblatorLossScale = res.maxAmount;   // stock scales the loss rate by the tank's max amount, not what is left
        }
    }

    public interface IReentryAeroFactory
    {
        IReentryAero Create(CelestialBody body, List<ReentryNode> nodes, Vector3 velDir, Vector3 liftAxis, Vector3 forward, Vector3 up,
                            double aoaDeg, bool shieldFirst, ReentryConstants k);
    }
}
