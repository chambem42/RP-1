using System;
using System.Collections.Generic;

namespace RP0.Reentry
{
    /// <summary>
    /// Snapshot of every constant the thermal chain reads: PhysicsGlobals (tuned by RO's RO_Physics.cfg),
    /// the body's own multipliers, and the difficulty reentry heat scale. Taken once per analysis so the
    /// solver core stays free of KSP statics.
    /// </summary>
    public struct ReentryConstants
    {
        public double NewtonianTemperatureFactor, NewtonianConvectionFactorBase, NewtonianConvectionFactorTotal;
        public double NewtonianDensityExponent, NewtonianVelocityExponent;
        public double NewtonianMachTempLerpStartMach, NewtonianMachTempLerpEndMach, NewtonianMachTempLerpExponent;
        public double MachConvectionFactor, MachConvectionDensityExponent, MachConvectionVelocityExponent;
        public double MachTemperatureScalar, MachTemperatureVelocityExponent;
        public double SpaceTemperature, StefanBoltzmann, StandardSpecificHeatCapacity;
        public double RadiationFactor, ConductionFactor, SkinInternalConductionFactor, SkinSkinConductionFactor;
        public double FullConvectionAreaMin, FullToCrossSectionLerpStart, FullToCrossSectionLerpEnd;
        public double TurbulentConvectionStart, TurbulentConvectionEnd, TurbulentConvectionMult;
        public double ThermalConvergenceFactor;
        public double DragCubeMultiplier, DragMultiplier, BodyLiftMultiplier;
        public bool ConvectionEnabled, RadiationEnabled, ConductionEnabled;

        // Per-body / per-game
        public double ShockTemperatureMultiplier, ConvectionMultiplier, AdiabaticIndex, ReentryHeatScale;

        public static ReentryConstants FromGame(CelestialBody body)
        {
            var c = new ReentryConstants
            {
                NewtonianTemperatureFactor = PhysicsGlobals.NewtonianTemperatureFactor,
                NewtonianConvectionFactorBase = PhysicsGlobals.NewtonianConvectionFactorBase,
                NewtonianConvectionFactorTotal = PhysicsGlobals.NewtonianConvectionFactorTotal,
                NewtonianDensityExponent = PhysicsGlobals.NewtonianDensityExponent,
                NewtonianVelocityExponent = PhysicsGlobals.NewtonianVelocityExponent,
                NewtonianMachTempLerpStartMach = PhysicsGlobals.NewtonianMachTempLerpStartMach,
                NewtonianMachTempLerpEndMach = PhysicsGlobals.NewtonianMachTempLerpEndMach,
                NewtonianMachTempLerpExponent = PhysicsGlobals.NewtonianMachTempLerpExponent,
                MachConvectionFactor = PhysicsGlobals.MachConvectionFactor,
                MachConvectionDensityExponent = PhysicsGlobals.MachConvectionDensityExponent,
                MachConvectionVelocityExponent = PhysicsGlobals.MachConvectionVelocityExponent,
                MachTemperatureScalar = PhysicsGlobals.MachTemperatureScalar,
                MachTemperatureVelocityExponent = PhysicsGlobals.MachTemperatureVelocityExponent,
                SpaceTemperature = PhysicsGlobals.SpaceTemperature,
                StefanBoltzmann = PhysicsGlobals.StefanBoltzmanConstant,
                StandardSpecificHeatCapacity = PhysicsGlobals.StandardSpecificHeatCapacity,
                RadiationFactor = PhysicsGlobals.RadiationFactor,
                ConductionFactor = PhysicsGlobals.ConductionFactor,
                SkinInternalConductionFactor = PhysicsGlobals.SkinInternalConductionFactor,
                SkinSkinConductionFactor = PhysicsGlobals.SkinSkinConductionFactor,
                FullConvectionAreaMin = PhysicsGlobals.FullConvectionAreaMin,
                FullToCrossSectionLerpStart = PhysicsGlobals.FullToCrossSectionLerpStart,
                FullToCrossSectionLerpEnd = PhysicsGlobals.FullToCrossSectionLerpEnd,
                TurbulentConvectionStart = PhysicsGlobals.TurbulentConvectionStart,
                TurbulentConvectionEnd = PhysicsGlobals.TurbulentConvectionEnd,
                TurbulentConvectionMult = PhysicsGlobals.TurbulentConvectionMult,
                ThermalConvergenceFactor = PhysicsGlobals.ThermalConvergenceFactor,
                DragCubeMultiplier = PhysicsGlobals.DragCubeMultiplier,
                DragMultiplier = PhysicsGlobals.DragMultiplier,
                BodyLiftMultiplier = PhysicsGlobals.BodyLiftMultiplier,
                ConvectionEnabled = PhysicsGlobals.ThermalConvectionEnabled,
                RadiationEnabled = PhysicsGlobals.ThermalRadiationEnabled,
                ConductionEnabled = PhysicsGlobals.ThermalConductionEnabled,
                ShockTemperatureMultiplier = body.shockTemperatureMultiplier,
                ConvectionMultiplier = body.convectionMultiplier,
                AdiabaticIndex = body.atmosphereAdiabaticIndex,
                ReentryHeatScale = HighLogic.CurrentGame != null ? HighLogic.CurrentGame.Parameters.Difficulty.ReentryHeatScale : 1.0
            };
            return c;
        }
    }

    /// <summary>
    /// Per-part thermal properties and the per-attitude aero numbers the thermal chain needs. Built once
    /// per attitude by <see cref="ReentryVesselBuilder"/>; the solver only mutates the state fields.
    /// </summary>
    public class ReentryNode
    {
        public string Name;
        public Part Part;

        // Static properties (from Part / ModuleAblator)
        public double ThermalMass;               // kJ/K, internal
        public double SkinThermalMass;           // kJ/K
        public double MaxTemp, SkinMaxTemp;
        public double EmissiveConstant, HeatConvectiveConstant;
        public double HeatConductivity;          // becomes ReentryConductivity when the shield is exposed
        public double SkinInternalConductionMult, SkinSkinConductionMult;
        public double DragCoeff;                 // DragCubes.DragCoeff, for the turbulent-convection pseudo-Re

        // Ablator (stock ModuleAblator); AblatorMax <= 0 means none. AblatorLossScale is the tank's max
        // amount, which stock multiplies the loss rate by.
        public double AblatorMax, AblatorLossScale, AblatorDensity, AblatorPyrolysisLoss;  // units, units, t/unit, kJ/t
        public double AblationTempThresh, LossConst, LossExp, ReentryConductivity;
        public bool AblatorExposed;

        // Per-Mach aero samples (parallel to ReentryAttitude.MachSamples)
        public double[] ExposedArea, RadiativeArea, ConvectionTempMult, ConvectionCoeffMult, ConvectionAreaMult;

        // State
        public double Temperature, SkinTemperature, SkinUnexposedTemperature;
        public double SkinExposedAreaFrac = 1.0;
        public bool Exposed;
        public double AblatorLeft;
        public double PeakSkin, PeakInternal, HeatLoad;

        public void Reset(double startTemp)
        {
            Temperature = SkinTemperature = SkinUnexposedTemperature = startTemp;
            SkinExposedAreaFrac = 1.0;
            Exposed = false;
            AblatorLeft = AblatorMax;
            PeakSkin = PeakInternal = startTemp;
            HeatLoad = 0.0;
        }

        public double SkinFrac => SkinMaxTemp > 0 ? Math.Max(PeakSkin, 0) / SkinMaxTemp : 0;
        public double InternalFrac => MaxTemp > 0 ? Math.Max(PeakInternal, 0) / MaxTemp : 0;
        public double WorstFrac => Math.Max(SkinFrac, InternalFrac);
    }

    /// <summary>Aerodynamic model of the whole vessel in the trajectory plane.</summary>
    public interface IReentryAero
    {
        string Description { get; }
        /// <summary>Drag and signed lift (kN) at the given dynamic pressure (kPa), Mach and pseudo-Reynolds drag multiplier.
        /// Positive lift is toward the vessel's "up" side; the caller flips it for a lift-down attitude.</summary>
        void Forces(double dynPresKPa, double mach, double pseudoReDragMult, out double dragKN, out double liftKN);
    }

    /// <summary>Everything about one attitude: the nodes with their aero samples, plus the vessel aero.</summary>
    public class ReentryAttitude
    {
        public double AoADeg;
        public bool ShieldFirst;
        public double[] MachSamples;
        public List<ReentryNode> Nodes = new List<ReentryNode>();
        public IReentryAero Aero;
        public double MassT;
    }

    public enum ReentryOutcome { Landed, SkippedOut, SkinOverheat, InternalOverheat, AblatorExhausted, TimedOut }

    public class ReentryResult
    {
        public double AoADeg, PeAltitude;
        public bool LiftDown;
        public ReentryOutcome Outcome;
        public double PeakG, PeakSkin, PeakSkinFrac, PeakInternalFrac, HeatLoadKJ, AblatorUsed;
        public double Duration, Downrange, FinalAltitude, FinalSpeed;
        public string LimitingPart;
        public double PeakHeatingSpeed, PeakHeatingAltitude;

        public bool Survived => Outcome == ReentryOutcome.Landed;
        public bool Acceptable(double gLimit) => Survived && PeakG <= gLimit;
        public double Margin => 1.0 - Math.Max(PeakSkinFrac, PeakInternalFrac);
    }
}
