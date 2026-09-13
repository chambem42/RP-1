using System;
using System.Collections.Generic;

namespace RP0.Reentry
{
    /// <summary>
    /// Integrates a planar 3-DOF entry and, alongside it, the stock FlightIntegrator thermal chain for every
    /// part. The thermal formulae are a transcription of FlightIntegrator 1.12 (CalculateShockTemperature,
    /// CalculateConvectiveCoefficient*, SetSkinProperties, PrecalcConvection, UpdateRadiation, the skin/internal
    /// conduction in UpdateConduction, and ModuleAblator.FixedUpdate) with the vessel-level inputs supplied by
    /// the trajectory instead of a live vessel. Part-to-part conduction and sun/body flux are omitted.
    /// </summary>
    public class ReentrySolver
    {
        public const double DtMax = 2.0, DtMin = 0.02;
        public const double MaxTime = 4000.0;

        private readonly CelestialBody _body;
        private readonly ReentryConstants _k;
        private readonly double _atmDepth, _radius, _mu, _surfaceRotSpeed;
        private readonly RealHeatBridge _realHeat;

        /// <summary>Seconds between trace samples when a trace list is supplied, matching the telemetry logger.</summary>
        public const double TraceInterval = 0.5;

        public ReentrySolver(CelestialBody body, ReentryConstants k)
        {
            _body = body;
            _k = k;
            _atmDepth = body.atmosphereDepth;
            _radius = body.Radius;
            _mu = body.gravParameter;
            // The game heats and drags with surface-relative speed. The trajectory is integrated inertially,
            // so for a prograde equatorial entry the airspeed is the inertial speed less the ground's.
            _surfaceRotSpeed = body.rotates && body.rotationPeriod != 0.0 ? 2.0 * Math.PI * body.Radius / Math.Abs(body.rotationPeriod) : 0.0;
            _realHeat = RealHeatBridge.For(body);
        }

        /// <summary>Which heating model the solver is following, for the UI.</summary>
        public string ThermalModelDescription => _realHeat != null ? "RealHeat" : "stock";

        public ReentryResult Simulate(ReentryAttitude att, double entryAlt, double entrySpeed, double entryFpaDeg,
                                      bool liftDown, double startTemp = 293.15, List<ReentryTraceSample> trace = null,
                                      List<ReentryPartSample> partsTrace = null)
        {
            var res = new ReentryResult { AoADeg = att.AoADeg, LiftDown = liftDown, Outcome = ReentryOutcome.TimedOut };
            foreach (ReentryNode n in att.Nodes) n.Reset(startTemp);

            double r = _radius + entryAlt;
            double v = entrySpeed;
            double gamma = entryFpaDeg * Math.PI / 180.0;
            double t = 0.0, downrange = 0.0, peakG = 0.0;
            double liftSign = liftDown ? -1.0 : 1.0;
            bool armed = false;
            double peakFluxRate = 0.0;
            double nextTrace = 0.0;

            while (t < MaxTime)
            {
                double alt = r - _radius;
                if (alt > _atmDepth && armed && gamma > 0.0) { res.Outcome = ReentryOutcome.SkippedOut; break; }
                if (alt <= 0.0) { res.Outcome = ReentryOutcome.Landed; break; }
                if (alt < _atmDepth) armed = true;

                // Airspeed: subtract the ground's rotation from the horizontal component.
                double vAir = Math.Max(1.0, Math.Sqrt(Math.Pow(v * Math.Cos(gamma) - _surfaceRotSpeed * (_radius / r), 2) + Math.Pow(v * Math.Sin(gamma), 2)));

                double pressure = _body.GetPressure(alt);
                double ambientT = _body.GetTemperature(alt);
                double density = pressure > 0.0 ? _body.GetDensity(pressure, ambientT) : 0.0;
                double sos = pressure > 0.0 ? _body.GetSpeedOfSound(pressure, density) : 0.0;
                double mach = sos > 0.0 ? vAir / sos : 0.0;
                double g = _mu / (r * r);
                double q = 0.0005 * density * vAir * vAir;   // kPa, as stock

                double pseudoRe = density * vAir;
                double pseudoReDragMult = density > 0.0 ? PhysicsGlobals.DragCurvePseudoReynolds.Evaluate((float)pseudoRe) : 1.0;
                att.Aero.Forces(q, mach, alt, pseudoReDragMult, out double dragKN, out double liftKN);
                double aDrag = dragKN / att.MassT;
                double aLift = liftSign * liftKN / att.MassT;

                // Step size: fine while decelerating hard or heating hard, coarse in the thin stuff.
                double dt = DtMax;
                if (density > 1e-9) dt = Math.Min(dt, Math.Max(DtMin, 0.5 / Math.Max(aDrag / 9.80665, 0.05)));
                if (vAir > 2000.0 && density > 1e-7) dt = Math.Min(dt, 0.25);
                if (trace != null) dt = Math.Min(dt, TraceInterval);

                double gees = Math.Sqrt(aDrag * aDrag + aLift * aLift) / 9.80665;
                if (gees > peakG) peakG = gees;

                if (density > 0.0)
                {
                    StepThermal(att, dt, density, vAir, mach, q, ambientT, ref peakFluxRate, ref res, alt);
                }

                if (trace != null && t >= nextTrace && density > 0.0)
                {
                    nextTrace = t + TraceInterval;
                    trace.Add(Sample(att, t, alt, vAir, mach, density, q, gees, ambientT, dragKN, liftSign * liftKN));
                    if (partsTrace != null) SampleParts(att, t, alt, partsTrace);
                }

                double dv = (-aDrag - g * Math.Sin(gamma)) * dt;
                double dgamma = (aLift / Math.Max(v, 1.0) + (v / r - g / Math.Max(v, 1.0)) * Math.Cos(gamma)) * dt;
                double dr = v * Math.Sin(gamma) * dt;
                double dtheta = v * Math.Cos(gamma) / r * dt;
                v = Math.Max(1.0, v + dv);
                gamma += dgamma;
                r += dr;
                downrange += dtheta * _radius;
                t += dt;

                if (v < 250.0 && alt < 30000.0) { res.Outcome = ReentryOutcome.Landed; break; }
            }

            ReentryNode worst = null;
            double worstFrac = -1.0;
            double heat = 0.0, abl = 0.0, peakSkin = 0.0, peakSkinFrac = 0.0, peakIntFrac = 0.0;
            foreach (ReentryNode n in att.Nodes)
            {
                heat += n.HeatLoad;
                abl += n.AblatorMax - n.AblatorLeft;
                if (n.PeakSkin > peakSkin) peakSkin = n.PeakSkin;
                if (n.SkinFrac > peakSkinFrac) peakSkinFrac = n.SkinFrac;
                if (n.InternalFrac > peakIntFrac) peakIntFrac = n.InternalFrac;
                if (n.WorstFrac > worstFrac) { worstFrac = n.WorstFrac; worst = n; }
            }
            res.PeakG = peakG;
            res.PeakSkin = peakSkin;
            res.PeakSkinFrac = peakSkinFrac;
            res.PeakInternalFrac = peakIntFrac;
            res.HeatLoadKJ = heat;
            res.AblatorUsed = abl;
            res.Duration = t;
            res.Downrange = downrange;
            res.FinalAltitude = r - _radius;
            res.FinalSpeed = v;
            res.LimitingPart = worst != null ? worst.Name : "";

            if (res.Outcome == ReentryOutcome.Landed || res.Outcome == ReentryOutcome.TimedOut)
            {
                if (peakSkinFrac >= 1.0) res.Outcome = ReentryOutcome.SkinOverheat;
                else if (peakIntFrac >= 1.0) res.Outcome = ReentryOutcome.InternalOverheat;
                else foreach (ReentryNode n in att.Nodes)
                    if (n.AblatorMax > 0.0 && n.AblatorLeft <= 0.0) { res.Outcome = ReentryOutcome.AblatorExhausted; break; }
            }
            else if (peakSkinFrac >= 1.0) res.Outcome = ReentryOutcome.SkinOverheat;
            else if (peakIntFrac >= 1.0) res.Outcome = ReentryOutcome.InternalOverheat;
            return res;
        }

        // ---- vessel-level thermal inputs, as CalculateConstantsAtmosphere ------------------------------

        private double ConvectiveMachLerp(double mach)
        {
            double t = (mach - _k.NewtonianMachTempLerpStartMach) / (_k.NewtonianMachTempLerpEndMach - _k.NewtonianMachTempLerpStartMach);
            if (t < 0.0) t = 0.0; else if (t > 1.0) t = 1.0;
            return Math.Pow(t, _k.NewtonianMachTempLerpExponent);
        }

        private double ShockTemperature(double spd, double machLerp)
        {
            double n = spd * _k.NewtonianTemperatureFactor;
            if (machLerp > 0.0)
            {
                double b = _k.MachTemperatureScalar * Math.Pow(spd, _k.MachTemperatureVelocityExponent);
                n = n + (b - n) * machLerp;
            }
            return n * _k.ReentryHeatScale * _k.ShockTemperatureMultiplier;
        }

        private double ConvectiveCoefficient(double density, double spd, double machLerp)
        {
            double rhoN = density > 1.0 ? density : Math.Pow(density, _k.NewtonianDensityExponent);
            double newton = rhoN * (_k.NewtonianConvectionFactorBase + Math.Pow(spd, _k.NewtonianVelocityExponent)) * _k.NewtonianConvectionFactorTotal;
            double c;
            if (machLerp <= 0.0) c = newton;
            else
            {
                double rhoM = density > 1.0 ? density : Math.Pow(density, _k.MachConvectionDensityExponent);
                double machC = 1E-07 * _k.MachConvectionFactor * rhoM * Math.Pow(spd, _k.MachConvectionVelocityExponent);
                c = machLerp >= 1.0 ? machC : newton + (machC - newton) * machLerp;
            }
            return c * _k.ConvectionMultiplier;
        }

        private double DensityThermalLerp(double density, double mach, double gamma)
        {
            double n = density;
            if (mach > 1.0)
            {
                double m2 = mach * mach;
                n = (gamma + 1.0) * m2 / (2.0 + (gamma - 1.0) * m2) * n;
            }
            if (n < 0.0625) return 1.0 - Math.Sqrt(Math.Sqrt(n));
            if (n < 0.25) return 0.75 - Math.Sqrt(n);
            return 0.0625 / n;
        }

        // ---- per-part thermal step -----------------------------------------------------------------

        private void StepThermal(ReentryAttitude att, double dt, double density, double spd, double mach, double q, double ambientT,
                                 ref double peakFluxRate, ref ReentryResult res, double alt)
        {
            double machLerp = ConvectiveMachLerp(mach);
            double extT, convCoeff, dtl;
            if (_realHeat != null)
            {
                // RealHeat.UpdateThermodynamicsPre: real-gas shock temperature, shock-density lerp with that
                // gas's gamma, and the convective coefficient cut at low dynamic pressure.
                extT = ambientT + _realHeat.TempRise(spd);
                convCoeff = ConvectiveCoefficient(density, spd, machLerp);
                double lerpVal = q * _realHeat.DynamicPressureMultiplier;
                if (lerpVal < 1.0)
                    convCoeff *= _realHeat.MinConvectiveCoefficientMultLowQ + (1.0 - _realHeat.MinConvectiveCoefficientMultLowQ) * lerpVal;
                dtl = DensityThermalLerp(density, mach, _realHeat.Gamma(spd));
            }
            else
            {
                extT = Math.Max(ambientT, ShockTemperature(spd, machLerp));
                convCoeff = ConvectiveCoefficient(density, spd, machLerp);
                dtl = DensityThermalLerp(density, mach, _k.AdiabaticIndex);
            }
            _lastExtT = extT;
            _lastConvCoeff = convCoeff;
            double brt = ambientT + (_k.SpaceTemperature - ambientT) * dtl;
            double brtExposed = extT + (_k.SpaceTemperature - extT) * dtl;
            _lastBrtExposed = brtExposed;
            double pseudoRe = density * spd;
            double turbRecip = 1.0 / (_k.TurbulentConvectionEnd - _k.TurbulentConvectionStart);
            double areaLerp = -_k.FullConvectionAreaMin + (mach - _k.FullToCrossSectionLerpStart) / (_k.FullToCrossSectionLerpEnd - _k.FullToCrossSectionLerpStart);
            if (areaLerp < 0.0) areaLerp = 0.0; else if (areaLerp > 1.0) areaLerp = 1.0;
            double cf = _k.ThermalConvergenceFactor;
            double totalConv = 0.0;

            double[] samples = att.MachSamples;
            int si = SampleIndex(samples, mach, out double sf);

            foreach (ReentryNode n in att.Nodes)
            {
                double radArea = Math.Max(Lerp(n.RadiativeArea, si, sf), 0.001);
                double expArea = Lerp(n.ExposedArea, si, sf);
                double tempMult = Lerp(n.ConvectionTempMult, si, sf);
                double coeffMult = Lerp(n.ConvectionCoeffMult, si, sf);
                double areaMult = Lerp(n.ConvectionAreaMult, si, sf);

                // SetSkinProperties: convection area and the exposed/unexposed skin split.
                double convArea = (radArea + (expArea - radArea) * areaLerp) * areaMult;
                double frac = convArea / radArea;
                bool exposed;
                if (frac < 0.999 && !double.IsNaN(frac))
                {
                    if (frac > 0.001) exposed = true;
                    else { frac = 0.0; exposed = false; }
                }
                else { frac = 1.0; exposed = true; }

                if (exposed)
                {
                    if (!n.Exposed) { n.SkinUnexposedTemperature = n.SkinTemperature; n.Exposed = true; }
                    RedistributeSkin(n, frac);
                }
                else
                {
                    if (n.Exposed) n.SkinTemperature = n.SkinExposedAreaFrac * n.SkinTemperature + (1.0 - n.SkinExposedAreaFrac) * n.SkinUnexposedTemperature;
                    n.Exposed = false;
                    convArea = 0.0;
                    frac = 1.0;
                }
                n.SkinExposedAreaFrac = frac;
                double expMassMult = 1.0 / frac;
                double unexpMassMult = frac < 1.0 ? 1.0 / (1.0 - frac) : 0.0;

                // Per-step invariants (PrecalcConvection / PrecalcRadiation / PrecalcConduction).
                double postShock = ambientT + (extT - ambientT) * tempMult;
                double finalCoeff = 0.0;
                if (_k.ConvectionEnabled && convArea > 0.0)
                {
                    finalCoeff = convCoeff * convArea * 0.001 * n.HeatConvectiveConstant * coeffMult;
                    double partRe = pseudoRe * n.DragCoeff;
                    if (partRe > _k.TurbulentConvectionStart)
                    {
                        if (partRe > _k.TurbulentConvectionEnd) finalCoeff *= _k.TurbulentConvectionMult;
                        else finalCoeff *= 1.0 + (_k.TurbulentConvectionMult - 1.0) * (partRe - _k.TurbulentConvectionStart) * turbRecip;
                    }
                    finalCoeff = Math.Min(finalCoeff, n.SkinThermalMass * frac);
                }

                double emiss = n.EmissiveConstant * _k.RadiationFactor * 0.001;
                double brtExp = brt + (brtExposed - brt) * tempMult;
                double aExp = radArea * frac, aUnexp = radArea * (1.0 - frac);
                bool splitSkin = n.Exposed && frac < 1.0;

                bool ablates = n.AblatorMax > 0.0;
                double conductivity = ablates && n.AblatorExposed ? n.ReentryConductivity : n.HeatConductivity;
                double sic = 0.0, skinSkinK = 0.0;
                if (_k.ConductionEnabled)
                {
                    double half = _k.ConductionFactor * 10.0 * conductivity * 0.5;
                    sic = half * _k.SkinInternalConductionFactor * n.SkinInternalConductionMult;
                    if (splitSkin)
                        skinSkinK = half * n.SkinSkinConductionMult * _k.SkinSkinConductionFactor * Math.Min(frac, 1.0 - frac) * 2.0 * Math.Sqrt(radArea);
                }

                // Explicit step, sub-stepped against the stiffest term acting on the exposed skin: convection
                // is already clamped to one e-fold per second by stock, but ablation near its threshold and
                // radiation at 3000 K are not.
                double skinRecip = 1.0 / n.SkinThermalMass;
                double dExpDT = finalCoeff + 4.0 * Pow3(n.SkinTemperature) * _k.StefanBoltzmann * emiss * aExp;
                if (ablates && n.AblatorLeft > 0.0 && n.SkinTemperature > n.AblationTempThresh)
                    dExpDT += AblationFlux(n, n.SkinTemperature) * (-n.LossExp) / (n.SkinTemperature * n.SkinTemperature);
                double stable = dExpDT > 0.0 ? 0.5 * n.SkinThermalMass * frac / dExpDT : dt;
                int sub = stable >= dt ? 1 : (int)Math.Min(64, Math.Ceiling(dt / Math.Max(stable, 1e-4)));
                double h = dt / sub;
                double convFlux = 0.0, lastRad = 0.0, lastSkinInt = 0.0;

                for (int s = 0; s < sub; ++s)
                {
                    convFlux = (postShock - n.SkinTemperature) * finalCoeff;
                    double expRad = 0.0, unexpRad = 0.0;
                    if (_k.RadiationEnabled)
                    {
                        if (aExp > 0.0) expRad = -(Pow4(n.SkinTemperature) - Pow4(brtExp)) * _k.StefanBoltzmann * emiss * aExp;
                        if (splitSkin && aUnexp > 0.0) unexpRad = -(Pow4(n.SkinUnexposedTemperature) - Pow4(brt)) * _k.StefanBoltzmann * emiss * aUnexp;
                    }

                    double ablFlux = 0.0;
                    if (ablates && n.AblatorLeft > 0.0 && n.SkinTemperature > n.AblationTempThresh)
                    {
                        double rate = n.LossConst * Math.Exp(n.LossExp / n.SkinTemperature) * n.AblatorLossScale;   // units/s
                        double used = rate * h;
                        if (used > n.AblatorLeft) { rate = n.AblatorLeft / h; used = n.AblatorLeft; }
                        n.AblatorLeft -= used;
                        ablFlux = n.AblatorPyrolysisLoss * rate * n.AblatorDensity;                          // kW
                    }

                    double skinInt = aExp * sic * (n.SkinTemperature - n.Temperature);
                    double unexpSkinInt = splitSkin ? aUnexp * sic * (n.SkinUnexposedTemperature - n.Temperature) : 0.0;
                    double skinSkin = splitSkin ? skinSkinK * (n.SkinTemperature - n.SkinUnexposedTemperature) : 0.0;

                    n.Temperature += (skinInt + unexpSkinInt) * cf / n.ThermalMass * h;
                    n.SkinTemperature += ((convFlux + expRad - ablFlux) + (-skinSkin - skinInt) * cf) * skinRecip * expMassMult * h;
                    if (splitSkin)
                        n.SkinUnexposedTemperature += (unexpRad + (skinSkin - unexpSkinInt) * cf) * skinRecip * unexpMassMult * h;

                    if (n.Temperature < _k.SpaceTemperature) n.Temperature = _k.SpaceTemperature;
                    if (n.SkinTemperature < _k.SpaceTemperature) n.SkinTemperature = _k.SpaceTemperature;
                    if (n.SkinUnexposedTemperature < _k.SpaceTemperature) n.SkinUnexposedTemperature = _k.SpaceTemperature;
                    if (convFlux > 0.0) n.HeatLoad += convFlux * h;
                    lastRad = expRad + unexpRad;
                    lastSkinInt = (skinInt + unexpSkinInt) * cf;
                }

                n.LastConvectionFlux = convFlux * cf;
                n.LastRadiationFlux = lastRad;
                n.LastSkinToInternalFlux = lastSkinInt;
                n.LastExposedArea = expArea;
                n.LastRadiativeArea = radArea;
                n.LastTempMult = tempMult;
                n.LastCoeffMult = coeffMult;
                n.LastAreaMult = areaMult;

                if (n.SkinTemperature > n.PeakSkin) n.PeakSkin = n.SkinTemperature;
                if (splitSkin && n.SkinUnexposedTemperature > n.PeakSkin) n.PeakSkin = n.SkinUnexposedTemperature;
                if (n.Temperature > n.PeakInternal) n.PeakInternal = n.Temperature;
                if (convFlux > 0.0) totalConv += convFlux;
            }

            if (totalConv > peakFluxRate)
            {
                peakFluxRate = totalConv;
                res.PeakHeatingSpeed = spd;
                res.PeakHeatingAltitude = alt;
            }
        }

        private double _lastExtT, _lastConvCoeff, _lastBrtExposed;

        // A trace row for the part closest to its limit, mirroring ReentryTelemetryLogger.FindHottestPart.
        private ReentryTraceSample Sample(ReentryAttitude att, double t, double alt, double vAir, double mach, double density,
                                          double q, double gees, double ambientT, double dragKN, double liftKN)
        {
            ReentryNode hot = null;
            double best = double.NegativeInfinity;
            foreach (ReentryNode n in att.Nodes)
            {
                double frac = n.SkinMaxTemp > 0 ? n.SkinTemperature / n.SkinMaxTemp : 0.0;
                if (n.MaxTemp > 0) frac = Math.Max(frac, n.Temperature / n.MaxTemp);
                if (frac > best) { best = frac; hot = n; }
            }
            var s = new ReentryTraceSample
            {
                Time = t, Altitude = alt, Airspeed = vAir, Mach = mach, Density = density, DynPresKPa = q, GeeForce = gees,
                AtmosphericTemperature = ambientT, ExternalTemperature = _lastExtT, ConvectiveCoefficient = _lastConvCoeff,
                Part = hot != null ? (hot.Part != null && hot.Part.partInfo != null ? hot.Part.partInfo.name : hot.Name) : "",
                AoA = att.AoADeg, RefArea = att.Aero.ReferenceArea, BackgroundTempExposed = _lastBrtExposed, TrimPitch = att.Aero.LastTrimPitch,
                Cl = q > 0.0 ? liftKN / (q * att.Aero.ReferenceArea) : 0.0,
                Cd = q > 0.0 ? dragKN / (q * att.Aero.ReferenceArea) : 0.0
            };
            if (hot != null)
            {
                s.SkinTemperature = hot.SkinTemperature; s.SkinMaxTemp = hot.SkinMaxTemp;
                s.Temperature = hot.Temperature; s.MaxTemp = hot.MaxTemp;
                s.ExposedArea = hot.LastExposedArea; s.RadiativeArea = hot.LastRadiativeArea;
                s.ConvectionFlux = hot.LastConvectionFlux; s.RadiationFlux = hot.LastRadiationFlux; s.SkinToInternalFlux = hot.LastSkinToInternalFlux;
            }
            return s;
        }

        /// <summary>Part rows for the hottest few parts plus every engine, matching the flight-side parts log.</summary>
        public const int PartsTraceHottest = 4;

        private static void SampleParts(ReentryAttitude att, double t, double alt, List<ReentryPartSample> partsTrace)
        {
            var picked = new List<ReentryNode>();
            var byFrac = new List<ReentryNode>(att.Nodes);
            byFrac.Sort((a, b) => Math.Max(b.SkinTemperature / b.SkinMaxTemp, b.Temperature / b.MaxTemp)
                                   .CompareTo(Math.Max(a.SkinTemperature / a.SkinMaxTemp, a.Temperature / a.MaxTemp)));
            for (int i = 0; i < byFrac.Count && picked.Count < PartsTraceHottest; ++i) picked.Add(byFrac[i]);
            foreach (ReentryNode n in att.Nodes) if (n.IsEngine && !picked.Contains(n)) picked.Add(n);
            foreach (ReentryNode n in picked)
            {
                partsTrace.Add(new ReentryPartSample
                {
                    Time = t, Altitude = alt,
                    Part = n.Part != null && n.Part.partInfo != null ? n.Part.partInfo.name : n.Name,
                    SkinTemperature = n.SkinTemperature, SkinMaxTemp = n.SkinMaxTemp, Temperature = n.Temperature, MaxTemp = n.MaxTemp,
                    SkinUnexposedTemperature = n.SkinUnexposedTemperature,
                    ExposedArea = n.LastExposedArea, RadiativeArea = n.LastRadiativeArea, SkinExposedAreaFrac = n.SkinExposedAreaFrac,
                    TempMult = n.LastTempMult, CoeffMult = n.LastCoeffMult, AreaMult = n.LastAreaMult,
                    ConvectionFlux = n.LastConvectionFlux, RadiationFlux = n.LastRadiationFlux, SkinToInternalFlux = n.LastSkinToInternalFlux,
                    SkinThermalMass = n.SkinThermalMass, ThermalMass = n.ThermalMass,
                    HeatConvectiveConstant = n.HeatConvectiveConstant, EmissiveConstant = n.EmissiveConstant
                });
            }
        }

        // SetSkinProperties: when the exposed fraction changes, move heat between the two skin nodes so
        // energy is conserved rather than letting the smaller node's temperature jump.
        private static void RedistributeSkin(ReentryNode n, double newFrac)
        {
            double oldFrac = n.SkinExposedAreaFrac;
            if (oldFrac == newFrac || n.SkinUnexposedTemperature == n.SkinTemperature) return;
            if (oldFrac <= 0.0 || oldFrac >= 1.0 || newFrac >= 1.0) return;
            double oldUnexp = 1.0 - oldFrac;
            double eExp = n.SkinTemperature * oldFrac * n.SkinThermalMass;
            double eUnexp = n.SkinUnexposedTemperature * oldUnexp * n.SkinThermalMass;
            double delta = newFrac - oldFrac;
            double moved = delta > 0.0 ? delta / oldUnexp * eUnexp : delta / oldFrac * eExp;
            n.SkinTemperature = (eExp + moved) / (newFrac * n.SkinThermalMass);
            n.SkinUnexposedTemperature = (eUnexp - moved) / ((1.0 - newFrac) * n.SkinThermalMass);
        }

        private static int SampleIndex(double[] samples, double mach, out double f)
        {
            if (mach <= samples[0]) { f = 0.0; return 0; }
            int last = samples.Length - 1;
            if (mach >= samples[last]) { f = 0.0; return last; }
            int i = 0;
            while (i < last - 1 && samples[i + 1] < mach) ++i;
            f = (mach - samples[i]) / (samples[i + 1] - samples[i]);
            return i;
        }

        private static double Lerp(double[] a, int i, double f)
        {
            if (f <= 0.0 || i + 1 >= a.Length) return a[i];
            return a[i] + (a[i + 1] - a[i]) * f;
        }

        private static double AblationFlux(ReentryNode n, double skinTemp)
        {
            return n.AblatorPyrolysisLoss * n.LossConst * Math.Exp(n.LossExp / skinTemp) * n.AblatorLossScale * n.AblatorDensity;
        }

        private static double Pow4(double x) { x *= x; return x * x; }
        private static double Pow3(double x) { return x * x * x; }

        // ---- sweeps ------------------------------------------------------------------------------------

        /// <summary>Entry-interface state for a periapsis + (apoapsis | entry speed) arrival, shared with the sim's Reentry mode.</summary>
        public bool TryEntryState(double peAlt, SimReentryArrival arrival, double apAlt, double entrySpeed, double entryAlt,
                                  out double speed, out double fpaDeg, out string error)
        {
            speed = fpaDeg = 0.0;
            if (!SimReentryUtils.TryComputeEntry(_body, peAlt, arrival, apAlt, entrySpeed, entryAlt, 0.0,
                                                 out SimReentryUtils.EntryState st, out error))
                return false;
            speed = st.EntrySpeed;
            fpaDeg = st.EntryFPA;
            return true;
        }
    }
}
