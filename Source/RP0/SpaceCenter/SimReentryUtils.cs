using System;

namespace RP0
{
    /// <summary>
    /// Orbit maths for the simulation's Reentry mode.
    /// <para>
    /// Circular/Elliptical placements are meant for parking orbits and need a hand-typed mean anomaly;
    /// Hyperbolic is built around an interplanetary transfer solve. Reentry mode instead describes
    /// the <i>arriving</i> trajectory the way a reentry is actually talked about - a periapsis (which
    /// may be inside the atmosphere) plus either an apoapsis or the speed at the entry interface -
    /// and drops the vessel straight onto the inbound leg just before that interface.
    /// </para>
    /// </summary>
    public static class SimReentryUtils
    {
        /// <summary>Default number of seconds of coast before the interface, to let the player set attitude.</summary>
        public const double DefaultLeadTime = 30d;

        public struct EntryState
        {
            public double SMA;
            public double Ecc;
            /// <summary>Mean anomaly (rad) at which the vessel should be spawned.</summary>
            public double MeanAnomaly;
            /// <summary>Inertial speed (m/s) at the entry interface.</summary>
            public double EntrySpeed;
            /// <summary>Flight path angle (deg, negative = descending) at the entry interface.</summary>
            public double EntryFPA;
            /// <summary>Altitude (m) the vessel is actually spawned at, once lead time is applied.</summary>
            public double SpawnAltitude;
            /// <summary>Apoapsis altitude (m) for an elliptical arrival; NaN when hyperbolic.</summary>
            public double ApoapsisAltitude;
            /// <summary>True anomaly (rad, negative inbound) at the entry altitude, for aims measured from it.</summary>
            public double TrueAnomalyAtEntry;
            /// <summary>Hyperbolic excess speed (m/s); 0 when elliptical.</summary>
            public double VInf;
        }

        /// <summary>
        /// Builds the arriving trajectory and the spawn point on it.
        /// </summary>
        /// <param name="body">Body being entered.</param>
        /// <param name="peAlt">Periapsis altitude (m). May be negative or inside the atmosphere - that is the point.</param>
        /// <param name="arrival">Whether <paramref name="apAlt"/> or <paramref name="entrySpeed"/> describes the arrival.</param>
        /// <param name="apAlt">Apoapsis altitude (m) of the arriving orbit.</param>
        /// <param name="entrySpeed">Speed (m/s) at the entry interface. Above escape speed gives a hyperbolic arrival.</param>
        /// <param name="entryAlt">Entry interface altitude (m).</param>
        /// <param name="leadTime">Seconds of coast before the interface to spawn at.</param>
        /// <param name="state">The resulting orbit and spawn point.</param>
        /// <param name="error">Human readable reason the arrival is impossible, when this returns false.</param>
        public static bool TryComputeEntry(CelestialBody body, double peAlt, SimReentryArrival arrival, double apAlt,
                                           double entrySpeed, double entryAlt, double leadTime,
                                           out EntryState state, out string error)
        {
            state = new EntryState();
            error = null;

            if (body == null)
            {
                error = "No body selected.";
                return false;
            }

            double mu = body.gravParameter;
            double radius = body.Radius;
            double rp = radius + peAlt;
            double re = radius + entryAlt;

            if (rp <= 0d)
            {
                error = "Periapsis is below the centre of the body.";
                return false;
            }
            if (re <= rp)
            {
                error = "The entry interface must be above the periapsis.";
                return false;
            }

            double sma;
            if (arrival == SimReentryArrival.EntrySpeed)
            {
                if (entrySpeed <= 0d)
                {
                    error = "Enter a positive entry speed.";
                    return false;
                }
                // vis-viva: a = -mu / (2 * specific energy). Circular-at-interface or slower can't reach a lower periapsis.
                double energy = 0.5d * entrySpeed * entrySpeed - mu / re;
                if (Math.Abs(energy) < 1e-9)
                    energy = 1e-9;    // exactly parabolic: nudge to a barely-hyperbolic arrival
                sma = -mu / (2d * energy);
                double ecc0 = 1d - rp / sma;
                if (ecc0 < 0d)
                {
                    double minSpeed = Math.Sqrt(mu * (2d / re - 2d / (re + rp)));
                    error = $"Entry speed is too low to have come from a periapsis that low; needs at least {minSpeed:N0} m/s.";
                    return false;
                }
            }
            else
            {
                double ra = radius + apAlt;
                if (ra < re)
                {
                    error = "Apoapsis is below the entry interface; the vessel never reaches it.";
                    return false;
                }
                if (ra > body.sphereOfInfluence)
                {
                    // Not clamped the way a parking orbit is, because the point of this mode is to
                    // reach real return speeds - but an apoapsis outside the SOI is not an orbit.
                    error = "Apoapsis is outside the sphere of influence; give an entry speed instead.";
                    return false;
                }
                sma = (ra + rp) * 0.5d;
            }

            double ecc = 1d - rp / sma;
            if (ecc < 0d)
            {
                error = "Apoapsis is below periapsis.";
                return false;
            }

            // True anomaly at the interface, taken on the inbound (negative) branch.
            double semiLatus = sma * (1d - ecc * ecc);
            double cosTa = ecc > 1e-9 ? (semiLatus / re - 1d) / ecc : 1d;
            if (cosTa > 1d) cosTa = 1d;
            else if (cosTa < -1d) cosTa = -1d;
            double trueAnomaly = -Math.Acos(cosTa);

            double meanAnomaly = MeanAnomalyFromTrueAnomaly(trueAnomaly, ecc);

            // Back the spawn point up along the trajectory by the lead time. Mean anomaly advances
            // linearly with time on both conic types, so this is exact rather than an approximation.
            double meanMotion = Math.Sqrt(mu / Math.Abs(sma * sma * sma));
            double spawnMeanAnomaly = meanAnomaly - meanMotion * Math.Max(leadTime, 0d);

            state.SMA = sma;
            state.Ecc = ecc;
            state.MeanAnomaly = spawnMeanAnomaly;
            state.EntrySpeed = Math.Sqrt(Math.Max(mu * (2d / re - 1d / sma), 0d));
            state.EntryFPA = FlightPathAngleDeg(re, sma, mu, rp);
            state.SpawnAltitude = RadiusAtMeanAnomaly(spawnMeanAnomaly, sma, ecc) - radius;
            state.ApoapsisAltitude = ecc < 1d ? sma * (1d + ecc) - radius : double.NaN;
            state.VInf = sma < 0d ? Math.Sqrt(-mu / sma) : 0d;
            return true;
        }

        /// <summary>
        /// The conic through a state below the top of the atmosphere - the inertial speed (m/s) and flight path
        /// angle (deg, negative down) at entryAlt (m) - spawning leadTime seconds before it. Used when the entry
        /// analysis has flown the leg above the start altitude: the drag there leaves the vehicle slower and steeper
        /// than the arrival conic says, and a simulation started on the conic would be optimistic.
        /// </summary>
        public static bool TryComputeEntryFromState(CelestialBody body, double entryAlt, double speed, double fpaDeg, double leadTime,
                                                    out EntryState state, out string error)
        {
            state = new EntryState();
            error = null;
            if (body == null) { error = "No body selected."; return false; }
            if (!(speed > 0d) || double.IsNaN(fpaDeg)) { error = "No entry state to spawn from."; return false; }
            double mu = body.gravParameter;
            double radius = body.Radius;
            double re = radius + entryAlt;
            double gamma = fpaDeg * Math.PI / 180d;
            double energy = 0.5d * speed * speed - mu / re;
            if (Math.Abs(energy) < 1e-9) energy = 1e-9;
            double sma = -mu / (2d * energy);
            double h = re * speed * Math.Cos(gamma);
            double semiLatus = h * h / mu;
            double ecc = Math.Sqrt(Math.Max(0d, 1d - semiLatus / sma));
            double rp = sma * (1d - ecc);
            if (rp <= 0d) { error = "The entry state dives into the centre of the body."; return false; }
            double cosTa = ecc > 1e-9 ? (semiLatus / re - 1d) / ecc : 1d;
            if (cosTa > 1d) cosTa = 1d; else if (cosTa < -1d) cosTa = -1d;
            double trueAnomaly = fpaDeg < 0d ? -Math.Acos(cosTa) : Math.Acos(cosTa);
            double meanAnomaly = MeanAnomalyFromTrueAnomaly(trueAnomaly, ecc);
            double meanMotion = Math.Sqrt(mu / Math.Abs(sma * sma * sma));
            double spawnMeanAnomaly = meanAnomaly - meanMotion * Math.Max(leadTime, 0d);
            state.SMA = sma;
            state.Ecc = ecc;
            state.MeanAnomaly = spawnMeanAnomaly;
            state.EntrySpeed = speed;
            state.EntryFPA = fpaDeg;
            state.SpawnAltitude = RadiusAtMeanAnomaly(spawnMeanAnomaly, sma, ecc) - radius;
            state.ApoapsisAltitude = ecc < 1d ? sma * (1d + ecc) - radius : double.NaN;
            state.VInf = sma < 0d ? Math.Sqrt(-mu / sma) : 0d;
            state.TrueAnomalyAtEntry = trueAnomaly;
            return true;
        }

        /// <summary>Mean anomaly for a true anomaly, on either an elliptical or a hyperbolic conic.</summary>
        public static double MeanAnomalyFromTrueAnomaly(double trueAnomaly, double ecc)
        {
            if (ecc < 1d)
            {
                double eccAnomaly = 2d * Math.Atan2(Math.Sqrt(1d - ecc) * Math.Sin(trueAnomaly * 0.5d),
                                                    Math.Sqrt(1d + ecc) * Math.Cos(trueAnomaly * 0.5d));
                return eccAnomaly - ecc * Math.Sin(eccAnomaly);
            }

            // Hyperbolic: H = 2 atanh( sqrt((e-1)/(e+1)) tan(nu/2) ), M = e sinh H - H
            double t = Math.Sqrt((ecc - 1d) / (ecc + 1d)) * Math.Tan(trueAnomaly * 0.5d);
            if (t >= 1d) t = 1d - 1e-12;
            else if (t <= -1d) t = -1d + 1e-12;
            double h = 2d * Atanh(t);
            return ecc * Math.Sinh(h) - h;
        }

        /// <summary>Radius at a mean anomaly, used only to report the spawn altitude back to the player.</summary>
        public static double RadiusAtMeanAnomaly(double meanAnomaly, double sma, double ecc)
        {
            if (ecc < 1d)
            {
                double e = SolveKepler(meanAnomaly, ecc);
                return sma * (1d - ecc * Math.Cos(e));
            }
            double h = SolveKeplerHyperbolic(meanAnomaly, ecc);
            return sma * (1d - ecc * Math.Cosh(h));
        }

        /// <summary>Flight path angle in degrees; negative because reentry mode always spawns inbound.</summary>
        private static double FlightPathAngleDeg(double r, double sma, double mu, double rp)
        {
            double speed = Math.Sqrt(Math.Max(mu * (2d / r - 1d / sma), 0d));
            double periSpeed = Math.Sqrt(Math.Max(mu * (2d / rp - 1d / sma), 0d));
            if (speed <= 0d) return 0d;
            double cosFpa = rp * periSpeed / (r * speed);    // angular momentum is conserved
            if (cosFpa > 1d) cosFpa = 1d;
            else if (cosFpa < -1d) cosFpa = -1d;
            return -Math.Acos(cosFpa) * (180d / Math.PI);
        }

        private static double SolveKepler(double meanAnomaly, double ecc)
        {
            double e = meanAnomaly;
            for (int i = 0; i < 40; ++i)
            {
                double f = e - ecc * Math.Sin(e) - meanAnomaly;
                double df = 1d - ecc * Math.Cos(e);
                if (Math.Abs(df) < 1e-12) break;
                double step = f / df;
                e -= step;
                if (Math.Abs(step) < 1e-12) break;
            }
            return e;
        }

        private static double SolveKeplerHyperbolic(double meanAnomaly, double ecc)
        {
            double h = Math.Sign(meanAnomaly) * Math.Log(2d * Math.Abs(meanAnomaly) / ecc + 1.8d);
            for (int i = 0; i < 60; ++i)
            {
                double f = ecc * Math.Sinh(h) - h - meanAnomaly;
                double df = ecc * Math.Cosh(h) - 1d;
                if (Math.Abs(df) < 1e-12) break;
                double step = f / df;
                h -= step;
                if (Math.Abs(step) < 1e-12) break;
            }
            return h;
        }

        private static double Atanh(double x)
        {
            return 0.5d * Math.Log((1d + x) / (1d - x));
        }
    }
}
