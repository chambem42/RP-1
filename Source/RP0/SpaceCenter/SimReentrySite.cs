using System;
using System.Collections.Generic;
using UnityEngine;

namespace RP0
{
    /// <summary>A place on the home body a Reentry-mode simulation can be aimed at.</summary>
    public struct ReentrySite
    {
        public string Name;
        public double Latitude, Longitude;
        public override string ToString() => $"{Name} ({Latitude:F2}, {Longitude:F2})";
    }

    /// <summary>
    /// Aiming a Reentry-mode simulation at a landing site: the orbit orientation (LAN, argument of
    /// periapsis) that puts the trajectory's periapsis over the site, at the moment the vessel gets there,
    /// on a chosen inclination and pass direction. Also the list of sites worth offering: the current space
    /// center, KSCSwitcher's launch complexes and every launch site (runways, Kerbal Konstructs bases) the
    /// game knows on the home body.
    /// </summary>
    public static class SimReentrySite
    {
        /// <summary>
        /// Orient the arriving conic so its periapsis passes over the site. The vessel is spawned now at
        /// meanAnomalyAtSpawn (negative, inbound) and reaches periapsis timeToPe later; the site moves with
        /// the body in the meantime, so the aim point is where the site will be then.
        /// </summary>
        /// <param name="descending">True for the southbound pass over the site, false for the northbound one.</param>
        public static bool TryOrient(CelestialBody body, ReentrySite site, double incDeg, bool descending,
                                     double sma, double ecc, double meanAnomalyAtSpawn, double ut,
                                     out double lanDeg, out double argPeDeg, out double timeToPe, out double headingDeg, out string error)
        {
            lanDeg = argPeDeg = timeToPe = headingDeg = 0d;
            error = null;
            double mu = body.gravParameter;
            double n = Math.Sqrt(mu / Math.Pow(Math.Abs(sma), 3));
            timeToPe = Math.Abs(meanAnomalyAtSpawn) / n;

            // The site's direction from the body's centre when the vessel arrives at periapsis.
            Vector3d axis = body.angularVelocity.normalized;
            double spinDeg = body.angularVelocity.magnitude * timeToPe * 180d / Math.PI;
            Vector3d siteNow = (body.GetWorldSurfacePosition(site.Latitude, site.Longitude, 0d) - body.position).normalized;
            Vector3d rHat = (QuaternionD.AngleAxis(spinDeg, axis) * siteNow).normalized;

            double sinLat = Vector3d.Dot(rHat, axis);
            double cosLat = Math.Sqrt(Math.Max(0d, 1d - sinLat * sinLat));
            double inc = incDeg * Math.PI / 180d;
            double cosTheta = cosLat > 1e-9 ? Math.Cos(inc) / cosLat : 2d;
            if (Math.Abs(cosTheta) > 1d)
            {
                error = $"An inclination of {incDeg:F1} deg never passes over {site.Name} (latitude {site.Latitude:F1}); use at least {Math.Abs(site.Latitude):F1} deg (or at most {180d - Math.Abs(site.Latitude):F1} for a retrograde pass).";
                return false;
            }
            double sinTheta = Math.Sqrt(1d - cosTheta * cosTheta);

            // Orbit normal: perpendicular to rHat, at the inclination to the spin axis; the two roots are the
            // northbound and southbound passes.
            Vector3d e1 = (axis - rHat * sinLat).normalized;          // "north" at the site
            Vector3d e2 = Vector3d.Cross(rHat, e1).normalized;         // completes the triad
            Vector3d nHat = e1 * cosTheta + e2 * sinTheta;
            Vector3d vHat = Vector3d.Cross(nHat, rHat).normalized;
            bool northbound = Vector3d.Dot(vHat, axis) > 0d;
            if (northbound == descending)
            {
                nHat = e1 * cosTheta - e2 * sinTheta;
                vHat = Vector3d.Cross(nHat, rHat).normalized;
            }

            double rp = sma * (1d - ecc);
            double vPe = Math.Sqrt(mu * (2d / rp - 1d / sma));
            Vector3d pos = rHat * rp, vel = vHat * vPe;

            // KSP's orbit frame swaps y and z relative to world space.
            var orbit = new Orbit();
            orbit.UpdateFromStateVectors(new Vector3d(pos.x, pos.z, pos.y), new Vector3d(vel.x, vel.z, vel.y), body, ut + timeToPe);
            lanDeg = orbit.LAN;
            argPeDeg = orbit.argumentOfPeriapsis;

            // Ground-track heading at periapsis, for the readout.
            Vector3d east = Vector3d.Cross(axis, rHat).normalized;
            Vector3d north = Vector3d.Cross(rHat, east).normalized;
            headingDeg = (Math.Atan2(Vector3d.Dot(vHat, east), Vector3d.Dot(vHat, north)) * 180d / Math.PI + 360d) % 360d;
            return true;
        }

        /// <summary>Sites on the home body, deduplicated by name, current space center first.</summary>
        public static List<ReentrySite> Collect()
        {
            var list = new List<ReentrySite>();
            var seen = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            CelestialBody home = Planetarium.fetch?.Home;

            if (SpaceCenter.Instance != null)
                Add(list, seen, "Space Center (current)", SpaceCenter.Instance.Latitude, SpaceCenter.Instance.Longitude);

            foreach (var s in KSCSwitcherInterop.GetAvailableSiteCoordinates())
                Add(list, seen, s.displayName, s.lat, s.lon);

            if (PSystemSetup.Instance?.LaunchSites != null && home != null)
            {
                foreach (LaunchSite ls in PSystemSetup.Instance.LaunchSites)
                {
                    if (ls == null || ls.Body != home) continue;
                    if (!TryLatLon(ls, home, out double lat, out double lon)) continue;
                    string name = !string.IsNullOrEmpty(ls.launchSiteName) ? ls.launchSiteName : ls.name;
                    Add(list, seen, name, lat, lon);
                }
            }
            return list;
        }

        private static void Add(List<ReentrySite> list, HashSet<string> seen, string name, double lat, double lon)
        {
            if (string.IsNullOrEmpty(name) || double.IsNaN(lat) || double.IsNaN(lon)) return;
            if (!seen.Add(name)) return;
            list.Add(new ReentrySite { Name = name, Latitude = lat, Longitude = lon });
        }

        private static bool TryLatLon(LaunchSite ls, CelestialBody body, out double lat, out double lon)
        {
            lat = lon = double.NaN;
            try
            {
                if (ls.pqsCity != null) { lat = ls.pqsCity.lat; lon = ls.pqsCity.lon; return true; }
                if (ls.pqsCity2 != null) { lat = ls.pqsCity2.lat; lon = ls.pqsCity2.lon; return true; }
                if (ls.launchSiteTransform != null)
                {
                    Vector3d p = ls.launchSiteTransform.position;
                    lat = body.GetLatitude(p); lon = body.GetLongitude(p);
                    return true;
                }
            }
            catch (Exception) { }
            return false;
        }
    }
}
