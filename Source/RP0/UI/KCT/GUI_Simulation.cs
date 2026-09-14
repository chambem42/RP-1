using System;
using System.Collections.Generic;
using UnityEngine;
using ROUtils;

namespace RP0
{
    public static partial class KCT_GUI
    {
        private static Rect _simulationWindowPosition = new Rect((Screen.width - 250) / 2, (Screen.height - 250) / 2, 250, 1);
        private static Rect _simulationConfigPosition = new Rect((Screen.width / 2) - 150, (Screen.height / 4), 300, 1);
        private static Vector2 _bodyChooserScrollPos;
        private static CelestialBody _bodyChooserRoot;
        private static Dictionary<CelestialBody, List<CelestialBody>> _bodyChooserChildren;
        private static bool _bodyChooserForOrigin;

        private static string _sOrbitAlt = "", _sOrbitPe = "", _sOrbitAp = "", _sOrbitInc = "", _sOrbitLAN = "", _sOrbitMNA = "", _sOrbitArgPe = "", _UTString = "", _sDelay = "0";
        private static string _sHypInsertionDV = "", _sHypPeAlt = "", _sHypTimeToPe = "", _sHypTransferTime = "", _sAutostageTarget = "";
        private static string _sEntryAlt = "", _sEntrySpeed = "", _sEntryLeadTime = "30", _sPropellantPct = "";
        private static List<ReentrySite> _reentrySites;
        private static bool _showSiteList;
        private static Vector2 _siteListScroll;
        private static readonly string[] _reentryPassLabels = { "Northbound pass", "Southbound pass" };
        private static bool _fromCurrentUT = true;

        // Cached KAC transfer-window lookup for the current hyperbolic target (re-scanned when the target or
        // origin body changes, or when invalidated), to offer a "load from KAC alarm" button. Keyed on the
        // CelestialBody references so the common unchanged path (every OnGUI frame) allocates nothing.
        private static CelestialBody _kacLookupTarget, _kacLookupOrigin;
        private static bool _kacHasWindow;
        private static string _kacDepartsLabel;            // cached "departs in …" text for the load button
        private static object _kacSubscribedInstance;      // the KACAPI instance we're subscribed to (re-init aware)
        private static ModIntegrations.KacTransferWindow _kacWindow;

        // Editor-side preview while the sim config window is up: parts with inverseStage >
        // _sAutostageTarget will be shed (and the shed vessels destroyed) when the sim begins —
        // highlighted red; the parts that will remain on the vessel are highlighted blue.
        private static int _autostageHighlightTarget = int.MinValue;
        private static readonly HashSet<Part> _autostageHighlightedParts = new HashSet<Part>();
        // Fairing parts faded out during the preview so the highlighted payload inside is visible.
        private static readonly HashSet<Part> _fadedFairingParts = new HashSet<Part>();

        public static void DrawSimulationWindow(int windowID)
        {
            GUILayout.BeginVertical();
            GUILayout.Label("This is a simulation.");
            GUILayout.Label("All progress will be lost after leaving the flight scene.");

            if (FlightDriver.CanRevertToPostInit && GUILayout.Button("Restart Simulation"))
            {
                GUIStates.ShowSimulationGUI = false;
                KCTUtilities.EnableSimulationLocks();
                FlightDriver.RevertToLaunch();
                SpaceCenterManagement.Instance.SimulationParams.Reset();
                _centralWindowPosition.height = 1;
            }

            if (FlightDriver.CanRevertToPrelaunch && GUILayout.Button("Revert to Editor"))
            {
                GUIStates.ShowSimulationGUI = false;
                KCTUtilities.DisableSimulationLocks();
                var facility = ShipConstruction.ShipType; // This uses stock behavior because the LaunchedVessel is no longer valid.
                FlightDriver.RevertToPrelaunch(facility);
                _centralWindowPosition.height = 1;
            }

            if (GUILayout.Button("Close"))
            {
                GUIStates.ShowSimulationGUI = !GUIStates.ShowSimulationGUI;
            }
            GUILayout.EndVertical();

            if (_simulationWindowPosition.width > 250)
                _simulationWindowPosition.width = 250;

            CenterWindow(ref _simulationWindowPosition);
        }

        public static void DrawSimulationConfigure(int windowID)
        {
            SimulationParams simParams = SpaceCenterManagement.Instance.SimulationParams;
            GUILayout.BeginVertical();

            if (simParams == null) simParams = SpaceCenterManagement.Instance.SimulationParams = new SimulationParams();
            if (simParams.SimulationBody == null) simParams.SimulationBody = Planetarium.fetch.Home;

            bool isHyperbolic = simParams.SimOrbitMode == SimOrbitMode.Hyperbolic;
            bool isReentry = simParams.SimOrbitMode == SimOrbitMode.Reentry;

            GUILayout.BeginHorizontal();
            GUILayout.Label(isHyperbolic ? "Target body: " : "Body: ");
            GUILayout.Label(simParams.SimulationBody.bodyName);
            if (GUILayout.Button("Select", GUILayout.ExpandWidth(false)))
            {
                _bodyChooserForOrigin = false;
                GUIStates.ShowSimConfig = false;
                GUIStates.ShowSimBodyChooser = true;
                _centralWindowPosition.height = 1;
                _simulationConfigPosition.height = 1;
            }
            GUILayout.EndHorizontal();

            {
                bool prev = simParams.SimulateInOrbit;
                simParams.SimulateInOrbit = GUILayout.Toggle(simParams.SimulateInOrbit, " Start in orbit");
                if (prev && !simParams.SimulateInOrbit)
                {
                    // "Start in orbit" can only be turned off for a pad start at Home. A non-Home target or a
                    // hyperbolic approach both require being in orbit, so reset those to a Home circular start.
                    simParams.SimulationBody = Planetarium.fetch.Home;
                    simParams.SimOrbitMode = SimOrbitMode.Circular;
                    isHyperbolic = false;
                    isReentry = false;
                }
                else if (simParams.SimulationBody != Planetarium.fetch.Home)
                {
                    // A non-Home body always implies starting in orbit (there's no pad to launch from there).
                    simParams.SimulateInOrbit = true;
                }
                if (simParams.SimulateInOrbit != prev)
                    _simulationConfigPosition.height = 1;
            }
            if (simParams.SimulationBody != Planetarium.fetch.Home || simParams.SimulateInOrbit || isHyperbolic || isReentry)
            {
                DrawOrbitModeRow(simParams);
                // Reentry needs an atmosphere to enter; fall back if the body was switched to an airless one.
                if (isReentry && !simParams.SimulationBody.atmosphere)
                {
                    simParams.SimOrbitMode = SimOrbitMode.Elliptical;
                    isReentry = false;
                    _simulationConfigPosition.height = 1;
                }

                if (simParams.SimOrbitMode == SimOrbitMode.Circular)
                {
                    GUILayout.BeginHorizontal();
                    GUILayout.Label("Orbit Altitude (km): ");
                    _sOrbitAlt = GUILayout.TextField(_sOrbitAlt, GUILayout.Width(100));
                    GUILayout.EndHorizontal();
                    GUILayout.BeginHorizontal();
                    GUILayout.Label("Min: " + simParams.SimulationBody.atmosphereDepth / 1000 + "km");
                    GUILayout.Label("Max: " + Math.Floor((simParams.SimulationBody.sphereOfInfluence - simParams.SimulationBody.Radius) / 1000) + "km");
                    GUILayout.EndHorizontal();
                }
                else if (simParams.SimOrbitMode == SimOrbitMode.Elliptical)
                {
                    GUILayout.BeginHorizontal();
                    GUILayout.Label("Orbit Periapsis (km): ");
                    _sOrbitPe = GUILayout.TextField(_sOrbitPe, GUILayout.Width(100));
                    GUILayout.EndHorizontal();
                    GUILayout.BeginHorizontal();
                    GUILayout.Label("Orbit Apoapsis (km): ");
                    _sOrbitAp = GUILayout.TextField(_sOrbitAp, GUILayout.Width(100));
                    GUILayout.EndHorizontal();
                }
                else if (isReentry)
                {
                    DrawReentryConfig(simParams);
                }
                else // Hyperbolic
                {
                    RefreshKacWindowCache(simParams.SimulationBody, simParams.SimOriginBody);
                    if (_kacHasWindow)
                    {
                        GUILayout.BeginHorizontal();
                        if (GUILayout.Button(new GUIContent($"⟳ Load from KAC alarm: {_kacWindow.AlarmName} (in {_kacDepartsLabel})",
                            "Fill origin, departure date, transfer time, insertion ΔV and periapsis from the matching Kerbal Alarm Clock transfer-window alarm.")))
                        {
                            // Re-scan on click so the freshest matching alarm is what gets loaded.
                            InvalidateKacCache();
                            RefreshKacWindowCache(simParams.SimulationBody, simParams.SimOriginBody);
                            if (_kacHasWindow) ApplyKacWindow(simParams);
                        }
                        GUILayout.EndHorizontal();
                    }

                    GUILayout.BeginHorizontal();
                    GUILayout.Label("Origin body: ");
                    GUILayout.Label(simParams.SimOriginBody != null ? simParams.SimOriginBody.bodyName : "(none)");
                    if (GUILayout.Button("Select", GUILayout.ExpandWidth(false)))
                    {
                        _bodyChooserForOrigin = true;
                        GUIStates.ShowSimConfig = false;
                        GUIStates.ShowSimBodyChooser = true;
                        _centralWindowPosition.height = 1;
                        _simulationConfigPosition.height = 1;
                    }
                    GUILayout.EndHorizontal();

                    GUILayout.BeginHorizontal();
                    GUILayout.Label(new GUIContent("Transfer time: ", "Heliocentric transfer duration from the origin body (TWP arrival date − departure date). Used to solve the arrival v∞ direction that orients the approach plane. Accepts \"180d\", \"1y 2d 3h\", or plain seconds."));
                    _sHypTransferTime = GUILayout.TextField(_sHypTransferTime, GUILayout.Width(80));
                    GUILayout.EndHorizontal();

                    GUILayout.BeginHorizontal();
                    GUILayout.Label(new GUIContent("Insertion ΔV (m/s): ", "Insertion / capture ΔV at the target body from a transfer-window planner. Should match the periapsis altitude TWP assumed."));
                    _sHypInsertionDV = GUILayout.TextField(_sHypInsertionDV, GUILayout.Width(80));
                    GUILayout.EndHorizontal();

                    GUILayout.BeginHorizontal();
                    GUILayout.Label("Periapsis altitude (km): ");
                    _sHypPeAlt = GUILayout.TextField(_sHypPeAlt, GUILayout.Width(80));
                    GUILayout.EndHorizontal();
                }

                if (!simParams.SimulateInOrbit) simParams.SimulateInOrbit = true;
            }

            if (simParams.SimulateInOrbit)
            {
                bool aimedAtSite = isReentry && simParams.HasReentrySite;
                if (!aimedAtSite)
                {
                    GUILayout.BeginHorizontal();
                    GUILayout.Label("Delay (s): ");
                    _sDelay = GUILayout.TextField(_sDelay, 3, GUILayout.Width(40));
                    GUILayout.EndHorizontal();
                }

                // In hyperbolic mode these override the transfer-derived approach plane per-axis; leave a
                // field blank to derive that axis from the v∞. (The KAC button fills inclination from TWP.)
                string approachHint = isHyperbolic ? "Leave blank to derive from the transfer."
                    : aimedAtSite ? $"Blank = the lowest inclination that passes over {simParams.SimReentrySiteName}: {Math.Abs(simParams.SimReentrySiteLat):F1} deg (a due-east pass at the site)." : "";
                GUILayout.BeginHorizontal();
                GUILayout.Label(new GUIContent("Inclination (degrees): ", approachHint));
                _sOrbitInc = GUILayout.TextField(_sOrbitInc, GUILayout.Width(50));
                if (aimedAtSite && string.IsNullOrWhiteSpace(_sOrbitInc))
                    GUILayout.Label($"= {Math.Abs(simParams.SimReentrySiteLat):F1} (minimum for the site)");
                GUILayout.EndHorizontal();

                if (!aimedAtSite)
                {
                    GUILayout.BeginHorizontal();
                    GUILayout.Label(new GUIContent("LAN (degrees): ", approachHint));
                    _sOrbitLAN = GUILayout.TextField(_sOrbitLAN, GUILayout.Width(50));
                    GUILayout.EndHorizontal();
                }

                if (isHyperbolic)
                {
                    GUILayout.BeginHorizontal();
                    GUILayout.Label(new GUIContent("Time to periapsis: ", "Seconds before periapsis at which to place the vessel. Accepts \"3h\", \"1d 2h\", or plain seconds. 0 = at periapsis."));
                    _sHypTimeToPe = GUILayout.TextField(_sHypTimeToPe, GUILayout.Width(80));
                    GUILayout.EndHorizontal();
                }
                else
                {
                    if (!isReentry)
                    {
                        // Reentry derives the mean anomaly from the entry interface and lead time.
                        GUILayout.BeginHorizontal();
                        GUILayout.Label("Mean Anomaly (radians): ");
                        _sOrbitMNA = GUILayout.TextField(_sOrbitMNA, GUILayout.Width(50));
                        GUILayout.EndHorizontal();
                    }

                    if (!aimedAtSite)
                    {
                        GUILayout.BeginHorizontal();
                        GUILayout.Label(new GUIContent("Argument of Periapsis (degrees): ",
                            isReentry ? "With inclination and LAN, sets where on the globe the entry happens." : ""));
                        _sOrbitArgPe = GUILayout.TextField(_sOrbitArgPe, GUILayout.Width(50));
                        GUILayout.EndHorizontal();
                    }
                }

                GUILayout.Space(4);
                GUILayout.BeginHorizontal();
                GUILayout.Label(new GUIContent("Autostage to stage: ", "Blank disables autostage. Parts in stages above this number will be shed and destroyed at sim start (highlighted red in the editor; retained parts blue)."));
                _sAutostageTarget = GUILayout.TextField(_sAutostageTarget, 3, GUILayout.Width(40));
                GUILayout.EndHorizontal();
                UpdateAutostageHighlights();
            }

            string s1 = "Valid formats: \"1y 2d 3h 4m 5s\" and \"31719845\".";
            string s2 = "Valid formats: \"1y 2d 3h 4m 5s\", \"31719845\", and \"1960-12-31 23:59:59\".";

            GUILayout.Space(4);
            GUILayout.BeginHorizontal();
            string utFormats = _fromCurrentUT ? s1 : s2;
            string utLabel = isHyperbolic ? "Departure: " : "Time: ";
            string utTooltip = isHyperbolic
                ? "Departure date (the sim warps to arrival = departure + transfer time). " + utFormats
                : utFormats;
            GUILayout.Label(new GUIContent(utLabel, utTooltip));
            _UTString = GUILayout.TextField(_UTString, GUILayout.Width(110));
            _fromCurrentUT = GUILayout.Toggle(_fromCurrentUT, new GUIContent(" From Now", "If selected the game will warp forwards by the entered value. Otherwise the date and time will be set to the entered value."));
            GUILayout.EndHorizontal();
            GUILayout.Space(4);

            if (ModUtils.IsTestFlightInstalled || ModUtils.IsTestLiteInstalled)
            {
                simParams.DisableFailures = !GUILayout.Toggle(!simParams.DisableFailures, " Enable Part Failures (TestFlight or TestLite)");
                GUILayout.Space(4);
            }

            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Simulate"))
            {
                ClearAutostageHighlights();
                StartSim(simParams);
            }

            if (GUILayout.Button("Cancel"))
            {
                ClearAutostageHighlights();
                GUIStates.ShowSimConfig = false;
                _centralWindowPosition.height = 1;
                _unlockEditor = true;
            }
            GUILayout.EndHorizontal();

            GUILayout.EndVertical();
            CheckEditorLock();
            GUI.DragWindow();  // movable instead of locked to screen centre
            ClampWindow(ref _simulationConfigPosition, strict: false);
        }

        private static readonly string[] _orbitModeLabels = { "Circular", "Elliptical", "Hyperbolic", "Reentry" };
        private static readonly string[] _reentryArrivalLabels = { "Apoapsis", "Entry speed" };
        private static readonly string[] _reentryAttitudeLabels = { "Prograde", "Retrograde", "Leave" };

        private static void DrawOrbitModeRow(SimulationParams simParams)
        {
            int prev = (int)simParams.SimOrbitMode;
            int sel = GUILayout.SelectionGrid(prev, _orbitModeLabels, 4);
            if (sel != prev)
            {
                simParams.SimOrbitMode = (SimOrbitMode)sel;
                _simulationConfigPosition.height = 1;
                InvalidateKacCache();
            }
        }

        // Reentry mode: describe the arriving trajectory rather than a parking orbit, and show what the
        // typed numbers actually produce at the entry interface before committing to a sim.
        private static void DrawReentryConfig(SimulationParams simParams)
        {
            CelestialBody body = simParams.SimulationBody;
            LoadReentryFields(simParams);
            if (string.IsNullOrEmpty(_sEntryAlt))
                _sEntryAlt = (body.atmosphereDepth / 1000d).ToString("F0");

            GUILayout.BeginHorizontal();
            GUILayout.Label(new GUIContent("Periapsis (km): ", "Target periapsis of the arriving trajectory - the entry corridor setting. May be inside the atmosphere, or negative."));
            _sOrbitPe = GUILayout.TextField(_sOrbitPe, GUILayout.Width(80));
            GUILayout.EndHorizontal();

            int prevArrival = (int)simParams.SimReentryArrival;
            int selArrival = GUILayout.SelectionGrid(prevArrival, _reentryArrivalLabels, 2);
            if (selArrival != prevArrival)
            {
                simParams.SimReentryArrival = (SimReentryArrival)selArrival;
                _simulationConfigPosition.height = 1;
            }

            GUILayout.BeginHorizontal();
            if (simParams.SimReentryArrival == SimReentryArrival.Apoapsis)
            {
                GUILayout.Label(new GUIContent("Apoapsis (km): ", "Apoapsis of the arriving orbit. Not clamped to a parking-orbit range; must stay inside the SOI."));
                _sOrbitAp = GUILayout.TextField(_sOrbitAp, GUILayout.Width(80));
            }
            else
            {
                GUILayout.Label(new GUIContent("Entry speed (m/s): ", "Inertial speed at the entry interface. Above escape speed gives a hyperbolic arrival - a lunar return to Earth is roughly 11,000 m/s."));
                _sEntrySpeed = GUILayout.TextField(_sEntrySpeed, GUILayout.Width(80));
            }
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label(new GUIContent("Entry interface (km): ", "Altitude the vessel is dropped onto, on the inbound leg. Defaults to the top of the atmosphere."));
            _sEntryAlt = GUILayout.TextField(_sEntryAlt, GUILayout.Width(80));
            GUILayout.EndHorizontal();

            DrawReentrySitePicker(simParams);

            GUILayout.BeginHorizontal();
            GUILayout.Label(new GUIContent("Lead time (s): ", "Coast this long before reaching the entry interface, to set attitude first."));
            _sEntryLeadTime = GUILayout.TextField(_sEntryLeadTime, GUILayout.Width(80));
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label(new GUIContent("Attitude: ", "Once on the trajectory, point the vessel this way and hold it with SAS. Prograde for a nose-first entry, retrograde for heat-shield first."));
            int prevAtt = (int)simParams.SimReentryAttitude;
            int selAtt = GUILayout.SelectionGrid(prevAtt, _reentryAttitudeLabels, 3);
            if (selAtt != prevAtt) simParams.SimReentryAttitude = (SimReentryAttitude)selAtt;
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label(new GUIContent("Propellant remaining (%): ", "Once on the trajectory, set every engine propellant to this share of its tanks. Blank = as built. Use the same number in the editor's reentry analysis."));
            _sPropellantPct = GUILayout.TextField(_sPropellantPct, GUILayout.Width(80));
            GUILayout.EndHorizontal();

            simParams.LogReentryTelemetry = GUILayout.Toggle(simParams.LogReentryTelemetry, new GUIContent(" Log thermal telemetry",
                "Write per-tick heating data to saves/<save>/RP-1_ReentryLogs/*.csv, for comparing a flown entry against the editor's prediction."));

            // Live feedback so the corridor being asked for is visible before the sim starts.
            var entry = default(SimReentryUtils.EntryState);
            bool ok = TryParseReentryInputs(simParams, out double peAlt, out double apAlt, out double entrySpeed,
                                            out double entryAlt, out double leadTime, out string problem);
            if (ok)
                ok = SimReentryUtils.TryComputeEntry(body, peAlt, simParams.SimReentryArrival, apAlt, entrySpeed,
                                                     entryAlt, leadTime, out entry, out problem);
            if (ok)
            {
                string conic = entry.Ecc < 1d
                    ? $"elliptical, Ap {entry.ApoapsisAltitude / 1000d:N0} km"
                    : $"hyperbolic, v-inf {entry.VInf:N0} m/s";
                GUILayout.Label($"Entry: {entry.EntrySpeed:N0} m/s at {entry.EntryFPA:N2} deg, {conic}");
                GUILayout.Label($"Spawns at {entry.SpawnAltitude / 1000d:N0} km, {leadTime:N0} s before interface");
            }
            else
            {
                GUILayout.Label(problem ?? "Invalid reentry setup");
            }
        }

        /// <summary>
        /// Fill the simulation config's Reentry mode from the editor's reentry analysis and open it, so the
        /// predicted corridor can be flown for real.
        /// </summary>
        public static void PrefillReentrySim(double peKm, SimReentryArrival arrival, double apKm, double entrySpeed, bool shieldFirst)
        {
            SimulationParams simParams = SpaceCenterManagement.Instance.SimulationParams;
            if (simParams == null) simParams = SpaceCenterManagement.Instance.SimulationParams = new SimulationParams();
            simParams.SimulationBody = Planetarium.fetch.Home;
            simParams.SimulateInOrbit = true;
            simParams.SimOrbitMode = SimOrbitMode.Reentry;
            simParams.SimReentryArrival = arrival;
            simParams.SimReentryAttitude = shieldFirst ? SimReentryAttitude.Retrograde : SimReentryAttitude.Prograde;
            _sOrbitPe = peKm.ToString("F0");
            _sOrbitAp = apKm.ToString("F0");
            _sEntrySpeed = entrySpeed.ToString("F0");
            _sEntryAlt = "";   // re-defaults to the top of the atmosphere
            GUIStates.ShowSimConfig = true;
            _simulationConfigPosition.height = 1;
            _centralWindowPosition.height = 1;
        }

        // The SimulationParams values persist with the save, but these text boxes are session statics that
        // start empty. Fill them from the saved reentry setup once per session, so a restart does not mean
        // retyping the entry that was being tested. Blank boxes only: never overwrite what the player typed.
        private static bool _reentryFieldsLoaded;

        private static void LoadReentryFields(SimulationParams simParams)
        {
            if (_reentryFieldsLoaded) return;
            _reentryFieldsLoaded = true;
            // Only a setup that was actually used in Reentry mode is worth restoring.
            if (simParams.SimEntryAltitude <= 0d) return;
            if (string.IsNullOrEmpty(_sOrbitPe)) _sOrbitPe = (simParams.SimOrbitPe / 1000d).ToString("F0");
            if (string.IsNullOrEmpty(_sOrbitAp) && simParams.SimOrbitAp > 0d) _sOrbitAp = (simParams.SimOrbitAp / 1000d).ToString("F0");
            if (string.IsNullOrEmpty(_sEntrySpeed) && simParams.SimEntrySpeed > 0d) _sEntrySpeed = simParams.SimEntrySpeed.ToString("F0");
            if (string.IsNullOrEmpty(_sEntryAlt)) _sEntryAlt = (simParams.SimEntryAltitude / 1000d).ToString("F0");
            _sEntryLeadTime = simParams.SimEntryLeadTime.ToString("F0");
            if (string.IsNullOrEmpty(_sPropellantPct) && simParams.SimPropellantPercent >= 0d) _sPropellantPct = simParams.SimPropellantPercent.ToString("F0");
        }

        // Landing-site aim: a dropdown of the home body's sites plus the pass direction. With a site chosen the
        // orbit's LAN and argument of periapsis are derived at placement, so those boxes (and the delay) are hidden.
        private static void DrawReentrySitePicker(SimulationParams simParams)
        {
            if (_reentrySites == null) _reentrySites = SimReentrySite.Collect();
            GUILayout.BeginHorizontal();
            GUILayout.Label(new GUIContent("Aim at: ", "Put the trajectory's periapsis over a landing site. The site moves with the planet while the vessel coasts in; that is accounted for."));
            string current = simParams.HasReentrySite ? simParams.SimReentrySiteName : "(none: LAN and argument of periapsis as typed)";
            if (GUILayout.Button(new GUIContent(current + (_showSiteList ? "  \u25B2" : "  \u25BC")), GUILayout.MinWidth(220)))
            {
                _showSiteList = !_showSiteList;
                if (_showSiteList) _reentrySites = SimReentrySite.Collect();
            }
            GUILayout.EndHorizontal();
            if (_showSiteList)
            {
                _siteListScroll = GUILayout.BeginScrollView(_siteListScroll, GUILayout.Height(Math.Min(200, 24 * (_reentrySites.Count + 2))));
                if (GUILayout.Button("(none: LAN and argument of periapsis as typed)"))
                {
                    simParams.SimReentrySiteName = "";
                    _showSiteList = false;
                }
                foreach (ReentrySite site in _reentrySites)
                {
                    if (GUILayout.Button(new GUIContent(site.Name, $"{site.Latitude:F2}, {site.Longitude:F2}")))
                    {
                        simParams.SimReentrySiteName = site.Name;
                        simParams.SimReentrySiteLat = site.Latitude;
                        simParams.SimReentrySiteLon = site.Longitude;
                        _showSiteList = false;
                        _centralWindowPosition.height = 1;
                    }
                }
                GUILayout.EndScrollView();
            }
            if (simParams.HasReentrySite)
            {
                int pass = GUILayout.SelectionGrid(simParams.SimReentryDescending ? 1 : 0, _reentryPassLabels, 2);
                simParams.SimReentryDescending = pass == 1;
                double lat = Math.Abs(simParams.SimReentrySiteLat);
                if (double.TryParse(_sOrbitInc, out double inc) && (inc < lat - 0.01 || inc > 180d - lat + 0.01))
                    GUILayout.Label($"Inclination must be between {lat:F1} and {180d - lat:F1} deg to pass over {simParams.SimReentrySiteName}.");
            }
        }

        // Parses the reentry text fields. Returns false, with a reason, if any of them is unusable.
        private static bool TryParseReentryInputs(SimulationParams simParams, out double peAlt, out double apAlt,
                                                  out double entrySpeed, out double entryAlt, out double leadTime, out string error)
        {
            peAlt = apAlt = entrySpeed = entryAlt = leadTime = 0d;
            error = null;

            if (!double.TryParse(_sOrbitPe, out peAlt))
            {
                error = "Enter a periapsis.";
                return false;
            }
            peAlt *= 1000d;

            if (simParams.SimReentryArrival == SimReentryArrival.Apoapsis)
            {
                if (!double.TryParse(_sOrbitAp, out apAlt))
                {
                    error = "Enter an apoapsis.";
                    return false;
                }
                apAlt *= 1000d;
            }
            else if (!double.TryParse(_sEntrySpeed, out entrySpeed) || entrySpeed <= 0d)
            {
                error = "Enter a positive entry speed.";
                return false;
            }

            if (!double.TryParse(_sEntryAlt, out entryAlt))
            {
                error = "Enter an entry interface altitude.";
                return false;
            }
            entryAlt *= 1000d;

            if (string.IsNullOrWhiteSpace(_sEntryLeadTime))
                leadTime = SimReentryUtils.DefaultLeadTime;
            else if (!double.TryParse(_sEntryLeadTime, out leadTime) || leadTime < 0d)
            {
                error = "Enter a non-negative lead time.";
                return false;
            }

            return true;
        }

        // Re-scan KAC for a transfer-window alarm to the (target, origin) pair, cached by that key — both
        // hits and misses, so it's not re-run every frame. The cache is invalidated on mode switch, on the
        // load button, when KAC reports an alarm state change, and when KAC (re)initialises its API instance.
        private static void RefreshKacWindowCache(CelestialBody target, CelestialBody origin)
        {
            // (Re)subscribe to the alarm-change event, re-subscribing if KACWrapper swapped its KAC instance
            // (e.g. across a scene load); an unrecognised instance also forces a re-scan (handles late KAC load).
            if (KACWrapper.APIReady && !ReferenceEquals(KACWrapper.KAC, _kacSubscribedInstance))
            {
                if (_kacSubscribedInstance is KACWrapper.KACAPI prev) prev.onAlarmStateChanged -= OnKacAlarmsChanged;
                KACWrapper.KAC.onAlarmStateChanged += OnKacAlarmsChanged;
                _kacSubscribedInstance = KACWrapper.KAC;
                InvalidateKacCache();
            }

            // Cheap reference guard so the common (unchanged) path does no work or allocation each frame.
            if (ReferenceEquals(target, _kacLookupTarget) && ReferenceEquals(origin, _kacLookupOrigin)) return;
            _kacLookupTarget = target;
            _kacLookupOrigin = origin;

            // Acceptable alarm targets: the selected body plus its parents up to the planet that orbits the
            // star, so a moon target (Titan) matches the interplanetary alarm to its parent planet
            // (Earth->Saturn). targetNames[0] stays the selected body so the helper can tell a parent match
            // from a direct one. Home is excluded from the parents: a transfer "to Home" is never the window
            // for a sim, and would otherwise let a moon-of-Home match Earth-return alarms.
            List<string> targetNames = null;
            if (target != null)
            {
                CelestialBody home = Planetarium.fetch.Home;
                targetNames = new List<string>();
                foreach (CelestialBody b in KCTUtilities.AncestorChainToPlanet(target))
                {
                    if (b != target && b == home) break;
                    targetNames.Add(b.bodyName);
                }
            }
            string originName = origin == null ? null : origin.bodyName;
            _kacHasWindow = targetNames != null &&
                ModIntegrations.AlarmHelper.TryGetTransferWindowToTarget(targetNames, originName, out _kacWindow);
            _kacDepartsLabel = _kacHasWindow && !double.IsNaN(_kacWindow.DepartureUT)
                ? RP0DTUtils.PrintDateDeltaCompact(_kacWindow.DepartureUT - Planetarium.GetUniversalTime(), true, false)
                : "?";
        }

        private static void OnKacAlarmsChanged(KACWrapper.KACAPI.AlarmStateChangedEventArgs e) => InvalidateKacCache();

        private static void InvalidateKacCache() => _kacLookupTarget = _kacLookupOrigin = null;

        // Fill the hyperbolic-mode fields from the cached KAC alarm. Departure goes into the Time field
        // as an absolute UT (the hyperbolic Time field is the departure date).
        private static void ApplyKacWindow(SimulationParams simParams)
        {
            var w = _kacWindow;
            CelestialBody origin = string.IsNullOrEmpty(w.OriginBodyName) ? null : FlightGlobals.GetBodyByName(w.OriginBodyName);
            if (origin != null) simParams.SimOriginBody = origin;
            if (!double.IsNaN(w.DepartureUT)) { _UTString = w.DepartureUT.ToString("F0"); _fromCurrentUT = false; }
            if (!double.IsNaN(w.TransferSeconds)) _sHypTransferTime = w.TransferSeconds.ToString("F0");
            if (w.IsParentMatch)
            {
                // A parent (planet) alarm matched a moon target: its insertion ΔV / periapsis are the planet's,
                // not the moon's. Clear both (don't leave a stale value) so the moon sim derives v∞ from the
                // transfer and defaults its periapsis.
                _sHypInsertionDV = "";
                _sHypPeAlt = "";
            }
            else
            {
                if (!double.IsNaN(w.InsertionDV)) _sHypInsertionDV = w.InsertionDV.ToString("F0");
                if (!double.IsNaN(w.PeriapsisAltKm)) _sHypPeAlt = w.PeriapsisAltKm.ToString("F0");
            }
            // Approach plane: clear both inclination and LAN so they're derived from the transfer. TWP
            // reports no insertion LAN, and the derived inclination already matches its insertion inc, so
            // filling one but not the other would just be inconsistent.
            _sOrbitInc = "";
            _sOrbitLAN = "";
            _simulationConfigPosition.height = 1;

            // The departure date and transfer time drive the whole solve; if either couldn't be read from
            // the alarm notes, warn rather than silently leaving a stale value in the field.
            if (double.IsNaN(w.DepartureUT) || double.IsNaN(w.TransferSeconds))
                ScreenMessages.PostScreenMessage(new ScreenMessage(
                    "Couldn't read departure/transfer time from the KAC alarm; enter those manually.", 8f, ScreenMessageStyle.UPPER_CENTER));

            RP0Debug.Log($"Loaded KAC window '{w.AlarmName}': origin={w.OriginBodyName} departUT={w.DepartureUT} tof={w.TransferSeconds} insDV={w.InsertionDV} peAlt={w.PeriapsisAltKm}km");
        }

        public static void DrawBodyChooser(int windowID)
        {
            _bodyChooserScrollPos = GUILayout.BeginScrollView(_bodyChooserScrollPos, GUILayout.Height(500));
            GUILayout.BeginVertical();

            if (_bodyChooserChildren == null)
                BuildBodyChooserHierarchy();

            if (_bodyChooserRoot != null)
            {
                if (GUILayout.Button(_bodyChooserRoot.bodyName))
                    SelectSimBody(_bodyChooserRoot);

                if (_bodyChooserChildren.TryGetValue(_bodyChooserRoot, out var planets))
                {
                    foreach (CelestialBody planet in planets)
                    {
                        if (GUILayout.Button(planet.bodyName))
                            SelectSimBody(planet);

                        if (_bodyChooserChildren.TryGetValue(planet, out var moons))
                        {
                            foreach (CelestialBody moon in moons)
                            {
                                GUILayout.BeginHorizontal();
                                GUILayout.Space(20f);
                                if (GUILayout.Button(moon.bodyName))
                                    SelectSimBody(moon);
                                GUILayout.EndHorizontal();
                            }
                        }
                    }
                }
            }

            GUILayout.EndVertical();
            GUILayout.EndScrollView();

            CheckEditorLock();
            CenterWindow(ref _centralWindowPosition);
        }

        private static void BuildBodyChooserHierarchy()
        {
            _bodyChooserChildren = new Dictionary<CelestialBody, List<CelestialBody>>();
            foreach (CelestialBody body in FlightGlobals.Bodies)
            {
                if (body.orbit == null)
                {
                    _bodyChooserRoot = body;
                }
                else
                {
                    var parent = body.referenceBody;
                    if (!_bodyChooserChildren.TryGetValue(parent, out var list))
                        _bodyChooserChildren[parent] = list = new List<CelestialBody>();
                    list.Add(body);
                }
            }
            foreach (List<CelestialBody> list in _bodyChooserChildren.Values)
            {
                list.Sort((a, b) => a.orbit.semiMajorAxis.CompareTo(b.orbit.semiMajorAxis));
            }
        }

        private static void SelectSimBody(CelestialBody body)
        {
            if (_bodyChooserForOrigin)
                SpaceCenterManagement.Instance.SimulationParams.SimOriginBody = body;
            else
                SpaceCenterManagement.Instance.SimulationParams.SimulationBody = body;
            _bodyChooserForOrigin = false;
            GUIStates.ShowSimBodyChooser = false;
            GUIStates.ShowSimConfig = true;
            _centralWindowPosition.height = 1;
        }

        private static void StartSim(SimulationParams simParams)
        {
            if (SpaceCenterManagement.Instance.IsSimulatedFlight)
            {
                string msg = "Current save already appears to be a simulation. Starting a simulation inside a simulation isn't allowed.";
                PopupDialog.SpawnPopupDialog(new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f), "simErrorPopup", "Simulation error", msg, "Understood", false, HighLogic.UISkin).HideGUIsWhilePopup();
                return;
            }

            if (EditorLogic.fetch.ship.Count == 0)
            {
                var message = new ScreenMessage("Can't simulate without a vessel", 6f, ScreenMessageStyle.UPPER_CENTER);
                ScreenMessages.PostScreenMessage(message);
                return;
            }

            CelestialBody body = simParams.SimulationBody;
            bool isHyperbolic = simParams.SimOrbitMode == SimOrbitMode.Hyperbolic;
            bool isReentry = simParams.SimOrbitMode == SimOrbitMode.Reentry;

            if (body != Planetarium.fetch.Home || isHyperbolic || isReentry)
                simParams.SimulateInOrbit = true;

            if (isHyperbolic && simParams.SimOriginBody == null)
            {
                ScreenMessages.PostScreenMessage(new ScreenMessage("Hyperbolic mode requires an origin body", 6f, ScreenMessageStyle.UPPER_CENTER));
                return;
            }

            if (simParams.SimulateInOrbit)
            {
                if (simParams.SimOrbitMode == SimOrbitMode.Circular)
                {
                    if (!double.TryParse(_sOrbitAlt, out simParams.SimOrbitAltitude))
                        simParams.SimOrbitAltitude = GetDefaultAltitudeForBody(body);
                    else
                        simParams.SimOrbitAltitude = EnsureSafeMaxAltitude(1000 * simParams.SimOrbitAltitude, body);

                    simParams.SimOrbitPe = simParams.SimOrbitAp = 0;
                }
                else if (simParams.SimOrbitMode == SimOrbitMode.Elliptical)
                {
                    if (!double.TryParse(_sOrbitPe, out simParams.SimOrbitPe))
                        simParams.SimOrbitPe = GetDefaultAltitudeForBody(body);

                    if (!double.TryParse(_sOrbitAp, out simParams.SimOrbitAp))
                        simParams.SimOrbitAp = GetDefaultAltitudeForBody(body);

                    simParams.SimOrbitAp = EnsureSafeMaxAltitude(1000 * simParams.SimOrbitAp, body);
                    simParams.SimOrbitPe = Math.Min(1000 * simParams.SimOrbitPe, simParams.SimOrbitAp);

                    simParams.SimOrbitAltitude = 0;
                }
                else if (isReentry)
                {
                    if (!body.atmosphere)
                    {
                        ScreenMessages.PostScreenMessage(new ScreenMessage($"{body.bodyName} has no atmosphere to reenter", 6f, ScreenMessageStyle.UPPER_CENTER));
                        return;
                    }
                    if (!TryParseReentryInputs(simParams, out double peAlt, out double apAlt, out double entrySpeed,
                                               out double entryAlt, out double leadTime, out string problem))
                    {
                        ScreenMessages.PostScreenMessage(new ScreenMessage(problem, 6f, ScreenMessageStyle.UPPER_CENTER));
                        return;
                    }
                    if (!SimReentryUtils.TryComputeEntry(body, peAlt, simParams.SimReentryArrival, apAlt, entrySpeed,
                                                         entryAlt, leadTime, out _, out problem))
                    {
                        ScreenMessages.PostScreenMessage(new ScreenMessage($"Cannot simulate that reentry: {problem}", 6f, ScreenMessageStyle.UPPER_CENTER));
                        return;
                    }

                    // Deliberately unclamped: a periapsis inside the atmosphere is the whole point of this mode.
                    simParams.SimOrbitPe = peAlt;
                    simParams.SimOrbitAp = apAlt;
                    simParams.SimEntrySpeed = entrySpeed;
                    simParams.SimEntryAltitude = entryAlt;
                    simParams.SimEntryLeadTime = leadTime;
                    simParams.SimPropellantPercent = double.TryParse(_sPropellantPct, out double propPct) && propPct >= 0d ? Math.Min(propPct, 100d) : -1d;
                    simParams.SimReentryAimAngleDeg = simParams.SimReentryAimSeconds = double.NaN;
                    if (simParams.HasReentrySite)
                    {
                        // Aim the predicted touchdown at the site, not the periapsis: a lifting vehicle lands well past it.
                        if (ReentryGUI.TryPredictTouchdown(simParams, out double aimAngle, out double aimSeconds, out double downrangeKm, out string aimProblem))
                        {
                            simParams.SimReentryAimAngleDeg = aimAngle;
                            simParams.SimReentryAimSeconds = leadTime + aimSeconds;
                            ScreenMessages.PostScreenMessage($"Aiming at {simParams.SimReentrySiteName}: predicted touchdown {downrangeKm:N0} km past the entry interface at {ReentryGUI.PredictAoA:N0} deg AoA", 8f, ScreenMessageStyle.UPPER_CENTER);
                        }
                        else
                        {
                            ScreenMessages.PostScreenMessage($"No touchdown prediction ({aimProblem}); aiming the periapsis at {simParams.SimReentrySiteName}", 8f, ScreenMessageStyle.UPPER_CENTER);
                        }
                    }
                    simParams.SimOrbitAltitude = 0;
                }
                else // Hyperbolic
                {
                    // Insertion ΔV is optional: blank/0 (e.g. a TWP flyby plan) means "no capture burn", and
                    // the approach speed is taken from the transfer solve instead.
                    if (!double.TryParse(_sHypInsertionDV, out simParams.SimHyperbolicInsertionDV) || simParams.SimHyperbolicInsertionDV < 0)
                        simParams.SimHyperbolicInsertionDV = 0;
                    // Periapsis altitude: blank/0 (a flyby plan reports Pe=0) falls back to a default.
                    if (!double.TryParse(_sHypPeAlt, out simParams.SimHyperbolicPeAlt) || simParams.SimHyperbolicPeAlt <= 0)
                        simParams.SimHyperbolicPeAlt = GetDefaultAltitudeForBody(body) / 1000.0;  // km
                    // For hyperbolic insertion we deliberately allow periapsis BELOW the atmosphere
                    // so the user can simulate aerobraking. Only cap the upper bound at the SOI.
                    simParams.SimHyperbolicPeAlt = Math.Min(
                        Math.Max(1000.0 * simParams.SimHyperbolicPeAlt, 1000.0),
                        body.sphereOfInfluence - body.Radius - 1000.0);
                    simParams.SimOrbitAltitude = simParams.SimOrbitPe = simParams.SimOrbitAp = 0;

                    // Transfer time orients the approach: departure UT = arrival UT − transfer time, used
                    // to Lambert-solve the arrival v∞. Required (the geometry can't be solved without it).
                    if ((!ROUtils.DTUtils.TryParseTimeString(_sHypTransferTime, isTimespan: true, out simParams.SimHypTransferTime)
                         && !double.TryParse(_sHypTransferTime, out simParams.SimHypTransferTime))
                        || simParams.SimHypTransferTime <= 0)
                    {
                        ScreenMessages.PostScreenMessage(new ScreenMessage("Enter a positive transfer time", 6f, ScreenMessageStyle.UPPER_CENTER));
                        return;
                    }
                }

                if (isHyperbolic)
                {
                    // Hyperbolic: a blank inclination/LAN is derived from the transfer (signalled by NaN);
                    // a supplied value overrides that axis of the approach plane.
                    simParams.SimInclination = !string.IsNullOrWhiteSpace(_sOrbitInc) && double.TryParse(_sOrbitInc, out double hypInc)
                        ? hypInc % 360 : double.NaN;
                    simParams.SimLAN = !string.IsNullOrWhiteSpace(_sOrbitLAN) && double.TryParse(_sOrbitLAN, out double hypLan)
                        ? hypLan % 360 : double.NaN;
                }
                else
                {
                    if (isReentry && simParams.HasReentrySite && string.IsNullOrWhiteSpace(_sOrbitInc))
                        simParams.SimInclination = Math.Abs(simParams.SimReentrySiteLat);   // the lowest inclination that reaches the site
                    else if (!double.TryParse(_sOrbitInc, out simParams.SimInclination))
                        simParams.SimInclination = 0;
                    else
                        simParams.SimInclination %= 360;

                    if (string.IsNullOrWhiteSpace(_sOrbitLAN))
                        simParams.SimLAN = double.NaN;  // SetSimOrbit treats NaN as 0
                    else if (!double.TryParse(_sOrbitLAN, out simParams.SimLAN))
                        simParams.SimLAN = 0;
                    else
                        simParams.SimLAN %= 360;
                }

                if (isHyperbolic)
                {
                    // SimMNA is computed from SimHypTimeToPe inside SetSimOrbit (needs sma and μ).
                    // NaN signals "use the earliest time still inside the target's SOI".
                    simParams.SimMNA = 0;
                    simParams.SimArgPe = 0;
                    if (string.IsNullOrWhiteSpace(_sHypTimeToPe))
                    {
                        simParams.SimHypTimeToPe = double.NaN;
                    }
                    else if (!ROUtils.DTUtils.TryParseTimeString(_sHypTimeToPe, isTimespan: true, out double tToPe)
                             && !double.TryParse(_sHypTimeToPe, out tToPe))
                    {
                        ScreenMessages.PostScreenMessage(new ScreenMessage("Couldn't parse time to periapsis; using earliest in SOI", 6f, ScreenMessageStyle.UPPER_CENTER));
                        simParams.SimHypTimeToPe = double.NaN;
                    }
                    else
                    {
                        simParams.SimHypTimeToPe = tToPe;
                    }
                }
                else
                {
                    if (!double.TryParse(_sOrbitMNA, out simParams.SimMNA))
                        simParams.SimMNA = Math.PI; // apoapsis, good for safety
                    else
                        simParams.SimMNA %= 2 * Math.PI;

                    if (!double.TryParse(_sOrbitArgPe, out simParams.SimArgPe))
                        simParams.SimArgPe = 0;
                    else
                        simParams.SimArgPe %= 360;
                }

                if (string.IsNullOrWhiteSpace(_sAutostageTarget) || !int.TryParse(_sAutostageTarget, out simParams.SimAutostageTarget))
                    simParams.SimAutostageTarget = -1;
            }

            double currentUT = Planetarium.GetUniversalTime();
            double ut = 0;
            if (_fromCurrentUT && (_UTString.Contains("-") || _UTString.Contains(":"))) // prevent the user from doing 1960-12-31, accidentally selecting "From Now", and then warping 1960 years forward
            {
                var message = new ScreenMessage("Value must be of format \"1y 2d 3h 4m 5s\" or \"31719845\" when \"From Now\" is selected.", 6f, ScreenMessageStyle.UPPER_CENTER);
                ScreenMessages.PostScreenMessage(message);
                return;
            }
            else if (!string.IsNullOrWhiteSpace(_UTString) &&
                     ((_UTString.Contains(":") && !System.Text.RegularExpressions.Regex.IsMatch(_UTString, @"^\d{4}-\d{2}-\d{2}")) ||
                      !ROUtils.DTUtils.TryParseTimeString(_UTString, isTimespan: !_fromCurrentUT, out ut))) // if string is not empty and ((string has HH:mm but no YYYY-MM-DD) or (string fails TryParseTimeString)), then output failure
            {
                var message = new ScreenMessage("Please enter a valid time value.", 6f, ScreenMessageStyle.UPPER_CENTER);
                ScreenMessages.PostScreenMessage(message);
                return;
            }
            simParams.DelayMoveSeconds = 0;
            if (_fromCurrentUT)
            {
                simParams.SimulationUT = ut != 0 ? currentUT + ut : 0;
            }
            else
            {
                simParams.SimulationUT = ut;
            }

            // Hyperbolic: the Time field is the DEPARTURE date. Shift the sim epoch to arrival
            // (departure + transfer time) so the vessel is placed at the target at arrival, and the
            // Lambert solve in SetSimOrbit samples origin@departure / target@arrival correctly.
            if (isHyperbolic)
            {
                double departureUT = simParams.SimulationUT > 0 ? simParams.SimulationUT : currentUT;
                simParams.SimulationUT = departureUT + simParams.SimHypTransferTime;
            }

            if (simParams.SimulationUT < 0)
            {
                var message = new ScreenMessage("Cannot set time further back than the game start", 6f, ScreenMessageStyle.UPPER_CENTER);
                ScreenMessages.PostScreenMessage(message);
                return;
            }

            if (ModUtils.IsPrincipiaInstalled && simParams.SimulationUT != 0 && simParams.SimulationUT < currentUT + 0.5)
            {
                var message = new ScreenMessage("Going backwards in time isn't allowed with Principia", 6f, ScreenMessageStyle.UPPER_CENTER);
                ScreenMessages.PostScreenMessage(message);
                return;
            }

            int.TryParse(_sDelay, out simParams.DelayMoveSeconds);
            if (simParams.SimulationUT < 0)
                simParams.SimulationUT = currentUT;

            //_unlockEditor = true;
            GUIStates.ShowSimConfig = false;
            _centralWindowPosition.height = 1;
            string tempFile = KSPUtil.ApplicationRootPath + "saves/" + HighLogic.SaveFolder + "/Ships/temp.craft";
            KCTUtilities.MakeSimulationSave();

            // Create the LaunchedVessel fresh instead of cloning the EditorVessel, since it's possible that the player
            // may have changed the vessel slightly since the last time the coroutine updated the EditorVessel.
            SpaceCenterManagement.Instance.LaunchedVessel = new VesselProject(EditorLogic.fetch.ship, EditorLogic.fetch.launchSiteName, EditorLogic.FlagURL, true);
            // Just in case, let's set the LCID
            SpaceCenterManagement.Instance.LaunchedVessel.LCID = SpaceCenterManagement.EditorShipEditingMode ? SpaceCenterManagement.Instance.EditedVessel.LCID : SpaceCenterManagement.Instance.ActiveSC.ActiveLC.ID;

            VesselCrewManifest manifest = KSP.UI.CrewAssignmentDialog.Instance.GetManifest();
            if (manifest == null)
            {
                manifest = HighLogic.CurrentGame.CrewRoster.DefaultCrewForVessel(EditorLogic.fetch.ship.SaveShip(), null, true);
            }
            EditorLogic.fetch.ship.SaveShip().Save(tempFile);
            SpaceCenterManagement.Instance.IsSimulatedFlight = true;
            // EditorLogic.fetch.launchSiteName will always default to LaunchPad or Runway when entering the editor, and can only be changed to a different launch site in the editor,
            // So we can use SpaceCenterManagement.Instance.ActiveSC.ActiveLC.ActiveLPInstance.launchSiteName if it isn't changed
            // If it is changed inside of the editor, then SpaceCenterManagement.Instance.ActiveSC.ActiveLC.ActiveLPInstance.launchSiteName might not match it,
            // So we use EditorLogic.fetch.launchSiteName to align with expected behavior (launch at the site the user picks)

            // There also exists a seemingly stock bug where if a vessel is loaded automatically in one editor after entering it,
            // Then the user switches to the other editor, EditorLogic.fetch.launchSiteName won't run ValidLaunchSite() and will stay stuck as what it was before,
            // So we need to check for ValidLaunchSite() as well.
            // SpaceCenterManagement.Instance.ActiveSC.ActiveLC.ActiveLPInstance.launchSiteName is always valid, but that only applies to the VAB of course
            string launchSiteName = EditorLogic.fetch.launchSiteName;
            if ((launchSiteName == "LaunchPad" || !EditorDriver.ValidLaunchSite(launchSiteName)) && SpaceCenterManagement.Instance.ActiveSC.ActiveLC.LCType == LaunchComplexType.Pad)
            {
                launchSiteName = SpaceCenterManagement.Instance.ActiveSC.ActiveLC.ActiveLPInstance.launchSiteName;
            }
            else if (!EditorDriver.ValidLaunchSite(launchSiteName) && SpaceCenterManagement.Instance.ActiveSC.ActiveLC.LCType == LaunchComplexType.Hangar)
            {
                // If some mechanic is added to select a different default launch site for the SPH, then this will need to be updated to reflect that
                // For now, we can just use "Runway", since that's what the SPH always defaults to when one enters it
                launchSiteName = "Runway";
            }
            SpaceCenterManagement.Instance.StartCoroutine(CallbackUtil.DelayedCallback(1, delegate
            {
                FlightDriver.StartWithNewLaunch(tempFile, EditorLogic.FlagURL, launchSiteName, manifest);
            }));
        }

        private static double EnsureSafeMaxAltitude(double altitudeMeters, CelestialBody body)
        {
            return Math.Min(Math.Max(altitudeMeters, body.atmosphereDepth), body.sphereOfInfluence - body.Radius - 1000);
        }

        private static double GetDefaultAltitudeForBody(CelestialBody body)
        {
            return body.atmosphere ? body.atmosphereDepth + 30000 : 30000;
        }

        private static void UpdateAutostageHighlights()
        {
            int target;
            if (string.IsNullOrWhiteSpace(_sAutostageTarget) || !int.TryParse(_sAutostageTarget, out target) || target < 0)
                target = -1;

            if (target == _autostageHighlightTarget) return;
            _autostageHighlightTarget = target;

            foreach (Part p in _autostageHighlightedParts)
            {
                if (p != null) p.SetHighlightDefault();
            }
            _autostageHighlightedParts.Clear();
            RestoreFairingOpacity();

            if (target < 0 || EditorLogic.fetch == null || EditorLogic.fetch.ship == null) return;

            // Red = will be shed at sim start (inverseStage > target); blue = will remain on the vessel.
            foreach (Part p in EditorLogic.fetch.ship.parts)
            {
                p.highlightType = Part.HighlightType.AlwaysOn;
                p.SetHighlightColor(p.inverseStage > target ? XKCDColors.Red : XKCDColors.Blue);
                p.SetHighlight(true, false);
                if (p.highlighter != null) p.highlighter.SeeThroughOn();
                _autostageHighlightedParts.Add(p);

                // Fade occluding shells (fairings, cargo bays) so the highlighted payload inside is visible.
                if (IsOccludingPart(p))
                {
                    p.SetOpacity(0.15f);
                    _fadedFairingParts.Add(p);
                }
            }
        }

        // True for parts that enclose/occlude others — fairings (by module name, mod-agnostic) and stock
        // cargo bays (ModuleCargoBay, which also covers service bays and fairing-like shrouds derived from it).
        private static bool IsOccludingPart(Part p)
        {
            foreach (PartModule m in p.Modules)
            {
                if (m is ModuleCargoBay) return true;
                string n = m.moduleName;
                if (!string.IsNullOrEmpty(n) && n.IndexOf("Fairing", StringComparison.OrdinalIgnoreCase) >= 0)
                    return true;
            }
            return false;
        }

        private static void RestoreFairingOpacity()
        {
            foreach (Part p in _fadedFairingParts)
            {
                if (p != null) p.SetOpacity(1f);
            }
            _fadedFairingParts.Clear();
        }

        private static void ClearAutostageHighlights()
        {
            foreach (Part p in _autostageHighlightedParts)
            {
                if (p != null) p.SetHighlightDefault();
            }
            _autostageHighlightedParts.Clear();
            RestoreFairingOpacity();
            _autostageHighlightTarget = int.MinValue;
        }
    }
}
