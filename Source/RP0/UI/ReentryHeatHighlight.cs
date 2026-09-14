using System;
using System.Collections.Generic;
using UnityEngine;
using RP0.Reentry;
using KSP.UI.Screens;

namespace RP0
{
    /// <summary>
    /// Tints the editor parts by how close a predicted entry took them to their limits, with the same edge
    /// highlight the flight scene's temperature gauges use (TemperatureGauge: nothing below the highlight
    /// threshold, then red brightening toward orange at the limit). Parts that went over their limit are
    /// drawn as red ghosts. Uses the stock part highlighter, so it clears cleanly.
    /// </summary>
    public static class ReentryHeatHighlight
    {
        private static readonly List<Part> _tinted = new List<Part>();
        private static readonly List<Part> _faded = new List<Part>();
        private static bool _hooked;

        public static bool Active => _tinted.Count > 0 || _faded.Count > 0;
        public const float OverLimitOpacity = 0.25f;

        /// <summary>Apply to every part the analysis knows; parts it skipped (no drag cube, launch clamps) are left alone.</summary>
        public static void Apply(ReentryAttitude att)
        {
            Clear();
            if (att?.Nodes == null) return;
            foreach (ReentryNode n in att.Nodes)
            {
                Part p = n.Part;
                if (p == null) continue;
                double frac = n.WorstFrac;
                if (frac >= 1.0)
                {
                    // Over the limit: the part is gone - a red ghost. Plain edge highlight, not see-through,
                    // which would paint a solid fill over the faded mesh.
                    p.highlightType = Part.HighlightType.AlwaysOn;
                    p.SetHighlightColor(XKCDColors.Red);
                    p.SetHighlight(true, false);
                    p.SetOpacity(OverLimitOpacity);
                    _tinted.Add(p);
                    _faded.Add(p);
                    continue;
                }
                Color? c = ColorFor(p, frac);
                if (!c.HasValue) continue;   // below the gauge threshold: no tint, as in flight
                p.highlightType = Part.HighlightType.AlwaysOn;
                p.SetHighlightColor(c.Value);
                p.SetHighlight(true, false);
                _tinted.Add(p);
            }
            Hook();
        }

        /// <summary>
        /// The flight scene's edge-highlight colour for a peak-over-limit fraction (TemperatureGauge.Update):
        /// none below PhysicsGlobals.TemperatureGaugeHighlightThreshold times the part's own multiplier, then
        /// red scaled by the fraction, running to orange at the limit.
        /// </summary>
        public static Color? ColorFor(Part p, double frac)
        {
            float num = (float)frac;
            float threshold = PhysicsGlobals.TemperatureGaugeHighlightThreshold * (p != null ? p.edgeHighlightThresholdMult : 1f);
            if (num <= threshold) return null;
            float edgeRatio = Mathf.InverseLerp(threshold, 1f, num);
            float colorScale = Mathf.Clamp01(num);
            return Color.Lerp(XKCDColors.Red * colorScale, XKCDColors.KSPNotSoGoodOrange * colorScale, edgeRatio);
        }

        public static void Clear()
        {
            foreach (Part p in _tinted)
            {
                if (p == null) continue;
                p.SetHighlightDefault();
                p.SetHighlight(false, false);
            }
            foreach (Part p in _faded)
                if (p != null) p.SetOpacity(1f);
            _tinted.Clear();
            _faded.Clear();
        }

        // Any change to the ship invalidates the prediction the tint came from. KSP's EventData.Add
        // dereferences the handler's target, so the handlers must live on an instance, not be static.
        private sealed class Listener
        {
            public void OnShipModified(ShipConstruct s) => Clear();
            public void OnEditorLoad(ShipConstruct s, CraftBrowserDialog.LoadType t) => Clear();
            public void OnSceneChange(GameScenes s) { _tinted.Clear(); _faded.Clear(); }
        }
        private static readonly Listener _listener = new Listener();

        private static void Hook()
        {
            if (_hooked) return;
            _hooked = true;
            GameEvents.onEditorShipModified.Add(_listener.OnShipModified);
            GameEvents.onEditorLoad.Add(_listener.OnEditorLoad);
            GameEvents.onGameSceneLoadRequested.Add(_listener.OnSceneChange);
        }
    }
}
