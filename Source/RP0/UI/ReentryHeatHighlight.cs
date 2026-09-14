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
    /// threshold, then red brightening toward orange at the limit). Parts that went over their limit pulse
    /// red, drawn through whatever encloses them (an engine inside a shroud, a tank under a fairing), the way
    /// the highlighter's see-through mode does for the autostage preview. Uses the stock part highlighter, so
    /// it clears cleanly.
    /// </summary>
    public static class ReentryHeatHighlight
    {
        private static readonly List<Part> _tinted = new List<Part>();
        private static readonly List<Part> _lost = new List<Part>();
        // What the hover tooltip says for each analysed part.
        private static readonly Dictionary<Part, string> _tips = new Dictionary<Part, string>();
        private static readonly GUIContent _tipContent = new GUIContent();
        private static bool _hooked;

        public static bool Active => _tinted.Count > 0;
        private static readonly Color LostBright = new Color(1f, 0.1f, 0.05f, 1f);
        private static readonly Color LostDim = new Color(0.5f, 0f, 0f, 0.35f);
        public const float LostPulseHz = 1.5f;

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
                _tips[p] = Describe(n);
                if (frac >= 1.0)
                {
                    // Over the limit: the part is gone - a pulsing red outline, visible through other parts.
                    p.highlightType = Part.HighlightType.AlwaysOn;
                    p.SetHighlightColor(LostBright);
                    p.SetHighlight(true, false);
                    _tinted.Add(p);
                    _lost.Add(p);
                    PulseLost(p);
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

        private static string Describe(ReentryNode n)
        {
            var sb = new System.Text.StringBuilder();
            sb.Append(n.Name);
            if (n.SkinMaxTemp > 0.0)
                sb.Append($"\nSkin peak {n.PeakSkin:N0} K of {n.SkinMaxTemp:N0} K ({n.SkinFrac * 100:N0}%)")
                  .Append(double.IsNaN(n.PeakSkinAltitude) ? "" : $" at {n.PeakSkinAltitude / 1000:N0} km");
            if (n.MaxTemp > 0.0)
                sb.Append($"\nInterior peak {n.PeakInternal:N0} K of {n.MaxTemp:N0} K ({n.InternalFrac * 100:N0}%)")
                  .Append(double.IsNaN(n.PeakInternalAltitude) ? "" : $" at {n.PeakInternalAltitude / 1000:N0} km");
            if (!double.IsNaN(n.LimitAltitude))
                sb.Append($"\nLOST at {n.LimitAltitude / 1000:N0} km");
            else if (n.WorstFrac >= 0.9)
                sb.Append($"\nWithin {(1.0 - n.WorstFrac) * 100:N0}% of its limit");
            return sb.ToString();
        }

        private static void PulseLost(Part p)
        {
            if (p.highlighter == null) return;
            p.highlighter.FlashingOn(LostDim, LostBright, LostPulseHz);
            p.highlighter.SeeThroughOn();
        }

        /// <summary>
        /// Keep the lost parts pulsing and see-through. The editor's own hover highlighting rewrites a part's
        /// highlighter state (Part.Highlight -> ConstantOn), which would leave a lost part solid after the
        /// mouse passed over it; re-asserting each frame is cheap (the highlighter calls only set flags).
        /// </summary>
        public static void Refresh()
        {
            foreach (Part p in _lost)
                if (p != null) PulseLost(p);
        }

        /// <summary>Draw the hover tooltip for a tinted part; call from an OnGUI.</summary>
        public static void DrawHoverTooltip()
        {
            if (_tips.Count == 0 || Event.current.type != EventType.Repaint) return;
            Part hovered = Mouse.HoveredPart;
            if (hovered == null || !_tips.TryGetValue(hovered, out string tip)) return;
            _tipContent.text = tip;
            GUIStyle style = HighLogic.Skin.box;
            Vector2 size = style.CalcSize(_tipContent);
            Vector2 m = Input.mousePosition;
            float x = Mathf.Min(m.x + 16f, Screen.width - size.x - 4f);
            float y = Mathf.Min(Screen.height - m.y + 16f, Screen.height - size.y - 4f);
            GUI.Box(new Rect(x, y, size.x + 8f, size.y + 4f), _tipContent, style);
        }

        public static void Clear()
        {
            _tips.Clear();
            foreach (Part p in _tinted)
            {
                if (p == null) continue;
                if (p.highlighter != null) { p.highlighter.FlashingOff(); p.highlighter.SeeThroughOff(); }
                p.SetHighlightDefault();
                p.SetHighlight(false, false);
            }
            _tinted.Clear();
            _lost.Clear();
        }

        // Any change to the ship invalidates the prediction the tint came from. KSP's EventData.Add
        // dereferences the handler's target, so the handlers must live on an instance, not be static.
        private sealed class Listener
        {
            public void OnShipModified(ShipConstruct s) => Clear();
            public void OnEditorLoad(ShipConstruct s, CraftBrowserDialog.LoadType t) => Clear();
            public void OnSceneChange(GameScenes s) { _tinted.Clear(); _lost.Clear(); _tips.Clear(); }
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
