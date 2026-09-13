using System;
using System.Collections.Generic;
using UnityEngine;
using RP0.Reentry;
using KSP.UI.Screens;

namespace RP0
{
    /// <summary>
    /// Tints the editor parts by how close a predicted entry took them to their limits: cold blue through
    /// yellow to red at the limit, and parts that went over it drawn half transparent. Uses the stock part
    /// highlighter (the same mechanism as the simulation dialog's autostage preview), so it clears cleanly.
    /// </summary>
    public static class ReentryHeatHighlight
    {
        private static readonly List<Part> _tinted = new List<Part>();
        private static readonly List<Part> _faded = new List<Part>();
        private static bool _hooked;

        public static bool Active => _tinted.Count > 0 || _faded.Count > 0;

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
                p.highlightType = Part.HighlightType.AlwaysOn;
                p.SetHighlightColor(ColorFor(frac));
                p.SetHighlight(true, false);
                if (p.highlighter != null) p.highlighter.SeeThroughOn();
                _tinted.Add(p);
                if (frac >= 1.0)
                {
                    p.SetOpacity(0.4f);
                    _faded.Add(p);
                }
            }
            Hook();
        }

        /// <summary>Peak-over-limit fraction to a colour: blue (cold) - cyan - green - yellow - red at the limit; past it stays red.</summary>
        public static Color ColorFor(double frac)
        {
            float f = (float)Math.Max(0.0, Math.Min(1.0, frac));
            // 0.0 blue, 0.35 cyan/green, 0.7 yellow, 1.0 red
            if (f < 0.35f) return Color.Lerp(new Color(0.15f, 0.3f, 1f), new Color(0.1f, 0.9f, 0.6f), f / 0.35f);
            if (f < 0.7f) return Color.Lerp(new Color(0.1f, 0.9f, 0.6f), new Color(1f, 0.95f, 0.1f), (f - 0.35f) / 0.35f);
            return Color.Lerp(new Color(1f, 0.95f, 0.1f), new Color(1f, 0.05f, 0.05f), (f - 0.7f) / 0.3f);
        }

        public static void Clear()
        {
            foreach (Part p in _tinted)
            {
                if (p == null) continue;
                p.SetHighlightDefault();
                p.SetHighlight(false, false);
                if (p.highlighter != null) p.highlighter.SeeThroughOff();
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
