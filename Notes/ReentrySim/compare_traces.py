"""Diff a predicted reentry trace against a flown one, by altitude.

    python compare_traces.py <dir-with-both-csvs> [--ship "Mk2 SSTO Big"] [--maxtime 800]
    python compare_traces.py --pred predicted_X.csv --flown X.csv

Reads the main traces (ReentryTelemetryLogger.HeaderLine format) and, when present, the .parts.csv
companions, and prints: trajectory (time, speed, q, g), aero forces (lift/drag/wing split, sideslip),
the hottest part, and per-tank-instance skin/interior temperatures. All numbers are pred | flown.
"""
import argparse, csv, glob, math, os, sys


def load(path):
    rows = list(csv.DictReader(open(path, newline='')))
    for r in rows:
        for k in r:
            if k != 'part':
                try: r[k] = float(r[k])
                except (TypeError, ValueError): r[k] = float('nan')
    return rows


def newest(pattern):
    xs = [x for x in sorted(glob.glob(pattern), key=os.path.getmtime) if not x.endswith('.parts.csv')]
    return xs[-1] if xs else None


def at(rows, alt):
    for r in rows:
        if r['altitude'] <= alt: return r
    return rows[-1]


def has(rows, col):
    return rows and col in rows[0] and not all(math.isnan(r[col]) for r in rows[:50])


def f(x, w=6, p=1):
    return f"{x:{w}.{p}f}" if x == x else " " * (w - 3) + "nan"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dir', nargs='?')
    ap.add_argument('--ship', default=None)
    ap.add_argument('--pred'); ap.add_argument('--flown')
    ap.add_argument('--maxtime', type=float, default=1e9, help='ignore flown rows after this missionTime (e.g. a post-breakup fragment)')
    ap.add_argument('--alts', default='100,98,96,94,92,90,88,86,84,82,80,78,76,74,72,70,65,60,55,50')
    a = ap.parse_args()
    pred, flown = a.pred, a.flown
    if a.dir:
        ship = a.ship or '*'
        pred = pred or newest(os.path.join(a.dir, f'predicted_{ship}_*.csv'))
        cands = [x for x in glob.glob(os.path.join(a.dir, f'{ship}_*.csv')) if not os.path.basename(x).startswith('predicted_') and not x.endswith('.parts.csv')]
        flown = flown or (sorted(cands, key=os.path.getmtime)[-1] if cands else None)
    if not pred or not flown:
        sys.exit(f"need both traces (pred={pred}, flown={flown})")
    print("pred :", pred); print("flown:", flown)
    p = load(pred)
    fl = [r for r in load(flown) if r['altitude'] > 500 and r['missionTime'] <= a.maxtime]
    alts = [float(x) for x in a.alts.split(',')]

    print(f"\npred rows {len(p)}, ends {p[-1]['altitude']/1000:.1f} km at {p[-1]['missionTime']:.0f} s; "
          f"flown rows {len(fl)}, ends {fl[-1]['altitude']/1000:.1f} km at {fl[-1]['missionTime']:.0f} s")

    print("\n== trajectory (pred | flown) ==")
    print(f"{'alt':>4} | {'t':>11} | {'srfSpeed':>11} | {'q kPa':>13} | {'g':>11} | {'aoa':>11} | {'pitch in':>11} | {'hottest':>44}")
    for alt in alts:
        x, y = at(p, alt * 1000), at(fl, alt * 1000)
        if x['altitude'] > alt * 1000 + 3000 or y['altitude'] > alt * 1000 + 3000: continue
        print(f"{alt:4.0f} | {f(x['missionTime'],5,0)} {f(y['missionTime'],5,0)} | {f(x['srfSpeed'],5,0)} {f(y['srfSpeed'],5,0)} | "
              f"{f(x['dynPreskPa'],6,3)} {f(y['dynPreskPa'],6,3)} | {f(x['geeForce'],5,2)} {f(y['geeForce'],5,2)} | {f(x['aoa'],5,1)} {f(y['aoa'],5,1)} | "
              f"{f(x['trimPitch'],5,2)} {f(y['trimPitch'],5,2)} | {x['part'][:14]:>14} {x['skinTemperature']/x['skinMaxTemp']*100:4.0f}% {y['part'][:14]:>14} {y['skinTemperature']/y['skinMaxTemp']*100:4.0f}%")

    if has(fl, 'liftKN') and has(p, 'liftKN'):
        print("\n== aero forces kN (pred | flown); lift is perpendicular to the airspeed in the symmetry plane ==")
        print(f"{'alt':>4} | {'lift':>13} | {'drag':>13} | {'side f':>6} | {'wing lift':>13} | {'wing drag':>13} | {'sideslip':>8} | {'Cl':>11} | {'Cd':>11}")
        for alt in alts:
            x, y = at(p, alt * 1000), at(fl, alt * 1000)
            if x['altitude'] > alt * 1000 + 3000 or y['altitude'] > alt * 1000 + 3000: continue
            qsx = x['dynPreskPa'] * x['refArea']; qsy = y['dynPreskPa'] * y['refArea']
            print(f"{alt:4.0f} | {f(x['liftKN'])} {f(y['liftKN'])} | {f(x['dragKN'])} {f(y['dragKN'])} | {f(y['sideKN'])} | "
                  f"{f(x['wingLiftKN'])} {f(y['wingLiftKN'])} | {f(x['wingDragKN'])} {f(y['wingDragKN'])} | {f(y['sideslip'],8,2)} | "
                  f"{f(x['liftKN']/qsx if qsx else float('nan'),5,2)} {f(y['liftKN']/qsy if qsy else float('nan'),5,2)} | "
                  f"{f(x['dragKN']/qsx if qsx else float('nan'),5,2)} {f(y['dragKN']/qsy if qsy else float('nan'),5,2)}")

    # parts companions
    pp_path = pred.replace('.csv', '') + '.parts.csv'; fp_path = flown.replace('.csv', '') + '.parts.csv'
    if not (os.path.exists(pp_path) and os.path.exists(fp_path)): return
    pp = load(pp_path); fp = [r for r in load(fp_path) if r['missionTime'] <= a.maxtime]

    # logs written since 2026-09-15 carry the craft's part id; older ones pair by skin thermal mass (nearest)
    by_id = 'partId' in pp[0] and 'partId' in fp[0] if pp and fp else False

    def instances(rows, name):
        keys = {}
        for r in rows:
            if r['part'] != name: continue
            k = int(r['partId']) if by_id else round(r['skinThermalMass'], 0)
            keys.setdefault(k, []).append(r)
        return keys

    names = sorted({r['part'] for r in fp} | {r['part'] for r in pp})
    for name in names:
        pi, fi = instances(pp, name), instances(fp, name)
        if not pi or not fi: continue
        for kf, frows in sorted(fi.items()):
            if by_id:
                if kf not in pi: continue
                kp = kf
            else:
                kp = min(pi.keys(), key=lambda k: abs(k - kf))
            prows = pi[kp]
            ident = f"id {kf}" if by_id else f"skinTM pred {kp:.0f} | flown {kf:.0f} kJ/K"
            print(f"\n== {name} ({ident}; TM {prows[0]['thermalMass']:.0f} | {frows[0]['thermalMass']:.0f}) ==")
            print(f"{'alt':>4} | {'skin K':>13} | {'skin%':>11} | {'int K':>13} | {'int%':>11} | {'s2i kW':>13} | {'conv kW':>15} | {'rad kW':>15} | {'expA':>13} | {'areaMult':>11}")
            for alt in alts:
                # parts logs only carry the hottest few parts per sample, so a part can be absent for a
                # stretch: take the nearest row and skip the altitude when it is not close
                x = min(prows, key=lambda r: abs(r['altitude'] - alt * 1000)); y = min(frows, key=lambda r: abs(r['altitude'] - alt * 1000))
                if abs(x['altitude'] - alt * 1000) > 1500 or abs(y['altitude'] - alt * 1000) > 1500: continue
                print(f"{alt:4.0f} | {f(x['skinTemperature'],6,0)} {f(y['skinTemperature'],6,0)} | {f(x['skinTemperature']/x['skinMaxTemp']*100,5,0)} {f(y['skinTemperature']/y['skinMaxTemp']*100,5,0)} | "
                      f"{f(x['temperature'],6,0)} {f(y['temperature'],6,0)} | {f(x['temperature']/x['maxTemp']*100,5,0)} {f(y['temperature']/y['maxTemp']*100,5,0)} | "
                      f"{f(x['skinToInternalFlux'],6,0)} {f(y['skinToInternalFlux'],6,0)} | {f(x['convectionFlux'],7,0)} {f(y['convectionFlux'],7,0)} | "
                      f"{f(x['radiationFlux'],7,0)} {f(y['radiationFlux'],7,0)} | {f(x['exposedArea'],6,1)} {f(y['exposedArea'],6,1)} | {f(x['convAreaMult'],5,2)} {f(y['convAreaMult'],5,2)}")


if __name__ == '__main__':
    main()
