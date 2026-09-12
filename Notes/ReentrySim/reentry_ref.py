#!/usr/bin/env python3
"""
Reference implementation of the RP-1 editor-side reentry survivability solver.

This is a standalone, dependency-free prototype of the algorithm proposed in
DESIGN.md.  It exists so that the numerics (integrator, step control, sweep
strategy, corridor search) can be exercised and reviewed without a running
copy of KSP, and so the C# port has something to be checked against.

It is deliberately written in the same shape as the intended C# solver:

    AtmosphereModel   -> CelestialBody.GetPressure/GetTemperature/GetDensity
    PhysicsConstants  -> PhysicsGlobals (values below are RO_Physics.cfg)
    Aero              -> FAR InstantConditionSim (or stock drag cubes)
    ThermalPart       -> Part + PartThermalData
    Solver            -> the new ReentrySolver

Everything marked "CALIBRATE" is a constant whose exact stock value could not
be recovered from public sources; see DESIGN.md section "Calibration".  The
in-game flight logger (reentry mode of the simulation) is what pins them down.

Units follow KSP: mass in tonnes, energy in kJ, power in kW, temperature in K,
length in m, time in s.  Thermal masses are kJ/K.
"""

import math
from dataclasses import dataclass, field

SIGMA = 5.670373e-8  # Stefan-Boltzmann, W/(m^2 K^4)


# --------------------------------------------------------------------------
# PhysicsGlobals -- values are RealismOverhaul's RO_Physics.cfg
# --------------------------------------------------------------------------
@dataclass
class PhysicsConstants:
    spaceTemperature: float = 4.0
    standardSpecificHeatCapacity: float = 800.0   # J/(kg K) == kJ/(t K)

    # Newtonian (subsonic/low supersonic) convection
    newtonianTemperatureFactor: float = 1.0
    newtonianConvectionFactorBase: float = 4.0
    newtonianConvectionFactorTotal: float = 1.5873
    newtonianDensityExponent: float = 0.5
    newtonianVelocityExponent: float = 1.0

    # Newtonian -> hypersonic blend
    newtonianMachTempLerpStartMach: float = 2.0
    newtonianMachTempLerpEndMach: float = 4.0
    newtonianMachTempLerpExponent: float = 3.0

    # Hypersonic convection
    machConvectionDensityExponent: float = 0.5
    machConvectionVelocityExponent: float = 3.0
    machConvectionFactor: float = 1.05
    machTemperatureScalar: float = 7.5
    machTemperatureVelocityExponent: float = 0.75

    # CALIBRATE: the internal scale factor stock applies to convectiveMachFlux
    # before it becomes a W/(m^2 K) coefficient.  See DESIGN.md "Calibration":
    # this is the one constant that could not be recovered from public sources,
    # and a single logged sim reentry pins it down exactly.
    machConvectionFluxScalar: float = 1.0e-4

    # Conduction / radiation
    conductionFactor: float = 5.0
    skinInternalConductionFactor: float = 0.2
    radiationFactor: float = 1.0
    partEmissivityExponent: float = 4.0


# --------------------------------------------------------------------------
# Atmosphere.  Stands in for CelestialBody.GetPressure/GetTemperature/
# GetDensity, which the real solver calls directly (so RSS/any body works).
# This is US-1976 up to 86 km plus an exponential tail, i.e. RSS Earth.
# --------------------------------------------------------------------------
_US76 = [  # (base alt m, base temp K, lapse K/m, base pressure Pa)
    (0.0, 288.15, -0.0065, 101325.0),
    (11000.0, 216.65, 0.0, 22632.06),
    (20000.0, 216.65, 0.001, 5474.889),
    (32000.0, 228.65, 0.0028, 868.0187),
    (47000.0, 270.65, 0.0, 110.9063),
    (51000.0, 270.65, -0.0028, 66.93887),
    (71000.0, 214.65, -0.002, 3.956420),
]
_R_AIR = 287.053


class AtmosphereModel:
    """RSS Earth.  atmosphereDepth is where KSP stops applying drag/heating."""

    def __init__(self, radius=6371000.0, mu=3.986004418e14,
                 atmosphere_depth=140000.0, rotation_period=86164.1):
        self.radius = radius
        self.mu = mu
        self.atmosphere_depth = atmosphere_depth
        self.rotation_period = rotation_period

    def gravity(self, r):
        return self.mu / (r * r)

    def temperature(self, alt):
        if alt >= 86000.0:
            return 186.87
        if alt < 0.0:
            alt = 0.0
        for i in range(len(_US76) - 1, -1, -1):
            h0, t0, lapse, _ = _US76[i]
            if alt >= h0:
                return t0 + lapse * (alt - h0)
        return 288.15

    def pressure(self, alt):
        if alt >= self.atmosphere_depth:
            return 0.0
        if alt >= 86000.0:
            # exponential tail, scale height ~7.2 km at the 86 km temperature
            p86 = self._p76(86000.0)
            return p86 * math.exp(-(alt - 86000.0) / 7200.0)
        return self._p76(max(alt, 0.0))

    def _p76(self, alt):
        for i in range(len(_US76) - 1, -1, -1):
            h0, t0, lapse, p0 = _US76[i]
            if alt >= h0:
                if lapse == 0.0:
                    return p0 * math.exp(-9.80665 * (alt - h0) / (_R_AIR * t0))
                t = t0 + lapse * (alt - h0)
                return p0 * (t / t0) ** (-9.80665 / (_R_AIR * lapse))
        return 101325.0

    def density(self, alt):
        p = self.pressure(alt)
        if p <= 0.0:
            return 0.0
        return p / (_R_AIR * self.temperature(alt))

    def speed_of_sound(self, alt):
        return math.sqrt(1.4 * _R_AIR * self.temperature(alt))


# --------------------------------------------------------------------------
# Aerodynamics.  In the real solver this is a table sampled once per (AoA,
# Mach) from FAR's InstantConditionSim, or from stock drag cubes.  Here it is
# modified-Newtonian for a blunt capsule so that the AoA sweep has the right
# qualitative shape (trim AoA buys L/D, which widens the corridor).
# --------------------------------------------------------------------------
@dataclass
class CapsuleAero:
    ref_area: float = 2.81          # m^2, Mercury-ish
    ca0: float = 1.60               # axial force coefficient at zero AoA
    cn0: float = 3.20               # normal force slope (tuned for L/D ~0.3)

    def coefficients(self, aoa_deg, mach):
        """Return (Cl, Cd) in the velocity frame for |AoA| = aoa_deg."""
        a = math.radians(abs(aoa_deg))
        ca = self.ca0 * math.cos(a) ** 2
        cn = self.cn0 * math.sin(a) * math.cos(a)
        cd = ca * math.cos(a) + cn * math.sin(a)
        cl = abs(cn * math.cos(a) - ca * math.sin(a))
        if mach < 5.0:   # crude subsonic/transonic fade, not the hot part of entry
            cd *= 0.75 + 0.05 * mach
        return cl, cd


# --------------------------------------------------------------------------
# Thermal.  One lumped skin node + one internal node per part, matching the
# stock two-node model.  The convection flux shape is taken verbatim from
# RP-1's own ModuleNonReentryRated, which is the authoritative in-repo
# statement of how stock assembles it:
#
#   postShockExtTemp = Lerp(atmosphericTemperature, externalTemperature,
#                           convectionTempMultiplier)
#   finalCoeff       = convectiveCoefficient * convectionArea * 0.001
#                      * heatConvectiveConstant * convectionCoeffMultiplier
#   convectionFlux   = finalCoeff * (postShockExtTemp - skinTemperature)
# --------------------------------------------------------------------------
@dataclass
class ThermalPart:
    name: str
    mass_t: float                   # tonnes (dry + resources)
    area: float                     # m^2 radiative/convective area
    skin_mass_per_area: float = 5.0        # kg/m^2
    skin_thermal_mass_modifier: float = 1.5
    thermal_mass_modifier: float = 1.0
    skin_internal_conduction_mult: float = 0.1
    emissive_constant: float = 0.4
    heat_convective_constant: float = 1.0
    skin_max_temp: float = 3600.0
    max_temp: float = 1144.0

    # PartThermalData occlusion multipliers.  1.0 == fully exposed to the
    # stagnation-region shock layer; a leeward or shielded part sees less.
    # RP-1's ModuleNonReentryRated floors all three at 0.75 for parts that are
    # not reentry rated, which is what makes those parts burn.
    convection_temp_mult: float = 1.0
    convection_coeff_mult: float = 1.0
    convection_area_mult: float = 1.0

    # ModuleAblator (stock) -- RO "Lunar" shield values by default
    ablator_units: float = 0.0
    ablation_temp_thresh: float = 1250.0
    loss_const: float = 150.0
    loss_exp: float = -25000.0
    pyrolysis_loss_factor: float = 145833.0
    reentry_conductivity: float = 0.0025

    # state
    skin_temp: float = 300.0
    internal_temp: float = 300.0
    ablator_left: float = 0.0
    peak_skin: float = 0.0
    peak_internal: float = 0.0
    heat_load: float = 0.0          # kJ absorbed through the skin

    def reset(self, t0=300.0):
        self.skin_temp = t0
        self.internal_temp = t0
        self.ablator_left = self.ablator_units
        self.peak_skin = t0
        self.peak_internal = t0
        self.heat_load = 0.0

    @property
    def skin_thermal_mass(self):
        # tonnes of skin * specific heat * modifier  -> kJ/K
        return max(1e-6, (self.skin_mass_per_area * self.area / 1000.0)
                   * 800.0 * self.skin_thermal_mass_modifier)

    @property
    def internal_thermal_mass(self):
        return max(1e-6, self.mass_t * 800.0 * self.thermal_mass_modifier)

    @property
    def skin_frac(self):
        return min(0.99, self.skin_temp / self.skin_max_temp)

    @property
    def internal_frac(self):
        return min(0.99, self.internal_temp / self.max_temp)


class ThermalSolver:
    def __init__(self, consts: PhysicsConstants):
        self.k = consts

    def shock_temperature(self, ambient_t, speed, mach):
        """FlightIntegrator.CalculateShockTemperature."""
        k = self.k
        newtonian = speed * k.newtonianTemperatureFactor
        hypersonic = k.machTemperatureScalar * speed ** k.machTemperatureVelocityExponent
        if mach <= k.newtonianMachTempLerpStartMach:
            shock = newtonian
        elif mach >= k.newtonianMachTempLerpEndMach:
            shock = hypersonic
        else:
            f = ((mach - k.newtonianMachTempLerpStartMach)
                 / (k.newtonianMachTempLerpEndMach - k.newtonianMachTempLerpStartMach))
            f = f ** k.newtonianMachTempLerpExponent
            shock = newtonian + (hypersonic - newtonian) * f
        return ambient_t + shock

    def convective_coefficient(self, density, speed, mach, shock_delta):
        """Lerp of CalculateConvectiveCoefficientNewtonian/Mach."""
        k = self.k
        newtonian = (density ** k.newtonianDensityExponent
                     * (k.newtonianConvectionFactorBase
                        + speed ** k.newtonianVelocityExponent)
                     * k.newtonianConvectionFactorTotal)
        mach_flux = (k.machConvectionFluxScalar * k.machConvectionFactor
                     * density ** k.machConvectionDensityExponent
                     * speed ** k.machConvectionVelocityExponent)
        hypersonic = mach_flux / max(shock_delta, 1.0)
        if mach <= k.newtonianMachTempLerpStartMach:
            return newtonian
        if mach >= k.newtonianMachTempLerpEndMach:
            return hypersonic
        f = ((mach - k.newtonianMachTempLerpStartMach)
             / (k.newtonianMachTempLerpEndMach - k.newtonianMachTempLerpStartMach))
        f = f ** k.newtonianMachTempLerpExponent
        return newtonian + (hypersonic - newtonian) * f

    def density_thermal_lerp(self, density):
        """FlightIntegrator.CalculateDensityThermalLerp (via RealHeat)."""
        if density < 0.0625:
            return 1.0 - math.sqrt(math.sqrt(density))
        if density < 0.25:
            return 0.75 - math.sqrt(density)
        return 0.0625 / density

    def step(self, part: ThermalPart, dt, ambient_t, ext_t, coeff, density):
        """Integrate one part's skin and internal node for dt seconds."""
        k = self.k
        # Convection into the skin (kW), assembled exactly as stock does.
        post_shock = ambient_t + (ext_t - ambient_t) * part.convection_temp_mult
        conv = (coeff * part.area * part.convection_area_mult * 0.001
                * part.heat_convective_constant * part.convection_coeff_mult
                * (post_shock - part.skin_temp))

        # Re-radiation from the skin (kW).
        lerp = self.density_thermal_lerp(density)
        bg = ambient_t + (k.spaceTemperature - ambient_t) * lerp
        rad = (part.emissive_constant * SIGMA * part.area
               * (part.skin_temp ** 4 - bg ** 4) * 0.001 * k.radiationFactor)

        # Ablation (stock ModuleAblator) -- a strong thermostat near 2500-3000 K.
        abl = 0.0
        cond_mult = part.skin_internal_conduction_mult
        if part.ablator_left > 0.0 and part.skin_temp > part.ablation_temp_thresh:
            rate = part.loss_const * math.exp(part.loss_exp / part.skin_temp)
            rate = min(rate, part.ablator_left / dt)
            abl = rate * part.pyrolysis_loss_factor * 0.001   # kW
            part.ablator_left -= rate * dt
            cond_mult = part.reentry_conductivity

        # Skin -> internal conduction (kW).
        cond = ((part.skin_temp - part.internal_temp)
                * k.skinInternalConductionFactor * k.conductionFactor
                * cond_mult * part.area * 0.001)

        d_skin = (conv - rad - abl - cond) / part.skin_thermal_mass
        d_int = cond / part.internal_thermal_mass

        part.skin_temp = max(4.0, part.skin_temp + d_skin * dt)
        part.internal_temp = max(4.0, part.internal_temp + d_int * dt)
        part.heat_load += max(conv, 0.0) * dt
        part.peak_skin = max(part.peak_skin, part.skin_temp)
        part.peak_internal = max(part.peak_internal, part.internal_temp)
        return conv


# --------------------------------------------------------------------------
# Trajectory + thermal integration
# --------------------------------------------------------------------------
@dataclass
class EntryResult:
    survived: bool = False
    outcome: str = ""
    peak_skin: float = 0.0
    peak_skin_frac: float = 0.0
    peak_internal_frac: float = 0.0
    peak_g: float = 0.0
    heat_load: float = 0.0
    ablator_used: float = 0.0
    duration: float = 0.0
    downrange: float = 0.0
    final_alt: float = 0.0
    final_speed: float = 0.0
    limiting_part: str = ""


@dataclass
class Vehicle:
    mass_t: float
    aero: CapsuleAero
    parts: list = field(default_factory=list)


def simulate_entry(body: AtmosphereModel, vehicle: Vehicle, consts: PhysicsConstants,
                   entry_alt, entry_speed, entry_fpa_deg, aoa_deg, bank_deg=0.0,
                   max_time=3000.0, dt_max=2.0, dt_min=0.02):
    """3-DOF planar entry, adaptive step on altitude-rate and heating rate."""
    thermal = ThermalSolver(consts)
    for p in vehicle.parts:
        p.reset()

    r = body.radius + entry_alt
    v = entry_speed
    gamma = math.radians(entry_fpa_deg)
    t = 0.0
    downrange = 0.0
    peak_g = 0.0
    cos_bank = math.cos(math.radians(bank_deg))

    result = EntryResult()
    apoapsis_check_armed = False

    while t < max_time:
        alt = r - body.radius
        if alt > body.atmosphere_depth and apoapsis_check_armed and gamma > 0:
            result.outcome = "skipped out of the atmosphere"
            break
        if alt <= 0.0:
            result.outcome = "reached the surface"
            break
        if alt < body.atmosphere_depth:
            apoapsis_check_armed = True

        rho = body.density(alt)
        a_snd = body.speed_of_sound(alt)
        mach = v / a_snd if a_snd > 0 else 0.0
        g = body.gravity(r)

        cl, cd = vehicle.aero.coefficients(aoa_deg, mach)
        q = 0.5 * rho * v * v
        drag = q * cd * vehicle.aero.ref_area / 1000.0        # kN
        lift = q * cl * vehicle.aero.ref_area / 1000.0        # kN
        a_drag = drag / vehicle.mass_t                         # m/s^2
        a_lift = lift / vehicle.mass_t

        # adaptive step: fine where the air is thick or we are hot
        dt = dt_max
        if rho > 1e-9:
            dt = min(dt, max(dt_min, 0.5 / max(a_drag / 9.80665, 0.05)))
        if v > 2000.0 and rho > 1e-7:
            dt = min(dt, 0.25)

        ambient = body.temperature(alt)
        ext = thermal.shock_temperature(ambient, v, mach)
        coeff = thermal.convective_coefficient(rho, v, mach, ext - ambient)
        for p in vehicle.parts:
            thermal.step(p, dt, ambient, ext, coeff, rho)

        gees = math.hypot(a_drag, a_lift) / 9.80665
        peak_g = max(peak_g, gees)

        dv = (-a_drag - g * math.sin(gamma)) * dt
        dgamma = ((a_lift * cos_bank) / max(v, 1.0)
                  + (v / r - g / max(v, 1.0)) * math.cos(gamma)) * dt
        dr = v * math.sin(gamma) * dt
        dtheta = v * math.cos(gamma) / r * dt

        v = max(1.0, v + dv)
        gamma += dgamma
        r += dr
        downrange += dtheta * body.radius
        t += dt

        if v < 250.0 and alt < 30000.0:
            result.outcome = "slowed to subsonic descent"
            break
    else:
        result.outcome = "timed out"

    worst = max(vehicle.parts, key=lambda p: max(p.skin_frac, p.internal_frac))
    result.peak_skin = max(p.peak_skin for p in vehicle.parts)
    result.peak_skin_frac = max(p.peak_skin / p.skin_max_temp for p in vehicle.parts)
    result.peak_internal_frac = max(p.peak_internal / p.max_temp for p in vehicle.parts)
    result.peak_g = peak_g
    result.heat_load = sum(p.heat_load for p in vehicle.parts)
    result.ablator_used = sum(p.ablator_units - p.ablator_left for p in vehicle.parts)
    result.duration = t
    result.downrange = downrange
    result.final_alt = r - body.radius
    result.final_speed = v
    result.limiting_part = worst.name
    result.survived = (result.peak_skin_frac < 1.0
                       and result.peak_internal_frac < 1.0
                       and result.outcome != "skipped out of the atmosphere"
                       and all(p.ablator_left > 0 or p.ablator_units == 0
                               for p in vehicle.parts))
    if result.peak_skin_frac >= 1.0:
        result.outcome = "skin burn-through"
    elif result.peak_internal_frac >= 1.0:
        result.outcome = "internal overheat"
    elif any(p.ablator_units > 0 and p.ablator_left <= 0 for p in vehicle.parts):
        result.outcome = "ablator exhausted"
    return result


# --------------------------------------------------------------------------
# Entry-interface state from an arriving orbit (elliptical or hyperbolic)
# --------------------------------------------------------------------------
def entry_state_from_orbit(body: AtmosphereModel, pe_alt, ap_alt=None, v_inf=None,
                           entry_alt=None):
    """Speed and flight path angle at the entry interface.

    ap_alt: apoapsis altitude for an elliptical return (None if hyperbolic)
    v_inf : hyperbolic excess speed (m/s) for an escape-velocity return
    """
    if entry_alt is None:
        entry_alt = body.atmosphere_depth
    rp = body.radius + pe_alt
    r = body.radius + entry_alt
    if v_inf is not None:
        energy = 0.5 * v_inf * v_inf
    else:
        ra = body.radius + ap_alt
        sma = 0.5 * (rp + ra)
        energy = -body.mu / (2.0 * sma)
    v = math.sqrt(2.0 * (energy + body.mu / r))
    vp = math.sqrt(2.0 * (energy + body.mu / rp))
    h = rp * vp                                    # angular momentum
    cos_fpa = min(1.0, h / (r * v))
    fpa = -math.degrees(math.acos(cos_fpa))        # descending
    return v, fpa


# --------------------------------------------------------------------------
# The sweep: for each (AoA, Pe) pair, run an entry and score it.
# --------------------------------------------------------------------------
def sweep(body, vehicle, consts, pe_list, aoa_list, ap_alt=None, v_inf=None):
    rows = []
    for pe in pe_list:
        for aoa in aoa_list:
            v, fpa = entry_state_from_orbit(body, pe, ap_alt=ap_alt, v_inf=v_inf)
            res = simulate_entry(body, vehicle, consts, body.atmosphere_depth,
                                 v, fpa, aoa)
            rows.append((pe, aoa, res))
    return rows


def acceptable(res: EntryResult, g_limit):
    """A pair is acceptable if it captures, survives, and stays under the
    structural/crew g limit."""
    return res.survived and res.peak_g <= g_limit


def score(res: EntryResult, g_limit):
    """Lower is better.  Prefers survival, then thermal margin, then gentleness."""
    if not acceptable(res, g_limit):
        return 1e6 + max(res.peak_skin_frac, res.peak_internal_frac) * 1000
    return (max(res.peak_skin_frac, res.peak_internal_frac) * 100.0
            + res.peak_g / max(g_limit, 1.0) * 25.0)


def corridor(rows, g_limit):
    """For each AoA, the contiguous band of periapsis altitudes that both
    capture the vehicle and keep it inside its thermal and g limits."""
    bands = {}
    for pe, aoa, res in rows:
        if acceptable(res, g_limit):
            lo, hi = bands.get(aoa, (pe, pe))
            bands[aoa] = (min(lo, pe), max(hi, pe))
    return bands


def mercury_capsule():
    shield = ThermalPart(name="Heat shield", mass_t=0.30, area=3.2,
                         skin_mass_per_area=5.0, skin_thermal_mass_modifier=1.5,
                         skin_internal_conduction_mult=0.1, emissive_constant=0.4,
                         skin_max_temp=3600.0, max_temp=1144.0,
                         ablator_units=96.0)
    hull = ThermalPart(name="Capsule hull", mass_t=1.10, area=9.0,
                       skin_mass_per_area=2.0, skin_thermal_mass_modifier=1.0,
                       skin_internal_conduction_mult=1.0, emissive_constant=0.6,
                       skin_max_temp=1300.0, max_temp=900.0,
                       heat_convective_constant=0.4,
                       convection_temp_mult=0.35, convection_coeff_mult=0.35,
                       convection_area_mult=0.6)
    return Vehicle(mass_t=1.40, aero=CapsuleAero(), parts=[shield, hull])


def report(title, rows, g_limit=12.0):
    print()
    print("=" * 78)
    print(title)
    print("=" * 78)
    print(f"{'Pe km':>7} {'AoA':>5} {'ok':>3} {'skin K':>8} {'skin%':>6} "
          f"{'int%':>6} {'g':>6} {'MJ':>8} {'abl':>6}  outcome")
    for pe, aoa, r in rows:
        print(f"{pe/1000:7.0f} {aoa:5.0f} {'Y' if acceptable(r, g_limit) else 'n':>3} "
              f"{r.peak_skin:8.0f} {r.peak_skin_frac*100:6.1f} "
              f"{r.peak_internal_frac*100:6.1f} {r.peak_g:6.1f} "
              f"{r.heat_load/1000:8.1f} {r.ablator_used:6.1f}  {r.outcome}")
    ok = [x for x in rows if acceptable(x[2], g_limit)]
    if ok:
        best = min(ok, key=lambda x: score(x[2], g_limit))
        print(f"\n  best pair: Pe {best[0]/1000:.0f} km, AoA {best[1]:.0f} deg "
              f"-> peak skin {best[2].peak_skin:.0f} K "
              f"({best[2].peak_skin_frac*100:.0f}% of limit), "
              f"{best[2].peak_g:.1f} g, limited by {best[2].limiting_part}")
        print(f"  entry corridor (g limit {g_limit:.0f}):")
        for aoa, (lo, hi) in sorted(corridor(rows, g_limit).items()):
            print(f"    AoA {aoa:3.0f} deg: Pe {lo/1000:6.0f} .. {hi/1000:5.0f} km")
    else:
        print(f"\n  no acceptable pair in this grid (g limit {g_limit:.0f})")


if __name__ == "__main__":
    body = AtmosphereModel()
    consts = PhysicsConstants()
    pe_list = [-200000, -100000, -50000, -20000, 0, 20000, 40000, 60000]
    aoa_list = [0, 10, 20, 30]

    report("LEO return (Ap 250 km), crewed 12 g limit",
           sweep(body, mercury_capsule(), consts, pe_list, aoa_list, ap_alt=250000))
    report("Lunar return (v_inf 1100 m/s, hyperbolic), crewed 12 g limit",
           sweep(body, mercury_capsule(), consts, pe_list, aoa_list, v_inf=1100.0))
