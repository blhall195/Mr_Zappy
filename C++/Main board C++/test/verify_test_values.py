#!/usr/bin/env python3
"""
Verification oracle for leg-checker test values.

For each (tolerance, distance, direction) combination:
  - Finds the critical angular delta where the endpoint spread exactly equals the
    Cartesian tolerance (or where the angular separation equals the angular tolerance)
    using scipy.optimize.brentq — an independent bisection solver.
  - Derives inside/outside values at ±MARGIN of that critical delta.
  - Confirms the expected pass/fail outcome using numpy/scipy independently of
    any C++ implementation.

Run with: python3 test/verify_test_values.py
"""

import sys
import numpy as np
from scipy.optimize import brentq
from scipy.spatial.distance import euclidean

CART_TOLERANCES_CM = [0, 5, 10, 30]
ANG_TOLERANCES_DEG = [0, 1.7, 5, 10]
DISTANCES_M        = [0, 5, 10, 100]
DIRECTIONS         = ['az', 'inc', 'diag']
MARGIN             = 0.005   # 0.5% — far enough from boundary to be reliable in float32

PASS_STR = "\033[32mPASS\033[0m"
FAIL_STR = "\033[31mFAIL\033[0m"

failures = 0

# ── Geometry helpers ───────────────────────────────────────────────────────────

def aer_to_enu(az_deg, inc_deg, dist):
    az  = np.radians(az_deg)
    inc = np.radians(inc_deg)
    return np.array([
        dist * np.cos(inc) * np.sin(az),
        dist * np.cos(inc) * np.cos(az),
        dist * np.sin(inc),
    ])

def cart_dist_m(s1, s2):
    return euclidean(aer_to_enu(*s1), aer_to_enu(*s2))

def angular_sep_deg(s1, s2):
    v1 = aer_to_enu(s1[0], s1[1], 1.0)
    v2 = aer_to_enu(s2[0], s2[1], 1.0)
    return np.degrees(np.arccos(np.clip(np.dot(v1, v2), -1.0, 1.0)))

BASE = (0.0, 0.0)   # base shot: az=0°, inc=0°

def varied_shot(direction, delta, dist):
    if direction == 'az':
        return (BASE[0] + delta, BASE[1],         dist)
    elif direction == 'inc':
        return (BASE[0],         BASE[1] + delta, dist)
    else:  # diag: az and inc vary equally
        return (BASE[0] + delta, BASE[1] + delta, dist)

# ── Critical-value search ──────────────────────────────────────────────────────

def find_critical_cart(direction, dist_m, tol_cm):
    """Angular delta where cart endpoint distance == tol_cm."""
    tol_m = tol_cm / 100.0
    base_shot = (*BASE, dist_m)

    def error(delta):
        s2 = varied_shot(direction, delta, dist_m)
        return cart_dist_m(base_shot, s2) - tol_m

    upper = 1e-6
    while error(upper) < 0:
        upper *= 2
        if upper > 170:
            return None
    return brentq(error, 0.0, upper, xtol=1e-10, rtol=1e-10)

def find_critical_angular(direction, tol_deg):
    """Angular delta where angular separation == tol_deg."""
    DIST = 5.0  # distance irrelevant for angular checker

    def error(delta):
        s2 = varied_shot(direction, delta, DIST)
        return angular_sep_deg((*BASE, DIST), s2) - tol_deg

    upper = 1e-6
    while error(upper) < 0:
        upper *= 2
        if upper > 170:
            return None
    return brentq(error, 0.0, upper, xtol=1e-10, rtol=1e-10)

# ── Checkers ───────────────────────────────────────────────────────────────────

def check(label, value, tol, unit, expect_pass):
    # C++ rejects only on strict >, so equality passes — mirror that with <=
    global failures
    ok = (value <= tol) == expect_pass
    tag = PASS_STR if ok else FAIL_STR
    verb = "pass" if expect_pass else "fail"
    if not ok:
        failures += 1
    print(f"  {tag}  {label}: {value:.6f}{unit} {'<=' if expect_pass else '>'} {tol}{unit} → {verb}")

# ── CartesianLegChecker matrix ─────────────────────────────────────────────────

print("=" * 70)
print("CartesianLegChecker — critical deltas (degrees) and boundary checks")
print("=" * 70)

print("\n--- Special case: tol=0cm (only identical shots pass; skip dist=0 where all shots collapse to origin) ---")
for dist in DISTANCES_M[1:]:   # dist=0 always produces 0cm regardless of angle — see dist=0 section
    base = (0.0, 0.0, float(dist))
    s2   = (0.001, 0.0, float(dist))
    d = cart_dist_m(base, s2) * 100
    check(f"dist={dist}m epsilon az", d, 0, "cm", False)

print("\n--- Special case: dist=0m (all shots at origin — always pass) ---")
for tol in CART_TOLERANCES_CM[1:]:
    base = (0.0, 0.0, 0.0)
    s2   = (45.0, 45.0, 0.0)
    d = cart_dist_m(base, s2) * 100
    check(f"tol={tol}cm any angle", d, tol, "cm", True)

print()
for tol in CART_TOLERANCES_CM[1:]:       # skip tol=0 (handled above)
    for dist in DISTANCES_M[1:]:         # skip dist=0 (handled above)
        print(f"--- tol={tol}cm  dist={dist}m ---")
        for direction in DIRECTIONS:
            crit = find_critical_cart(direction, dist, tol)
            if crit is None:
                print(f"  SKIP  {direction}: cannot reach tolerance at this distance")
                continue
            inside  = crit * (1 - MARGIN)
            outside = crit * (1 + MARGIN)
            base = (*BASE, float(dist))
            d_in  = cart_dist_m(base, varied_shot(direction, inside,  dist)) * 100
            d_out = cart_dist_m(base, varied_shot(direction, outside, dist)) * 100
            check(f"{direction} inside  Δ={inside:.6f}°", d_in,  tol, "cm", True)
            check(f"{direction} outside Δ={outside:.6f}°", d_out, tol, "cm", False)
            print(f"         critical Δ = {crit:.8f}°  "
                  f"inside={inside:.8f}°  outside={outside:.8f}°")

# ── AngularLegChecker matrix ───────────────────────────────────────────────────

print()
print("=" * 70)
print("AngularLegChecker — critical deltas and boundary checks")
print("=" * 70)

print("\n--- Special case: tol=0° (only identical shots pass) ---")
for direction in DIRECTIONS:
    DIST = 5.0
    s2 = varied_shot(direction, 0.001, DIST)
    a = angular_sep_deg((*BASE, DIST), s2)
    check(f"{direction} epsilon", a, 0, "°", False)

print()
for tol in ANG_TOLERANCES_DEG[1:]:       # skip tol=0 (handled above)
    print(f"--- tol={tol}° ---")
    for direction in DIRECTIONS:
        crit = find_critical_angular(direction, tol)
        if crit is None:
            print(f"  SKIP  {direction}: cannot reach tolerance")
            continue
        inside  = crit * (1 - MARGIN)
        outside = crit * (1 + MARGIN)
        DIST = 5.0
        a_in  = angular_sep_deg((*BASE, DIST), varied_shot(direction, inside,  DIST))
        a_out = angular_sep_deg((*BASE, DIST), varied_shot(direction, outside, DIST))
        check(f"{direction} inside  Δ={inside:.6f}°", a_in,  tol, "°", True)
        check(f"{direction} outside Δ={outside:.6f}°", a_out, tol, "°", False)
        print(f"         critical Δ = {crit:.8f}°  "
              f"inside={inside:.8f}°  outside={outside:.8f}°")

print()
print(f"{'All checks passed.' if failures == 0 else f'{failures} check(s) FAILED.'}")
sys.exit(failures)
