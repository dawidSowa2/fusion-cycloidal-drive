"""
Capstan (cable) drive geometry for a robot-arm joint (pure Python, lengths in cm).

Layout: the output sector turns about the origin; the motor drum sits at (C, 0). The cable
is anchored in the middle of the drum, wraps it in a helix, and leaves along the two external
tangents to the sector rim, where each end is clamped at a sector end.

"Pitch" radius means the radius of the cable centreline.
"""

import math

CABLES = {
    # name: (smallest recommended drum pitch diameter / cable diameter, typical note)
    'Dyneema / braided line': (8.0, 'braided line'),
    'Steel wire rope': (18.0, 'steel rope'),
}


def layout(p):
    """Derived dimensions from a capstan parameter dict."""
    dc = p['cableDiameter']
    r_p = p['drumDiameter'] / 2.0
    R_p = p['ratio'] * r_p
    lip_h = p['lipHeight']
    groove = bool(p['helicalGroove'])
    L = {}
    L['dc'], L['r_p'], L['R_p'] = dc, r_p, R_p
    # Drum: with a groove the cable sits half-buried, so the drum surface is at the cable
    # centre; without one the cable rests on the surface.
    L['drum_r'] = r_p if groove else r_p - dc / 2.0
    L['drum_lip_r'] = r_p + dc / 2.0 + lip_h
    L['rim_r'] = R_p - dc / 2.0
    L['rim_lip_r'] = R_p + dc / 2.0 + lip_h
    # Centre distance keeps the drum lips clear of the sector lips.
    L['C'] = L['rim_lip_r'] + L['drum_lip_r'] + p['drumGap']
    # The cable leaves the rim and the drum along the external tangents, at +/- alpha.
    L['alpha'] = math.acos((R_p - r_p) / L['C'])
    half_range = math.radians(p['jointRange']) / 2.0
    anchor = math.radians(p['anchorAngle'])
    L['half_span'] = L['alpha'] + half_range + anchor
    # The drum winds on one side and off the other; keep wrapsMin turns on each side.
    L['travel_turns'] = p['ratio'] * p['jointRange'] / 360.0
    L['turns'] = L['travel_turns'] + 2 * p['minWraps']
    L['helix_pitch'] = dc + 0.03 + 2 * p['grooveClearance']
    L['track'] = L['turns'] * L['helix_pitch'] + 0.2
    free = math.sqrt(max(0.0, L['C'] ** 2 - (R_p - r_p) ** 2))
    rim_wrap = (L['half_span'] - L['alpha']) * R_p
    drum_wrap = math.pi * 2 * r_p * L['turns'] / 2.0
    # two halves of cable, plus ~6 cm per end for tying / clamping
    L['cable_length'] = 2 * (free + rim_wrap + drum_wrap + 6.0)
    return L


def helix_points(cx, cy, r, z_mid, turns, pitch, per_turn=24, phase_at_mid=math.pi / 2):
    """Points on the cable helix, centred on z_mid, passing angle phase_at_mid at z_mid
    (so the drum's cross-hole along Y meets the groove)."""
    n = max(8, int(per_turn * turns))
    a_mid = turns * math.pi
    a0 = phase_at_mid - a_mid
    pts = []
    for i in range(n + 1):
        a = 2 * math.pi * turns * i / n
        pts.append((cx + r * math.cos(a0 + a), cy + r * math.sin(a0 + a),
                    z_mid - turns * pitch / 2.0 + pitch * a / (2 * math.pi)))
    return pts


def analyze(p):
    """Validate a capstan parameter dict (cm). Returns (errors, warnings, info)."""
    errors, warnings = [], []
    if p['ratio'] <= 1:
        return ['Ratio must be greater than 1.'], warnings, {}
    if p['cableDiameter'] <= 0 or p['drumDiameter'] <= p['cableDiameter']:
        return ['Drum pitch diameter must be bigger than the cable diameter.'], warnings, {}
    L = layout(p)
    info = dict(L)

    if L['half_span'] >= math.pi - 0.05:
        max_range = 2 * math.degrees(math.pi - 0.05 - L['alpha'] - math.radians(p['anchorAngle']))
        errors.append('Joint range too large for one sector: max about %.0f deg with these '
                      'settings.' % max(0.0, max_range))
    min_ratio, label = CABLES.get(p['cable'], (8.0, 'cable'))
    d_ratio = p['drumDiameter'] / p['cableDiameter']
    info['d_ratio'] = d_ratio
    if d_ratio < min_ratio:
        warnings.append('The drum is only %.1fx the cable diameter; %s lasts longer at %.0fx '
                        'or more (bigger drum or thinner cable).' % (d_ratio, label, min_ratio))

    shaft_r = (p['shaftDiameter'] + p['shaftClearance']) / 2.0
    core_r = L['drum_r'] - (L['dc'] / 2.0 + p['grooveClearance'] if p['helicalGroove'] else 0)
    if core_r - shaft_r < 0.12:
        errors.append('Drum too thin around the motor shaft: increase the drum diameter.')
    if p['setScrew'] and L['drum_lip_r'] - shaft_r < 0.3:
        warnings.append('Drum hub is thin for an M3 set screw.')

    hub_r = p['hubDiameter'] / 2.0
    rim_in = L['rim_r'] - p['rimThickness']
    if hub_r - p['outputBore'] / 2.0 < 0.2:
        errors.append('Output hub too thin around the bore: increase the hub diameter.')
    if rim_in - p['anchorDepth'] - hub_r < 0.3:
        errors.append('Sector too small for the hub and cable anchors: increase the ratio or '
                      'drum diameter, or shrink the hub.')
    if p['boltCount'] > 0:
        bc, br = p['boltCircleDiameter'] / 2.0, p['boltDiameter'] / 2.0
        if bc - br - p['outputBore'] / 2.0 < 0.1 or hub_r - (bc + br) < 0.1:
            errors.append('Hub bolt circle does not fit between the bore and the hub edge.')
    return errors, warnings, info
