"""
Timing-belt drive geometry (pure Python, lengths in cm).

Pulley grooves are a printable U-shaped approximation of each belt's tooth: a round bottom of
the belt tooth's radius, straight sides, and the belt tooth's height for depth, plus clearance.
That is close enough for FDM pulleys; check the fit with a short test print first.
"""

import math

# pitch, pitch-line offset (PLD), tooth height, tooth radius, overall belt thickness   [cm]
PROFILES = {
    'GT2 2mm': dict(pitch=0.2, pld=0.0254, h=0.075, r=0.0555, thick=0.138),
    'GT2 3mm': dict(pitch=0.3, pld=0.0381, h=0.114, r=0.085, thick=0.241),
    'HTD 3M':  dict(pitch=0.3, pld=0.0381, h=0.117, r=0.086, thick=0.240),
    'HTD 5M':  dict(pitch=0.5, pld=0.05715, h=0.206, r=0.149, thick=0.380),
}


def pitch_diameter(teeth, pitch):
    return teeth * pitch / math.pi


def outer_diameter(teeth, prof):
    return pitch_diameter(teeth, prof['pitch']) - 2 * prof['pld']


def groove_polygon(teeth, prof, clearance, segments=12):
    """One tooth groove as a closed (x, y) polygon, centred on the +X axis. It reaches a
    little past the pulley OD so cutting it leaves a clean gap."""
    ro = outer_diameter(teeth, prof) / 2.0
    depth = prof['h'] + clearance
    w = prof['r'] + clearance           # half width of the U
    c = ro - depth + w                  # centre of the round bottom
    x_out = max(ro + 0.05, c + 0.01)
    pts = [(x_out, w), (c, w)]
    for i in range(1, segments):
        a = math.pi / 2 + math.pi * i / segments
        pts.append((c + w * math.cos(a), w * math.sin(a)))
    pts += [(c, -w), (x_out, -w)]
    return pts


def pulley_outline(teeth, prof, clearance, segments=10, land_points=3):
    """Closed counter-clockwise (x, y) outline of a whole toothed pulley, centred on the
    origin: OD lands joined by U-shaped grooves."""
    ro = outer_diameter(teeth, prof) / 2.0
    depth = prof['h'] + clearance
    w = prof['r'] + clearance
    c = ro - depth + w
    xe = math.sqrt(ro * ro - w * w)          # where the groove side meets the OD
    beta = math.atan2(w, xe)                 # half-angle of the groove opening
    step = 2 * math.pi / teeth
    local = [(xe, -w)]
    if c < xe:
        local.append((c, -w))
    for i in range(1, segments):
        a = 1.5 * math.pi - math.pi * i / segments
        local.append((c + w * math.cos(a), w * math.sin(a)))
    if c < xe:
        local.append((c, w))
    local.append((xe, w))
    pts = []
    for k in range(teeth):
        th = k * step
        cs, sn = math.cos(th), math.sin(th)
        pts += [(cs * x - sn * y, sn * x + cs * y) for x, y in local]
        for j in range(1, land_points + 1):
            a = th + beta + (step - 2 * beta) * j / (land_points + 1)
            pts.append((ro * math.cos(a), ro * math.sin(a)))
    return pts


def belt_length(C, D1, D2):
    """Exact pitch length of an open belt around two pitch diameters at centre distance C."""
    phi = math.asin((D2 - D1) / (2 * C))
    return 2 * C * math.cos(phi) + math.pi * (D1 + D2) / 2 + phi * (D2 - D1)


def centre_distance(belt_teeth, z1, z2, pitch):
    """Centre distance for a closed belt with belt_teeth teeth (None if it is too short)."""
    D1, D2 = sorted((pitch_diameter(z1, pitch), pitch_diameter(z2, pitch)))
    L = belt_teeth * pitch
    lo = (D2 - D1) / 2 + 1e-6
    if belt_length(lo, D1, D2) >= L:
        return None
    hi = L
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if belt_length(mid, D1, D2) < L:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def teeth_in_mesh(C, z_small, z_big, pitch):
    D1, D2 = pitch_diameter(z_small, pitch), pitch_diameter(z_big, pitch)
    phi = math.asin((D2 - D1) / (2 * C))
    wrap = math.pi - 2 * phi
    return z_small * wrap / (2 * math.pi)


def tangent_points(r1, r2, C):
    """Upper external tangent between circles (0,0,r1) and (C,0,r2): returns (p1, p2).
    The lower tangent is the mirror image in the X axis."""
    s = (r2 - r1) / C
    c = math.sqrt(max(0.0, 1 - s * s))
    nx, ny = -s, c
    return (r1 * nx, r1 * ny), (C + r2 * nx, r2 * ny)


def suggest_belt_teeth(z1, z2, pitch, target_C):
    """Closed belt size (teeth) whose centre distance is nearest target_C."""
    D1, D2 = pitch_diameter(z1, pitch), pitch_diameter(z2, pitch)
    return int(round(belt_length(target_C, min(D1, D2), max(D1, D2)) / pitch))


def analyze(p):
    """Validate a belt-drive parameter dict (cm). Returns (errors, warnings, info)."""
    errors, warnings, info = [], [], {}
    prof = PROFILES.get(p['profile'])
    if prof is None:
        return ['Unknown belt profile.'], warnings, info
    z1, z2 = int(p['motorTeeth']), int(p['outputTeeth'])
    pitch = prof['pitch']
    info['ratio'] = z2 / float(z1)
    info['pd1'], info['pd2'] = pitch_diameter(z1, pitch), pitch_diameter(z2, pitch)
    info['od1'], info['od2'] = outer_diameter(z1, prof), outer_diameter(z2, prof)
    flange = p['flangeHeight'] if p['flanges'] else 0.0
    C = centre_distance(int(p['beltTeeth']), z1, z2, pitch)
    if C is None:
        errors.append('Belt is too short to go round both pulleys.')
        return errors, warnings, info
    info['C'] = C
    info['beltLength'] = int(p['beltTeeth']) * pitch

    if C < (info['od1'] + info['od2']) / 2 + flange * 2 + 0.1:
        errors.append('Pulleys collide: use a longer belt (more belt teeth).')
    mesh = teeth_in_mesh(C, min(z1, z2), max(z1, z2), pitch)
    info['mesh'] = mesh
    if mesh < 6:
        warnings.append('Only %.1f teeth in mesh on the small pulley; 6 or more is '
                        'recommended to stop the belt skipping.' % mesh)

    for name, z in (('motor', z1), ('output', z2)):
        ro = outer_diameter(z, prof) / 2
        land = math.pi * 2 * ro / z - 2 * (prof['r'] + p['grooveClearance'])
        if land < 0.03:
            errors.append('Teeth on the %s pulley are too thin: lower the groove clearance '
                          'or use more teeth.' % name)

    root2 = info['od2'] / 2 - prof['h'] - p['grooveClearance']
    bore2 = p['outputBore'] / 2
    if p['boltCount'] > 0:
        bc = p['boltCircleDiameter'] / 2
        br = p['boltDiameter'] / 2
        if bc - br - bore2 < 0.1:
            errors.append('Output bolt holes hit the output bore: increase the bolt circle.')
        if root2 - (bc + br) < 0.15:
            errors.append('Output bolt holes break into the teeth: decrease the bolt circle.')
    if root2 - bore2 < 0.2:
        errors.append('Output bore is too big for the output pulley.')

    shaft_r = (p['shaftDiameter'] + p['shaftClearance']) / 2
    root1 = info['od1'] / 2 - prof['h'] - p['grooveClearance']
    if root1 - shaft_r < 0.1:
        errors.append('Motor pulley is too small for the shaft: use more motor teeth.')
    if p['setScrew'] and p['hubDiameter'] / 2 - shaft_r < 0.25:
        errors.append('Hub too thin for an M3 set screw: increase the hub diameter.')
    if z1 > z2:
        warnings.append('The motor pulley is bigger than the output pulley, so this '
                        'speeds the output up instead of reducing it.')
    return errors, warnings, info
