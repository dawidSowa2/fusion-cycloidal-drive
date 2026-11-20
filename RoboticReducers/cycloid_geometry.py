"""
Pure-Python cycloidal drive geometry (no Fusion dependencies, so it can be tested
outside Fusion). All lengths are in centimetres, because that is Fusion's internal unit.

Conventions
-----------
N   = number of ring pins. The disc has N-1 lobes and the reduction ratio is (N-1):1
      (output turns opposite to the input).
R   = radius of the ring-pin circle (pin centres).
Rr  = ring-pin radius (plus any profile clearance, which shrinks the disc).
E   = eccentricity.

Pitch curve (path of a pin centre seen from the disc):
    P(t) = ( R cos t - E cos(N t),  -R sin t + E sin(N t) )
The disc profile is P(t) offset inwards by Rr.

With the disc centre at (E, 0) and no rotation, the profile meshes with pins placed at
angles 2*pi*j/N. In general, when the eccentric points at angle theta, the disc is at
E*(cos theta, sin theta) and rotated by -theta/(N-1).
"""

import math


def _p(t, R, E, N):
    return (R * math.cos(t) - E * math.cos(N * t),
            -R * math.sin(t) + E * math.sin(N * t))


def _d1(t, R, E, N):
    return (-R * math.sin(t) + E * N * math.sin(N * t),
            -R * math.cos(t) + E * N * math.cos(N * t))


def _d2(t, R, E, N):
    return (-R * math.cos(t) + E * N * N * math.cos(N * t),
            R * math.sin(t) - E * N * N * math.sin(N * t))


def disc_profile(R, Rr, E, N, points_per_lobe=30):
    """Closed list of (x, y) points of the disc outline, centred on the origin,
    unrotated. The first point is not repeated at the end."""
    lobes = N - 1
    count = max(8, int(points_per_lobe)) * lobes

    def point(t):
        x, y = _p(t, R, E, N)
        dx, dy = _d1(t, R, E, N)
        L = math.hypot(dx, dy)
        # Curve runs clockwise, so the outward normal is (-dy, dx)/L.
        return (x + Rr * dy / L, y - Rr * dx / L)

    # Uniform t crowds points onto the lobe tips; resample evenly by arc length so the
    # roots (where the pins sit) get as many spline points as the tips.
    dense = 40 * count
    ts = [2.0 * math.pi * i / dense for i in range(dense + 1)]
    dp = [point(t) for t in ts]
    s = [0.0]
    for i in range(1, len(dp)):
        s.append(s[-1] + math.hypot(dp[i][0] - dp[i - 1][0], dp[i][1] - dp[i - 1][1]))
    total = s[-1]
    pts, j = [], 0
    for i in range(count):
        target = total * i / count
        while s[j + 1] < target:
            j += 1
        f = (target - s[j]) / (s[j + 1] - s[j]) if s[j + 1] > s[j] else 0.0
        pts.append(point(ts[j] + f * (ts[j + 1] - ts[j])))
    return pts


def min_convex_radius(R, E, N, samples=20000):
    """Smallest radius of curvature on the convex (lobe tip) parts of the pitch curve.
    The pin radius must be smaller than this or the profile undercuts / self-intersects."""
    best = float('inf')
    for i in range(samples):
        t = 2.0 * math.pi * i / samples
        x1, y1 = _d1(t, R, E, N)
        x2, y2 = _d2(t, R, E, N)
        cross = x1 * y2 - y1 * x2
        speed = math.hypot(x1, y1)
        if speed < 1e-12:
            return 0.0
        k = cross / speed ** 3
        if k < 0:  # clockwise curve: negative curvature = bending towards the inside
            best = min(best, 1.0 / -k)
    return best


def disc_placements(disc_count, E, N):
    """(cx, cy, rotation) for each disc at the assembly position (input angle 0).
    Disc k's eccentric is at k*360/disc_count degrees."""
    out = []
    for k in range(disc_count):
        theta = 2.0 * math.pi * k / disc_count
        out.append((E * math.cos(theta), E * math.sin(theta), -theta / (N - 1)))
    return out


def transform(points, cx, cy, rot):
    c, s = math.cos(rot), math.sin(rot)
    return [(cx + c * x - s * y, cy + s * x + c * y) for x, y in points]


HOUSING_CLEARANCE = 0.05  # 0.5 mm radial gap between the lobe tips and the housing


def disc_reach(R, Rr_eff, E):
    """Furthest any disc point gets from the input axis while running: the lobe tip
    radius (R + E - Rr) measured from the disc centre, plus the eccentric offset E."""
    return R + 2 * E - Rr_eff


def housing_bore_radius(R, Rr_eff, E):
    return disc_reach(R, Rr_eff, E) + HOUSING_CLEARANCE


def ring_pin_centres(R, N):
    return [(R * math.cos(2 * math.pi * j / N), R * math.sin(2 * math.pi * j / N))
            for j in range(N)]


def output_pin_centres(Rop, M):
    return [(Rop * math.cos(2 * math.pi * j / M), Rop * math.sin(2 * math.pi * j / M))
            for j in range(M)]


def analyze(p):
    """Validate a parameter dict (cm). Returns (errors, warnings, info)."""
    errors, warnings, info = [], [], {}
    N = int(p['pinCount'])
    R = p['ringDiameter'] / 2.0
    Rr = p['pinDiameter'] / 2.0
    E = p['eccentricity']
    Rr_eff = Rr + p['profileClearance']
    n = int(p['discCount'])
    M = int(p['outputPinCount'])
    Rop = p['outputCircleDiameter'] / 2.0
    hole_r = (p['outputPinDiameter'] + 2 * E + p['outputHoleClearance']) / 2.0
    bore_r = p['bearingOD'] / 2.0
    cam_r = p['bearingID'] / 2.0
    shaft_r = (p['shaftDiameter'] + p['shaftClearance']) / 2.0
    wall = 0.1  # 1 mm minimum wall

    info['ratio'] = N - 1
    info['lobes'] = N - 1
    info['k'] = E * N / R if R > 0 else 0

    if N < 4:
        errors.append('Need at least 4 ring pins.')
        return errors, warnings, info
    if E <= 0 or R <= 0 or Rr <= 0:
        errors.append('Ring diameter, pin diameter and eccentricity must be positive.')
        return errors, warnings, info
    if E * N >= R:
        errors.append('Eccentricity too large: E x pins must be < ring radius '
                      '(max E = %.2f mm).' % (R / N * 10))
        return errors, warnings, info

    rho = min_convex_radius(R, E, N)
    info['minRadius'] = rho
    if Rr_eff >= rho:
        errors.append('Lobes undercut: pin radius + clearance (%.2f mm) must be below the '
                      'lobe curvature radius (%.2f mm). Use smaller pins, smaller '
                      'eccentricity or a bigger ring.' % (Rr_eff * 10, rho * 10))
    elif Rr_eff > 0.85 * rho:
        warnings.append('Lobe tips are sharp (pin radius is %.0f%% of the lobe curvature '
                        'limit).' % (100 * Rr_eff / rho))

    if 2 * R * math.sin(math.pi / N) <= 2 * Rr + 0.02:
        errors.append('Ring pins overlap each other: fewer/thinner pins or bigger ring.')

    outer = R + E - Rr_eff
    root = R - E - Rr_eff
    info['discOuterDiameter'] = 2 * outer
    info['discRootDiameter'] = 2 * root

    reach = disc_reach(R, Rr_eff, E)
    house_in = housing_bore_radius(R, Rr_eff, E)
    house_out = R + Rr + p.get('housingWall', 0.5)
    info['discReachDiameter'] = 2 * reach
    info['housingBoreDiameter'] = 2 * house_in
    if p.get('makeHousing', True):
        if house_out - house_in < 0.1:
            errors.append('Housing wall too thin behind the disc relief: increase the housing '
                          'wall.')
        if house_in > R:
            warnings.append('The lobe tips swing past the pin centres, so the housing bore is '
                            'relieved and pins are held mainly by the seats in the base plate '
                            'and top cover.')

    if cam_r >= bore_r:
        errors.append('Bearing inner diameter must be smaller than its outer diameter.')
    if cam_r - E - shaft_r < 0.05:
        errors.append('Eccentric cam too thin around the shaft: use a bearing with a larger '
                      'inner diameter or a smaller eccentricity.')
    if M > 0:
        if Rop - hole_r - bore_r < wall:
            errors.append('Output holes hit the bearing bore: increase the output pin circle '
                          'or use a smaller bearing.')
        if root - (Rop + hole_r) < wall:
            errors.append('Output holes break through the lobe roots: decrease the output pin '
                          'circle or output pin diameter.')
        if M > 1 and 2 * Rop * math.sin(math.pi / M) - 2 * hole_r < wall:
            errors.append('Output holes overlap each other: fewer or thinner output pins.')
    info['outputHoleDiameter'] = 2 * hole_r

    if n == 1:
        warnings.append('A single disc is not balanced and will vibrate at speed.')
    elif n == 2:
        info['phase'] = '0 / 180 deg'
    elif n == 3:
        info['phase'] = '0 / 120 / 240 deg'
    return errors, warnings, info
