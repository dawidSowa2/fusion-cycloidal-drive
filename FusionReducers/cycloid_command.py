"""
Cycloidal drive command: 1, 2 or 3 phased discs, eccentric cam, ring pins, housing with
NEMA base plate, top cover and output carrier. Lengths in cm (Fusion's internal unit).
"""

from . import cycloid_geometry as cg
from .fusion_ui import *  # noqa: F401,F403


SPEC = [
    ('gGear', 'Cycloidal stage', 'group', None, True),
    ('pinCount', 'Ring pins (ratio = pins - 1)', 'int', 20, (5, 150)),
    ('ringDiameter', 'Ring pin circle diameter', 'len', 7.0, None),
    ('pinDiameter', 'Ring pin diameter', 'len', 0.5, None),
    ('eccentricity', 'Eccentricity', 'len', 0.15, None),
    ('profileClearance', 'Profile clearance (shrinks disc)', 'len', 0.01, None),
    ('discCount', 'Number of discs', 'drop', '3', ['1', '2', '3']),
    ('discThickness', 'Disc thickness', 'len', 0.6, None),
    ('discGap', 'Gap between parts', 'len', 0.05, None),
    ('pointsPerLobe', 'Spline points per lobe', 'int', 40, (10, 200)),

    ('gShaft', 'Bearing and motor', 'group', None, True),
    ('bearingOD', 'Disc bearing outer diameter (disc bore)', 'len', 2.7, None),
    ('bearingID', 'Disc bearing inner diameter (cam OD)', 'len', 2.0, None),
] + motor_spec() + [

    ('gOutput', 'Output', 'group', None, None),
    ('outputPinCount', 'Output pins', 'int', 8, (0, 36)),
    ('outputCircleDiameter', 'Output pin circle diameter', 'len', 4.4, None),
    ('outputPinDiameter', 'Output pin diameter (incl. roller)', 'len', 0.5, None),
    ('outputHoleClearance', 'Output hole clearance', 'len', 0.02, None),

    ('gParts', 'Parts to generate', 'group', None, None),
    ('makeDiscs', 'Cycloidal discs', 'bool', True, None),
    ('makeCam', 'Eccentric cam', 'bool', True, None),
    ('makeRingPins', 'Ring pins', 'bool', True, None),
    ('makeHousing', 'Housing ring', 'bool', True, None),
    ('makeBasePlate', 'Motor base plate', 'bool', True, None),
    ('makeCover', 'Top cover ring (holds pins)', 'bool', True, None),
    ('makeCarrier', 'Output carrier + pins', 'bool', True, None),
    ('housingWall', 'Housing wall outside pins', 'len', 0.5, None),
    ('pinHoleClearance', 'Ring pin seat clearance', 'len', 0.0, None),
    ('baseThickness', 'Base plate thickness', 'len', 0.4, None),
    ('carrierThickness', 'Output carrier thickness', 'len', 0.5, None),
    ('coverThickness', 'Top cover thickness', 'len', 0.4, None),

    ('preview', 'Live preview (slower)', 'bool', False, None),
]



def info_html(p):
    p = dict(p, discCount=int(p['discCount']))
    errors, warnings, info = cg.analyze(p)
    lines = []
    if 'ratio' in info:
        lines.append('<b>Ratio %d : 1</b> (output reverses), %d lobes, %d disc(s)'
                     % (info['ratio'], info['lobes'], int(p['discCount'])))
    if 'discOuterDiameter' in info:
        lines.append('Disc OD %s, root %s, lobe curvature limit %s'
                     % (mm(info['discOuterDiameter']), mm(info['discRootDiameter']),
                        mm(info['minRadius'])))
        lines.append('Lobe tips sweep %s, housing bore %s, housing OD %s'
                     % (mm(info['discReachDiameter']), mm(info['housingBoreDiameter']),
                        mm(p['ringDiameter'] + p['pinDiameter'] + 2 * p['housingWall'])))
        lines.append('Output hole diameter %s' % mm(info['outputHoleDiameter']))
    if 'phase' in info:
        lines.append('Eccentric phases: ' + info['phase'])
    for e in errors:
        lines.append('<font color="#d33">ERROR: %s</font>' % e)
    for w in warnings:
        lines.append('<font color="#c80">Note: %s</font>' % w)
    return '<br>'.join(lines), errors


def build(p):
    root = root_component()

    N = int(p['pinCount'])
    n = int(p['discCount'])
    R = p['ringDiameter'] / 2.0
    Rr = p['pinDiameter'] / 2.0
    E = p['eccentricity']
    Rr_eff = Rr + p['profileClearance']
    t = p['discThickness']
    gap = p['discGap']
    M = int(p['outputPinCount'])
    Rop = p['outputCircleDiameter'] / 2.0
    out_r = p['outputPinDiameter'] / 2.0
    hole_r = out_r + E + p['outputHoleClearance'] / 2.0
    bore_r = p['bearingOD'] / 2.0
    cam_r = p['bearingID'] / 2.0
    shaft_r = (p['shaftDiameter'] + p['shaftClearance']) / 2.0

    # Z layout (stack grows along +Z, base plate sits below Z = 0)
    z0 = gap
    disc_z = [z0 + k * (t + gap) for k in range(n)]
    stack_top = z0 + n * t + (n - 1) * gap
    carrier_z = stack_top + gap
    top_z = carrier_z + (p['carrierThickness'] if p['makeCarrier'] else 0.0)

    # The disc centre orbits at E, so its lobe tips sweep out to R + 2E - Rr from the axis.
    house_in = cg.housing_bore_radius(R, Rr_eff, E)
    house_out = R + Rr + p['housingWall']
    seat_r = Rr + p['pinHoleClearance'] / 2.0
    bt = p['baseThickness']
    ct = p['coverThickness']
    # Ring pins sit in blind seats, half the plate/cover deep, at each end.
    pin_bottom = -bt / 2.0 if p['makeBasePlate'] else 0.0
    pin_top = top_z + (ct / 2.0 if (p['makeHousing'] and p['makeCover']) else 0.0)
    pin_clear_r = R - Rr - 0.05   # inner edge of the ring pins, minus clearance

    top = add_comp(root, 'Cycloidal Drive %dto1 %d-disc' % (N - 1, n))

    placements = cg.disc_placements(n, E, N)
    profile = cg.disc_profile(R, Rr_eff, E, N, p['pointsPerLobe'])

    # --- discs
    if p['makeDiscs']:
        for k, (cx, cy, rot) in enumerate(placements):
            phase = 360.0 * k / n
            comp = add_comp(top, 'Disc %d (%.0f deg)' % (k + 1, phase))
            z = disc_z[k]
            sk = sketch_at(comp, z, 'Disc profile')
            sk.isComputeDeferred = True
            pts = adsk.core.ObjectCollection.create()
            for x, y in cg.transform(profile, cx, cy, rot):
                pts.add(to_sk(sk, x, y, z))
            spline = sk.sketchCurves.sketchFittedSplines.add(pts)
            spline.isClosed = True
            circle(sk, cx, cy, z, bore_r)
            for ox, oy in cg.output_pin_centres(Rop, M):
                circle(sk, ox + cx, oy + cy, z, hole_r)
            sk.isComputeDeferred = False
            feat = extrude(comp, profile_with_most_loops(sk), t, NEW)
            feat.bodies.item(0).name = 'Disc %d' % (k + 1)

    # --- eccentric cam
    if p['makeCam']:
        comp = add_comp(top, 'Eccentric Cam')
        body = None
        for k, (cx, cy, _) in enumerate(placements):
            sk = sketch_at(comp, disc_z[k], 'Lobe %d' % (k + 1))
            circle(sk, cx, cy, disc_z[k], cam_r)
            h = t + (gap if k < n - 1 else 0.0)
            prof = sk.profiles.item(0)
            if body is None:
                body = extrude(comp, prof, h, NEW).bodies.item(0)
                body.name = 'Eccentric Cam'
            else:
                extrude(comp, prof, h, JOIN, [body])
        shaft_bore(comp, body, 0.0, 0.0, z0, stack_top - z0, p)

    # --- ring pins
    if p['makeRingPins']:
        comp = add_comp(top, 'Ring Pins')
        sk = sketch_at(comp, pin_bottom, 'Ring pins')
        sk.isComputeDeferred = True
        for x, y in cg.ring_pin_centres(R, N):
            circle(sk, x, y, pin_bottom, Rr)
        sk.isComputeDeferred = False
        feat = extrude(comp, all_profiles(sk), pin_top - pin_bottom, NEW)
        for i in range(feat.bodies.count):
            feat.bodies.item(i).name = 'Ring pin %d' % (i + 1)

    # --- housing ring + base plate
    if p['makeHousing'] or p['makeBasePlate']:
        comp = add_comp(top, 'Housing')
        body = None
        if p['makeHousing']:
            sk = sketch_at(comp, 0.0, 'Housing outer')
            circle(sk, 0, 0, 0.0, house_out)
            body = extrude(comp, sk.profiles.item(0), top_z, NEW).bodies.item(0)
            body.name = 'Housing'
            sk = sketch_at(comp, 0.0, 'Housing inner')
            circle(sk, 0, 0, 0.0, house_in)
            extrude(comp, sk.profiles.item(0), top_z, CUT, [body])
            sk = sketch_at(comp, 0.0, 'Pin seats')
            sk.isComputeDeferred = True
            for x, y in cg.ring_pin_centres(R, N):
                circle(sk, x, y, 0.0, seat_r)
            sk.isComputeDeferred = False
            extrude(comp, all_profiles(sk), top_z, CUT, [body])

        if p['makeBasePlate']:
            sk = sketch_at(comp, -bt, 'Base plate')
            circle(sk, 0, 0, -bt, house_out)
            if body is None:
                body = extrude(comp, sk.profiles.item(0), bt, NEW).bodies.item(0)
                body.name = 'Base plate'
            else:
                extrude(comp, sk.profiles.item(0), bt, JOIN, [body])
            sk = sketch_at(comp, -bt, 'Motor holes')
            motor_holes(sk, 0.0, 0.0, -bt, p['motor'], shaft_r)
            extrude(comp, all_profiles(sk), bt, CUT, [body])
            # blind ring-pin seats in the top half of the plate
            sk = sketch_at(comp, -bt / 2.0, 'Plate pin seats')
            sk.isComputeDeferred = True
            for x, y in cg.ring_pin_centres(R, N):
                circle(sk, x, y, -bt / 2.0, seat_r)
            sk.isComputeDeferred = False
            extrude(comp, all_profiles(sk), bt / 2.0, CUT, [body])

        if p['makeHousing'] and p['makeCover']:
            cover = add_comp(top, 'Top Cover')
            sk = sketch_at(cover, top_z, 'Cover')
            circle(sk, 0, 0, top_z, house_out)
            circle(sk, 0, 0, top_z, pin_clear_r)
            cbody = extrude(cover, profile_with_most_loops(sk), ct, NEW).bodies.item(0)
            cbody.name = 'Top Cover'
            sk = sketch_at(cover, top_z, 'Cover pin seats')
            sk.isComputeDeferred = True
            for x, y in cg.ring_pin_centres(R, N):
                circle(sk, x, y, top_z, seat_r)
            sk.isComputeDeferred = False
            extrude(cover, all_profiles(sk), ct / 2.0, CUT, [cbody])

    # --- output carrier with integrated pins
    if p['makeCarrier']:
        comp = add_comp(top, 'Output Carrier')
        # Stay inside the ring pins (and therefore inside the housing and cover bore).
        cr = min(Rop + out_r + 0.4, pin_clear_r - 0.05)
        sk = sketch_at(comp, carrier_z, 'Carrier')
        circle(sk, 0, 0, carrier_z, cr)
        body = extrude(comp, sk.profiles.item(0), p['carrierThickness'], NEW).bodies.item(0)
        body.name = 'Output Carrier'
        if M > 0:
            sk = sketch_at(comp, z0, 'Output pins')
            sk.isComputeDeferred = True
            for x, y in cg.output_pin_centres(Rop, M):
                circle(sk, x, y, z0, out_r)
            sk.isComputeDeferred = False
            extrude(comp, all_profiles(sk), carrier_z - z0, JOIN, [body])

    return top



def summary(p):
    return info_html(p)


COMMAND = ReducerCommand(
    'cycloidalDriveGeneratorCmd', 'Cycloidal Drive',
    'Generate a 1, 2 or 3 disc cycloidal reducer for a stepper motor.',
    SPEC, summary, lambda p: build(dict(p, discCount=int(p['discCount']))),
    positive=('ringDiameter', 'pinDiameter', 'eccentricity', 'discThickness', 'bearingOD',
              'bearingID', 'shaftDiameter', 'outputCircleDiameter', 'outputPinDiameter',
              'housingWall', 'baseThickness', 'carrierThickness', 'coverThickness'))
