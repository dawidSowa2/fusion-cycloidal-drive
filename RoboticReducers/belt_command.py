"""
Belt drive command: GT2 / HTD timing pulleys for a closed belt, with a motor plate whose
slotted holes let you tension the belt, and an output pulley ready to bolt to an arm link.

Layout: motor pulley at the origin, output pulley at (C, 0). The plate's top face is Z = 0.
Lengths in cm (Fusion's internal unit).
"""

import math

from . import belt_geometry as bg
from .fusion_ui import *  # noqa: F401,F403

GAP = 0.1   # clearance between the plate and the pulleys

SPEC = [
    ('gBelt', 'Belt and pulleys', 'group', None, True),
    ('profile', 'Belt profile', 'drop', 'GT2 2mm', list(bg.PROFILES.keys())),
    ('beltWidth', 'Belt width', 'len', 0.6, None),
    ('motorTeeth', 'Motor pulley teeth', 'int', 20, (8, 150)),
    ('outputTeeth', 'Output pulley teeth', 'int', 60, (8, 300)),
    ('beltTeeth', 'Closed belt length (teeth)', 'int', 140, (20, 3000)),
    ('grooveClearance', 'Groove clearance', 'len', 0.005, None),
    ('flanges', 'Flanges on pulleys', 'bool', True, None),
    ('flangeHeight', 'Flange height above teeth', 'len', 0.15, None),
    ('flangeThickness', 'Flange thickness', 'len', 0.1, None),

    ('gMotor', 'Motor pulley', 'group', None, True),
] + motor_spec() + [
    ('hubDiameter', 'Hub diameter', 'len', 1.6, None),
    ('hubHeight', 'Hub height', 'len', 0.7, None),
    ('setScrew', 'M3 set screw holes in hub', 'bool', True, None),

    ('gOutput', 'Output pulley (joint)', 'group', None, None),
    ('outputBore', 'Output bore (axle)', 'len', 0.8, None),
    ('spacerDiameter', 'Spacer under output pulley', 'len', 1.2, None),
    ('boltCount', 'Bolt holes for the arm link', 'int', 4, (0, 12)),
    ('boltCircleDiameter', 'Bolt circle diameter', 'len', 2.4, None),
    ('boltDiameter', 'Bolt hole diameter', 'len', 0.34, None),
    ('bearingOD', 'Plate bearing outer diameter', 'len', 2.2, None),

    ('gParts', 'Parts to generate', 'group', None, None),
    ('makeMotorPulley', 'Motor pulley', 'bool', True, None),
    ('makeOutputPulley', 'Output pulley', 'bool', True, None),
    ('makeBelt', 'Belt (reference only)', 'bool', True, None),
    ('makePlate', 'Motor plate', 'bool', True, None),
    ('plateThickness', 'Plate thickness', 'len', 0.6, None),
    ('tensionTravel', 'Tensioning slot travel (each way)', 'len', 0.2, None),
    ('plateMargin', 'Plate margin', 'len', 0.4, None),

    ('preview', 'Live preview (slower)', 'bool', False, None),
]

POSITIVE = ('beltWidth', 'flangeThickness', 'shaftDiameter', 'hubDiameter', 'hubHeight',
            'outputBore', 'spacerDiameter', 'bearingOD', 'plateThickness')


def summary(p):
    errors, warnings, info = bg.analyze(p)
    lines = []
    if 'ratio' in info:
        lines.append('<b>Ratio %.3g : 1</b> (same direction as the motor)' % info['ratio'])
        lines.append('Pitch diameters %s / %s, outside %s / %s'
                     % (mm(info['pd1']), mm(info['pd2']), mm(info['od1']), mm(info['od2'])))
    if 'C' in info:
        lines.append('Centre distance %s for a %s belt (%d teeth), %.1f teeth in mesh'
                     % (mm(info['C']), mm(info['beltLength']), int(p['beltTeeth']),
                        info['mesh']))
    return summary_html(lines, errors, warnings), errors


def _pulley(comp, cx, teeth, prof, p, band_z, band_h, name):
    """Toothed band with optional flanges. Returns the body."""
    sk = sketch_at(comp, band_z, name + ' teeth')
    sk.isComputeDeferred = True
    pts = [(cx + x, y) for x, y in bg.pulley_outline(teeth, prof, p['grooveClearance'])]
    polygon(sk, pts, band_z)
    sk.isComputeDeferred = False
    body = extrude(comp, largest_profile(sk), band_h, NEW).bodies.item(0)
    body.name = name
    if p['flanges']:
        fr = bg.outer_diameter(teeth, prof) / 2.0 + p['flangeHeight']
        ft = p['flangeThickness']
        cylinder(comp, cx, 0.0, band_z - ft, fr, ft, JOIN, [body], 'Bottom flange')
        cylinder(comp, cx, 0.0, band_z + band_h, fr, ft, JOIN, [body], 'Top flange')
    return body


def build(p):
    errors, _, info = bg.analyze(p)
    if errors:
        raise RuntimeError('\n'.join(errors))
    root = root_component()
    prof = bg.PROFILES[p['profile']]
    z1, z2 = int(p['motorTeeth']), int(p['outputTeeth'])
    C = info['C']
    ft = p['flangeThickness'] if p['flanges'] else 0.0
    band_h = p['beltWidth'] + 0.1
    band_z = GAP + p['hubHeight'] + ft
    top_z = band_z + band_h + ft
    shaft_r = (p['shaftDiameter'] + p['shaftClearance']) / 2.0

    top = add_comp(root, 'Belt Drive %s %dT-%dT' % (p['profile'].replace(' ', ''), z1, z2))

    if p['makeMotorPulley']:
        comp = add_comp(top, 'Motor Pulley %dT' % z1)
        body = _pulley(comp, 0.0, z1, prof, p, band_z, band_h, 'Motor Pulley')
        cylinder(comp, 0.0, 0.0, GAP, p['hubDiameter'] / 2.0, p['hubHeight'], JOIN, [body], 'Hub')
        shaft_bore(comp, body, 0.0, 0.0, GAP, top_z - GAP, p)
        if p['setScrew']:
            set_screw_holes(comp, body, 0.0, 0.0, GAP + p['hubHeight'] / 2.0)

    if p['makeOutputPulley']:
        comp = add_comp(top, 'Output Pulley %dT' % z2)
        body = _pulley(comp, C, z2, prof, p, band_z, band_h, 'Output Pulley')
        cylinder(comp, C, 0.0, GAP, p['spacerDiameter'] / 2.0, band_z - ft - GAP, JOIN, [body],
                 'Spacer')
        sk = sketch_at(comp, GAP, 'Bore and bolts')
        circle(sk, C, 0.0, GAP, p['outputBore'] / 2.0)
        for k in range(int(p['boltCount'])):
            a = 2 * math.pi * k / p['boltCount']
            bc = p['boltCircleDiameter'] / 2.0
            circle(sk, C + bc * math.cos(a), bc * math.sin(a), GAP, p['boltDiameter'] / 2.0)
        extrude(comp, all_profiles(sk), top_z - GAP, CUT, [body])

    if p['makeBelt']:
        comp = add_comp(top, 'Belt (reference)')
        z = band_z + 0.05
        sk = sketch_at(comp, z, 'Belt path')
        back = prof['thick'] - prof['h']
        for r1, r2 in ((info['od1'] / 2, info['od2'] / 2),
                       (info['od1'] / 2 + back, info['od2'] / 2 + back)):
            (ux1, uy1), (ux2, uy2) = bg.tangent_points(r1, r2, C)
            a1 = arc3(sk, (ux1, uy1), (-r1, 0.0), (ux1, -uy1), z)
            a2 = arc3(sk, (ux2, uy2), (C + r2, 0.0), (ux2, -uy2), z)
            join_line(sk, a1, (ux1, uy1), a2, (ux2, uy2), z)
            join_line(sk, a1, (ux1, -uy1), a2, (ux2, -uy2), z)
        belt = extrude(comp, profile_with_most_loops(sk), p['beltWidth'], NEW).bodies.item(0)
        belt.name = 'Belt %d teeth (reference, buy this)' % int(p['beltTeeth'])

    if p['makePlate']:
        comp = add_comp(top, 'Motor Plate')
        t = p['plateThickness']
        m = p['plateMargin']
        half = motor_size(p['motor'], p['hubDiameter'] + 1.0) / 2.0
        x0 = -half - m - p['tensionTravel']
        x1 = C + p['bearingOD'] / 2.0 + 0.6 + m
        y = max(half, p['bearingOD'] / 2.0 + 0.6) + m
        sk = sketch_at(comp, -t, 'Plate')
        polygon(sk, [(x0, -y), (x1, -y), (x1, y), (x0, y)], -t)
        body = extrude(comp, sk.profiles.item(0), t, NEW).bodies.item(0)
        body.name = 'Motor Plate'
        sk = sketch_at(comp, -t, 'Plate holes')
        motor_holes(sk, 0.0, 0.0, -t, p['motor'], shaft_r, p['tensionTravel'])
        circle(sk, C, 0.0, -t, p['bearingOD'] / 2.0)
        extrude(comp, all_profiles(sk), t, CUT, [body])
    return None


COMMAND = ReducerCommand(
    'beltDriveGeneratorCmd', 'Belt Drive',
    'Generate GT2 / HTD timing pulleys, a reference belt and a tensioning motor plate.',
    SPEC, summary, build, positive=POSITIVE)
