"""
Capstan drive command for robot-arm joints: a grooved motor drum, an output sector that
turns on the joint axle, and a base plate that holds the motor and the joint bearing.

Layout: sector (joint) axis at the origin, motor drum at (C, 0). The plate's top face is
Z = 0. Lengths in cm (Fusion's internal unit).
"""

import math

import adsk.core
import adsk.fusion

from . import capstan_geometry as cpg
from .fusion_ui import *  # noqa: F401,F403

GAP = 0.1   # clearance between the plate and the moving parts

SPEC = [
    ('gCable', 'Cable and ratio', 'group', None, True),
    ('cable', 'Cable type', 'drop', 'Dyneema / braided line', list(cpg.CABLES.keys())),
    ('cableDiameter', 'Cable diameter', 'len', 0.1, None),
    ('ratio', 'Reduction ratio', 'float', 8.0, None),
    ('drumDiameter', 'Drum pitch diameter (cable centre)', 'len', 1.4, None),
    ('jointRange', 'Joint range (degrees)', 'float', 180.0, None),
    ('minWraps', 'Minimum wraps each side', 'float', 1.5, None),
    ('helicalGroove', 'Helical groove on drum', 'bool', True, None),
    ('grooveClearance', 'Groove clearance', 'len', 0.01, None),
    ('lipHeight', 'Lip height above cable', 'len', 0.1, None),
    ('lipThickness', 'Lip thickness', 'len', 0.1, None),
    ('drumGap', 'Gap between drum and sector lips', 'len', 0.1, None),

    ('gMotor', 'Motor drum', 'group', None, True),
] + motor_spec() + [
    ('hubHeight', 'Drum hub height', 'len', 0.6, None),
    ('setScrew', 'M3 set screw holes in hub', 'bool', True, None),

    ('gOutput', 'Output sector (joint)', 'group', None, None),
    ('outputBore', 'Joint bore (axle)', 'len', 0.8, None),
    ('hubDiameter', 'Sector hub diameter', 'len', 3.0, None),
    ('rimThickness', 'Rim thickness', 'len', 0.4, None),
    ('webThickness', 'Web thickness', 'len', 0.4, None),
    ('anchorAngle', 'Extra rim for cable anchors (deg)', 'float', 20.0, None),
    ('anchorDepth', 'Anchor block depth', 'len', 0.8, None),
    ('boltCount', 'Bolt holes for the arm link', 'int', 4, (0, 12)),
    ('boltCircleDiameter', 'Bolt circle diameter', 'len', 2.0, None),
    ('boltDiameter', 'Bolt hole diameter', 'len', 0.34, None),
    ('bearingOD', 'Plate bearing outer diameter', 'len', 2.2, None),

    ('gParts', 'Parts to generate', 'group', None, None),
    ('makeDrum', 'Motor drum', 'bool', True, None),
    ('makeSector', 'Output sector', 'bool', True, None),
    ('makePlate', 'Base plate', 'bool', True, None),
    ('plateThickness', 'Plate thickness', 'len', 0.6, None),
    ('plateMargin', 'Plate margin', 'len', 0.4, None),

    ('preview', 'Live preview (slower)', 'bool', False, None),
]

POSITIVE = ('cableDiameter', 'ratio', 'drumDiameter', 'jointRange', 'lipThickness',
            'shaftDiameter', 'hubHeight', 'outputBore', 'hubDiameter', 'rimThickness',
            'webThickness', 'anchorDepth', 'bearingOD', 'plateThickness')


def summary(p):
    errors, warnings, L = cpg.analyze(p)
    lines = []
    if 'R_p' in L:
        lines.append('<b>Ratio %.3g : 1</b> (output turns opposite to the motor)' % p['ratio'])
        lines.append('Sector pitch diameter %s, centre distance %s'
                     % (mm(2 * L['R_p']), mm(L['C'])))
        lines.append('Sector span %.0f deg, drum %.1f turns (%.1f of travel), track %s'
                     % (math.degrees(2 * L['half_span']), L['turns'], L['travel_turns'],
                        mm(L['track'])))
        lines.append('Drum is %.1fx the cable diameter. Cable needed: about %.0f cm'
                     % (L['d_ratio'], L['cable_length']))
    return summary_html(lines, errors, warnings), errors


def _helical_groove(comp, body, L, p, cx, z_mid):
    """Cut the cable helix into the drum. Returns an error string if Fusion refuses."""
    try:
        pts = cpg.helix_points(cx, 0.0, L['r_p'], z_mid, L['turns'], L['helix_pitch'])
        sk = comp.sketches.add(comp.xYConstructionPlane)
        sk.name = 'Cable helix'
        col = adsk.core.ObjectCollection.create()
        for x, y, z in pts:
            col.add(to_sk(sk, x, y, z))
        spline = sk.sketchCurves.sketchFittedSplines.add(col)
        path = adsk.fusion.Path.create(spline, adsk.fusion.ChainedCurveOptions.noChainedCurves)
        pin = comp.constructionPlanes.createInput()
        pin.setByDistanceOnPath(spline, V(0.0))
        plane = comp.constructionPlanes.add(pin)
        plane.isLightBulbOn = False
        psk = comp.sketches.add(plane)
        psk.name = 'Groove section'
        x, y, z = pts[0]
        psk.sketchCurves.sketchCircles.addByCenterRadius(
            psk.modelToSketchSpace(P(x, y, z)), L['dc'] / 2.0 + p['grooveClearance'])
        sweeps = comp.features.sweepFeatures
        si = sweeps.createInput(psk.profiles.item(0), path, CUT)
        si.orientation = adsk.fusion.SweepOrientationTypes.PerpendicularOrientationType
        si.participantBodies = [body]
        sweeps.add(si)
        sk.isVisible = False
        return None
    except Exception as e:
        return ('The helical groove could not be cut (%s), so the drum was left smooth. '
                'Turn off "Helical groove" and set the drum diameter to the drum surface '
                'plus one cable diameter to keep the ratio exact.' % e)


def build(p):
    errors, _, L = cpg.analyze(p)
    if errors:
        raise RuntimeError('\n'.join(errors))
    root = root_component()
    C = L['C']
    lip_t = p['lipThickness']
    z_lip0 = GAP + p['hubHeight']          # bottom of the lower lips
    z_track0 = z_lip0 + lip_t
    z_track1 = z_track0 + L['track']
    z_top = z_track1 + lip_t
    z_mid = 0.5 * (z_track0 + z_track1)
    shaft_r = (p['shaftDiameter'] + p['shaftClearance']) / 2.0
    messages = []

    top = add_comp(root, 'Capstan Drive %.3gto1' % p['ratio'])

    # --- motor drum
    if p['makeDrum']:
        comp = add_comp(top, 'Motor Drum')
        body = cylinder(comp, C, 0.0, GAP, L['drum_lip_r'], z_top - GAP, NEW, None, 'Drum')
        body.name = 'Motor Drum'
        sk = sketch_at(comp, z_track0, 'Cable track')
        circle(sk, C, 0.0, z_track0, L['drum_r'])
        circle(sk, C, 0.0, z_track0, L['drum_lip_r'] + 0.05)
        extrude(comp, profile_with_most_loops(sk), L['track'], CUT, [body])
        if p['helicalGroove']:
            msg = _helical_groove(comp, body, L, p, C, z_mid)
            if msg:
                messages.append(msg)
        # cross hole in the middle of the track: the cable's midpoint is threaded through it
        sk = comp.sketches.add(comp.xZConstructionPlane)
        sk.name = 'Cable anchor hole'
        circle(sk, C, 0.0, z_mid, L['dc'] / 2.0 + 0.02)
        extrude_symmetric(comp, sk.profiles.item(0), 2 * L['drum_lip_r'] + 0.2, CUT, [body])
        shaft_bore(comp, body, C, 0.0, GAP, z_top - GAP, p)
        if p['setScrew']:
            set_screw_holes(comp, body, C, 0.0, GAP + p['hubHeight'] / 2.0)

    # --- output sector
    if p['makeSector']:
        comp = add_comp(top, 'Output Sector')
        hs = L['half_span']
        rim_in = L['rim_r'] - p['rimThickness']
        sk = sketch_at(comp, z_lip0, 'Rim')
        annular_sector(sk, rim_in, L['rim_r'], -hs, hs, z_lip0)
        body = extrude(comp, sk.profiles.item(0), z_top - z_lip0, NEW).bodies.item(0)
        body.name = 'Output Sector'
        for z in (z_lip0, z_track1):
            sk = sketch_at(comp, z, 'Lip')
            annular_sector(sk, L['rim_r'] - p['rimThickness'] / 2.0, L['rim_lip_r'], -hs, hs, z)
            extrude(comp, sk.profiles.item(0), lip_t, JOIN, [body])
        sk = sketch_at(comp, z_lip0, 'Web')
        annular_sector(sk, p['hubDiameter'] / 2.0 - 0.05, rim_in + 0.05, -hs, hs, z_lip0)
        extrude(comp, sk.profiles.item(0), p['webThickness'], JOIN, [body])
        cylinder(comp, 0.0, 0.0, GAP, p['hubDiameter'] / 2.0, z_top - GAP, JOIN, [body], 'Hub')

        # cable anchors: a block at each end, a slot from the rim down to the cable height,
        # and a vertical M3 screw the cable is wrapped round and clamped under.
        tab = min(math.radians(p['anchorAngle']), 1.4 / L['rim_r'])
        r_screw = rim_in - p['anchorDepth'] / 2.0
        for sign in (1, -1):
            a0, a1 = (hs - tab, hs) if sign > 0 else (-hs, -hs + tab)
            sk = sketch_at(comp, z_lip0, 'Anchor block')
            annular_sector(sk, rim_in - p['anchorDepth'], rim_in + 0.05, a0, a1, z_lip0)
            extrude(comp, sk.profiles.item(0), z_top - z_lip0, JOIN, [body])
        for sign in (1, -1):
            a = sign * (hs - tab / 2.0)
            ux, uy = math.cos(a), math.sin(a)
            nx, ny = -uy, ux
            w = L['dc'] / 2.0 + 0.03
            r0, r1 = r_screw, L['rim_lip_r'] + 0.1
            z_slot = z_mid - L['dc'] / 2.0
            sk = sketch_at(comp, z_slot, 'Cable slot')
            polygon(sk, [(r0 * ux - w * nx, r0 * uy - w * ny), (r1 * ux - w * nx, r1 * uy - w * ny),
                         (r1 * ux + w * nx, r1 * uy + w * ny), (r0 * ux + w * nx, r0 * uy + w * ny)],
                    z_slot)
            extrude(comp, sk.profiles.item(0), z_top - z_slot + 0.01, CUT, [body])
            sk = sketch_at(comp, z_lip0, 'Anchor screw')
            circle(sk, r_screw * ux, r_screw * uy, z_lip0, M3_TAP / 2.0)
            extrude(comp, sk.profiles.item(0), z_top - z_lip0, CUT, [body])

        sk = sketch_at(comp, GAP, 'Bore and bolts')
        circle(sk, 0.0, 0.0, GAP, p['outputBore'] / 2.0)
        bc = p['boltCircleDiameter'] / 2.0
        for k in range(int(p['boltCount'])):
            a = 2 * math.pi * k / p['boltCount']
            circle(sk, bc * math.cos(a), bc * math.sin(a), GAP, p['boltDiameter'] / 2.0)
        extrude(comp, all_profiles(sk), z_top - GAP, CUT, [body])

    # --- base plate
    if p['makePlate']:
        comp = add_comp(top, 'Base Plate')
        t = p['plateThickness']
        m = p['plateMargin']
        half = motor_size(p['motor'], 2 * L['drum_lip_r'] + 1.0) / 2.0
        x0 = -(p['bearingOD'] / 2.0 + 0.6 + m)
        x1 = C + half + m
        y = max(half, p['bearingOD'] / 2.0 + 0.6) + m
        sk = sketch_at(comp, -t, 'Plate')
        polygon(sk, [(x0, -y), (x1, -y), (x1, y), (x0, y)], -t)
        body = extrude(comp, sk.profiles.item(0), t, NEW).bodies.item(0)
        body.name = 'Base Plate'
        sk = sketch_at(comp, -t, 'Plate holes')
        motor_holes(sk, C, 0.0, -t, p['motor'], shaft_r)
        circle(sk, 0.0, 0.0, -t, p['bearingOD'] / 2.0)
        extrude(comp, all_profiles(sk), t, CUT, [body])

    return '\n\n'.join(messages) if messages else None


COMMAND = ReducerCommand(
    'capstanDriveGeneratorCmd', 'Capstan Drive',
    'Generate a cable capstan drive with a sector output for robot arm joints.',
    SPEC, summary, build, positive=POSITIVE)
