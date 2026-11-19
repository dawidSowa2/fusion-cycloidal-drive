"""
Cycloidal Drive generator for Autodesk Fusion.

Adds a "Cycloidal Drive" command to SOLID > CREATE. It builds a complete, phase-correct
1, 2 or 3 disc cycloidal reducer sized for a stepper motor:
  - cycloidal discs (each disc rotated/phased correctly for its eccentric)
  - eccentric cam with a (D-flat) motor shaft bore
  - ring pins and a housing ring that holds them
  - optional NEMA 14/17/23 base plate
  - optional output carrier with output pins

Fusion works in centimetres internally; every length in this file is in cm.
"""

import adsk.core
import adsk.fusion
import traceback
import json
import math
import os

from . import cycloid_geometry as cg

CMD_ID = 'cycloidalDriveGeneratorCmd'
CMD_NAME = 'Cycloidal Drive'
CMD_TOOLTIP = 'Generate a 1, 2 or 3 disc cycloidal reducer for a stepper motor.'
PANEL_ID = 'SolidCreatePanel'
SETTINGS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'last_settings.json')

app = None
ui = None
handlers = []

# name -> (pilot diameter, mounting hole spacing, mounting hole diameter, shaft diameter)  [cm]
MOTORS = {
    'None':    None,
    'NEMA 14': (2.2, 2.6, 0.34, 0.5),
    'NEMA 17': (2.2, 3.1, 0.34, 0.5),
    'NEMA 23': (3.81, 4.714, 0.51, 0.635),
}

# Input specification: (id, label, kind, default, extra)
#   kind: 'len' (cm), 'int' (min, max), 'bool', 'drop' (choices), 'group'
SPEC = [
    ('gGear', 'Cycloidal stage', 'group', None, None),
    ('pinCount', 'Ring pins (ratio = pins - 1)', 'int', 20, (5, 150)),
    ('ringDiameter', 'Ring pin circle diameter', 'len', 7.0, None),
    ('pinDiameter', 'Ring pin diameter', 'len', 0.5, None),
    ('eccentricity', 'Eccentricity', 'len', 0.15, None),
    ('profileClearance', 'Profile clearance (shrinks disc)', 'len', 0.01, None),
    ('discCount', 'Number of discs', 'drop', '3', ['1', '2', '3']),
    ('discThickness', 'Disc thickness', 'len', 0.6, None),
    ('discGap', 'Gap between parts', 'len', 0.05, None),
    ('pointsPerLobe', 'Spline points per lobe', 'int', 40, (10, 200)),

    ('gShaft', 'Bearing and motor', 'group', None, None),
    ('bearingOD', 'Disc bearing outer diameter (disc bore)', 'len', 2.7, None),
    ('bearingID', 'Disc bearing inner diameter (cam OD)', 'len', 2.0, None),
    ('motor', 'Motor', 'drop', 'NEMA 17', list(MOTORS.keys())),
    ('shaftDiameter', 'Motor shaft diameter', 'len', 0.5, None),
    ('shaftClearance', 'Shaft bore clearance', 'len', 0.01, None),
    ('dFlat', 'D-flat on shaft bore', 'bool', True, None),
    ('flatDepth', 'D-flat depth', 'len', 0.05, None),

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


# ----------------------------------------------------------------------------- settings

def load_settings():
    vals = {s[0]: s[3] for s in SPEC if s[2] != 'group'}
    try:
        with open(SETTINGS_FILE, 'r') as f:
            saved = json.load(f)
        for k, v in saved.items():
            if k in vals:
                vals[k] = v
    except Exception:
        pass
    return vals


def save_settings(vals):
    try:
        with open(SETTINGS_FILE, 'w') as f:
            json.dump(vals, f, indent=2)
    except Exception:
        pass


def read_inputs(inputs):
    vals = {}
    for sid, _, kind, _, _ in SPEC:
        if kind == 'group':
            continue
        inp = inputs.itemById(sid)
        if kind == 'len':
            vals[sid] = adsk.core.ValueCommandInput.cast(inp).value
        elif kind == 'int':
            vals[sid] = adsk.core.IntegerSpinnerCommandInput.cast(inp).value
        elif kind == 'bool':
            vals[sid] = adsk.core.BoolValueCommandInput.cast(inp).value
        elif kind == 'drop':
            vals[sid] = adsk.core.DropDownCommandInput.cast(inp).selectedItem.name
    vals['discCount'] = int(vals['discCount'])
    return vals


# ----------------------------------------------------------------------------- info text

def mm(v):
    return '%.2f mm' % (v * 10)


def info_html(p):
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


# ----------------------------------------------------------------------------- modelling helpers

def P(x, y, z=0.0):
    return adsk.core.Point3D.create(x, y, z)


def V(v):
    return adsk.core.ValueInput.createByReal(v)


def add_comp(parent, name):
    occ = parent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component
    comp.name = name
    return comp


def sketch_at(comp, z, name=None):
    if abs(z) < 1e-9:
        plane = comp.xYConstructionPlane
    else:
        pin = comp.constructionPlanes.createInput()
        pin.setByOffset(comp.xYConstructionPlane, V(z))
        plane = comp.constructionPlanes.add(pin)
        plane.isLightBulbOn = False
    sk = comp.sketches.add(plane)
    if name:
        sk.name = name
    return sk


def to_sk(sk, x, y, z):
    """Model coordinates -> sketch coordinates (robust for offset planes)."""
    return sk.modelToSketchSpace(P(x, y, z))


def circle(sk, x, y, z, r):
    return sk.sketchCurves.sketchCircles.addByCenterRadius(to_sk(sk, x, y, z), r)


def all_profiles(sk):
    col = adsk.core.ObjectCollection.create()
    for i in range(sk.profiles.count):
        col.add(sk.profiles.item(i))
    return col


def profile_with_most_loops(sk):
    best = None
    for i in range(sk.profiles.count):
        pr = sk.profiles.item(i)
        if best is None or pr.profileLoops.count > best.profileLoops.count:
            best = pr
    return best


def extrude(comp, profiles, dist, op, bodies=None):
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(profiles, op)
    inp.setDistanceExtent(False, V(dist))
    if bodies:
        inp.participantBodies = bodies
    return ext.add(inp)


NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation


def d_shaft_sketch(sk, z, r, flat_depth, use_flat):
    if not use_flat or flat_depth <= 0 or flat_depth >= r:
        circle(sk, 0, 0, z, r)
        return
    d = r - flat_depth
    w = math.sqrt(r * r - d * d)
    a_start = math.atan2(d, -w)     # left end of the flat
    a_end = math.atan2(d, w)        # right end of the flat
    sweep = (a_end + 2 * math.pi) - a_start
    arcs = sk.sketchCurves.sketchArcs
    arc = arcs.addByCenterStartSweep(to_sk(sk, 0, 0, z), to_sk(sk, -w, d, z), sweep)
    sk.sketchCurves.sketchLines.addByTwoPoints(arc.startSketchPoint, arc.endSketchPoint)


# ----------------------------------------------------------------------------- builder

def build(p):
    design = adsk.fusion.Design.cast(app.activeProduct)
    if not design:
        raise RuntimeError('Open a Fusion design first.')
    root = design.rootComponent

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
        sk = sketch_at(comp, z0, 'Shaft bore')
        d_shaft_sketch(sk, z0, shaft_r, p['flatDepth'], p['dFlat'])
        extrude(comp, all_profiles(sk), stack_top - z0, CUT, [body])

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
            motor = MOTORS.get(p['motor'])
            if motor:
                pilot, spacing, hole_d, _ = motor
                circle(sk, 0, 0, -bt, pilot / 2.0 + 0.02)
                s = spacing / 2.0
                for sx, sy in ((s, s), (-s, s), (-s, -s), (s, -s)):
                    circle(sk, sx, sy, -bt, hole_d / 2.0)
            else:
                circle(sk, 0, 0, -bt, shaft_r + 0.1)
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


# ----------------------------------------------------------------------------- event handlers

class CreatedHandler(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        try:
            cmd = args.command
            cmd.setDialogMinimumSize(420, 600)
            cmd.okButtonText = 'Generate'
            vals = load_settings()
            inputs = cmd.commandInputs
            parent = inputs
            for sid, label, kind, _, extra in SPEC:
                v = vals.get(sid)
                if kind == 'group':
                    g = inputs.addGroupCommandInput(sid, label)
                    g.isExpanded = sid in ('gGear', 'gShaft')
                    parent = g.children
                elif sid == 'preview':
                    parent = inputs
                    parent.addBoolValueInput(sid, label, True, '', bool(v))
                elif kind == 'len':
                    parent.addValueInput(sid, label, 'mm', V(float(v)))
                elif kind == 'int':
                    parent.addIntegerSpinnerCommandInput(sid, label, extra[0], extra[1], 1, int(v))
                elif kind == 'bool':
                    parent.addBoolValueInput(sid, label, True, '', bool(v))
                elif kind == 'drop':
                    dd = parent.addDropDownCommandInput(
                        sid, label, adsk.core.DropDownStyles.TextListDropDownStyle)
                    for choice in extra:
                        dd.listItems.add(choice, choice == str(v), '')
            html, _ = info_html(read_inputs(inputs))
            inputs.addTextBoxCommandInput('info', 'Summary', html, 9, True)

            for evt, h in ((cmd.execute, ExecuteHandler()),
                           (cmd.executePreview, PreviewHandler()),
                           (cmd.inputChanged, InputChangedHandler()),
                           (cmd.validateInputs, ValidateHandler()),
                           (cmd.destroy, DestroyHandler())):
                evt.add(h)
                handlers.append(h)
        except Exception:
            ui.messageBox('Cycloidal Drive failed:\n' + traceback.format_exc())


class InputChangedHandler(adsk.core.InputChangedEventHandler):
    def notify(self, args):
        try:
            inputs = args.firingEvent.sender.commandInputs
            if args.input.id == 'motor':
                motor = MOTORS.get(args.input.selectedItem.name)
                if motor:
                    inputs.itemById('shaftDiameter').value = motor[3]
            html, _ = info_html(read_inputs(inputs))
            inputs.itemById('info').formattedText = html
        except Exception:
            pass


class ValidateHandler(adsk.core.ValidateInputsEventHandler):
    def notify(self, args):
        try:
            p = read_inputs(args.firingEvent.sender.commandInputs)
            errors, _, _ = cg.analyze(p)
            positive = ('ringDiameter', 'pinDiameter', 'eccentricity', 'discThickness',
                        'bearingOD', 'bearingID', 'shaftDiameter', 'outputCircleDiameter',
                        'outputPinDiameter', 'housingWall', 'baseThickness', 'carrierThickness', 'coverThickness')
            ok = not errors and all(p[k] > 0 for k in positive) and p['discGap'] >= 0
            args.areInputsValid = ok
        except Exception:
            args.areInputsValid = False


class PreviewHandler(adsk.core.CommandEventHandler):
    def notify(self, args):
        try:
            p = read_inputs(args.command.commandInputs)
            if p['preview']:
                build(p)
        except Exception:
            pass


class ExecuteHandler(adsk.core.CommandEventHandler):
    def notify(self, args):
        try:
            p = read_inputs(args.command.commandInputs)
            save_settings(p)
            build(p)
        except Exception:
            ui.messageBox('Cycloidal Drive failed:\n' + traceback.format_exc())


class DestroyHandler(adsk.core.CommandEventHandler):
    def notify(self, args):
        pass


# ----------------------------------------------------------------------------- add-in entry points

def run(context):
    global app, ui
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        cmd_def = ui.commandDefinitions.itemById(CMD_ID)
        if cmd_def:
            cmd_def.deleteMe()
        res = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'resources')
        cmd_def = ui.commandDefinitions.addButtonDefinition(
            CMD_ID, CMD_NAME, CMD_TOOLTIP, res if os.path.isdir(res) else '')
        h = CreatedHandler()
        cmd_def.commandCreated.add(h)
        handlers.append(h)
        panel = ui.allToolbarPanels.itemById(PANEL_ID)
        if panel and not panel.controls.itemById(CMD_ID):
            panel.controls.addCommand(cmd_def)
    except Exception:
        if ui:
            ui.messageBox('Cycloidal Drive add-in failed to start:\n' + traceback.format_exc())


def stop(context):
    try:
        panel = ui.allToolbarPanels.itemById(PANEL_ID)
        if panel:
            ctrl = panel.controls.itemById(CMD_ID)
            if ctrl:
                ctrl.deleteMe()
        cmd_def = ui.commandDefinitions.itemById(CMD_ID)
        if cmd_def:
            cmd_def.deleteMe()
        handlers.clear()
    except Exception:
        if ui:
            ui.messageBox('Cycloidal Drive add-in failed to stop:\n' + traceback.format_exc())
