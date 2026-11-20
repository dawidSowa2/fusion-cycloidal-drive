"""
Shared Fusion helpers: modelling shortcuts plus a small framework that turns a parameter
SPEC into a command dialog with saved settings, a live summary and validation.

Fusion works in centimetres internally; every length here is in cm.
"""

import adsk.core
import adsk.fusion
import json
import math
import os
import traceback

ADDIN_DIR = os.path.dirname(os.path.abspath(__file__))
SETTINGS_DIR = os.path.join(ADDIN_DIR, 'settings')

NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
CUT = adsk.fusion.FeatureOperations.CutFeatureOperation

# name -> (pilot diameter, hole spacing, hole diameter, shaft diameter, body size)  [cm]
MOTORS = {
    'None':    None,
    'NEMA 14': (2.2, 2.6, 0.34, 0.5, 3.5),
    'NEMA 17': (2.2, 3.1, 0.34, 0.5, 4.23),
    'NEMA 23': (3.81, 4.714, 0.51, 0.635, 5.64),
}

M3_TAP = 0.25        # hole for an M3 screw to cut its own thread in plastic
M3_CLEAR = 0.34

_handlers = []       # Fusion only keeps weak references to event handlers


# ----------------------------------------------------------------------------- modelling helpers

def app():
    return adsk.core.Application.get()


def root_component():
    design = adsk.fusion.Design.cast(app().activeProduct)
    if not design:
        raise RuntimeError('Open a Fusion design first.')
    return design.rootComponent


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
    """Sketch on a plane parallel to XY at height z."""
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
    """Model coordinates -> sketch coordinates (works for any sketch plane)."""
    return sk.modelToSketchSpace(P(x, y, z))


def circle(sk, x, y, z, r):
    return sk.sketchCurves.sketchCircles.addByCenterRadius(to_sk(sk, x, y, z), r)


def polygon(sk, pts, z):
    """Closed polygon of sketch lines through (x, y) points at height z."""
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(to_sk(sk, pts[0][0], pts[0][1], z),
                                 to_sk(sk, pts[1][0], pts[1][1], z))
    prev = first
    for x, y in pts[2:]:
        prev = lines.addByTwoPoints(prev.endSketchPoint, to_sk(sk, x, y, z))
    lines.addByTwoPoints(prev.endSketchPoint, first.startSketchPoint)


def _nearest_end(curve, x, y, z, sk):
    target = to_sk(sk, x, y, z)
    a, b = curve.startSketchPoint, curve.endSketchPoint
    return a if a.geometry.distanceTo(target) <= b.geometry.distanceTo(target) else b


def arc3(sk, a, m, b, z):
    """Arc through three (x, y) points at height z."""
    return sk.sketchCurves.sketchArcs.addByThreePoints(
        to_sk(sk, a[0], a[1], z), to_sk(sk, m[0], m[1], z), to_sk(sk, b[0], b[1], z))


def join_line(sk, c1, p1, c2, p2, z):
    """Line between the ends of two curves nearest to p1 and p2."""
    return sk.sketchCurves.sketchLines.addByTwoPoints(
        _nearest_end(c1, p1[0], p1[1], z, sk), _nearest_end(c2, p2[0], p2[1], z, sk))


def slot(sk, x1, y1, x2, y2, r, z):
    """Obround slot of radius r between two centres (a circle if they coincide)."""
    dx, dy = x2 - x1, y2 - y1
    L = math.hypot(dx, dy)
    if L < 1e-6:
        return circle(sk, x1, y1, z, r)
    ux, uy = dx / L, dy / L
    nx, ny = -uy, ux
    a1, a2 = (x1 + nx * r, y1 + ny * r), (x1 - nx * r, y1 - ny * r)
    b1, b2 = (x2 + nx * r, y2 + ny * r), (x2 - nx * r, y2 - ny * r)
    arc_a = arc3(sk, a1, (x1 - ux * r, y1 - uy * r), a2, z)
    arc_b = arc3(sk, b1, (x2 + ux * r, y2 + uy * r), b2, z)
    join_line(sk, arc_a, a1, arc_b, b1, z)
    join_line(sk, arc_a, a2, arc_b, b2, z)


def annular_sector(sk, r_in, r_out, a0, a1, z):
    """Closed region between radii r_in..r_out and angles a0..a1 (radians, a1 > a0)."""
    am = 0.5 * (a0 + a1)
    pol = lambda r, a: (r * math.cos(a), r * math.sin(a))
    outer = arc3(sk, pol(r_out, a0), pol(r_out, am), pol(r_out, a1), z)
    inner = arc3(sk, pol(r_in, a0), pol(r_in, am), pol(r_in, a1), z)
    join_line(sk, outer, pol(r_out, a0), inner, pol(r_in, a0), z)
    join_line(sk, outer, pol(r_out, a1), inner, pol(r_in, a1), z)


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


def largest_profile(sk):
    best, area = None, -1.0
    for i in range(sk.profiles.count):
        pr = sk.profiles.item(i)
        a = pr.areaProperties().area
        if a > area:
            best, area = pr, a
    return best


def extrude(comp, profiles, dist, op, bodies=None):
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(profiles, op)
    inp.setDistanceExtent(False, V(dist))
    if bodies:
        inp.participantBodies = bodies
    return ext.add(inp)


def extrude_symmetric(comp, profiles, total, op, bodies=None):
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(profiles, op)
    inp.setSymmetricExtent(V(total), True)
    if bodies:
        inp.participantBodies = bodies
    return ext.add(inp)


def cylinder(comp, x, y, z, r, h, op=NEW, bodies=None, name=None):
    sk = sketch_at(comp, z, name)
    circle(sk, x, y, z, r)
    feat = extrude(comp, sk.profiles.item(0), h, op, bodies)
    return feat.bodies.item(0) if op == NEW else feat


def d_shaft_sketch(sk, cx, cy, z, r, flat_depth, use_flat):
    """Motor shaft bore, optionally with a D-flat on the +Y side."""
    if not use_flat or flat_depth <= 0 or flat_depth >= r:
        circle(sk, cx, cy, z, r)
        return
    d = r - flat_depth
    w = math.sqrt(r * r - d * d)
    arc = arc3(sk, (cx - w, cy + d), (cx, cy - r), (cx + w, cy + d), z)
    sk.sketchCurves.sketchLines.addByTwoPoints(arc.startSketchPoint, arc.endSketchPoint)


def shaft_bore(comp, body, cx, cy, z, h, p):
    """Cut the motor shaft bore (with optional D-flat) through a body."""
    sk = sketch_at(comp, z, 'Shaft bore')
    r = (p['shaftDiameter'] + p['shaftClearance']) / 2.0
    d_shaft_sketch(sk, cx, cy, z, r, p['flatDepth'], p['dFlat'])
    extrude(comp, all_profiles(sk), h, CUT, [body])


def set_screw_holes(comp, body, cx, cy, z):
    """M3 grub-screw holes through a hub along Y, at height z (both sides).
    The hub must be centred on the X axis (cy == 0), which every generator here uses."""
    sk = comp.sketches.add(comp.xZConstructionPlane)
    sk.name = 'Set screw'
    circle(sk, cx, 0.0, z, M3_TAP / 2.0)
    extrude_symmetric(comp, sk.profiles.item(0), 2 * (abs(cy) + 10.0), CUT, [body])


def motor_holes(sk, cx, cy, z, motor_name, shaft_r, travel=0.0):
    """Pilot + 4 screw holes for a NEMA motor centred at (cx, cy). With travel > 0 the
    holes become slots along X (for belt tensioning)."""
    motor = MOTORS.get(motor_name)
    if not motor:
        slot(sk, cx - travel, cy, cx + travel, cy, shaft_r + 0.1, z)
        return
    pilot, spacing, hole_d, _, _ = motor
    slot(sk, cx - travel, cy, cx + travel, cy, pilot / 2.0 + 0.02, z)
    s = spacing / 2.0
    for sx, sy in ((s, s), (-s, s), (-s, -s), (s, -s)):
        slot(sk, cx + sx - travel, cy + sy, cx + sx + travel, cy + sy, hole_d / 2.0, z)


def motor_size(motor_name, fallback):
    motor = MOTORS.get(motor_name)
    return motor[4] if motor else fallback


def mm(v):
    return '%.2f mm' % (v * 10)


def summary_html(lines, errors, warnings):
    out = list(lines)
    out += ['<font color="#d33">ERROR: %s</font>' % e for e in errors]
    out += ['<font color="#c80">Note: %s</font>' % w for w in warnings]
    return '<br>'.join(out)


# ----------------------------------------------------------------------------- shared SPEC pieces

def motor_spec(default='NEMA 17'):
    return [
        ('motor', 'Motor', 'drop', default, list(MOTORS.keys())),
        ('shaftDiameter', 'Motor shaft diameter', 'len', 0.5, None),
        ('shaftClearance', 'Shaft bore clearance', 'len', 0.01, None),
        ('dFlat', 'D-flat on shaft bore', 'bool', True, None),
        ('flatDepth', 'D-flat depth', 'len', 0.05, None),
    ]


# ----------------------------------------------------------------------------- command framework

class ReducerCommand(object):
    """One toolbar command built from a SPEC list.

    SPEC entries: (id, label, kind, default, extra)
      kind 'group' starts a group (extra=True to expand it), 'len' is a length in cm,
      'int' takes extra=(min, max), 'float' a plain number, 'bool', 'drop' (extra=choices).
      An entry whose id is 'preview' is placed at the top level after the groups.
    summary(p) -> (html, errors);  build(p) creates the geometry.
    """

    def __init__(self, cmd_id, name, tooltip, spec, summary, build, positive=()):
        self.cmd_id, self.name, self.tooltip = cmd_id, name, tooltip
        self.spec, self.summary, self.build_fn = spec, summary, build
        self.positive = positive
        self.settings_file = os.path.join(SETTINGS_DIR, cmd_id + '.json')

    # settings ---------------------------------------------------------------
    def defaults(self):
        return {s[0]: s[3] for s in self.spec if s[2] != 'group'}

    def load(self):
        vals = self.defaults()
        try:
            with open(self.settings_file, 'r') as f:
                for k, v in json.load(f).items():
                    if k in vals:
                        vals[k] = v
        except Exception:
            pass
        return vals

    def save(self, vals):
        try:
            os.makedirs(SETTINGS_DIR, exist_ok=True)
            with open(self.settings_file, 'w') as f:
                json.dump(vals, f, indent=2)
        except Exception:
            pass

    def read(self, inputs):
        vals = {}
        for sid, _, kind, default, _ in self.spec:
            if kind == 'group':
                continue
            inp = inputs.itemById(sid)
            if kind in ('len', 'float'):
                vals[sid] = adsk.core.ValueCommandInput.cast(inp).value
            elif kind == 'int':
                vals[sid] = adsk.core.IntegerSpinnerCommandInput.cast(inp).value
            elif kind == 'bool':
                vals[sid] = adsk.core.BoolValueCommandInput.cast(inp).value
            elif kind == 'drop':
                vals[sid] = adsk.core.DropDownCommandInput.cast(inp).selectedItem.name
        return vals

    # dialog -----------------------------------------------------------------
    def create_inputs(self, cmd):
        vals = self.load()
        inputs = cmd.commandInputs
        parent = inputs
        for sid, label, kind, _, extra in self.spec:
            v = vals.get(sid)
            if kind == 'group':
                g = inputs.addGroupCommandInput(sid, label)
                g.isExpanded = bool(extra)
                parent = g.children
                continue
            target = inputs if sid == 'preview' else parent
            if kind == 'len':
                target.addValueInput(sid, label, 'mm', V(float(v)))
            elif kind == 'float':
                target.addValueInput(sid, label, '', V(float(v)))
            elif kind == 'int':
                target.addIntegerSpinnerCommandInput(sid, label, extra[0], extra[1], 1, int(v))
            elif kind == 'bool':
                target.addBoolValueInput(sid, label, True, '', bool(v))
            elif kind == 'drop':
                dd = target.addDropDownCommandInput(
                    sid, label, adsk.core.DropDownStyles.TextListDropDownStyle)
                choices = list(extra)
                if str(v) not in choices:
                    v = choices[0]
                for choice in choices:
                    dd.listItems.add(choice, choice == str(v), '')
        html, _ = self.summary(self.read(inputs))
        inputs.addTextBoxCommandInput('info', 'Summary', html, 10, True)

    def register(self, ui, panel):
        old = ui.commandDefinitions.itemById(self.cmd_id)
        if old:
            old.deleteMe()
        cmd_def = ui.commandDefinitions.addButtonDefinition(
            self.cmd_id, self.name, self.tooltip, '')
        owner = self

        class Created(adsk.core.CommandCreatedEventHandler):
            def notify(self, args):
                try:
                    cmd = args.command
                    cmd.setDialogMinimumSize(420, 600)
                    cmd.okButtonText = 'Generate'
                    owner.create_inputs(cmd)
                    for evt, h in ((cmd.execute, Execute()), (cmd.executePreview, Preview()),
                                   (cmd.inputChanged, Changed()), (cmd.validateInputs, Validate())):
                        evt.add(h)
                        _handlers.append(h)
                except Exception:
                    ui.messageBox('%s failed:\n%s' % (owner.name, traceback.format_exc()))

        class Changed(adsk.core.InputChangedEventHandler):
            def notify(self, args):
                try:
                    inputs = args.firingEvent.sender.commandInputs
                    if args.input.id == 'motor':
                        motor = MOTORS.get(args.input.selectedItem.name)
                        shaft = inputs.itemById('shaftDiameter')
                        if motor and shaft:
                            shaft.value = motor[3]
                    html, _ = owner.summary(owner.read(inputs))
                    inputs.itemById('info').formattedText = html
                except Exception:
                    pass

        class Validate(adsk.core.ValidateInputsEventHandler):
            def notify(self, args):
                try:
                    p = owner.read(args.firingEvent.sender.commandInputs)
                    _, errors = owner.summary(p)
                    args.areInputsValid = not errors and all(p[k] > 0 for k in owner.positive)
                except Exception:
                    args.areInputsValid = False

        class Preview(adsk.core.CommandEventHandler):
            def notify(self, args):
                try:
                    p = owner.read(args.command.commandInputs)
                    if p.get('preview'):
                        owner.build_fn(p)
                except Exception:
                    pass

        class Execute(adsk.core.CommandEventHandler):
            def notify(self, args):
                try:
                    p = owner.read(args.command.commandInputs)
                    owner.save(p)
                    msg = owner.build_fn(p)
                    if msg:
                        ui.messageBox(msg, owner.name)
                except Exception:
                    ui.messageBox('%s failed:\n%s' % (owner.name, traceback.format_exc()))

        h = Created()
        cmd_def.commandCreated.add(h)
        _handlers.append(h)
        if panel and not panel.controls.itemById(self.cmd_id):
            panel.controls.addCommand(cmd_def)

    def unregister(self, ui, panel):
        if panel:
            ctrl = panel.controls.itemById(self.cmd_id)
            if ctrl:
                ctrl.deleteMe()
        cmd_def = ui.commandDefinitions.itemById(self.cmd_id)
        if cmd_def:
            cmd_def.deleteMe()


def clear_handlers():
    del _handlers[:]
