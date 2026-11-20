"""Runs every command's build() against a stand-in for Fusion's API (unittest.mock), so
typos, wrong argument counts and bad parameter plumbing show up without Fusion.
It does not check the resulting geometry; the *_geometry tests do that."""
import importlib
import os
import sys
import types
import unittest
from unittest import mock

ROOT = os.path.join(os.path.dirname(__file__), '..')


def load_addin():
    adsk = mock.MagicMock()
    for name, mod in (('adsk', adsk), ('adsk.core', adsk.core), ('adsk.fusion', adsk.fusion)):
        sys.modules[name] = mod
    for m in [k for k in sys.modules if k.startswith('FusionReducers')]:
        del sys.modules[m]
    # Register the add-in folder as a package, the way Fusion loads it.
    package = types.ModuleType('FusionReducers')
    package.__path__ = [os.path.join(ROOT, 'FusionReducers')]
    sys.modules['FusionReducers'] = package
    pkg = importlib.import_module('FusionReducers.FusionReducers')
    comp = mock.MagicMock()
    comp.occurrences.addNewComponent.return_value.component = comp
    sketch = comp.sketches.add.return_value
    sketch.profiles.count = 2
    sketch.profiles.item.return_value.profileLoops.count = 1
    sketch.profiles.item.return_value.areaProperties.return_value.area = 1.0
    arc = sketch.sketchCurves.sketchArcs.addByThreePoints.return_value
    arc.startSketchPoint.geometry.distanceTo.return_value = 0.0
    arc.endSketchPoint.geometry.distanceTo.return_value = 1.0
    comp.features.extrudeFeatures.add.return_value.bodies.count = 1
    design = adsk.fusion.Design.cast.return_value
    design.rootComponent.occurrences.addNewComponent.return_value.component = comp
    return pkg, comp


class CommandStubTests(unittest.TestCase):
    def setUp(self):
        self.pkg, self.comp = load_addin()

    def test_every_command_builds_with_defaults(self):
        for cmd in self.pkg.COMMANDS:
            p = cmd.defaults()
            html, errors = cmd.summary(p)
            self.assertEqual(errors, [], cmd.name)
            self.assertIn('Ratio', html, cmd.name)
            cmd.build_fn(p)

    def test_part_toggles_and_motor_choices(self):
        for cmd in self.pkg.COMMANDS:
            flags = [k for k, _, kind, d, _ in cmd.spec if kind == 'bool' and k.startswith('make')]
            for motor in ('None', 'NEMA 14', 'NEMA 17', 'NEMA 23'):
                for off in [None] + flags:
                    p = cmd.defaults()
                    p['motor'] = motor
                    if off:
                        p[off] = False
                    cmd.build_fn(p)

    def test_cycloid_disc_counts(self):
        cmd = self.pkg.cycloid_command.COMMAND
        for n in ('1', '2', '3'):
            p = cmd.defaults()
            p['discCount'] = n
            cmd.build_fn(p)

    def test_belt_profiles(self):
        bg = self.pkg.belt_command.bg
        cmd = self.pkg.belt_command.COMMAND
        for name in bg.PROFILES:
            p = cmd.defaults()
            p['profile'] = name
            p['beltTeeth'] = int(40.0 / bg.PROFILES[name]['pitch'])
            if name == 'HTD 5M':
                p.update(motorTeeth=15, outputTeeth=45)
            self.assertEqual(cmd.summary(p)[1], [], name)
            cmd.build_fn(p)

    def test_capstan_without_groove(self):
        cmd = self.pkg.capstan_command.COMMAND
        p = cmd.defaults()
        p['helicalGroove'] = False
        self.assertEqual(cmd.summary(p)[1], [])
        cmd.build_fn(p)


if __name__ == '__main__':
    unittest.main()
