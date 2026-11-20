import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'RoboticReducers'))
import capstan_geometry as cpg  # noqa: E402

DEFAULTS = dict(cable='Dyneema / braided line', cableDiameter=0.1, drumDiameter=1.4, ratio=8.0,
                jointRange=180.0, minWraps=1.5, anchorAngle=20.0, lipHeight=0.1,
                drumGap=0.1, helicalGroove=True, grooveClearance=0.01, shaftDiameter=0.5,
                shaftClearance=0.01, setScrew=True, hubDiameter=3.0, outputBore=0.8,
                rimThickness=0.4, anchorDepth=0.8, boltCount=4, boltCircleDiameter=2.0,
                boltDiameter=0.34)


class CapstanTests(unittest.TestCase):
    def test_defaults_valid(self):
        errors, _, info = cpg.analyze(DEFAULTS)
        self.assertEqual(errors, [])
        self.assertAlmostEqual(info['R_p'] / info['r_p'], 8.0)

    def test_cable_is_tangent_to_both_pitch_circles(self):
        L = cpg.layout(DEFAULTS)
        a, R, r, C = L['alpha'], L['R_p'], L['r_p'], L['C']
        p1 = (R * math.cos(a), R * math.sin(a))
        p2 = (C + r * math.cos(a), r * math.sin(a))
        t = (p2[0] - p1[0], p2[1] - p1[1])
        self.assertAlmostEqual(t[0] * p1[0] + t[1] * p1[1], 0, places=9)
        self.assertAlmostEqual(t[0] * (p2[0] - C) + t[1] * p2[1], 0, places=9)

    def test_drums_do_not_touch(self):
        L = cpg.layout(DEFAULTS)
        self.assertGreater(L['C'] - L['rim_lip_r'] - L['drum_lip_r'], 0.05)

    def test_cable_stays_on_rim_over_full_range(self):
        L = cpg.layout(DEFAULTS)
        half = math.radians(DEFAULTS['jointRange']) / 2
        for k in range(-10, 11):
            th = half * k / 10.0
            # each anchor is at +/-half_span on the sector, which has turned by th
            for end in (+1, -1):
                anchor = end * L['half_span'] + th
                self.assertGreater(abs(anchor), L['alpha'] + math.radians(19.9))

    def test_helix_mid_point_is_on_y_axis(self):
        pts = cpg.helix_points(5.0, 0.0, 0.7, 1.0, 7.0, 0.15)
        mid = pts[len(pts) // 2]
        self.assertAlmostEqual(mid[0], 5.0, places=6)
        self.assertAlmostEqual(mid[2], 1.0, places=6)
        self.assertAlmostEqual(pts[-1][2] - pts[0][2], 7.0 * 0.15, places=9)

    def test_drum_has_enough_turns(self):
        L = cpg.layout(DEFAULTS)
        self.assertAlmostEqual(L['travel_turns'], 4.0)
        self.assertAlmostEqual(L['turns'], 7.0)

    def test_range_too_large_is_rejected(self):
        errors, _, _ = cpg.analyze(dict(DEFAULTS, jointRange=300.0))
        self.assertTrue(errors)

    def test_thin_steel_drum_warns(self):
        _, warnings, _ = cpg.analyze(dict(DEFAULTS, cable='Steel wire rope'))
        self.assertTrue(warnings)


if __name__ == '__main__':
    unittest.main()
