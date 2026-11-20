import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'RoboticReducers'))
import belt_geometry as bg  # noqa: E402

def _cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _self_intersects(pts):
    n = len(pts)
    segs = [(pts[i], pts[(i + 1) % n]) for i in range(n)]
    for i in range(n):
        for j in range(i + 2, n):
            if i == 0 and j == n - 1:
                continue
            (p1, p2), (q1, q2) = segs[i], segs[j]
            if (_cross(p1, p2, q1) * _cross(p1, p2, q2) < 0 and
                    _cross(q1, q2, p1) * _cross(q1, q2, p2) < 0):
                return True
    return False


DEFAULTS = dict(profile='GT2 2mm', motorTeeth=20, outputTeeth=60, beltTeeth=200,
                grooveClearance=0.005, flanges=True, flangeHeight=0.15, outputBore=0.8,
                boltCount=4, boltCircleDiameter=2.4, boltDiameter=0.34, shaftDiameter=0.5,
                shaftClearance=0.01, setScrew=True, hubDiameter=1.6)


class BeltTests(unittest.TestCase):
    def test_gt2_20t_pulley_size(self):
        # A 20T GT2 pulley has a 12.73 mm pitch diameter and a ~12.22 mm outside diameter.
        self.assertAlmostEqual(bg.pitch_diameter(20, 0.2) * 10, 12.732, places=2)
        self.assertAlmostEqual(bg.outer_diameter(20, bg.PROFILES['GT2 2mm']) * 10, 12.224, places=2)

    def test_centre_distance_round_trips(self):
        for z1, z2, n in ((20, 60, 200), (16, 16, 100), (12, 80, 400)):
            C = bg.centre_distance(n, z1, z2, 0.2)
            L = bg.belt_length(C, bg.pitch_diameter(z1, .2), bg.pitch_diameter(z2, .2))
            self.assertAlmostEqual(L, n * 0.2, places=6)
            self.assertEqual(bg.suggest_belt_teeth(z1, z2, 0.2, C), n)

    def test_equal_pulleys_centre_distance(self):
        C = bg.centre_distance(100, 20, 20, 0.2)          # (L - pi*D) / 2
        self.assertAlmostEqual(C, (20.0 - 4.0) / 2, places=6)

    def test_too_short_belt(self):
        self.assertIsNone(bg.centre_distance(30, 20, 60, 0.2))

    def test_tangent_is_perpendicular_to_radii(self):
        r1, r2, C = 0.6, 1.9, 7.0
        (x1, y1), (x2, y2) = bg.tangent_points(r1, r2, C)
        tx, ty = x2 - x1, y2 - y1
        self.assertAlmostEqual(tx * x1 + ty * y1, 0, places=9)
        self.assertAlmostEqual(tx * (x2 - C) + ty * y2, 0, places=9)

    def test_groove_depth_and_width(self):
        prof = bg.PROFILES['GT2 2mm']
        pts = bg.groove_polygon(20, prof, 0.005)
        ro = bg.outer_diameter(20, prof) / 2
        self.assertAlmostEqual(ro - min(x for x, _ in pts), prof['h'] + 0.005, places=9)
        self.assertAlmostEqual(max(y for _, y in pts), prof['r'] + 0.005, places=9)

    def test_pulley_outline_is_a_simple_toothed_ring(self):
        for name, prof in bg.PROFILES.items():
            z = 20
            pts = bg.pulley_outline(z, prof, 0.005)
            ro = bg.outer_diameter(z, prof) / 2
            radii = [math.hypot(x, y) for x, y in pts]
            self.assertAlmostEqual(max(radii), ro, places=6, msg=name)
            self.assertAlmostEqual(min(radii), ro - prof['h'] - 0.005, places=4, msg=name)
            self.assertFalse(_self_intersects(pts), name)
            area = 0.5 * sum(x1 * y2 - x2 * y1 for (x1, y1), (x2, y2) in zip(pts, pts[1:] + pts[:1]))
            self.assertGreater(area, 0, name)          # counter-clockwise

    def test_defaults_valid(self):
        errors, _, info = bg.analyze(DEFAULTS)
        self.assertEqual(errors, [])
        self.assertAlmostEqual(info['ratio'], 3.0)

    def test_every_profile_valid_with_defaults(self):
        for name in bg.PROFILES:
            p = dict(DEFAULTS, profile=name, beltTeeth=int(400 / bg.PROFILES[name]['pitch'] / 10))
            if name == 'HTD 5M':
                p.update(motorTeeth=15, outputTeeth=45, outputBore=0.8)
            errors, _, _ = bg.analyze(p)
            self.assertEqual(errors, [], name)


if __name__ == '__main__':
    unittest.main()
