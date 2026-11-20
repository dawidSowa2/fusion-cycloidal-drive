"""Geometry checks that run without Fusion:  python3 -m unittest discover tests"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'FusionReducers'))
import cycloid_geometry as cg  # noqa: E402

DESIGNS = [  # (pins, ring radius, pin radius, eccentricity) in cm
    (20, 3.5, 0.26, 0.15),
    (12, 2.5, 0.20, 0.12),
    (16, 3.0, 0.16, 0.17),
    (10, 3.0, 0.41, 0.25),
]


def _seg_dist(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy)))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def _poly_dist(p, pts):
    return min(_seg_dist(p, pts[i], pts[(i + 1) % len(pts)]) for i in range(len(pts)))


def _poses(n, E, N, inp):
    for k in range(n):
        th = inp + 2 * math.pi * k / n
        yield E * math.cos(th), E * math.sin(th), -th / (N - 1)


class GeometryTests(unittest.TestCase):
    def test_designs_are_valid(self):
        for N, R, Rr, E in DESIGNS:
            self.assertLess(E * N, R)
            self.assertLess(Rr, cg.min_convex_radius(R, E, N))

    def test_discs_touch_every_ring_pin(self):
        for N, R, Rr, E in DESIGNS:
            prof = cg.disc_profile(R, Rr, E, N, 40)
            for n in (2, 3):
                for inp in (0.0, 0.37, 1.9):
                    for cx, cy, rot in _poses(n, E, N, inp):
                        disc = cg.transform(prof, cx, cy, rot)
                        for pin in cg.ring_pin_centres(R, N):
                            self.assertAlmostEqual(_poly_dist(pin, disc), Rr, delta=0.002)

    def test_discs_clear_housing(self):
        for N, R, Rr, E in DESIGNS:
            prof = cg.disc_profile(R, Rr, E, N, 40)
            bore = cg.housing_bore_radius(R, Rr, E)
            for n in (2, 3):
                for step in range(60):
                    for cx, cy, rot in _poses(n, E, N, math.radians(step * 7.3)):
                        reach = max(math.hypot(x, y) for x, y in cg.transform(prof, cx, cy, rot))
                        self.assertLess(reach, bore - 0.04)

    def test_default_settings_pass_validation(self):
        p = dict(pinCount=20, ringDiameter=7.0, pinDiameter=.5, eccentricity=.15,
                 profileClearance=.01, discCount=3, outputPinCount=8,
                 outputCircleDiameter=4.4, outputPinDiameter=.5, outputHoleClearance=.02,
                 bearingOD=2.7, bearingID=2.0, shaftDiameter=.5, shaftClearance=.01,
                 housingWall=.5, makeHousing=True)
        errors, _, info = cg.analyze(p)
        self.assertEqual(errors, [])
        self.assertEqual(info['ratio'], 19)

    def test_overlarge_eccentricity_is_rejected(self):
        p = dict(pinCount=16, ringDiameter=6.0, pinDiameter=.6, eccentricity=.25,
                 profileClearance=0, discCount=2, outputPinCount=6,
                 outputCircleDiameter=3.0, outputPinDiameter=.4, outputHoleClearance=0,
                 bearingOD=2.0, bearingID=1.5, shaftDiameter=.5, shaftClearance=0)
        errors, _, _ = cg.analyze(p)
        self.assertTrue(errors)


if __name__ == '__main__':
    unittest.main()
