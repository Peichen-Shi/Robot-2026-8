#!/usr/bin/env python3
import math
import unittest


def unwrap(previous, wrapped):
    return previous + math.atan2(math.sin(wrapped - previous),
                                 math.cos(wrapped - previous))


class YawUnwrapTest(unittest.TestCase):
    def test_positive_boundary(self):
        start = math.radians(179.0)
        result = unwrap(start, math.radians(-179.0))
        self.assertAlmostEqual(math.degrees(result - start), 2.0, places=12)

    def test_negative_boundary(self):
        start = math.radians(-179.0)
        result = unwrap(start, math.radians(179.0))
        self.assertAlmostEqual(math.degrees(result - start), -2.0, places=12)

    def test_sequence_is_locally_continuous(self):
        angles = [170, 179, -179, -170, -160]
        continuous = [math.radians(angles[0])]
        for angle in angles[1:]:
            continuous.append(unwrap(continuous[-1], math.radians(angle)))
        self.assertTrue(all(abs(b - a) < math.radians(20)
                            for a, b in zip(continuous, continuous[1:])))


if __name__ == "__main__":
    unittest.main()
