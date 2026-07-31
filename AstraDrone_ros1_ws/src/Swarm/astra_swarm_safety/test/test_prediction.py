import unittest

from astra_swarm_safety.prediction import (
    ellipsoid_distance,
    pairwise_ids,
    predicted_vertical_minimum,
    separation_clear,
)


class PredictionTest(unittest.TestCase):
    def test_three_vehicle_pairs_cover_uav3(self):
        self.assertEqual(
            pairwise_ids([1, 2, 3]), [(1, 2), (1, 3), (2, 3)])

    def test_vehicle_ids_must_be_sorted_and_unique(self):
        with self.assertRaises(ValueError):
            pairwise_ids([1, 3, 2])
        with self.assertRaises(ValueError):
            pairwise_ids([1, 1])

    def test_future_crossing_is_rejected(self):
        clear, current, predicted = separation_clear(
            (0, 0, 34), (0, 0, 28),
            [(0, 0, 34), (1, 0, 31)],
            [(2, 0, 28), (1, 0, 31)],
            3.0, 1.5)
        self.assertFalse(clear)
        self.assertEqual(predicted, 0.0)
        self.assertEqual(current, 6.0)

    def test_nominal_layers_are_clear(self):
        clear, _, _ = separation_clear(
            (0, 0, 34), (0, 0, 28),
            [(0, 0, 34)], [(0, 0, 28)], 3.0, 1.5)
        self.assertTrue(clear)

    def test_horizontal_phase_allows_close_fixed_layers(self):
        first = [(0, 0, 10), (0, 0, 8)]
        second = [(8, 0, 16), (8, 0, 10)]
        self.assertEqual(predicted_vertical_minimum(first, second), 2)
        clear, _, _ = separation_clear(
            first[0], second[0], first, second, 3.0, 1.5)
        self.assertTrue(clear)

    def test_same_xy_two_three_four_layers_fail_ellipsoid(self):
        self.assertLess(ellipsoid_distance((0, 0, 2), (0, 0, 4)), 3.0)
        clear, _, _ = separation_clear(
            (0, 0, 2), (0, 0, 4),
            [(0, 0, 2)], [(0, 0, 4)], 3.0, 1.5)
        self.assertFalse(clear)

    def test_one_metre_vertical_is_safe_with_large_phase(self):
        clear, _, _ = separation_clear(
            (0, 0, 2), (12, 0, 3),
            [(0, 0, 2)], [(12, 0, 3)], 3.0, 1.5)
        self.assertTrue(clear)


if __name__ == "__main__":
    unittest.main()
