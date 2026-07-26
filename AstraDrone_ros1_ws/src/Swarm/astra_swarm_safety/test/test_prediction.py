import unittest

from astra_swarm_safety.prediction import (
    predicted_vertical_minimum,
    separation_clear,
)


class PredictionTest(unittest.TestCase):
    def test_future_crossing_is_rejected(self):
        clear, current, predicted = separation_clear(
            (0, 0, 34), (0, 0, 28),
            [(0, 0, 34), (1, 0, 31)],
            [(2, 0, 28), (1, 0, 31)],
            3.0, 4.0, True)
        self.assertFalse(clear)
        self.assertEqual(predicted, 0.0)
        self.assertEqual(current, 6.0)

    def test_nominal_layers_are_clear(self):
        clear, _, _ = separation_clear(
            (0, 0, 34), (0, 0, 28),
            [(0, 0, 34)], [(0, 0, 28)], 3.0, 4.0, True)
        self.assertTrue(clear)

    def test_future_vertical_conflict_is_rejected(self):
        first = [(0, 0, 10), (0, 0, 8)]
        second = [(8, 0, 16), (8, 0, 10)]
        self.assertEqual(predicted_vertical_minimum(first, second), 2)
        clear, _, _ = separation_clear(
            first[0], second[0], first, second, 3.0, 4.0, True)
        self.assertFalse(clear)


if __name__ == "__main__":
    unittest.main()
