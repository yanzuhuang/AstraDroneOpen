#!/usr/bin/env python3

import unittest

from learning_speed_rl.observation.latest_sample import LatestSampleMailbox


class LatestSampleMailboxTest(unittest.TestCase):
    def test_busy_producer_coalesces_to_newest_without_queue_growth(self):
        mailbox = LatestSampleMailbox()
        self.assertFalse(mailbox.push((1, "old")))
        self.assertTrue(mailbox.push((2, "newer")))
        self.assertTrue(mailbox.push((3, "latest")))
        self.assertEqual(mailbox.input_count, 3)
        self.assertEqual(mailbox.replacement_count, 2)
        self.assertEqual(mailbox.pop(), (3, "latest"))
        self.assertFalse(mailbox.pending)
        self.assertIsNone(mailbox.pop())

    def test_reset_clear_discards_pending_sample(self):
        mailbox = LatestSampleMailbox()
        mailbox.push("old_generation")
        mailbox.clear()
        self.assertFalse(mailbox.pending)
        self.assertIsNone(mailbox.pop())


if __name__ == "__main__":
    unittest.main()
