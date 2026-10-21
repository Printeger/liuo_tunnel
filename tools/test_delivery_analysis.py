import unittest
from delivery_analysis import analyze


class DeliveryAnalysisTest(unittest.TestCase):
    def fixture(self):
        def row(sequence, **extra):
            return dict(producer="1", session="10", sequence=str(sequence), **extra)

        profile = dict(outages=[[1, 6]])
        delivery = [
            row(1, action="delayed", source_elapsed_s="6.1"),
            row(1, action="duplicate", source_elapsed_s="6.1"),
            row(2, action="expired_delay", source_elapsed_s="0"),
            row(3, action="outage_drop", source_elapsed_s="2"),
        ]
        decisions = [
            row(1, reason="ACCEPT_TA", decision_wall_ns="6200000000"),
            row(1, reason="DUPLICATE"),
            row(2, reason="WINDOW_EXPIRED"),
        ]
        trajectory = [
            dict(stamp=str(2.5 + 0.1 * i), state_reason="UWB_ABSENT_LIO_CONTINUES")
            for i in range(45)
        ]
        return [
            profile,
            delivery,
            decisions,
            trajectory,
            [dict(reset="0", fallback="0")],
            [dict(session="1", stamp="1", scheduled_wall_ns="0")],
            dict(delivery_queue_peak=5),
        ]

    def test_correct_fault_recovery(self):
        result = analyze(*self.fixture())
        self.assertEqual(result["result"], "PASS")
        self.assertAlmostEqual(result["outages"][0]["fresh_ta_recovery_wall_s"], 0.2)
        self.assertEqual(result["expired_delayed_rejections"], 1)

    def test_duplicate_ingestion_and_hidden_gap_fail(self):
        data = self.fixture()
        data[2].append(data[2][0])
        data[3].pop()
        result = analyze(*data)
        self.assertIn("physical measurement entered graph twice", result["failures"])
        self.assertTrue(any("publication gap" in s for s in result["failures"]))

    def test_false_tracking_reason_fails(self):
        data = self.fixture()
        data[3][0]["state_reason"] = "JOINT_LIDAR_IMU_UWB"
        self.assertTrue(any("absence reason" in s for s in analyze(*data)["failures"]))
