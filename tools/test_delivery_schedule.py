import unittest
from delivery_schedule import DeliverySchedule


class DeliveryScheduleTest(unittest.TestCase):
    def test_nominal_preserves_all_bytes_and_order(self):
        source = [
            ("/clock", b"clock", 0),
            ("/sensors/uwb/range", b"physical", 0),
            ("/clock", b"later", 5_000_000),
        ]
        schedule = DeliverySchedule()
        result = list(schedule.events(iter(source)))
        self.assertEqual([(t, b, s) for t, b, s, d, a in result], source)
        self.assertTrue(all(s == d and a == "original" for t, b, s, d, a in result))
        self.assertLessEqual(schedule.peak, 1)

    def test_delays_reorder_duplicate_and_drop_without_changing_measurements(self):
        schedule = DeliverySchedule(
            dict(
                schema_version="liuo_delivery_v1",
                begin_s=0,
                end_s=1,
                delay_cycle_ms=[50, 25],
                duplicate_every=2,
                duplicate_delay_ms=1,
                outages=[[0.1, 0.2]],
            )
        )
        source = [
            ("/clock", b"clock", 0),
            ("/sensors/uwb/range", b"first", 0),
            ("/sensors/uwb/range", b"second", 5_000_000),
            ("/sensors/uwb/range", b"outage", 100_000_000),
        ]
        result = list(schedule.events(iter(source)))
        self.assertEqual(
            [r[1] for r in result], [b"clock", b"second", b"second", b"first", b"outage"]
        )
        self.assertEqual(
            [r[3] for r in result], [0, 30_000_000, 31_000_000, 50_000_000, 100_000_000]
        )
        self.assertEqual(result[2][2], 5_000_000)
        self.assertEqual(result[2][4], "duplicate")
        self.assertEqual(result[-1][4], "outage_drop")

    def test_expiry_and_queue_limit_are_explicit(self):
        profile = dict(
            schema_version="liuo_delivery_v1",
            begin_s=0,
            end_s=1,
            expired_every=1,
            expired_delay_s=12,
        )
        event = list(
            DeliverySchedule(profile).events(iter([("/sensors/uwb/range", b"unchanged", 0)]))
        )[0]
        self.assertEqual(event[3], 12_000_000_000)
        self.assertEqual(event[4], "expired_delay")
        with self.assertRaisesRegex(ValueError, "queue limit"):
            list(
                DeliverySchedule(profile, capacity=1).events(
                    iter([("/sensors/uwb/range", b"one", 0), ("/sensors/uwb/range", b"two", 1)])
                )
            )
        with self.assertRaisesRegex(ValueError, "nonmonotonic"):
            list(DeliverySchedule().events(iter([("/clock", b"a", 2), ("/clock", b"b", 1)])))

    def test_invalid_profiles_rejected(self):
        for change in [
            dict(delay_cycle_ms=[]),
            dict(delay_cycle_ms=[float("nan")]),
            dict(duplicate_every=-1),
            dict(duplicate_every=True),
            dict(end_s=-1),
            dict(outages=[[0, 2]]),
            dict(silent_drop=True),
        ]:
            with self.subTest(change=change), self.assertRaises(ValueError):
                profile = dict(schema_version="liuo_delivery_v1", begin_s=0, end_s=1)
                profile.update(change)
                DeliverySchedule(profile)
