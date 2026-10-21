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

    def test_anchor_outage_ta_aa_boundaries_and_unaffected_bytes(self):
        profile = dict(
            schema_version="liuo_delivery_v1",
            begin_s=0,
            end_s=3,
            anchor_outages=[dict(anchor_id=7, begin_s=1, end_s=2)],
        )
        source = [
            ("/clock", (), 0),
            ("/sensors/uwb/range", (7,), 999999999),
            ("/sensors/uwb/range", (7,), 1000000000),
            ("/sensors/uwb/range", (4, 7), 1500000000),
            ("/sensors/uwb/range", (4, 8), 1600000000),
            ("/sensors/uwb/range", (7,), 2000000000),
        ]
        events = list(DeliverySchedule(profile, anchor_ids=lambda data: data).events(source))
        self.assertEqual(
            [e[4] for e in events],
            [
                "original",
                "original",
                "anchor_outage_drop",
                "anchor_outage_drop",
                "original",
                "original",
            ],
        )
        self.assertEqual([(e[0], e[1], e[2]) for e in events], source)
        with self.assertRaisesRegex(ValueError, "decoder"):
            list(DeliverySchedule(profile).events(source))
        profile["anchor_outages"][0]["anchor_id"] = True
        with self.assertRaisesRegex(ValueError, "ID"):
            DeliverySchedule(profile, anchor_ids=lambda data: data)

    def test_nlos_is_positive_correlated_and_independent_of_delivery_order(self):
        fault = dict(
            anchor_id=4,
            begin_s=1,
            end_s=21,
            seed=38591,
            mean_m=2.0,
            log_sigma=0.25,
            correlation_s=2.0,
            step_s=0.1,
        )
        profile = dict(schema_version="liuo_delivery_v1", begin_s=0, end_s=22, nlos_bias=fault)
        forward, reverse = DeliverySchedule(profile), DeliverySchedule(profile)
        times = [1.05 + 0.1 * i for i in range(200)]
        values = [forward.range_bias(t, [4]) for t in times]
        backward = {t: reverse.range_bias(t, [4]) for t in reversed(times)}
        self.assertTrue(all(v > 0 for v in values))
        self.assertEqual(values, [backward[t] for t in times])
        self.assertEqual(forward.range_bias(0.99, [4]), 0)
        self.assertEqual(forward.range_bias(21, [4]), 0)
        self.assertEqual(forward.range_bias(5, [3]), 0)
        self.assertNotEqual(forward.range_bias(5, [3, 4]), forward.range_bias(5, [4]))
        near = sum((a - b) ** 2 for a, b in zip(values, values[1:])) / 199
        far = sum((a - b) ** 2 for a, b in zip(values, values[40:])) / 160
        self.assertLess(near, far)
        self.assertLessEqual(forward.bias_values, 65536)
        fault["step_s"] = 0.00001
        with self.assertRaisesRegex(ValueError, "bound"):
            DeliverySchedule(profile)
