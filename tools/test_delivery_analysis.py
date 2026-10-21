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

    def test_delayed_rejected_packet_is_distinguished_from_no_packet(self):
        data = self.fixture()
        data[3][0]["state_reason"] = "UWB_REJECTED_LIO_CONTINUES:WINDOW_EXPIRED"
        self.assertEqual(analyze(*data)["result"], "PASS")
        data[3][0]["state_reason"] = "UWB_REJECTED_LIO_CONTINUES:INVENTED_REASON"
        self.assertTrue(any("absence reason" in s for s in analyze(*data)["failures"]))

    def test_anchor_outage_checks_selected_and_unrelated_links(self):
        data = self.fixture()
        data[0]["anchor_outages"] = [dict(anchor_id=7, begin_s=7, end_s=8)]
        for row in data[1]:
            row.update(a_type="0", a="0", b_type="1", b="9")

        def link(sequence, b, action):
            return dict(
                producer="1",
                session="10",
                sequence=str(sequence),
                source_elapsed_s="7.1",
                a_type="0",
                a="0",
                b_type="1",
                b=str(b),
                action=action,
            )

        data[1] += [link(4, 7, "anchor_outage_drop"), link(5, 8, "original")]
        data[3] += [
            dict(stamp=str(8 + 0.1 * i), state_reason="JOINT_LIDAR_IMU_UWB") for i in range(10)
        ]
        restored = link(6, 7, "original")
        restored["source_elapsed_s"] = "8.1"
        data[1].insert(0, restored)
        for sequence, wall in [(5, "7200000000"), (6, "8200000000")]:
            data[2].append(
                dict(
                    producer="1",
                    session="10",
                    sequence=str(sequence),
                    reason="ACCEPT_TA",
                    decision_wall_ns=wall,
                )
            )
        result = analyze(*data)
        self.assertEqual(result["result"], "PASS")
        self.assertAlmostEqual(
            result["anchor_outages"][0]["failed_anchor_first_ta_recovery_wall_s"], 0.2
        )
        last_frame = data[3].pop()
        self.assertIn("LIO publication gap during single-anchor outage", analyze(*data)["failures"])
        data[3].append(last_frame)
        restored_decision = data[2].pop()
        self.assertIn(
            "failed anchor TA never restored after single-anchor outage", analyze(*data)["failures"]
        )
        data[2].append(restored_decision)
        data[1][-1]["action"] = "anchor_outage_drop"
        self.assertIn(
            "anchor outage incorrectly dropped unrelated link", analyze(*data)["failures"]
        )
        data[1][-2]["action"] = "original"
        self.assertIn(
            "anchor outage missed selected physical measurement", analyze(*data)["failures"]
        )

    def test_nlos_model_and_estimator_value_must_match_with_variance_preserved(self):
        from delivery_schedule import DeliverySchedule

        profile = dict(
            schema_version="liuo_delivery_v1",
            begin_s=0,
            end_s=4,
            nlos_bias=dict(
                anchor_id=4,
                begin_s=1,
                end_s=3,
                seed=7,
                mean_m=2.0,
                log_sigma=0.25,
                correlation_s=2.0,
                step_s=0.1,
            ),
        )
        bias = DeliverySchedule(profile).range_bias(2.0, [4])
        row = dict(
            producer="1",
            session="10",
            sequence="1",
            action="original",
            source_elapsed_s="2",
            a_type="0",
            a="0",
            b_type="1",
            b="4",
            base_range_m="799.9",
            range_after_stress_m=str(799.9 + bias),
            injected_bias_m=str(bias),
            propagation_status="2",
            base_propagation_status="0",
            base_variance_m2=".01",
            variance_after_stress_m2=".01",
        )
        decision = dict(
            producer="1",
            session="10",
            sequence="1",
            reason="ACCEPT_TA",
            range=str(799.9 + bias),
            decision_wall_ns="2100000000",
        )
        data = [
            profile,
            [row],
            [decision],
            [],
            [dict(reset="0", fallback="0")],
            [dict(session="1", stamp="1", scheduled_wall_ns="0")],
            dict(delivery_queue_peak=1),
        ]
        self.assertEqual(analyze(*data)["result"], "PASS")
        self.assertGreater(float(row["range_after_stress_m"]), 800)
        decision["range"] = "nan"
        self.assertIn(
            "estimator NLOS measurement differs from delivered value", analyze(*data)["failures"]
        )
        decision["range"] = row["range_after_stress_m"]
        row["variance_after_stress_m2"] = ".1"
        self.assertIn("NLOS stress changed baseline variance", analyze(*data)["failures"])
        row["range_after_stress_m"] = "800"
        self.assertIn(
            "NLOS measurement transform differs from declared model", analyze(*data)["failures"]
        )
