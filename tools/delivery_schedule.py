"""Bounded deterministic transport faults; message bytes and effective time never change."""

import heapq
import math


class DeliverySchedule:
    def __init__(self, profile=None, capacity=4096):
        self.profile = profile or {}
        allowed = {
            "schema_version",
            "begin_s",
            "end_s",
            "delay_cycle_ms",
            "duplicate_every",
            "duplicate_delay_ms",
            "expired_every",
            "expired_delay_s",
            "outages",
        }
        if set(self.profile) - allowed:
            raise ValueError("unknown delivery profile fields")
        if profile and profile.get("schema_version") != "liuo_delivery_v1":
            raise ValueError("unsupported delivery profile schema")
        for k in ["duplicate_every", "expired_every"]:
            value = self.profile.get(k, 0)
            if type(value) is not int or value < 0:
                raise ValueError("invalid " + k)
        self.begin = self.profile.get("begin_s", 0)
        self.end = self.profile.get("end_s", 0)
        self.delays = self.profile.get("delay_cycle_ms", [0])
        self.outages = self.profile.get("outages", [])
        numbers = [
            self.begin,
            self.end,
            self.profile.get("duplicate_delay_ms", 1),
            self.profile.get("expired_delay_s", 12),
        ] + self.delays
        if not self.delays or any(
            type(x) not in (int, float) or not math.isfinite(x) or x < 0 for x in numbers
        ):
            raise ValueError("delivery delays and bounds must be finite and nonnegative")
        if self.end < self.begin:
            raise ValueError("delivery window reversed")
        for interval in self.outages:
            if len(interval) != 2 or not all(
                type(x) in (int, float) and math.isfinite(x) for x in interval
            ):
                raise ValueError("invalid outage interval")
            if not self.begin <= interval[0] < interval[1] <= self.end:
                raise ValueError("outage outside delivery window")
        self.capacity = capacity
        self.peak = 0

    def events(self, records):
        pending = []
        order = 0
        count = 0
        first = None
        previous = None
        for topic, data, stamp in records:
            if previous is not None and stamp < previous:
                raise ValueError("nonmonotonic source record schedule")
            previous = stamp
            if first is None:
                first = stamp
            while pending and pending[0][0] <= stamp:
                yield heapq.heappop(pending)[2]
            elapsed = (stamp - first) * 1e-9
            delay, action, duplicate = 0, "original", False
            if topic == "/sensors/uwb/range":
                count += 1
                if self.begin <= elapsed < self.end:
                    delay = round(self.delays[(count - 1) % len(self.delays)] * 1e6)
                    action = "delayed" if delay else "original"
                    if (
                        self.profile.get("expired_every", 0)
                        and count % self.profile["expired_every"] == 0
                    ):
                        delay = round(self.profile.get("expired_delay_s", 12) * 1e9)
                        action = "expired_delay"
                    duplicate = bool(
                        self.profile.get("duplicate_every", 0)
                        and count % self.profile["duplicate_every"] == 0
                    )
                    if any(a <= elapsed < b for a, b in self.outages):
                        delay, action, duplicate = 0, "outage_drop", False
            events = [(stamp + delay, action)]
            if duplicate:
                events.append(
                    (
                        stamp + delay + round(self.profile.get("duplicate_delay_ms", 1) * 1e6),
                        "duplicate",
                    )
                )
            for delivery, kind in events:
                if len(pending) >= self.capacity:
                    raise ValueError("delivery queue limit exceeded")
                heapq.heappush(pending, (delivery, order, (topic, data, stamp, delivery, kind)))
                order += 1
                self.peak = max(self.peak, len(pending))
        while pending:
            yield heapq.heappop(pending)[2]
