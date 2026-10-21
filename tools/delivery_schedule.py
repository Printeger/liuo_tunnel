"""Bounded transport scheduling preserves bytes; optional range bias is applied explicitly by replay."""

import heapq
import math
import hashlib
import random


class DeliverySchedule:
    def __init__(self, profile=None, capacity=4096, anchor_ids=None):
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
            "anchor_outages",
            "nlos_bias",
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
        self.anchor_outages = self.profile.get("anchor_outages", [])
        self.anchor_ids = anchor_ids
        for fault in self.anchor_outages:
            if set(fault) != {"anchor_id", "begin_s", "end_s"}:
                raise ValueError("invalid anchor outage fields")
            if type(fault["anchor_id"]) is not int or fault["anchor_id"] < 0:
                raise ValueError("invalid outage anchor ID")
            a, b = fault["begin_s"], fault["end_s"]
            if not all(type(x) in (int, float) and math.isfinite(x) for x in [a, b]):
                raise ValueError("invalid anchor outage time")
            if not self.begin <= a < b <= self.end:
                raise ValueError("anchor outage outside delivery window")
        self.nlos = self.profile.get("nlos_bias")
        self.bias_series = {}
        self.bias_values = 0
        if self.nlos is not None:
            required = {
                "anchor_id",
                "begin_s",
                "end_s",
                "seed",
                "mean_m",
                "log_sigma",
                "correlation_s",
                "step_s",
            }
            if set(self.nlos) != required:
                raise ValueError("invalid NLOS bias fields")
            for key in ["anchor_id", "seed"]:
                if type(self.nlos[key]) is not int or self.nlos[key] < 0:
                    raise ValueError("invalid NLOS " + key)
            for key in required - {"anchor_id", "seed"}:
                value = self.nlos[key]
                if (
                    type(value) not in (int, float)
                    or not math.isfinite(value)
                    or value < 0
                    or (value == 0 and key != "begin_s")
                ):
                    raise ValueError("invalid NLOS " + key)
            if not self.begin <= self.nlos["begin_s"] < self.nlos["end_s"] <= self.end:
                raise ValueError("NLOS bias outside delivery window")
            self.bias_steps = math.ceil(
                (self.nlos["end_s"] - self.nlos["begin_s"]) / self.nlos["step_s"]
            )
            if not 1 <= self.bias_steps <= 10000:
                raise ValueError("NLOS time series bound exceeded")
        self.capacity = capacity
        self.peak = 0

    def range_bias(self, elapsed, anchor_ids):
        """Stationary lognormal AR(1) per physical link; effective record time only.

        Discrete values are held over step_s. Query order, delays and duplicate
        delivery never advance a random stream or change a physical sample.
        """
        fault = self.nlos
        endpoints = tuple(sorted(anchor_ids))
        if (
            fault is None
            or fault["anchor_id"] not in endpoints
            or not fault["begin_s"] <= elapsed < fault["end_s"]
        ):
            return 0.0
        if endpoints not in self.bias_series:
            if self.bias_values + self.bias_steps > 65536:
                raise ValueError("NLOS aggregate series bound exceeded")
            seed = int.from_bytes(
                hashlib.sha256(f"{fault['seed']}:{endpoints}".encode()).digest()[:8], "big"
            )
            rng = random.Random(seed)
            sigma = fault["log_sigma"]
            rho = math.exp(-fault["step_s"] / fault["correlation_s"])
            innovation = sigma * math.sqrt(1 - rho * rho)
            state = rng.gauss(0, sigma)
            values = []
            for index in range(self.bias_steps):
                if index:
                    state = rho * state + rng.gauss(0, innovation)
                value = fault["mean_m"] * math.exp(state - 0.5 * sigma * sigma)
                if not math.isfinite(value) or value < 0 or (value == 0 and key != "begin_s"):
                    raise ValueError("nonfinite NLOS bias")
                values.append(value)
            self.bias_series[endpoints] = values
            self.bias_values += len(values)
        index = min(self.bias_steps - 1, int((elapsed - fault["begin_s"]) / fault["step_s"]))
        return self.bias_series[endpoints][index]

    def events(self, records):
        if self.anchor_outages and self.anchor_ids is None:
            raise ValueError("anchor outage requires endpoint decoder")
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
                    elif self.anchor_outages:
                        endpoints = set(self.anchor_ids(data))
                        if any(
                            f["begin_s"] <= elapsed < f["end_s"] and f["anchor_id"] in endpoints
                            for f in self.anchor_outages
                        ):
                            delay, action, duplicate = 0, "anchor_outage_drop", False
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
