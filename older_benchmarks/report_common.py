#!/usr/bin/env python3
"""Shared parsing/helpers for benchmark reports."""

from __future__ import annotations

import csv
import math
import re
from dataclasses import dataclass
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]

SEQUENCE_ORDER = [
    "MH_01_easy",
    "MH_02_easy",
    "MH_03_medium",
    "MH_04_difficult",
    "MH_05_difficult",
    "V1_01_easy",
    "V1_02_medium",
    "V1_03_difficult",
    "V2_01_easy",
    "V2_02_medium",
]

ROLE_ORDER_HINT = [
    "ov2_est",
    "ov2_lc",
    "ov2_main",
    "ov2_map",
    "ov2_feed",
    "ov2slam_node_main",
    "ov2_merge",
    "ov2_vizfr",
    "ov2_vizkf",
]


@dataclass
class EvoMetrics:
    gt_count: int
    est_count: int
    matched_found: int
    matched_max: int
    ape_mean: float
    ape_rmse: float
    ape_std: float


@dataclass
class CpuMetrics:
    mean_pct: float
    std_pct: float
    max_pct: float


@dataclass
class RoleMetrics:
    name: str
    mean_pct: float
    std_pct: float
    unique_tids: int


@dataclass
class SequenceData:
    name: str
    seq_dir: Path
    evo: EvoMetrics
    cpu: CpuMetrics
    timers: dict[str, tuple[float, float]]
    roles: list[RoleMetrics]

    @property
    def fe_mean_ms(self) -> float:
        return self.timers["0.Full-Front_End"][0]

    @property
    def fe_std_ms(self) -> float:
        return self.timers["0.Full-Front_End"][1]

    @property
    def fe_hz(self) -> float:
        return 1000.0 / self.fe_mean_ms if self.fe_mean_ms > 0 else 0.0

    @property
    def matched_pct(self) -> float:
        return 100.0 * self.evo.matched_found / self.evo.matched_max if self.evo.matched_max else 0.0

    @property
    def top_roles(self) -> list[RoleMetrics]:
        return [role for role in self.roles if role.mean_pct > 0][:3]


def parse_summary_header(summary_path: Path) -> dict[str, str]:
    data: dict[str, str] = {}
    if not summary_path.exists():
        return data
    for line in summary_path.read_text(encoding="utf-8", errors="ignore").splitlines():
        if ":" not in line:
            continue
        key, value = line.split(":", 1)
        data[key.strip()] = value.strip()
    return data


def parse_evo_metrics(path: Path) -> EvoMetrics:
    text = path.read_text(encoding="utf-8", errors="ignore")

    def extract(pattern: str, cast=float) -> float | int:
        match = re.search(pattern, text, re.MULTILINE)
        if not match:
            raise ValueError(f"Could not parse {pattern!r} from {path}")
        return cast(match.group(1))

    gt_count = extract(r"Loaded\s+(\d+)\s+stamps and poses from: .*?(\.txt|gt\.tum)", int)
    est_count = extract(r"Loaded\s+(\d+)\s+stamps and poses from: .*ov2slam_kfs_traj\.txt", int)
    matched_found = extract(r"Found\s+(\d+)\s+of max\.\s+\d+\s+possible matching timestamps", int)
    matched_max = extract(r"Found\s+\d+\s+of max\.\s+(\d+)\s+possible matching timestamps", int)
    ape_mean = extract(r"^\s*mean\s+([0-9.]+)", float)
    ape_rmse = extract(r"^\s*rmse\s+([0-9.]+)", float)
    ape_std = extract(r"^\s*std\s+([0-9.]+)", float)
    return EvoMetrics(
        gt_count=gt_count,
        est_count=est_count,
        matched_found=matched_found,
        matched_max=matched_max,
        ape_mean=ape_mean,
        ape_rmse=ape_rmse,
        ape_std=ape_std,
    )


def parse_timing_csv(path: Path) -> dict[str, tuple[float, float]]:
    timings: dict[str, tuple[float, float]] = {}
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                timings[row["timer"]] = (float(row["mean_ms"]), float(row["std_ms"]))
            except Exception:
                continue
    return timings


def parse_timing_log(path: Path) -> dict[str, tuple[float, float]]:
    pattern = re.compile(r"^>>> (.+?) : ([0-9.]+) ± ([0-9.]+) ")
    timings: dict[str, tuple[float, float]] = {}
    for raw_line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        line = raw_line.strip()
        match = pattern.match(line)
        if not match:
            continue
        timings[match.group(1)] = (float(match.group(2)), float(match.group(3)))
    return timings


def parse_process_cpu_csv(path: Path) -> CpuMetrics:
    values: list[float] = []
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                values.append(float(row["process_cpu_percent"]))
            except Exception:
                continue
    if not values:
        raise ValueError(f"No process CPU samples in {path}")
    mu = sum(values) / len(values)
    variance = sum((value - mu) ** 2 for value in values) / len(values)
    return CpuMetrics(mean_pct=mu, std_pct=math.sqrt(variance), max_pct=max(values))


def parse_role_csv(path: Path) -> list[RoleMetrics]:
    roles: list[RoleMetrics] = []
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                roles.append(
                    RoleMetrics(
                        name=row["role"],
                        mean_pct=float(row["mean_cpu_all"]),
                        std_pct=float(row["std_cpu_all"]),
                        unique_tids=int(row["unique_tid_count"]),
                    )
                )
            except Exception:
                continue
    hint_order = {name: idx for idx, name in enumerate(ROLE_ORDER_HINT)}
    roles.sort(key=lambda role: (-role.mean_pct, hint_order.get(role.name, 999), role.name))
    return roles


def aggregate_role_means(sequences: list[SequenceData]) -> list[tuple[str, float, float, int]]:
    buckets: dict[str, list[float]] = {}
    counts: dict[str, int] = {}
    for seq in sequences:
        for role in seq.roles:
            buckets.setdefault(role.name, []).append(role.mean_pct)
            counts[role.name] = counts.get(role.name, 0) + 1
    rows = []
    for name, vals in buckets.items():
        mu = sum(vals) / len(vals)
        var = sum((v - mu) ** 2 for v in vals) / len(vals)
        rows.append((name, mu, math.sqrt(var), counts[name]))
    hint_order = {name: idx for idx, name in enumerate(ROLE_ORDER_HINT)}
    rows.sort(key=lambda row: (-row[1], hint_order.get(row[0], 999), row[0]))
    return rows
