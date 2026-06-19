"""Load and validate absolute-NED trajectory CSV files."""

import csv
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Tuple

from px4_mocap_hover.trajectory_csv import CSV_HEADER


@dataclass(frozen=True)
class TrajectorySample:
    """One validated trajectory sample from the recorder CSV."""

    elapsed_s: float
    ros_time_ns: int
    north_m: float
    east_m: float
    down_m: float


def load_trajectory_csv(path_value: str) -> Tuple[TrajectorySample, ...]:
    """Load every CSV row, rejecting malformed or non-finite data."""
    if not path_value:
        raise ValueError('trajectory_file must not be empty')

    path = Path(path_value).expanduser()
    samples = []
    with path.open(newline='', encoding='utf-8') as input_file:
        reader = csv.reader(input_file)
        try:
            header = tuple(next(reader))
        except StopIteration as error:
            raise ValueError('trajectory CSV is empty') from error

        if header != CSV_HEADER:
            raise ValueError(
                f'invalid trajectory CSV header: expected {CSV_HEADER}, '
                f'got {header}')

        for line_number, row in enumerate(reader, start=2):
            if len(row) != len(CSV_HEADER):
                raise ValueError(
                    f'trajectory CSV line {line_number} must contain '
                    f'{len(CSV_HEADER)} columns')
            try:
                elapsed_s = float(row[0])
                ros_time_ns = int(row[1])
                north_m = float(row[2])
                east_m = float(row[3])
                down_m = float(row[4])
            except ValueError as error:
                raise ValueError(
                    f'trajectory CSV line {line_number} contains an '
                    'invalid number') from error

            finite_values = (elapsed_s, north_m, east_m, down_m)
            if not all(math.isfinite(value) for value in finite_values):
                raise ValueError(
                    f'trajectory CSV line {line_number} contains a '
                    'non-finite value')

            samples.append(TrajectorySample(
                elapsed_s=elapsed_s,
                ros_time_ns=ros_time_ns,
                north_m=north_m,
                east_m=east_m,
                down_m=down_m,
            ))

    if not samples:
        raise ValueError('trajectory CSV contains no data rows')
    return tuple(samples)
