"""CSV recording helpers for motion-capture trajectories."""

import csv
from datetime import datetime
import math
from pathlib import Path
from typing import Callable, Optional, Sequence, TextIO, Tuple

from px4_mocap_hover.transforms import mocap_to_ned_position


CSV_HEADER = (
    'elapsed_s',
    'ros_time_ns',
    'north_m',
    'east_m',
    'down_m',
)
NedPosition = Tuple[float, float, float]


def valid_mocap_ned_position(
        position: Sequence[float]) -> Optional[NedPosition]:
    """Return the converted NED position, or None for invalid input."""
    if len(position) != 3:
        return None
    if not all(math.isfinite(value) for value in position):
        return None
    return mocap_to_ned_position(position)


class TrajectoryCsvWriter:
    """Create a collision-safe CSV lazily and append trajectory samples."""

    def __init__(
        self,
        output_directory: str,
        file_prefix: str,
        timestamp_factory: Callable[[], datetime] = datetime.now,
    ) -> None:
        """Configure an output file without creating it yet."""
        if not file_prefix:
            raise ValueError('file_prefix must not be empty')
        if Path(file_prefix).name != file_prefix:
            raise ValueError('file_prefix must not contain path separators')

        self.output_directory = Path(output_directory).expanduser()
        self.file_prefix = file_prefix
        self.timestamp_factory = timestamp_factory
        self.path: Optional[Path] = None
        self.sample_count = 0
        self._start_ros_time_ns: Optional[int] = None
        self._file: Optional[TextIO] = None
        self._writer: Optional[csv.writer] = None

    def record(self, ros_time_ns: int, position: Sequence[float]) -> Path:
        """Write and flush one NED sample, creating the CSV on first use."""
        if len(position) != 3:
            raise ValueError('position must contain three components')
        ned_position = tuple(float(value) for value in position)
        if not all(math.isfinite(value) for value in ned_position):
            raise ValueError('position must contain only finite values')

        if self._file is None:
            self._open_new_file()
            self._start_ros_time_ns = int(ros_time_ns)

        assert self._writer is not None
        assert self._file is not None
        assert self._start_ros_time_ns is not None
        assert self.path is not None

        elapsed_s = (int(ros_time_ns) - self._start_ros_time_ns) / 1e9
        self._writer.writerow((
            f'{elapsed_s:.9f}',
            str(int(ros_time_ns)),
            *(f'{value:.9f}' for value in ned_position),
        ))
        self._file.flush()
        self.sample_count += 1
        return self.path

    def close(self) -> None:
        """Close the output file if recording has started."""
        if self._file is not None:
            self._file.close()
            self._file = None
            self._writer = None

    def _open_new_file(self) -> None:
        self.output_directory.mkdir(parents=True, exist_ok=True)
        timestamp = self.timestamp_factory().strftime('%Y%m%d_%H%M%S')
        stem = f'{self.file_prefix}_{timestamp}'

        suffix = 0
        while True:
            collision_suffix = '' if suffix == 0 else f'_{suffix}'
            path = self.output_directory / f'{stem}{collision_suffix}.csv'
            try:
                output_file = path.open('x', newline='', encoding='utf-8')
            except FileExistsError:
                suffix += 1
                continue
            self.path = path
            self._file = output_file
            self._writer = csv.writer(output_file)
            self._writer.writerow(CSV_HEADER)
            return
