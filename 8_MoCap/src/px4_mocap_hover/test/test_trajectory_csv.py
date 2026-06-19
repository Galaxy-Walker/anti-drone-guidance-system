import csv
from datetime import datetime
import math

import pytest

from px4_mocap_hover.trajectory_csv import (
    CSV_HEADER,
    TrajectoryCsvWriter,
    valid_mocap_ned_position,
)


FIXED_TIMESTAMP = datetime(2026, 6, 19, 14, 30, 45)


def fixed_timestamp():
    return FIXED_TIMESTAMP


def read_rows(path):
    with path.open(newline='', encoding='utf-8') as input_file:
        return list(csv.reader(input_file))


def test_valid_position_is_converted_to_absolute_ned():
    assert valid_mocap_ned_position(
        (1.0, 2.0, 3.0)) == (1.0, 3.0, -2.0)


@pytest.mark.parametrize(
    'position',
    [
        (math.nan, 0.0, 0.0),
        (0.0, math.inf, 0.0),
        (0.0, 0.0),
    ],
)
def test_invalid_position_is_rejected(position):
    assert valid_mocap_ned_position(position) is None


def test_file_is_created_on_first_sample_with_expected_rows(tmp_path):
    writer = TrajectoryCsvWriter(
        str(tmp_path),
        'target_trajectory',
        timestamp_factory=fixed_timestamp,
    )

    assert list(tmp_path.iterdir()) == []

    path = writer.record(5_000_000_000, (1.0, 3.0, -2.0))
    writer.record(5_250_000_000, (1.5, 3.5, -2.5))
    writer.close()

    assert path.name == 'target_trajectory_20260619_143045.csv'
    assert writer.sample_count == 2
    rows = read_rows(path)
    assert rows[0] == list(CSV_HEADER)
    assert rows[1] == [
        '0.000000000',
        '5000000000',
        '1.000000000',
        '3.000000000',
        '-2.000000000',
    ]
    assert rows[2] == [
        '0.250000000',
        '5250000000',
        '1.500000000',
        '3.500000000',
        '-2.500000000',
    ]


def test_invalid_samples_can_be_skipped_while_valid_samples_are_recorded(
        tmp_path):
    writer = TrajectoryCsvWriter(
        str(tmp_path),
        'target_trajectory',
        timestamp_factory=fixed_timestamp,
    )

    positions = [
        (1.0, 2.0, 3.0),
        (math.nan, 2.0, 3.0),
        (4.0, 5.0, 6.0),
    ]
    for index, position in enumerate(positions):
        ned_position = valid_mocap_ned_position(position)
        if ned_position is not None:
            writer.record(index * 100_000_000, ned_position)
    writer.close()

    assert writer.sample_count == 2
    assert len(read_rows(writer.path)) == 3


def test_existing_file_is_not_overwritten(tmp_path):
    existing_path = (
        tmp_path / 'target_trajectory_20260619_143045.csv')
    existing_path.write_text('existing data\n', encoding='utf-8')
    writer = TrajectoryCsvWriter(
        str(tmp_path),
        'target_trajectory',
        timestamp_factory=fixed_timestamp,
    )

    path = writer.record(1_000_000_000, (1.0, 2.0, 3.0))
    writer.close()

    assert existing_path.read_text(encoding='utf-8') == 'existing data\n'
    assert path.name == 'target_trajectory_20260619_143045_1.csv'


def test_nested_output_directory_is_created_lazily(tmp_path):
    output_directory = tmp_path / 'nested' / 'trajectory_logs'
    writer = TrajectoryCsvWriter(
        str(output_directory),
        'target_trajectory',
        timestamp_factory=fixed_timestamp,
    )

    assert not output_directory.exists()
    writer.record(1_000_000_000, (1.0, 2.0, 3.0))
    writer.close()

    assert output_directory.is_dir()
