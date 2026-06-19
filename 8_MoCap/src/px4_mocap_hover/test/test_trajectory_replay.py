import csv

import pytest

from px4_mocap_hover.trajectory_csv import CSV_HEADER
from px4_mocap_hover.trajectory_replay import load_trajectory_csv


def write_rows(path, rows, header=CSV_HEADER):
    with path.open('w', newline='', encoding='utf-8') as output_file:
        writer = csv.writer(output_file)
        writer.writerow(header)
        writer.writerows(rows)


def test_loads_all_4458_rows_in_original_order(tmp_path):
    path = tmp_path / 'trajectory.csv'
    rows = [
        (
            f'{index / 100.0:.2f}',
            str(1_000_000_000 + index),
            f'{index * 0.01:.2f}',
            f'{-index * 0.02:.2f}',
            '-0.30',
        )
        for index in range(4458)
    ]
    write_rows(path, rows)

    samples = load_trajectory_csv(str(path))

    assert len(samples) == 4458
    assert samples[0].north_m == 0.0
    assert samples[0].east_m == 0.0
    assert samples[-1].north_m == pytest.approx(44.57)
    assert samples[-1].east_m == pytest.approx(-89.14)
    assert samples[-1].ros_time_ns == 1_000_004_457


def test_rejects_missing_file(tmp_path):
    with pytest.raises(FileNotFoundError):
        load_trajectory_csv(str(tmp_path / 'missing.csv'))


@pytest.mark.parametrize(
    'header,rows,error_match',
    [
        (None, None, 'empty'),
        (
            ('elapsed_s', 'north_m'),
            [('0.0', '1.0')],
            'header',
        ),
        (CSV_HEADER, [], 'no data rows'),
        (
            CSV_HEADER,
            [('0.0', '100', '1.0', '2.0')],
            '5 columns',
        ),
        (
            CSV_HEADER,
            [('0.0', 'bad', '1.0', '2.0', '3.0')],
            'invalid number',
        ),
        (
            CSV_HEADER,
            [('0.0', '100', 'nan', '2.0', '3.0')],
            'non-finite',
        ),
    ],
)
def test_rejects_invalid_csv(
        tmp_path, header, rows, error_match):
    path = tmp_path / 'invalid.csv'
    if header is None:
        path.touch()
    else:
        write_rows(path, rows, header=header)

    with pytest.raises(ValueError, match=error_match):
        load_trajectory_csv(str(path))
