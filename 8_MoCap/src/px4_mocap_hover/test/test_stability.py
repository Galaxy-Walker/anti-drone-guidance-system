from px4_mocap_hover.stability import PositionStabilityMonitor


def test_position_must_remain_within_tolerance_for_required_duration():
    monitor = PositionStabilityMonitor(required_duration=10.0, tolerance=0.2)

    assert not monitor.update(0.0, (0.0, 0.0, 0.0))
    assert not monitor.update(9.9, (0.1, 0.0, 0.0))
    assert monitor.update(10.0, (0.0, 0.1, 0.0))


def test_large_position_change_restarts_stability_window():
    monitor = PositionStabilityMonitor(required_duration=10.0, tolerance=0.2)

    monitor.update(0.0, (0.0, 0.0, 0.0))
    assert not monitor.update(9.0, (0.21, 0.0, 0.0))
    assert not monitor.update(18.9, (0.22, 0.0, 0.0))
    assert monitor.update(19.0, (0.21, 0.0, 0.0))


def test_invalid_position_restarts_stability_window():
    monitor = PositionStabilityMonitor(required_duration=10.0, tolerance=0.2)

    monitor.update(0.0, (0.0, 0.0, 0.0))
    assert not monitor.update(10.0, (0.0, 0.0, 0.0), valid=False)
    assert not monitor.update(10.1, (0.0, 0.0, 0.0))
    assert monitor.update(20.1, (0.0, 0.0, 0.0))
