from px4_mocap_hover.sample_gate import ConsecutiveSampleGate


def test_gate_becomes_ready_after_required_consecutive_samples():
    gate = ConsecutiveSampleGate(required_samples=5, timeout=0.5)

    for index in range(4):
        assert not gate.accept(index * 0.1)
    assert gate.accept(0.4)
    assert gate.ready


def test_timeout_resets_reacquisition_count():
    gate = ConsecutiveSampleGate(required_samples=5, timeout=0.5)

    gate.accept(0.0)
    gate.accept(0.1)
    gate.accept(0.2)
    assert not gate.fresh(0.8)
    assert gate.samples == 0

    for index in range(4):
        assert not gate.accept(0.8 + index * 0.1)
    assert gate.accept(1.2)


def test_gap_between_samples_starts_a_new_sequence():
    gate = ConsecutiveSampleGate(required_samples=3, timeout=0.5)

    gate.accept(0.0)
    gate.accept(0.1)
    assert not gate.accept(0.7)
    assert gate.samples == 1
