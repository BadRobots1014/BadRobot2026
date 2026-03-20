"""Regression tests for Limelight hardware implementation."""

from unittest.mock import MagicMock, patch

import pytest

from hardware.impl.limelight import Limelight


@pytest.fixture
def limelight() -> Limelight:
    with patch("hardware.impl.limelight.NetworkTableInstance") as mock_nt:
        mock_table = MagicMock()
        mock_nt.getDefault.return_value.getTable.return_value = mock_table
        # Each call to getDoubleArrayTopic returns a distinct mock so pose_sub
        # and stddevs_sub don't share the same object.
        mock_table.getDoubleArrayTopic.side_effect = lambda _: MagicMock()
        mock_nt.getDefault.return_value.addListener.return_value = MagicMock()

        ll = Limelight("limelight-test")
        return ll


def test_timestamp_uses_ms_to_seconds_conversion(limelight: Limelight) -> None:
    """Regression: arr[6] is latency in ms and must be divided (not multiplied) by 1000
    to convert to seconds before subtracting from getFPGATimestamp().

    Bug: timestamp = getFPGATimestamp() - (arr[6] * 1000.0)  ← wrong
    Fix: timestamp = getFPGATimestamp() - (arr[6] / 1000.0)  ← correct
    """
    fpga_time = 10.0
    latency_ms = 20.0  # 20 ms latency → 0.02 s offset
    expected_timestamp = fpga_time - (latency_ms / 1000.0)  # 9.98

    # [x, y, z, roll, pitch, yaw, latency_ms]
    pose_array = [1.0, 2.0, 0.0, 0.0, 0.0, 45.0, latency_ms]
    limelight.pose_sub.get.return_value = pose_array
    limelight.stddevs_sub.get.return_value = [0.0] * 12

    with patch(
        "hardware.impl.limelight.Timer.getFPGATimestamp", return_value=fpga_time
    ):
        _, timestamp, _ = limelight.get_vision_measurement()

    assert timestamp == pytest.approx(expected_timestamp), (
        f"Expected timestamp {expected_timestamp} (latency divided by 1000), "
        f"got {timestamp}. Ensure arr[6] / 1000.0, not arr[6] * 1000.0."
    )


def test_timestamp_not_far_in_past(limelight: Limelight) -> None:
    """A typical latency (< 500 ms) should produce a timestamp close to now, not thousands of seconds ago."""
    fpga_time = 100.0
    latency_ms = 100.0  # 100 ms — realistic camera latency

    pose_array = [3.0, 4.0, 0.0, 0.0, 0.0, 90.0, latency_ms]
    limelight.pose_sub.get.return_value = pose_array
    limelight.stddevs_sub.get.return_value = [0.0] * 12

    with patch(
        "hardware.impl.limelight.Timer.getFPGATimestamp", return_value=fpga_time
    ):
        _, timestamp, _ = limelight.get_vision_measurement()

    # With the bug (*1000), timestamp would be 100 - 100000 = -99900 — absurdly far in the past
    assert timestamp > 0, "Timestamp should be positive for realistic latency values"
    assert abs(fpga_time - timestamp) < 1.0, (
        "Timestamp offset should be sub-second for realistic camera latency, "
        f"but offset was {fpga_time - timestamp:.1f}s"
    )
