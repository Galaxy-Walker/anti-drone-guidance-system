"""桌下任务、实际遮挡区间与 Gazebo 追踪指标的离线回归测试。"""

from __future__ import annotations

import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")

ROOT = Path(__file__).resolve().parents[1]
for candidate in (ROOT, ROOT / "src", ROOT / "src/gazebosimulation2d"):
    if str(candidate) not in sys.path:
        sys.path.insert(0, str(candidate))

from matplotlib import pyplot as plt

from gazebosimulation2d.coordinates import (
    camera_pose_from_odometry,
    local_ned_to_world_enu,
    origin_enu_from_xy,
    world_enu_to_local_ned,
)
from plot_gazebo_csv import compute_gazebo_metrics, read_table_mask
from pythonsimulation2d.config import SimulationConfig
from pythonsimulation2d.plotting import _plot_distance_error, plot_scenario
from pythonsimulation2d.state import SimulationResult, TargetState
from pythonsimulation2d.target import TableOcclusionMission, target_state, target_under_table


def make_result() -> SimulationResult:
    # 出桌但尚未重获的误差 3 m 也必须计入平均值，桌下的 20 m 则排除。
    distances = np.array([1.0, 20.0, 3.0])
    target = np.array([[4.0, 0.0, 1.0], [6.0, 0.0, 1.0], [8.0, 0.0, 1.0]])
    pursuer = target.copy()
    pursuer[:, 0] -= distances
    pursuer[:, 2] = 8.0
    zeros = np.zeros((3, 3))
    return SimulationResult(
        "table_occlusion", "pn_nmpc", np.arange(3.0), pursuer, zeros.copy(),
        target, zeros.copy(), zeros.copy(), np.zeros(3), distances,
    )


class TableMissionTest(unittest.TestCase):
    def test_reference_speed_stop_and_continuity(self) -> None:
        config = SimulationConfig()
        times = np.arange(0.0, 40.0, 0.01)
        states = [target_state("table_occlusion", t, config) for t in times]
        positions = np.array([state.position[0] for state in states])
        velocities = np.array([state.velocity[0] for state in states])
        self.assertLessEqual(float(np.max(velocities)), 0.5)
        self.assertTrue(np.all(np.diff(positions) >= 0.0))
        self.assertLessEqual(float(np.max(np.diff(positions))), 0.005001)
        for t in (13.0, 14.5, 16.0):
            state = target_state("table_occlusion", t, config)
            np.testing.assert_allclose(state.position, [6.0, 0.0, 1.0])
            np.testing.assert_allclose(state.velocity, 0.0)
        np.testing.assert_allclose(states[-1].position, [12.0, 0.0, 1.0])
        np.testing.assert_allclose(states[-1].velocity, 0.0)

    def test_hover_waits_for_actual_arrival_and_restarts_if_unstable(self) -> None:
        mission = TableOcclusionMission(SimulationConfig())
        actual = TargetState(np.array([5.5, 0.0, 1.0]), np.zeros(3), np.zeros(3))
        mission.reference(20.0, actual)
        self.assertIsNone(mission.hover_start_s)
        actual.position[0] = 6.0
        mission.reference(21.0, actual)
        actual.velocity[0] = 0.2
        mission.reference(23.0, actual)
        self.assertIsNone(mission.hover_start_s)
        actual.velocity[0] = 0.0
        mission.reference(24.0, actual)
        mission.reference(26.99, actual)
        self.assertIsNone(mission.departure_s)
        mission.reference(27.0, actual)
        self.assertEqual(mission.departure_s, 27.0)
        departing = mission.reference(28.0, actual)
        self.assertGreater(departing.position[0], 6.0)
        self.assertAlmostEqual(departing.velocity[0], 0.5)
        np.testing.assert_allclose(mission.reference(45.0, actual).position, [12.0, 0.0, 1.0])

    def test_world_geometry_matches_mask_and_path_clears_legs(self) -> None:
        table = SimulationConfig().target.table
        world = ET.parse(ROOT / "worlds/table_occlusion.sdf").find("world")
        self.assertEqual(world.attrib["name"], "default")
        model = world.find("model[@name='occlusion_table']")
        pose = np.fromstring(model.findtext("pose"), sep=" ")
        np.testing.assert_allclose(pose[:2], [table.center_x, table.center_y])
        top = model.find("link[@name='top']")
        size = np.fromstring(top.findtext("collision/geometry/box/size"), sep=" ")
        top_pose = np.fromstring(top.findtext("pose"), sep=" ")
        np.testing.assert_allclose(size[:2], [table.length, table.width])
        self.assertAlmostEqual(top_pose[2] - size[2] * 0.5, table.underside_height)
        for leg in model.findall("link"):
            self.assertIsNotNone(leg.find("collision"))
            self.assertIsNotNone(leg.find("visual"))
            if leg.attrib["name"] != "top":
                leg_pose = np.fromstring(leg.findtext("pose"), sep=" ")
                self.assertGreater(abs(leg_pose[1]), 0.5)
        positions = np.array([[5, 0, 1], [7, 0, 1], [7.01, 0, 1], [6, 1.01, 1], [6, 0, 8]])
        np.testing.assert_array_equal(target_under_table(positions, table), [True, True, False, False, False])


class GazeboTrackingPlotTest(unittest.TestCase):
    def tearDown(self) -> None:
        plt.close("all")

    def test_gap_and_mean_use_same_samples(self) -> None:
        config = SimulationConfig()
        result = make_result()
        results = {"pn_nmpc": result}
        masks = {"pn_nmpc": target_under_table(result.target_position, config.target.table)}
        metrics = compute_gazebo_metrics(results, config, masks)
        self.assertAlmostEqual(metrics["pn_nmpc"]["mean_distance"], 2.0)
        with tempfile.TemporaryDirectory() as directory:
            _plot_distance_error(result.scenario, results, Path(directory), config, masks)
        curve = plt.gcf().axes[0].lines[0].get_ydata()
        np.testing.assert_allclose(curve, [1.0, np.nan, 3.0], equal_nan=True)
        np.testing.assert_allclose(result.distance, [1.0, 20.0, 3.0])

    def test_only_gazebo_metrics_plot_replaces_intercept_time(self) -> None:
        config = SimulationConfig()
        result = make_result()
        results = {"pn_nmpc": result}
        metrics = compute_gazebo_metrics(results, config, {"pn_nmpc": np.zeros(3, dtype=bool)})
        with tempfile.TemporaryDirectory() as directory:
            for tracking in (False, True):
                plot_scenario(
                    result.scenario, results, metrics, Path(directory), config,
                    show=True, include_trajectory=False, tracking_metrics=tracking,
                )
                expected = "Mean horizontal tracking error [m]" if tracking else "Intercept time [s]"
                self.assertEqual(plt.gcf().axes[1].get_title(), expected)
                plt.close("all")

    def test_old_csv_mask_and_recorded_mask(self) -> None:
        result = make_result()
        config = SimulationConfig()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "gazebo_samples.csv"
            path.write_text("time\n0\n1\n2\n")
            np.testing.assert_array_equal(read_table_mask(path, result, config), [False, True, False])
            path.write_text("time,target_under_table\n0,0\n1,0\n2,1\n")
            np.testing.assert_array_equal(read_table_mask(path, result, config), [False, False, True])
            result.scenario = "circle"
            np.testing.assert_array_equal(read_table_mask(path, result, config), [False, False, False])

    def test_no_visible_samples_reports_nan(self) -> None:
        result = make_result()
        config = SimulationConfig()
        results = {"pn_nmpc": result}
        masks = {"pn_nmpc": np.ones(3, dtype=bool)}
        metrics = compute_gazebo_metrics(results, config, masks)
        self.assertTrue(np.isnan(metrics["pn_nmpc"]["mean_distance"]))
        with tempfile.TemporaryDirectory() as directory:
            plot_scenario(
                result.scenario, results, metrics, Path(directory), config, show=True,
                include_trajectory=False, tracking_metrics=True, distance_masks=masks,
            )
        self.assertIn("N/A", [text.get_text() for text in plt.gcf().axes[1].texts])


class WorldOriginTest(unittest.TestCase):
    def test_position_roundtrip_and_camera_translation(self) -> None:
        origin = origin_enu_from_xy("[-2.0, 3.0]")
        ned = np.array([4.0, 5.0, -8.0])
        world = local_ned_to_world_enu(ned, origin)
        np.testing.assert_allclose(world, [3.0, 7.0, 8.0])
        np.testing.assert_allclose(world_enu_to_local_ned(world, origin), ned)
        local_pose = camera_pose_from_odometry(ned, [1, 0, 0, 0], [0, 0, 0.1], [0, np.pi / 2, 0])
        world_pose = camera_pose_from_odometry(ned, [1, 0, 0, 0], [0, 0, 0.1], [0, np.pi / 2, 0], origin)
        np.testing.assert_allclose(world_pose[0], local_pose[0] + origin)
        np.testing.assert_allclose(world_pose[1], local_pose[1])


if __name__ == "__main__":
    unittest.main()
