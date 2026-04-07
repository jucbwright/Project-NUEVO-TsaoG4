from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np


package_root = Path(__file__).resolve().parents[1]
if str(package_root) not in sys.path:
    sys.path.insert(0, str(package_root))

from robot.path_planner import APFPlanner, PurePursuitPlanner


class PurePursuitPlannerTests(unittest.TestCase):
    def test_lookahead_uses_ordered_remaining_path(self) -> None:
        planner = PurePursuitPlanner(lookahead_dist=50.0)
        path = np.array([
            [0.0, 500.0],
            [25.0, 500.0],
            [50.0, 500.0],
            [75.0, 500.0],
        ])

        lookahead = planner._lookahead_point(0.0, 480.0, path)

        self.assertEqual(tuple(lookahead), (50.0, 500.0))

    def test_compute_velocity_is_straight_on_for_aligned_target(self) -> None:
        planner = PurePursuitPlanner(lookahead_dist=100.0, max_angular=1.0)

        linear, angular = planner.compute_velocity(
            pose=(0.0, 0.0, 0.0),
            waypoints=np.array([[100.0, 0.0]]),
            max_linear=100.0,
        )

        self.assertAlmostEqual(linear, 100.0, places=3)
        self.assertAlmostEqual(angular, 0.0, places=3)

    def test_compute_velocity_slows_and_turns_for_corner(self) -> None:
        planner = PurePursuitPlanner(lookahead_dist=100.0, max_angular=1.0)

        linear, angular = planner.compute_velocity(
            pose=(0.0, 0.0, 0.0),
            waypoints=np.array([[0.0, 100.0]]),
            max_linear=100.0,
        )

        self.assertAlmostEqual(linear, 0.0, places=3)
        self.assertGreater(angular, 0.0)
        self.assertLessEqual(angular, 1.0)


class APFPlannerTests(unittest.TestCase):
    def test_compute_velocity_is_straight_on_without_obstacles(self) -> None:
        planner = APFPlanner(
            lookahead_dist=100.0,
            max_linear=100.0,
            max_angular=1.0,
        )

        linear, angular = planner.compute_velocity(
            pose=(0.0, 0.0, 0.0),
            waypoints=np.array([[100.0, 0.0]]),
            max_linear=100.0,
        )

        self.assertGreater(linear, 0.0)
        self.assertAlmostEqual(angular, 0.0, places=3)

    def test_compute_velocity_turns_away_from_front_left_obstacle(self) -> None:
        planner = APFPlanner(
            lookahead_dist=100.0,
            max_linear=100.0,
            max_angular=1.0,
            repulsion_gain=800.0,
            repulsion_range=200.0,
            obstacle_provider=lambda: [(120.0, 40.0)],
        )

        linear, angular = planner.compute_velocity(
            pose=(0.0, 0.0, 0.0),
            waypoints=np.array([[200.0, 0.0]]),
            max_linear=100.0,
        )

        self.assertGreaterEqual(linear, 0.0)
        self.assertLess(angular, 0.0)

    def test_get_obstacles_uses_provider(self) -> None:
        planner = APFPlanner(obstacle_provider=lambda: [(10.0, 20.0), (30.0, -40.0)])

        self.assertEqual(planner.get_obstacles(), [(10.0, 20.0), (30.0, -40.0)])


if __name__ == "__main__":
    unittest.main()
