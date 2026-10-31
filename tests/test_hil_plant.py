"""Checks the Burger plant against the turtlebot3_fake_node wheel update."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from hil.plant import BRIDGE_MAX_ANGULAR_RPS, BRIDGE_MAX_LINEAR_MPS, BurgerPlant


class TestBurgerPlant(unittest.TestCase):
    def _drive(self, plant, linear, angular, seconds, dt=0.02):
        """Re-stamps the command every step, the same way the 20 Hz sender does."""
        now = 1_000.0
        steps = int(round(seconds / dt))
        for i in range(steps):
            t = now + i * dt
            plant.set_body_velocity(linear, angular, now=t)
            plant.step(dt, now=t + dt)

    def test_straight_line_matches_distance(self):
        plant = BurgerPlant()
        self._drive(plant, 0.22, 0.0, 1.0)
        self.assertAlmostEqual(plant.x, 0.22, places=3)
        self.assertAlmostEqual(plant.y, 0.0, places=3)
        self.assertAlmostEqual(plant.theta, 0.0, places=3)

    def test_spin_in_place_keeps_position(self):
        plant = BurgerPlant()
        self._drive(plant, 0.0, 1.0, 1.0)
        self.assertAlmostEqual(plant.x, 0.0, places=3)
        self.assertAlmostEqual(plant.y, 0.0, places=3)
        self.assertAlmostEqual(plant.theta, 1.0, places=3)

    def test_bridge_clamps_match_the_pi_node(self):
        plant = BurgerPlant()
        plant.set_body_velocity(5.0, -9.0, now=10.0)
        plant.step(0.02, now=10.02)
        self.assertAlmostEqual(plant.v, BRIDGE_MAX_LINEAR_MPS, places=3)
        self.assertAlmostEqual(plant.w, -BRIDGE_MAX_ANGULAR_RPS, places=3)

    def test_watchdog_stops_the_plant(self):
        plant = BurgerPlant(cmd_timeout=0.6)
        plant.set_body_velocity(0.2, 0.0, now=10.0)
        plant.step(0.2, now=10.7)
        self.assertAlmostEqual(plant.v, 0.0, places=3)


if __name__ == "__main__":
    unittest.main()
