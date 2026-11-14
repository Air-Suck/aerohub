"""Offline HUD preview shares the user-facing display configuration."""
from pathlib import Path
import tempfile
import unittest

from simulator.projectairsim.display_preview.render import load_display, render


class DisplayPreviewTest(unittest.TestCase):
    def test_loads_vehicle_and_scale_and_renders_requested_size(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "simulator.yaml"
            path.write_text("vehicles:\n  - name: Drone7\ndisplay:\n  speed_max_mps: 30\n")
            config = load_display(path)
            self.assertEqual(config, {"vehicle": "Drone7", "speed_max_mps": 30.0})
            self.assertEqual(render(config, "running", 640, 360).size, (640, 360))


if __name__ == "__main__":
    unittest.main()
