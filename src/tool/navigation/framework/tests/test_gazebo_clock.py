import unittest

from uav_planner_framework.clock_parser import ClockTextParser


class ClockTextParserTest(unittest.TestCase):
    def test_extracts_sim_time_only(self):
        parser = ClockTextParser()
        lines = [
            "sim {", "  sec: 12", "  nsec: 345", "}",
            "real {", "  sec: 99", "}",
        ]
        self.assertEqual(
            [value for line in lines if (value := parser.feed(line)) is not None],
            [(12, 345)],
        )

    def test_proto_defaults_are_zero(self):
        parser = ClockTextParser()
        self.assertIsNone(parser.feed("sim {"))
        self.assertEqual(parser.feed("}"), (0, 0))


if __name__ == "__main__":
    unittest.main()
