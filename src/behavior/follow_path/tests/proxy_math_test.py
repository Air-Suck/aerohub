"""Verify sampling of ROS 1 polynomial coefficients without ROS imports."""
import ast
import pathlib
import unittest

source = pathlib.Path(__file__).resolve().parents[4] / "src/tool/navigation/planner/swarm_formation/ros1_proxy/ros1_proxy.py"
tree = ast.parse(source.read_text())
functions = [node for node in tree.body if isinstance(node, ast.FunctionDef)
             and node.name in {"evaluate", "sample"}]
namespace = {}
exec(compile(ast.Module(body=functions, type_ignores=[]), str(source), "exec"), namespace)


class Seconds(float):
    def __sub__(self, other):
        return Seconds(float(self)-float(other))

    def to_sec(self):
        return float(self)


class ProxyMathTest(unittest.TestCase):
    def test_polynomial_derivatives_and_terminal_hold(self):
        class Polynomial:
            order = 3
            start_time = Seconds(10)
            duration = [2.0]
            # p(t) = t^3 + 2t^2 + 3t + 4
            coef_x = coef_y = coef_z = [1., 2., 3., 4.]
        sample = namespace["sample"]
        self.assertEqual(sample(Polynomial(), Seconds(11), .1, .1)[0], [10.]*3)
        self.assertEqual(sample(Polynomial(), Seconds(11), .1, .1, 1)[0], [10.]*3)
        self.assertEqual(sample(Polynomial(), Seconds(11), .1, .1, 2)[0], [10.]*3)
        self.assertEqual(sample(Polynomial(), Seconds(13), .1, .1)[0], [26.]*3)
        self.assertEqual(sample(Polynomial(), Seconds(13), .1, .1, 1)[0], [0.]*3)
        self.assertEqual(sample(Polynomial(), Seconds(13), .1, .1, 2)[0], [0.]*3)
        with self.assertRaisesRegex(ValueError, "native start_time"):
            sample(Polynomial(), Seconds(9), .1, .1)


if __name__ == "__main__":
    unittest.main()
