import argparse
import importlib.util
from pathlib import Path
import tempfile
import unittest

APP = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('town05_compute', APP / 'compute.py')
compute = importlib.util.module_from_spec(spec)
spec.loader.exec_module(compute)


class ComputeCommandsTest(unittest.TestCase):
    def test_local_worker_and_center_have_distinct_responsibilities(self):
        for central in (False, True):
            with tempfile.TemporaryDirectory() as root:
                args = argparse.Namespace(app=APP, output=Path(root)/'runtime', vehicle='drone0',
                    as2_launcher=Path('/prepared/run_as2.py'), central=central, registry=Path('/registry'))
                env, commands = compute.commands(args)
                self.assertEqual('follow_path', env['AS2_EXECUTION_BACKEND'])
                self.assertNotIn('AS2_BEHAVIORS', env)
                self.assertEqual(['as2', 'state-adapter'] + (['cat-ora', 'proxy', 'planner'] if central else []),
                                 [name for name, _ in commands])
                self.assertFalse(any('mission.py' in item or 'legacy_setpoint' in item
                                     for _, command in commands for item in command))

    def test_invalid_center_does_not_create_partial_runtime(self):
        with tempfile.TemporaryDirectory() as root:
            args = argparse.Namespace(app=APP, output=Path(root)/'runtime', vehicle='drone0',
                as2_launcher=Path('/prepared/run_as2.py'), central=True, registry=None)
            with self.assertRaises(ValueError):
                compute.commands(args)
            self.assertFalse(args.output.exists())
