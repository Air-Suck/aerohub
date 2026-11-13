import importlib.util
import hashlib
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location('offboard_source_patches', ROOT/'tools/robopi/offboard_source_patches.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class LockedSourcePatchTest(unittest.TestCase):
    def test_patch_replay_matches_working_fix_and_is_idempotent(self):
        cases = [
            ('as2/aerostack2', 'as2-core-offboard-nonblocking.patch', ['as2_core/include/as2_core/aerial_platform.hpp']),
            ('as2/as2_platform_pixhawk', 'pixhawk-offboard-nonblocking.patch',
             ['include/as2_platform_pixhawk/pixhawk_platform.hpp', 'src/pixhawk_platform.cpp']),
        ]
        for repository, name, files in cases:
            with tempfile.TemporaryDirectory() as temporary:
                tree=Path(temporary)/'source';tree.mkdir()
                for relative in files:
                    path=tree/relative;path.parent.mkdir(parents=True,exist_ok=True)
                    path.write_bytes(subprocess.check_output(['git','-C',str(ROOT/repository),'show','HEAD:'+relative]))
                patch=ROOT/'deployment/platform/patches'/name
                self.assertEqual('applied',module.apply_source_patch(tree,patch))
                self.assertEqual('already_in_locked_source',module.apply_source_patch(tree,patch))
                for relative in files:
                    expected = {'as2/aerostack2/as2_core/include/as2_core/aerial_platform.hpp': 'f1d9809bf76f4faf554dde9807b4be526c5e8aafbabe949d4db1a4d88fc67e36', 'as2/as2_platform_pixhawk/include/as2_platform_pixhawk/pixhawk_platform.hpp': '1484748d2593349ac77fc2a1ababcb961485cc1ad3f09c1cb90bc7e19498e666', 'as2/as2_platform_pixhawk/src/pixhawk_platform.cpp': '98f61f7201987c4dfc7edb3fac408bb89765b25005b2d12fa3c9370526aea1e3'}
                    self.assertEqual(expected[repository+'/'+relative],hashlib.sha256((tree/relative).read_bytes()).hexdigest())

    def test_unexpected_source_is_not_silently_accepted(self):
        with tempfile.TemporaryDirectory() as temporary:
            tree=Path(temporary)/'source';tree.mkdir()
            with self.assertRaises(RuntimeError):
                module.apply_source_patch(tree,ROOT/'deployment/platform/patches/as2-core-offboard-nonblocking.patch')
