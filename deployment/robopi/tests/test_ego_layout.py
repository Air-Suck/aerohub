"""EGO candidate packaging contracts; no HDC or Docker side effects."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

path=Path(__file__).resolve().parents[1]/'deploy_lib.py'
spec=importlib.util.spec_from_file_location('ego_packager',path)
packager=importlib.util.module_from_spec(spec);spec.loader.exec_module(packager)

class EgoLayoutTests(unittest.TestCase):
    def test_private_candidate_contains_exact_packages_and_relocates(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);source=root/'source';payload=root/'payload'
            required=packager.AS2_PACKAGES|packager.EGO_PACKAGES
            for name in required:
                share=source/name/'share'/name;share.mkdir(parents=True)
                (share/'package.xml').write_text('<package><name>'+name+'</name></package>')
                (share/'path.sh').write_text('/data/aerohub/validation/old-candidate/install/'+name+'/lib')
            prefix='/data/aerohub/validation/ego-v2-pr1'
            packager.package(source,payload,'as2',prefix,include_ego=True)
            self.assertEqual({p.name for p in (payload/'as2/install').iterdir()},required)
            self.assertNotIn('uav_swarm_formation_adapter',required)
            self.assertEqual((payload/'as2/install/ego_planner/share/ego_planner/path.sh').read_text(),prefix+'/install/ego_planner/lib')
            self.assertFalse((payload/'as2/install/ego_planner/share/ego_planner/path.sh').read_text().startswith('/data/aerohub/as2/'))
    def test_missing_ego_messages_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaisesRegex(RuntimeError,'Missing packages'):
                packager.package(Path(tmp),Path(tmp)/'out','as2','/data/aerohub/validation/ego-v2-pr1',include_ego=True)
    def test_shared_prefix_or_other_algorithm_rejected(self):
        for kwargs in (dict(),dict(as2_root='/data/aerohub/validation/ego-v2-pr1',include_super=True)):
            with self.assertRaises(ValueError):packager.package(Path('/unused'),Path('/unused'),'as2',include_ego=True,**kwargs)

if __name__=='__main__':unittest.main()
