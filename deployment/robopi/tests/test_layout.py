import hashlib
import importlib.util
import json
from pathlib import Path
import tarfile
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT/(name+'.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

packager = load('deploy_lib')
installer = load('install_overlay')

class LayoutTest(unittest.TestCase):
    def test_split_relocation_and_replacement(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            source, payload, target = root/'source', root/'payload', root/'device'
            names = set.union(*packager.APPLICATIONS.values()) | packager.AS2_PACKAGES
            for name in names:
                share = source/name/'share'/name
                share.mkdir(parents=True)
                (share/'package.xml').write_text('<package/>')
                file = share/'path.sh'
                file.write_text('/data/local/tmp/aerohub-as2/install/'+name+'/lib\n/work/ws/install/as2_core/lib\n')
            packager.package(source, payload, 'all')
            for group, required in {'as2':packager.AS2_PACKAGES, **packager.APPLICATIONS}.items():
                for name in required:
                    text=(payload/group/'install'/name/'share'/name/'path.sh').read_text()
                    prefix=packager.AS2 if group=='as2' else '/data/aerohub/lib/'+group
                    self.assertIn(prefix+'/install/'+name+'/lib',text)
                    self.assertIn('/data/aerohub/as2/install/as2_core/lib',text)
                    self.assertNotIn('/data/local/tmp/',text)
            (payload/'as2/super_app').mkdir()
            (payload/'as2/super_app/mission.py').write_text('current mission')
            (target/'.deploy').mkdir(parents=True)
            bundle = target/'.deploy/overlay.tar.gz'
            def send():
                with tarfile.open(bundle,'w:gz',format=tarfile.GNU_FORMAT) as archive:
                    for p in payload.iterdir(): archive.add(p,arcname=p.name)
                installer.install(target,bundle)
            send()
            (target/'as2/install/obsolete').write_text('old')
            (target/'as2/super_app/obsolete.py').write_text('old')
            (target/'as2/runtime/keep.log').write_text('evidence')
            send()
            self.assertFalse((target/'as2/install/obsolete').exists())
            self.assertFalse((target/'as2/super_app/obsolete.py').exists())
            self.assertEqual((target/'as2/super_app/mission.py').read_text(),'current mission')
            self.assertEqual((target/'as2/runtime/keep.log').read_text(),'evidence')
            self.assertFalse((target/'.deploy/stage').exists())
            self.assertTrue((target/'lib/catora/install/mrs_formation_reshaping').is_dir())
            # A corrupt payload fails before replacing either installed component.
            (payload/'as2/install/as2_core/share/as2_core/path.sh').write_text('corrupted')
            with self.assertRaises(AssertionError): send()
            self.assertTrue((target/'as2/install/as2_core').is_dir())
            self.assertNotEqual((target/'as2/install/as2_core/share/as2_core/path.sh').read_text(),'corrupted')

class CandidateTest(unittest.TestCase):
    def test_candidate_relocation_and_install_preserves_default(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            source, payload, device = root/'source', root/'payload', root/'device'
            prefix = '/data/aerohub/validation/super-follow-path/as2'
            for name in packager.AS2_PACKAGES:
                share = source/name/'share'/name
                share.mkdir(parents=True)
                (share/'package.xml').write_text('<package/>')
                (share/'path.sh').write_text(packager.AS2+'/install/'+name+'/lib')
            packager.package(source, payload, 'as2', prefix)
            text = (payload/'as2/install/as2_core/share/as2_core/path.sh').read_text()
            self.assertEqual(text, prefix+'/install/as2_core/lib')
            (device/'.deploy').mkdir(parents=True)
            (device/'as2').mkdir()
            (device/'as2/sentinel').write_text('existing')
            bundle = device/'.deploy/overlay.tar.gz'
            with tarfile.open(bundle,'w:gz') as archive:
                archive.add(payload/'as2', arcname='as2')
            installer.install(device, bundle, device/'validation/super-follow-path/as2')
            self.assertTrue((device/'validation/super-follow-path/as2/install/uav_follow_path_execution').is_dir())
            self.assertEqual((device/'as2/sentinel').read_text(), 'existing')
            with self.assertRaises(ValueError):
                packager.package(source, root/'bad', 'all', prefix)

    def test_super_candidate_includes_native_and_bridge_at_same_prefix(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);source=root/'source'
            for name in packager.AS2_PACKAGES | packager.SUPER_PACKAGES:
                share=source/name/'share'/name;share.mkdir(parents=True)
                (share/'package.xml').write_text('<package/>')
                (share/'path.sh').write_text('/work/ws/install/'+name+'/lib')
            prefix='/data/aerohub/validation/super-follow-path/as2'
            packager.package(source,root/'payload','as2',prefix,include_super=True)
            for name in packager.SUPER_PACKAGES:
                self.assertEqual((root/'payload/as2/install'/name/'share'/name/'path.sh').read_text(),prefix+'/install/'+name+'/lib')

    def test_incomplete_execution_install_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder)/'source'
            source.mkdir()
            with self.assertRaisesRegex(RuntimeError, 'Missing packages'):
                packager.package(source, Path(folder)/'payload', 'as2')

class PythonFreshnessTest(unittest.TestCase):
    def test_newer_install_timestamp_does_not_hide_stale_module(self):
        import os
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)/'ws'
            source=root/'install'
            for name in packager.APP_PACKAGES:
                share=source/name/'share'/name
                share.mkdir(parents=True)
                (share/'package.xml').write_text('<package/>')
            name='uav_as2_setpoint_adapter'
            namespace=root/'src'/name/name
            namespace.mkdir(parents=True)
            (namespace.parent/'setup.py').write_text('')
            (namespace/'node.py').write_text('current source')
            installed=source/name/'lib/python3.12/site-packages'/name/'node.py'
            installed.parent.mkdir(parents=True)
            installed.write_text('old install')
            os.utime(installed,(2000000000,2000000000))
            with self.assertRaisesRegex(RuntimeError,'Stale Python installation'):
                packager.package(source,Path(folder)/'payload','catora')

if __name__ == '__main__': unittest.main()
