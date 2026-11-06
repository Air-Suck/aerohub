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
            names = set.union(*packager.APPLICATIONS.values()) | {'as2_core'}
            for name in names:
                share = source/name/'share'/name
                share.mkdir(parents=True)
                (share/'package.xml').write_text('<package/>')
                file = share/'path.sh'
                file.write_text('/data/local/tmp/aerohub-as2/install/'+name+'/lib\n/work/ws/install/as2_core/lib\n')
            packager.package(source, payload, 'all')
            for group, required in {'as2':{'as2_core'}, **packager.APPLICATIONS}.items():
                for name in required:
                    text=(payload/group/'install'/name/'share'/name/'path.sh').read_text()
                    prefix=packager.AS2 if group=='as2' else '/data/aerohub/lib/'+group
                    self.assertIn(prefix+'/install/'+name+'/lib',text)
                    self.assertIn('/data/aerohub/as2/install/as2_core/lib',text)
                    self.assertNotIn('/data/local/tmp/',text)
            (target/'.deploy').mkdir(parents=True)
            bundle = target/'.deploy/overlay.tar.gz'
            def send():
                with tarfile.open(bundle,'w:gz',format=tarfile.GNU_FORMAT) as archive:
                    for p in payload.iterdir(): archive.add(p,arcname=p.name)
                installer.install(target,bundle)
            send()
            (target/'as2/install/obsolete').write_text('old')
            send()
            self.assertFalse((target/'as2/install/obsolete').exists())
            self.assertFalse((target/'.deploy/stage').exists())
            self.assertTrue((target/'lib/catora/install/mrs_formation_reshaping').is_dir())
            # A corrupt payload fails before replacing either installed component.
            (payload/'as2/install/as2_core/share/as2_core/path.sh').write_text('corrupted')
            with self.assertRaises(AssertionError): send()
            self.assertTrue((target/'as2/install/as2_core').is_dir())
            self.assertNotEqual((target/'as2/install/as2_core/share/as2_core/path.sh').read_text(),'corrupted')

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
