"""Reject modified algorithm files without touching the actual checkout."""
from pathlib import Path
import shutil
import tempfile
import unittest
from adapter import verify_source


class SourceBaselineTest(unittest.TestCase):
    def test_verified_source_passes_and_changed_source_fails(self):
        model = Path(__file__).resolve().parent
        source = model.parents[4] / "src/tool/navigation/planner/yopo/upstream/YOPO"
        verify_source(source)
        with tempfile.TemporaryDirectory() as temporary:
            copied = Path(temporary)
            for line in (model / "source.sha256").read_text().splitlines():
                _, name = line.split(maxsplit=1)
                target = copied / name
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(source / name, target)
            changed = copied / "policy/yopo_network.py"
            changed.write_bytes(changed.read_bytes() + b"\n# modified model\n")
            with self.assertRaisesRegex(RuntimeError, "verified baseline"):
                verify_source(copied)


if __name__ == "__main__":
    unittest.main()
