import importlib.util
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import yaml


spec = importlib.util.spec_from_file_location("setup_models", Path(__file__).resolve().parents[1] / "setup_models.py")
selection = importlib.util.module_from_spec(spec)
spec.loader.exec_module(selection)


class SelectionTest(unittest.TestCase):
    def setUp(self):
        self.workspace = tempfile.TemporaryDirectory()
        self.addCleanup(self.workspace.cleanup)
        self.root = Path(self.workspace.name)
        model = self.root / "src/inference/models/example"
        model.mkdir(parents=True)
        for filename in ("README.md", "setup.sh", "pyproject.toml", "uv.lock"):
            (model / filename).touch()
        manifest = self.root / "src/inference/manifest.json"
        manifest.write_text('{"schema_version":1,"models":{"example":{"directory":"src/inference/models/example","description":"test fixture"}}}')
        for name, value in (("REPO", self.root), ("MANIFEST", manifest)):
            mock = patch.object(selection, name, value)
            mock.start()
            self.addCleanup(mock.stop)

    def run_config(self, models, *flags):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "selection.yaml"
            config.write_text(yaml.safe_dump({"inference": {"models": models}, "other_component": {"enabled": True}}))
            return selection.main(["--config", str(config)] + list(flags))

    def test_invalid_selection_causes_no_partial_preparation(self):
        for models in (["example", "unsupported"], ["example", "example"], [12]):
            with patch.object(selection.subprocess, "run") as execute:
                self.assertEqual(self.run_config(models), 1)
                execute.assert_not_called()

    def test_empty_and_dry_run_never_prepare(self):
        with patch.object(selection.subprocess, "run") as execute:
            self.assertEqual(self.run_config([]), 0)
            self.assertEqual(self.run_config(["example"], "--dry-run"), 0)
            execute.assert_not_called()

    def test_only_selected_model_receives_cuda_flag(self):
        with patch.object(selection.subprocess, "run") as execute:
            self.assertEqual(self.run_config(["example"], "--check-cuda"), 0)
            execute.assert_called_once_with(
                ["bash", str(selection.REPO / "src/inference/models/example/setup.sh"), "--check-cuda"],
                cwd=str(selection.REPO), check=True,
            )


if __name__ == "__main__":
    unittest.main()
