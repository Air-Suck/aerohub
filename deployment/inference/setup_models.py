#!/usr/bin/env python3
"""Prepare models selected by user YAML from the shared manifest."""

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path

import yaml


REPO = Path(__file__).resolve().parents[2]
MANIFEST = REPO / "src/inference/manifest.json"


def read_document(path):
    with path.open(encoding="utf-8") as stream:
        document = json.load(stream)
    if not isinstance(document, dict) or type(document.get("schema_version")) is not int or document["schema_version"] != 1:
        raise ValueError("{}: expected schema_version 1 object".format(path))
    return document


def supported_models():
    document = read_document(MANIFEST)
    if set(document) != {"schema_version", "models"} or not isinstance(document["models"], dict):
        raise ValueError("manifest must contain a models object")
    models = {}
    for model_id, entry in document["models"].items():
        if not re.fullmatch(r"[a-z][a-z0-9_-]*", model_id) or not isinstance(entry, dict):
            raise ValueError("invalid model entry: {}".format(model_id))
        if set(entry) != {"directory", "description"} or not isinstance(entry["description"], str):
            raise ValueError("{}: expected directory and description".format(model_id))
        directory = entry["directory"]
        if not isinstance(directory, str) or Path(directory).is_absolute():
            raise ValueError("{}: directory must be repository-relative".format(model_id))
        path = (REPO / directory).resolve()
        try:
            path.relative_to((REPO / "src/inference/models").resolve())
        except ValueError as error:
            raise ValueError("{}: directory must be within src/inference/models".format(model_id)) from error
        for filename in ("README.md", "setup.sh", "pyproject.toml", "uv.lock"):
            if not (path / filename).is_file():
                raise ValueError("{}: missing {}".format(model_id, filename))
        models[model_id] = path
    return models


def selected_models(config, models):
    with config.open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise ValueError("user YAML must contain an inference.models list")
    inference = document.get("inference")
    if not isinstance(inference, dict) or set(inference) != {"models"} or not isinstance(inference["models"], list):
        raise ValueError("user YAML must contain an inference.models list of model IDs")
    selected = inference["models"]
    if any(not isinstance(model_id, str) for model_id in selected):
        raise ValueError("model IDs must be strings")
    if len(selected) != len(set(selected)):
        raise ValueError("config contains duplicate model IDs")
    unknown = set(selected) - set(models)
    if unknown:
        raise ValueError("unsupported model IDs: {}".format(", ".join(sorted(unknown))))
    return [(model_id, models[model_id]) for model_id in selected]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=REPO / "config/inference.local.yaml")
    parser.add_argument("--list", action="store_true", help="list supported models without reading user config")
    parser.add_argument("--dry-run", action="store_true", help="validate config and show selection without preparing")
    parser.add_argument("--check-cuda", action="store_true", help="forward --check-cuda to selected model setup scripts")
    args = parser.parse_args(argv)
    try:
        models = supported_models()
        if args.list:
            for model_id, path in models.items():
                print("{}: {}".format(model_id, path.relative_to(REPO)))
            return 0
        if not args.config.is_file():
            raise ValueError("config missing: {}; supply --config with your user YAML containing inference.models".format(args.config))
        selection = selected_models(args.config, models)
        if not selection:
            print("No models selected; nothing to prepare.")
        for model_id, path in selection:
            print("{}: {}".format("Selected" if args.dry_run else "Preparing", model_id), flush=True)
            if not args.dry_run:
                command = ["bash", str(path / "setup.sh")]
                if args.check_cuda:
                    command.append("--check-cuda")
                subprocess.run(command, cwd=str(REPO), check=True)
        return 0
    except (OSError, ValueError, yaml.YAMLError, subprocess.CalledProcessError) as error:
        print("Model preparation failed: {}".format(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
