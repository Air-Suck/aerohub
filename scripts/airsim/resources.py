"""External path mapping, following LeUAV's user/workspace resource convention."""
import os
from pathlib import Path


def resolve_resource(name):
    import yaml

    root = Path(__file__).resolve().parents[2]
    config_home = Path(os.environ.get('XDG_CONFIG_HOME') or Path.home() / '.config')
    resources = {}
    for config in (config_home / 'aerohub/resources.yaml',
                   root / 'config/resources.local.yaml'):
        if not config.exists():
            continue
        try:
            document = yaml.safe_load(config.read_text())
        except yaml.YAMLError as exc:
            raise ValueError(f'Invalid resource YAML: {config}: {exc}') from exc
        if document is None:
            continue
        if not isinstance(document, dict) or set(document) - {'resources'}:
            raise ValueError(f'{config}: expected a resources mapping')
        entries = document.get('resources', {})
        if not isinstance(entries, dict):
            raise ValueError(f'{config}: resources must be a mapping')
        for key, entry in entries.items():
            if (not isinstance(key, str) or not key or not isinstance(entry, dict)
                    or set(entry) != {'path'} or not isinstance(entry['path'], str)
                    or not entry['path'].strip()):
                raise ValueError(f'{config}: each resource requires a nonempty path')
            path = Path(entry['path']).expanduser()
            resources[key] = (config.parent / path).resolve()
    if name not in resources:
        raise ValueError(f'Resource {name!r} is not configured; set resources.{name}.path '
                         'in config/resources.local.yaml or ~/.config/aerohub/resources.yaml')
    return resources[name]
