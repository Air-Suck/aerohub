"""Check SVG token drift only; rendering and responsibility review remain mandatory."""
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from diagram_style import TOKENS


def colors(value):
    if isinstance(value, dict):
        return {c for v in value.values() for c in colors(v)}
    return {value.lower()} if isinstance(value, str) and value.startswith('#') else set()


allowed = colors(TOKENS)
errors = []
for filename in sys.argv[1:]:
    path = Path(filename)
    source = path.read_text()
    root = ET.fromstring(source)
    unknown = set(re.findall(r'#[0-9a-fA-F]{6}\b', source.lower())) - allowed
    if unknown:
        errors.append(f'{path}: colors outside tokens: {sorted(unknown)}')
    if root.find('{http://www.w3.org/2000/svg}title') is None or root.find('{http://www.w3.org/2000/svg}desc') is None:
        errors.append(f'{path}: missing title/desc')
    groups = root.findall('.//{http://www.w3.org/2000/svg}g')
    if not any(g.get('font-family') == TOKENS['font'] for g in groups):
        errors.append(f'{path}: font family differs from reference')
if errors:
    raise SystemExit('\n'.join(errors))
if not sys.argv[1:]:
    raise SystemExit('Usage: python3 check_diagram_style.py <svg> ...')
print(f'PASS: {len(sys.argv) - 1} SVGs use shared palette/font and contain accessible metadata; visual/semantic review still required.')
