"""Shared native-SVG primitives. Layout and verified semantics belong to each entry."""
import json
from html import escape
from pathlib import Path

TOKENS = json.loads((Path(__file__).resolve().parents[1] / 'references/diagram-tokens.json').read_text())


def text(x, y, value, size=None, color=None, weight=None):
    size = TOKENS["bodySize"] if size is None else size
    attrs = f' font-weight="{weight}"' if weight else ''
    return f'<text x="{x}" y="{y}" font-size="{size}" fill="{color or TOKENS["muted"]}"{attrs}>{escape(value)}</text>'


def node(x, y, width, height, title, lines=(), kind='algorithm'):
    style = TOKENS[kind]
    return (f'<rect x="{x}" y="{y}" width="{width}" height="{height}" rx="{TOKENS["nodeRadius"]}" '
            f'fill="{style["fill"]}" stroke="{style["stroke"]}"/>\n'
            + text(x + 18, y + 34, title, TOKENS["nodeTitleSize"], style.get('title', TOKENS['foreground']), '600')
            + '\n' + '\n'.join(text(x + 18, y + 66 + i * 27, line) for i, line in enumerate(lines)))


def edge(path, data=False):
    dash = f' stroke-dasharray="{TOKENS["dash"]}"' if data else ''
    return f'<path d="{path}" fill="none" stroke="{TOKENS["line"]}" stroke-width="{TOKENS["lineWidth"]}"{dash} marker-end="url(#arrow)"/>'


def frame(x, y, width, height, label):
    return (f'<rect x="{x}" y="{y}" width="{width}" height="{height}" rx="{TOKENS["nodeRadius"]}" fill="none" stroke="{TOKENS["frame"]}"/>'
            + text(x + 20, y + 30, label, 17, weight='600'))


def svg(width, height, title, description, body):
    return f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}" role="img" aria-labelledby="title desc">
<title id="title">{escape(title)}</title><desc id="desc">{escape(description)}</desc>
<defs><marker id="arrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0 0 L10 5 L0 10z" fill="{TOKENS['line']}"/></marker></defs>
<rect width="{width}" height="{height}" rx="{TOKENS["canvasRadius"]}" fill="{TOKENS['background']}"/>
<g font-family="{TOKENS['font']}" fill="{TOKENS['foreground']}">
{text(32, 38, title, TOKENS["titleSize"], TOKENS['foreground'], '600')}
{body}
</g></svg>\n'''
