"""Configure the optional UE observation plugin and send explicit timer events."""
import json
from pathlib import Path
from urllib.request import Request, urlopen


def write_display_config(profile, run_dir, attach=False):
    if "display" not in profile:
        return None
    if attach:
        target = Path(profile["editor_project"]).parent / "Saved/Aerohub/display.json"
    else:
        target = run_dir / "display.json"
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(profile["display"], indent=2) + "\n")
    temporary.replace(target)
    return target


def request_display(profile, event="status"):
    settings = profile.get("display", {})
    port = settings.get("port", 8991)
    path = "status" if event == "status" else f"mission/{event}"
    request = Request(f"http://127.0.0.1:{port}/aerohub/{path}",
                      data=None if event == "status" else b"",
                      method="GET" if event == "status" else "POST")
    with urlopen(request, timeout=5) as response:
        return json.load(response)
