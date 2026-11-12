#!/usr/bin/env python3
"""Render deterministic HUD review images without Unreal or ROS."""
from __future__ import annotations

import argparse
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont
import yaml


FONT_DIR = Path(__file__).resolve().parents[1] / "unreal/AerohubDisplay/Resources/Fonts"
FONT_MONO = FONT_DIR / "NimbusMonoPS-Regular.otf"
FONT_MONO_BOLD = FONT_DIR / "NimbusMonoPS-Bold.otf"
STATES = {
    "idle": {"speed": 0.0, "peak": 0.0, "elapsed": 0.0, "accent": "#768494"},
    "running": {"speed": 12.4, "peak": 16.8, "elapsed": 21.3, "accent": "#35e2c2"},
    "finished": {"speed": 0.0, "peak": 17.6, "elapsed": 48.7, "accent": "#62a9ff"},
}


def font(path: Path, size: int) -> ImageFont.FreeTypeFont:
    return ImageFont.truetype(str(path), size)


def text_center(draw: ImageDraw.ImageDraw, xy, value, selected_font, fill):
    box = draw.textbbox((0, 0), value, font=selected_font)
    draw.text((xy[0] - (box[2] - box[0]) / 2, xy[1] - (box[3] - box[1]) / 2),
              value, font=selected_font, fill=fill)


def mix_color(stops, progress: float):
    progress = min(1.0, max(0.0, progress))
    for index in range(len(stops) - 1):
        left, left_color = stops[index]
        right, right_color = stops[index + 1]
        if progress <= right:
            amount = (progress - left) / (right - left)
            return tuple(round(left_color[channel] +
                               (right_color[channel] - left_color[channel]) * amount)
                         for channel in range(3)) + (235,)
    return stops[-1][1] + (235,)


def draw_ring_segment(draw, cx, cy, inner_radius, outer_radius, start, end, fill):
    angles = [start + (end - start) * index / 4 for index in range(5)]
    polygon = [(cx + math.cos(math.radians(angle)) * outer_radius,
                cy + math.sin(math.radians(angle)) * outer_radius) for angle in angles]
    polygon += [(cx + math.cos(math.radians(angle)) * inner_radius,
                 cy + math.sin(math.radians(angle)) * inner_radius)
                for angle in reversed(angles)]
    draw.polygon(polygon, fill=fill)


def load_display(config_path: Path) -> dict:
    document = yaml.safe_load(config_path.read_text())
    settings = document.get("display", {})
    return {"vehicle": settings.get("vehicle") or document["vehicles"][0]["name"],
            "speed_max_mps": float(settings.get("speed_max_mps", 20))}


def background(width: int, height: int) -> Image.Image:
    image = Image.new("RGB", (width, height), "#87a9c0")
    draw = ImageDraw.Draw(image)
    horizon = int(height * 0.43)
    draw.rectangle((0, horizon, width, height), fill="#777876")
    draw.rectangle((width * .28, horizon * .58, width * .72, horizon), fill="#555957")
    for index, color in enumerate(("#ef623d", "#37c9d4", "#a7db35", "#f3dd34")):
        x = width * (.43 + (index % 2) * .075)
        y = horizon * (.62 + (index // 2) * .14)
        draw.rectangle((x, y, x + width * .065, y + height * .06), fill=color)
    draw.polygon(((width * .20, horizon), (width * .23, horizon * .66),
                  (width * .26, horizon)), fill="#18a5af")
    draw.ellipse((width * .70, horizon * .70, width * .76, horizon * .94), fill="#f08a35")
    return image


def render(config: dict, state_name: str, width: int, height: int) -> Image.Image:
    state = STATES[state_name]
    image = background(width, height).convert("RGBA")
    overlay = Image.new("RGBA", image.size)
    draw = ImageDraw.Draw(overlay)
    scale = min(width / 1280, height / 720)
    primary = "#edf4fa"
    secondary = "#91a3b4"
    panel = (28, 33, 41, 136)
    accent = state["accent"]

    top_width, top_height = int(550 * scale), int(38 * scale)
    left = (width - top_width) // 2
    top = int(14 * scale)
    border = max(1, int(2 * scale))
    draw.rectangle((left, top, left + top_width, top + top_height), fill=panel,
                   outline=(224, 232, 239, 218), width=border)
    inset = int(7 * scale)
    vehicle_x = left + int(17 * scale)
    state_x = left + int(154 * scale)
    state_width = int(146 * scale)
    timer_x = left + int(330 * scale)
    baseline = top + int(9 * scale)
    draw.text((vehicle_x, baseline), f"[ {config['vehicle'].upper()} ]",
              font=font(FONT_MONO_BOLD, max(11, int(14 * scale))), fill=primary)
    draw.rectangle((state_x, top + inset, state_x + state_width,
                    top + top_height - inset), fill=accent)
    text_center(draw, (state_x + state_width / 2, top + top_height / 2),
                state_name.upper(), font(FONT_MONO_BOLD, max(11, int(14 * scale))),
                "#15232c")
    draw.line((timer_x - int(20 * scale), top + int(8 * scale),
               timer_x - int(20 * scale), top + top_height - int(8 * scale)),
              fill=(226, 235, 241, 185), width=max(1, int(scale)))
    draw.text((timer_x, baseline), "TIME",
              font=font(FONT_MONO, max(10, int(13 * scale))), fill=secondary)
    draw.text((timer_x + int(67 * scale), baseline), f"{state['elapsed']:05.1f} s",
              font=font(FONT_MONO_BOLD, max(11, int(14 * scale))), fill=accent)

    radius = int(132 * scale)
    cx, cy = width - radius - int(24 * scale), height - radius + int(4 * scale)
    start, sweep = 150, 240
    outer_inner, outer_outer = radius - 10 * scale, radius
    fill_inner, fill_outer = radius - 35 * scale, radius - 17 * scale
    speed = min(state["speed"], config["speed_max_mps"])
    normalized = speed / config["speed_max_mps"]

    # The reference uses a transparent horseshoe gauge, with the scene visible
    # through its center and around its open lower edge.
    segments = 60
    for index in range(segments):
        segment_start = start + sweep * index / segments
        segment_end = start + sweep * (index + .78) / segments
        draw_ring_segment(draw, cx, cy, outer_inner, outer_outer,
                          segment_start, segment_end, (206, 235, 246, 220))

    gradient_stops = [
        (0.0, (19, 40, 180)),
        (.24, (20, 126, 239)),
        (.48, (21, 208, 189)),
        (.72, (112, 225, 38)),
        (1.0, (255, 210, 22)),
    ]
    active_segments = round(normalized * segments)
    for index in range(active_segments):
        segment_start = start + sweep * index / segments
        segment_end = start + sweep * (index + 1.03) / segments
        draw_ring_segment(draw, cx, cy, fill_inner, fill_outer,
                          segment_start, segment_end,
                          mix_color(gradient_stops, index / max(1, segments - 1)))

    for index in range(11):
        angle = math.radians(start + sweep * index / 10)
        direction = math.cos(angle), math.sin(angle)
        inner = (cx + direction[0] * (radius - 9 * scale),
                 cy + direction[1] * (radius - 9 * scale))
        outer = (cx + direction[0] * (radius + 3 * scale),
                 cy + direction[1] * (radius + 3 * scale))
        draw.line((inner, outer), fill=primary, width=max(1, int(2 * scale)))
        label_radius = radius + 17 * scale
        text_center(draw, (cx + direction[0] * label_radius,
                           cy + direction[1] * label_radius),
                    f"{config['speed_max_mps'] * index / 10:.0f}",
                    font(FONT_MONO_BOLD, max(9, int(10 * scale))), primary)

    pointer_angle = math.radians(start + sweep * normalized)
    pointer_color = mix_color(gradient_stops, normalized)[:3] + (255,)
    pointer_inner = (cx + math.cos(pointer_angle) * (fill_inner - 3 * scale),
                     cy + math.sin(pointer_angle) * (fill_inner - 3 * scale))
    pointer_outer = (cx + math.cos(pointer_angle) * (outer_outer + 3 * scale),
                     cy + math.sin(pointer_angle) * (outer_outer + 3 * scale))
    draw.line((pointer_inner, pointer_outer), fill=pointer_color,
              width=max(3, int(4 * scale)))

    current_color = "#22f3ec"
    peak_color = "#ff5964"
    status_color = "#5bea72" if state_name == "running" else accent
    text_center(draw, (cx, cy - 34 * scale), "CURRENT  m/s",
                font(FONT_MONO_BOLD, max(10, int(13 * scale))), current_color)
    text_center(draw, (cx, cy - 8 * scale), f"{state['speed']:.1f}",
                font(FONT_MONO_BOLD, max(23, int(31 * scale))), current_color)
    text_center(draw, (cx, cy + 23 * scale), "PEAK  m/s",
                font(FONT_MONO_BOLD, max(9, int(11 * scale))), peak_color)
    text_center(draw, (cx, cy + 45 * scale), f"{state['peak']:.1f}",
                font(FONT_MONO_BOLD, max(17, int(22 * scale))), peak_color)
    status_text = {"idle": "READY", "running": "TRACKING", "finished": "FINISHED"}[state_name]
    text_center(draw, (cx, cy + 70 * scale), status_text,
                font(FONT_MONO_BOLD, max(11, int(13 * scale))), status_color)
    return Image.alpha_composite(image, overlay).convert("RGB")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--sizes", nargs="+", default=["1280x720", "1920x1080"])
    args = parser.parse_args()
    config = load_display(args.config)
    args.output.mkdir(parents=True, exist_ok=True)
    for size in args.sizes:
        width, height = map(int, size.lower().split("x", 1))
        for state in STATES:
            render(config, state, width, height).save(args.output / f"{state}-{size}.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
