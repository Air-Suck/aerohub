#!/usr/bin/env python3
"""Render measured UE actor footprints and a proposed course; never writes UE assets."""
import argparse
import json
import math
from pathlib import Path

import yaml
from PIL import Image, ImageDraw, ImageFont


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inventory', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    inventory = json.loads(args.inventory.read_text())
    cfg = yaml.safe_load(args.config.read_text())
    args.output.mkdir(parents=True, exist_ok=True)
    version = cfg['version']
    length, width = cfg['length_m'], cfg['clear_width_m']
    before, after = cfg['buffer_before_m'], cfg['buffer_after_m']
    thick = cfg['wall_thickness_m']
    yaw = math.radians(cfg['yaw_deg'])
    ox, oy, _ = cfg['origin_m']

    def world(s, t):
        return ox + s * math.cos(yaw) - t * math.sin(yaw), oy + s * math.sin(yaw) + t * math.cos(yaw)

    corners = [world(s, t) for s, t in [(-before, -width/2-thick),
               (length+after, -width/2-thick), (length+after, width/2+thick),
               (-before, width/2+thick)]]
    lo = [min(p[i] for p in corners) for i in range(2)]
    hi = [max(p[i] for p in corners) for i in range(2)]
    obstacles = [a for a in inventory['actors'] if a['label'] not in ('Ground', 'SunSky')]
    overlaps = [a['label'] for a in obstacles if all(
        a['center_m'][i]+a['extent_m'][i] > lo[i] and
        a['center_m'][i]-a['extent_m'][i] < hi[i] for i in range(2))]
    report = {'map': inventory['map'], 'proposal_version': cfg['version'],
              'course_aabb_xy_m': [lo, hi], 'overlapping_mesh_actors': overlaps,
              'interpretation': '60m between lines; 8m outside each line; 76m overall',
              'status': 'awaiting_location_review',
              'green_center_m': cfg['origin_m'],
              'red_center_m': [*world(length, 0), cfg['origin_m'][2]],
              'yaw_deg': cfg['yaw_deg'],
              'limits': 'Static mesh XY AABBs only; not collision or visual acceptance.'}
    (args.output / 'placement-review.json').write_text(json.dumps(report, indent=2)+'\n')
    font_path = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    font = ImageFont.truetype(font_path, 18)
    title = ImageFont.truetype(font_path, 26)

    def canvas(heading):
        im = Image.new('RGB', (1280, 720), '#f4f5f7')
        d = ImageDraw.Draw(im)
        d.text((35, 20), heading, font=title, fill='#17212b')
        d.text((35, 675), f'BlocksMap | proposal v{version} / empty | measured plan, NOT a UE rendered screenshot', font=font, fill='#374151')
        return im, d

    im, d = canvas('Blocks: actual actor footprints + proposed corridor')
    scale = 2.15

    def pt(x, y):
        return 220 + (x+60)*scale, 630 - (y+140)*scale

    for a in obstacles:
        x, y, _ = a['center_m']; ex, ey, _ = a['extent_m']
        p, q = pt(x-ex, y+ey), pt(x+ex, y-ey)
        d.rectangle([p, q], fill='#a1a8b0', outline='#737b84')
    d.polygon([pt(*p) for p in corners], fill='#d7ebff', outline='#1864ab', width=3)
    for s, color in [(0, '#16803d'), (length, '#dc2626')]:
        d.line([pt(*world(s, -width/2)), pt(*world(s, width/2))], fill=color, width=4)
    d.text((825, 115), 'Candidate: +X side of buildings', font=font, fill='#1864ab')
    d.text((825, 150), f'Green origin: ({ox:g}, {oy:g}, {cfg["origin_m"][2]:g}) m', font=font, fill='#17212b')
    d.text((825, 185), f'Travel yaw: {cfg["yaw_deg"]:g} degrees', font=font, fill='#17212b')
    d.text((825, 220), 'Nearest building X edge: 120 m', font=font, fill='#17212b')
    d.text((825, 255), f'Entrance buffer edge X: {lo[0]:g} m', font=font, fill='#17212b')
    d.text((825, 290), f'{lo[0]-120:g} m separation in X', font=font, fill='#17212b')
    d.text((825, 340), 'Blue: proposed 76 x 13 m footprint', font=font, fill='#1864ab')
    d.text((825, 375), 'Grey: measured existing meshes', font=font, fill='#59616a')
    d.text((825, 430), 'Green entrance faces buildings (-X).', font=font, fill='#17212b')
    d.line([(70, 600), (170, 600)], fill='black', width=2)
    d.line([(70, 600), (70, 500)], fill='black', width=2)
    d.text((155, 605), '+X', font=font, fill='black')
    d.text((40, 470), '+Y', font=font, fill='black')
    im.save(args.output / f'blocks-location-v{version}.png')

    im, d = canvas(f'Empty corridor: dimensions and buffers / proposal v{version}')
    x0, y0, k = 100, 270, 14
    x1 = x0+(before+length+after)*k
    y1 = y0+width*k
    d.rectangle((x0, y0, x1, y1), fill='#e4e8ed')
    for y in (y0-thick*k, y1):
        d.rectangle((x0, y, x1, y+thick*k), fill='#626973')
    start, finish = x0+before*k, x0+(before+length)*k
    for x, color in [(start, '#16803d'), (finish, '#dc2626')]:
        d.rectangle((x-1, y0, x+1, y1), fill=color)
    for x, text in [(x0, '8 m buffer'), (start+160, '60 m start to finish'), (finish, '8 m buffer')]:
        d.text((x, 220), text, font=font, fill='#17212b')
    d.text((100, 165), 'Building side <- GREEN entrance | travel +X -> RED finish', font=font, fill='#17212b')
    d.text((start-20, 465), 'START', font=font, fill='#16803d')
    d.text((finish-40, 465), 'FINISH', font=font, fill='#dc2626')
    d.text((400, 345), '12 m clear internal width', font=font, fill='#17212b')
    d.text((100, 110), '76 m overall | continuous walls 6 m high, 0.5 m thick | open top', font=font, fill='#17212b')
    d.text((100, 555), 'Floor surface proposed at UE Z=1.1 m (existing ground top Z=1.0 m).', font=font, fill='#17212b')
    d.text((100, 595), 'No pillars in this stage. Wall/floor geometry and collisions have not been generated.', font=font, fill='#17212b')
    im.save(args.output / f'course-plan-v{version}.png')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
