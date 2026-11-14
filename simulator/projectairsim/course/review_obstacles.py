#!/usr/bin/env python3
"""Validate and draw obstacle proposals without modifying a UE map."""
import argparse
import json
import math
from pathlib import Path
import yaml
from PIL import Image, ImageDraw, ImageFont


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    cfg = yaml.safe_load(args.config.read_text())
    base = yaml.safe_load((args.config.parent/cfg['base_config']).read_text())
    version=cfg['version']
    shape=cfg.get('pillar_shape','cube')
    points = cfg['pillar_centers_st_m']
    dx, dy, height = cfg['pillar_size_m']
    length, width = base['length_m'], base['clear_width_m']
    assert dx > 0 and dy > 0 and height > 0 and points
    for i, (s,t) in enumerate(points):
        assert dx/2 < s < length-dx/2
        assert abs(t)+dy/2 < width/2
        for u,v in points[:i]:
            assert abs(s-u)>=dx or abs(t-v)>=dy, 'Overlapping pillars'
    rows = sorted(set(s for s,t in points))
    gaps = []
    for s in rows:
        ts = sorted(t for u,t in points if u==s)
        gaps.extend(ts[i+1]-ts[i]-dy for i in range(len(ts)-1))
    report = {
        'version':cfg['version'], 'pillar_shape':shape, 'layout':cfg['layout'], 'status':cfg['review_status'],
        'pillar_count':len(points), 'pillar_size_m':[dx,dy,height],
        'row_pitch_m':min(b-a for a,b in zip(rows,rows[1:])),
        'minimum_same_row_pillar_gap_m':min(gaps),
        'minimum_wall_gap_m':min(width/2-abs(t)-dy/2 for s,t in points),
        'start_line_to_first_pillar_face_m':min(s for s,t in points)-dx/2,
        'last_pillar_face_to_finish_m':length-max(s for s,t in points)-dx/2,
        'minimum_pillar_surface_distance_m':min(math.hypot(max(abs(s-u)-dx,0),max(abs(t-v)-dy,0))
            for i,(s,t) in enumerate(points) for u,v in points[:i]),
        'drone_reference':cfg['drone_reference'],
        'limits':'Plan geometry only; no drone envelope, dynamic clearance or flight validation.',
    }
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/f'obstacle-plan-v{version}.json').write_text(json.dumps(report,indent=2)+'\n')
    im=Image.new('RGB',(1280,720),'#f4f5f7'); d=ImageDraw.Draw(im)
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',18)
    title=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',26)
    def text(x,y,msg,color='#17212b'):
        d.text((x,y),msg,font=font,fill=color)
    d.text((35,20),f'BlocksCourse v{version}: sparse straight pillars / PLAN REVIEW',font=title,fill='#17212b')
    text(40,85,f'12 {shape} pillars, 1 m wide, 6 m high; original grey building material')
    text(40,120,'6 staggered rows, 8 m pitch; minimum pillar gap 3 m; minimum wall gap 2.5 m')
    text(40,155,'Green start faces buildings; travel +X -> red finish; buffers stay obstacle-free')
    k=14; x0=100; y0=270; before=base['buffer_before_m']
    def pt(s,t):
        return x0+(s+before)*k,y0+(width/2-t)*k
    p=pt(-before,width/2); q=pt(length+base['buffer_after_m'],-width/2)
    d.rectangle((p,q),fill='#e0e5eb')
    for t in [-width/2,width/2]:
        d.line([pt(-before,t),pt(length+base['buffer_after_m'],t)],fill='#626973',width=7)
    for s,col in [(0,'#16803d'),(length,'#dc2626')]:
        d.line([pt(s,-width/2),pt(s,width/2)],fill=col,width=3)
    for i,(s,t) in enumerate(points,1):
        draw=d.ellipse if shape=='cylinder' else d.rectangle
        draw((pt(s-dx/2,t+dy/2),pt(s+dx/2,t-dy/2)),fill='#59616a',outline='black')
        x,y=pt(s,t); text(x-8,y+12,str(i))
    x,y=pt(0,0)
    d.ellipse((x-6,y-6,x+6,y+6),fill='#2563eb')
    # Marker intentionally has no implied vehicle dimensions before asset inspection.
    text(100,485,'Blue dot: drone at green line (position marker only; NOT scaled drone footprint)','#2563eb')
    text(100,530,'First / last pillar faces: 9.5 m from the start / finish line')
    text(100,565,'Next UE images: three fixed views + a close view with the real-size drone at start')
    text(35,675,'1280 x 720 plan | UE rendering evidence is delivered separately | No flight started')
    im.save(args.output/f'obstacle-plan-v{version}.png')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
