"""Replay recorded depth/masks against saved measurement values; not accuracy scoring."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from uav_object_detection.measurement import locate


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();rows=[]
    for path in sorted(args.directory.glob('*.npz')):
        with np.load(path,allow_pickle=False) as data:
            m=json.loads(str(data['metadata']))
            r=np.array(m['body_rotation']);p=np.array(m['body_position'])
            re=np.array(m['extrinsic_rotation']).reshape(3,3);pe=np.array(m['extrinsic_position'])
            for d in m['diagnostics']:
                if not d.get('accepted') or 'mask_key' not in d:continue
                diagnostic={}
                point,_=locate(data['depth'],d['box'],m['intrinsics'],r,p,(re,pe),
                    {'depth_method':'gemini2_reference'},diagnostic,data[d['mask_key']])
                if diagnostic['depth_m']!=d['depth_m'] or diagnostic['representative_pixel']!=d['representative_pixel']:
                    raise AssertionError('Depth selection changed relative to saved stage2 measurement')
                fx,fy,cx,cy=m['intrinsics'];u,v=d['representative_pixel'];z=d['depth_m']
                expected=r@(re@np.array([(u-cx)*z/fx,(v-cy)*z/fy,z])+pe)+p
                direct,_=locate(data['depth'],d['box'],m['intrinsics'],r@re,r@pe+p,
                    (np.eye(3),np.zeros(3)),{'depth_method':'gemini2_reference'},mask=data[d['mask_key']])
                np.testing.assert_allclose(point,expected,rtol=1e-12,atol=1e-12)
                np.testing.assert_allclose(direct,point,rtol=1e-12,atol=1e-12)
                rows.append(dict(file=path.name,sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                    saved_projection_max_error=float(np.max(np.abs(point-expected))),
                    direct_camera_tf_max_error=float(np.max(np.abs(direct-point)))))
    if not rows:raise AssertionError('No accepted recorded mask/depth measurements found')
    report=dict(passed=True,scope='recorded stage2 measurement and composed-camera-transform numerical equivalence; no corrected pose or accuracy claim',
        count=len(rows),maximum_component_error=max(max(x['saved_projection_max_error'],x['direct_camera_tf_max_error']) for x in rows),rows=rows)
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(passed=True,count=report['count'],maximum_component_error=report['maximum_component_error'])))


if __name__=='__main__':main()
