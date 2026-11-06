#!/usr/bin/env python3
"""Check locally loaded release images against the frozen delivery manifest."""
import argparse
import json
from pathlib import Path
import subprocess

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,default=ROOT/'deployment/releases/v0.1.0.json')
    args=parser.parse_args();manifest=json.loads(args.manifest.read_text());checked={}
    for name,entry in manifest['images'].items():
        value=json.loads(subprocess.check_output(['docker','image','inspect',entry['reference']],text=True))[0]
        platform=value['Os']+'/'+value['Architecture']
        if value['Id']!=entry['image_id'] or platform!=entry['platform']:
            raise RuntimeError(f"{entry['reference']}: got {value['Id']} {platform}; expected {entry['image_id']} {entry['platform']}")
        checked[name]={'reference':entry['reference'],'image_id':value['Id'],'platform':platform}
    print(json.dumps({'passed':True,'version':manifest['version'],'images':checked},indent=2))


if __name__=='__main__':main()
