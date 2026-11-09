"""Check AS2 process-tree RSS while the PC sensor/clock publishers are running."""
import argparse
import json
import os
from pathlib import Path
import shlex
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=int, default=180)
    parser.add_argument('--warmup', type=int, default=15)
    parser.add_argument('--max-growth-mib', type=int, default=64)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not 0 <= args.warmup < args.duration or args.max_growth_mib <= 0:
        parser.error('Require 0 <= warmup < duration and positive max growth')
    remote = os.environ['ROBOPI_ROOT']
    code = '''import json, os
from pathlib import Path
root = Path(REMOTE)
state = json.loads((root/'runtime/state.json').read_text())
pid = state['pid']
assert (root/'run_as2.py').as_posix().encode() in Path(f'/proc/{pid}/cmdline').read_bytes().split(bytes([0]))
fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
assert fields[0] != 'Z' and fields[19] == state['ticks']
processes = {}
for path in Path('/proc').glob('[0-9]*/status'):
    try:
        fields = dict(line.split(':', 1) for line in path.read_text().splitlines() if ':' in line)
        processes[int(path.parent.name)] = (int(fields['PPid']), int(fields.get('VmRSS', '0 kB').split()[0]))
    except (OSError, ValueError, KeyError):
        continue
owned = {pid}
while True:
    children = {p for p, (parent, _) in processes.items() if parent in owned}
    if children <= owned:
        break
    owned.update(children)
print(json.dumps(dict(pid=pid, rss_kib=sum(processes[p][1] for p in owned if p in processes))))
'''.replace('REMOTE', repr(remote), 1)
    command = ['python3', '-c', code]
    shell = ' '.join(map(shlex.quote, command)) + '; result=$?; echo __AEROHUB_RC=$result'
    samples = []
    started = time.monotonic()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    while True:
        result = subprocess.run([os.environ['HDC_BIN'], '-t', os.environ['HDC_TARGET'],
                                 'shell', shell], check=True, capture_output=True, text=True, timeout=15)
        output, marker, rc = result.stdout.rpartition('__AEROHUB_RC=')
        if not marker or rc.strip() != '0':
            raise RuntimeError(result.stdout)
        sample = json.loads(output.strip())
        sample['elapsed_s'] = round(time.monotonic() - started, 1)
        if samples and sample['pid'] != samples[0]['pid']:
            raise RuntimeError('AS2 restarted during the memory check')
        samples.append(sample)
        args.output.write_text(json.dumps({'samples': samples}, indent=2) + '\n')
        print(json.dumps(sample), flush=True)
        steady = [sample['rss_kib'] for sample in samples if sample['elapsed_s'] >= args.warmup]
        if (steady and max(steady) - steady[0] > args.max_growth_mib*1024
                or sample['elapsed_s'] >= args.duration):
            break
        time.sleep(min(5, args.duration - sample['elapsed_s']))
    steady = [sample['rss_kib'] for sample in samples if sample['elapsed_s'] >= args.warmup]
    growth = max(steady) - steady[0]
    report = dict(samples=samples, growth_mib=round(growth/1024, 1),
                  max_growth_mib=args.max_growth_mib, passed=growth <= args.max_growth_mib*1024)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    if not report['passed']:
        raise SystemExit('AS2 process-tree memory growth exceeds the limit')


if __name__ == '__main__':
    main()
