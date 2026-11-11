"""Verify recorded source -> AS2 -> PX4 numerical mapping; no interpolation claims."""
import argparse
import bisect
import collections
import json
import re
from pathlib import Path

def main():
    parser = argparse.ArgumentParser(); parser.add_argument('output', type=Path)
    out = parser.parse_args().output
    result = json.loads((out/'result.json').read_text())
    manifest = json.loads((out/'manifest.json').read_text()) if (out/'manifest.json').exists() else {}
    requested_clock_hz = manifest.get('clock_rate_hz_requested', 50.)
    data = collections.defaultdict(list)
    for line in (out/'samples.jsonl').read_text().splitlines():
        record = json.loads(line); data[record['topic']].append(record)
    # Invalid diagnostic frames are never accepted by the Behavior. During
    # handoff their clock stamps can coincide with unrelated Hover commands.
    source = [r for r in data['external_acro/control'] if r.get('solver_ok',True)]
    by_stamp = {r['stamp_ns']:r for r in source}
    body_samples=data['fmu/in/vehicle_rates_setpoint']
    assert body_samples, 'no PX4 body-rate samples'
    begin,end=body_samples[0]['wall'],body_samples[-1]['wall']
    actuator = [r for r in data['actuator_command/thrust'] if begin<=r['wall']<=end]
    pairs = [(r,by_stamp[r['stamp_ns']]) for r in actuator if r['stamp_ns'] in by_stamp]
    ordered_source=sorted(source,key=lambda r:r['wall'])
    source_walls=[r['wall'] for r in ordered_source]
    rate_pairs=[(r,by_stamp[r['stamp_ns']]) for r in data['actuator_command/twist'] if r['stamp_ns'] in by_stamp and begin<=r['wall']<=end]
    assert len(rate_pairs)>=10, 'too few exact source/actuator rate stamp pairs'
    max_rate_error=max(max(abs(a['rate'][i]-b['rate'][i]) for i in range(3)) for a,b in rate_pairs)
    assert max_rate_error<1e-5, 'AS2 rates differ from source with same stamp'
    errors = []
    for px4 in data['fmu/in/vehicle_rates_setpoint']:
        candidates = ordered_source[bisect.bisect_right(source_walls,px4['wall']-.5):
                                    bisect.bisect_left(source_walls,px4['wall']+.5)]
        if not candidates: continue
        # This is a bounded value match, not a unique sample-ID match: PX4 uses
        # a different stamp and may repeat a previous setpoint.
        errors.append(min(max(abs(px4['rate'][i]-r['rate'][i]*(1 if i == 0 else -1))
            for i in range(3))+abs(px4['thrust'][2]+r['thrust']/15.) for r in candidates))
    matched = sum(e < 1e-5 for e in errors)
    assert len(pairs) >= 10, 'too few exact source/actuator stamp pairs'
    max_thrust_error = max(abs(a['thrust']-b['thrust']) for a,b in pairs)
    assert max_thrust_error < 1e-5, 'AS2 thrust differs from source'
    assert len(errors) >= 10 and matched == len(errors), 'PX4 rates/thrust mapping differs'
    mode = (out/'px4-control-mode-uorb.txt').read_text()
    for condition in ('flag_control_offboard_enabled: True','flag_control_rates_enabled: True',
                      'flag_control_attitude_enabled: False','flag_control_velocity_enabled: False',
                      'flag_control_position_enabled: False'):
        assert condition in mode, 'PX4 internal control mode differs: '+condition
    internal_rates = (out/'px4-rates-uorb.txt').read_text()
    yaw_values = [float(x) for x in re.findall(r'\byaw:\s*([-+\d.eE]+)',internal_rates)]
    assert any(x < -.02 for x in yaw_values), 'no external yaw command in PX4 uORB'
    frequencies = {}
    for name in ['external_acro/control','actuator_command/thrust','fmu/in/vehicle_rates_setpoint']:
        values = data[name]
        frequencies[name] = {clock+'_hz':(len(values)-1)/(values[-1][clock]-values[0][clock])
                             for clock in ('wall','sim')}
    report = {'success':bool(result['success']), 'source_to_actuator_stamp_pairs':len(pairs),
        'max_thrust_error_newtons':max_thrust_error,'source_to_actuator_rate_stamp_pairs':len(rate_pairs),
        'max_actuator_rate_error_rad_s':max_rate_error, 'px4_value_matches':matched,
        'px4_samples_compared':len(errors), 'max_combined_rate_throttle_error':max(errors),
        'px4_internal_body_rate_mode_confirmed':True, 'px4_internal_yaw_samples_rad_s':yaw_values,
        'frequency_observations':frequencies,
        'clock_rate_hz_requested':requested_clock_hz,
        'actuator_mapping_window_wall_s':[begin,end],
        'limitations':'Exact actuator mapping is scoped to the observed PX4 BODY_RATES interval; preceding Hover commands are excluded. PX4 bounded value matching permits repeated samples; no unique sequence mapping. '
                      f'Clock bridge requested rate is {requested_clock_hz:g}Hz; this is not a 100Hz closed-loop acceptance.'}
    (out/'analysis.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    if not report['success']: raise SystemExit(1)

if __name__ == '__main__': main()
