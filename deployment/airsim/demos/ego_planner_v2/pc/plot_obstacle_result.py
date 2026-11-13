"""Standalone evidence figure from recorded real flight, no simulator access."""
import json,math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root=Path('/evidence')
report=json.loads((root/'result.json').read_text())
cloud=np.load(root/'preflight-cloud.npz')
track=np.asarray(json.loads((root/'actual-track.json').read_text()))
refs=json.loads((root/'references.json').read_text())
reference=np.asarray([r['position'] for r in refs])
points=cloud['points'];start=cloud['start'];goal=cloud['goal']
band=points[abs(points[:,2]-start[2])<=.5]
rows=[json.loads(line) for line in (root/'flight.jsonl').read_text().splitlines()]
odom=[r for r in rows if r.get('phase')=='ego' and r.get('topic')=='odom']
fig=plt.figure(figsize=(12,7),layout='constrained')
grid=fig.add_gridspec(2,2,width_ratios=[1.4,1])
ax=fig.add_subplot(grid[:,0])
ax.scatter(band[:,0],band[:,1],s=5,c='#65717c',alpha=.45,label='Observed obstacle points near flight height')
blocked=cloud['blocked'];ax.scatter(blocked[:,0],blocked[:,1],s=18,c='#bd2450',label='Points blocking the direct route')
ax.plot([start[0],goal[0]],[start[1],goal[1]],'--',color='#bd2450',lw=1.5,label='Direct route (blocked)')
ax.plot(reference[:,0],reference[:,1],color='#2766ba',lw=2,label='Executed AS2 reference')
ax.plot(track[:,1],track[:,2],color='#ed8b23',lw=1.5,label='Measured aircraft trajectory')
ax.scatter(*start[:2],marker='o',c='#187b4f',s=65,zorder=5,label='Start')
ax.scatter(*goal[:2],marker='*',c='#b32232',s=120,zorder=5,label='Goal')
extent=np.vstack([track[:,1:3],start[:2],goal[:2]])
ax.set_xlim(extent[:,0].min()-1.5,extent[:,0].max()+1.5)
ax.set_ylim(extent[:,1].min()-1.5,extent[:,1].max()+1.5)
ax.set_aspect('equal');ax.grid(alpha=.2);ax.set(xlabel='Local ENU x (m)',ylabel='Local ENU y (m)')
ax.legend(fontsize=8,loc='upper left')
t0=track[0,0]
ax2=fig.add_subplot(grid[0,1]);ax2.plot(track[:,0]-t0,track[:,3],color='#ed8b23',label='Measured altitude')
ax2.plot([(r['stamp_ns']/1e9-t0) for r in refs],reference[:,2],color='#2766ba',label='Reference altitude')
ax2.set(xlabel='Time in trajectory execution (s)',ylabel='Local ENU z (m)');ax2.grid(alpha=.2);ax2.legend(fontsize=8)
ax3=fig.add_subplot(grid[1,1]);ax3.plot([r['sim']-t0 for r in odom],[r['speed'] for r in odom],color='#ed8b23',label='Measured speed')
speed=[math.sqrt(sum(v*v for v in r['velocity'])) for r in refs]
ax3.plot([r['stamp_ns']/1e9-t0 for r in refs],speed,color='#2766ba',label='Reference speed')
ax3.axhline(.2415,ls='--',color='#555555',label='Reference speed limit')
ax3.set(xlabel='Time in trajectory execution (s)',ylabel='Speed (m/s)');ax3.grid(alpha=.2);ax3.legend(fontsize=8)
fig.suptitle('EGO / AS2 / PX4 / AirSim: single-aircraft obstacle detour\n'
             f"Result: {'PASS' if report['passed'] else 'FAIL'} | Goal error: {report.get('goal_error_m',math.nan):.3f} m | "
             f"Observed-point clearance: {report.get('minimum_live_point_clearance_m',math.nan):.3f} m")
fig.savefig(root/'trajectory.png',dpi=170)
fig.savefig(root/'trajectory.pdf')
